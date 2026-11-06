#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "arm_controllers/computed_torque_controller.hpp"
#include "controller_test_utils.hpp"

using arm_controllers::ComputedTorqueController;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
namespace test = arm_controllers::test;
using namespace std::chrono_literals;

class ComputedTorqueControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    controller_ = std::make_shared<ComputedTorqueController>();
    test::initController(*controller_, "ctc_controller", {});
  }

  void configureAndActivate(const arm_control_core::Vector & q)
  {
    ASSERT_EQ(
      controller_->get_node()->configure().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    arm_.setState(q, arm_control_core::Vector::Zero(6));
    arm_.assignTo(*controller_);
    ASSERT_EQ(
      controller_->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  std::shared_ptr<ComputedTorqueController> controller_;
  test::FakeArm arm_;
};

TEST_F(ComputedTorqueControllerTest, ClaimsEffortCommandsAndPositionVelocityStates)
{
  ASSERT_EQ(
    controller_->get_node()->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  const auto commands = controller_->command_interface_configuration().names;
  const auto states = controller_->state_interface_configuration().names;
  ASSERT_EQ(commands.size(), 6u);
  ASSERT_EQ(states.size(), 12u);
  EXPECT_EQ(commands.front(), "shoulder_pan_joint/effort");
  EXPECT_EQ(states[6], "shoulder_pan_joint/velocity");
}

// At rest with no trajectory the controller holds the current configuration: the torque is
// exactly the gravity torque of the model.
TEST_F(ComputedTorqueControllerTest, HoldingAtRestCompensatesGravity)
{
  const arm_control_core::Vector q = test::homeConfiguration();
  configureAndActivate(q);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(1, 0), rclcpp::Duration(1ms)),
    controller_interface::return_type::OK);
  arm_control_core::RobotModel model(test::urdf(), test::joints(), "wrist_3_joint");
  EXPECT_TRUE(arm_.efforts().isApprox(model.gravity(q), 1e-6));
}

// End to end: FollowJointTrajectory goal -> real-time loop at 1 kHz on a simulated UR5e
// (forward dynamics) -> SUCCEEDED result with the arm at the goal.
TEST_F(ComputedTorqueControllerTest, ExecutesAFollowJointTrajectoryGoal)
{
  const arm_control_core::Vector q0 = test::homeConfiguration();
  configureAndActivate(q0);

  auto client_node = std::make_shared<rclcpp::Node>("ctc_test_client");
  auto client = rclcpp_action::create_client<FollowJointTrajectory>(
    client_node, "/ctc_controller/follow_joint_trajectory");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client_node);
  executor.add_node(controller_->get_node()->get_node_base_interface());
  std::thread spinner([&executor]() { executor.spin(); });
  ASSERT_TRUE(client->wait_for_action_server(5s));

  arm_control_core::Vector q_goal = q0;
  q_goal += (arm_control_core::Vector(6) << 0.3, -0.2, 0.2, 0.3, 0.3, 0.3).finished();
  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = test::joints();
  trajectory_msgs::msg::JointTrajectoryPoint end;
  end.positions.assign(q_goal.data(), q_goal.data() + 6);
  end.velocities.assign(6, 0.0);
  end.time_from_start = rclcpp::Duration(2s);
  goal.trajectory.points.push_back(end);
  auto goal_future = client->async_send_goal(goal);
  ASSERT_EQ(goal_future.wait_for(5s), std::future_status::ready);
  auto goal_handle = goal_future.get();
  ASSERT_TRUE(goal_handle);
  auto result_future = client->async_get_result(goal_handle);

  // Simulated plant, integrated faster than real time (the trajectory clock is the time
  // passed to update()).
  arm_control_core::RobotModel plant(test::urdf(), test::joints(), "wrist_3_joint");
  arm_control_core::Vector q = q0;
  arm_control_core::Vector v = arm_control_core::Vector::Zero(6);
  const rclcpp::Time t0 = controller_->get_node()->now();
  const double dt = 1e-3;
  for (int k = 0; k < 3500; ++k) {
    arm_.setState(q, v);
    controller_->update(
      t0 + rclcpp::Duration::from_seconds(k * dt), rclcpp::Duration::from_seconds(dt));
    const arm_control_core::Vector a = plant.forwardDynamics(q, v, arm_.efforts());
    v += dt * a;
    q += dt * v;
  }
  ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(result.result->error_code, FollowJointTrajectory::Result::SUCCESSFUL);
  EXPECT_LT((q - q_goal).cwiseAbs().maxCoeff(), 0.01);

  executor.cancel();
  spinner.join();
}

// MoveIt sends a single point at t = 0 when the arm is already at the goal: the goal is answered
// at once (no update() here), instead of waiting for the end of a trajectory that has none.
TEST_F(ComputedTorqueControllerTest, AnswersGoalsWithoutDurationAtOnce)
{
  const arm_control_core::Vector q0 = test::homeConfiguration();
  configureAndActivate(q0);

  auto client_node = std::make_shared<rclcpp::Node>("ctc_test_client");
  auto client = rclcpp_action::create_client<FollowJointTrajectory>(
    client_node, "/ctc_controller/follow_joint_trajectory");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client_node);
  executor.add_node(controller_->get_node()->get_node_base_interface());
  std::thread spinner([&executor]() { executor.spin(); });
  ASSERT_TRUE(client->wait_for_action_server(5s));

  using WrappedResult = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult;
  const auto send = [&client](const arm_control_core::Vector & q) -> WrappedResult {
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = test::joints();
    trajectory_msgs::msg::JointTrajectoryPoint point;  // time_from_start = 0
    point.positions.assign(q.data(), q.data() + q.size());
    goal.trajectory.points.push_back(point);
    auto goal_future = client->async_send_goal(goal);
    if (goal_future.wait_for(5s) != std::future_status::ready || !goal_future.get()) {
      return {};
    }
    auto result_future = client->async_get_result(goal_future.get());
    if (result_future.wait_for(5s) != std::future_status::ready) {
      return {};
    }
    return result_future.get();
  };

  const WrappedResult at_goal = send(q0);
  EXPECT_EQ(at_goal.code, rclcpp_action::ResultCode::SUCCEEDED);

  arm_control_core::Vector away = q0;
  away(0) += 0.5;
  const WrappedResult jump = send(away);
  ASSERT_EQ(jump.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(jump.result->error_code, FollowJointTrajectory::Result::INVALID_GOAL);

  executor.cancel();
  spinner.join();
}

TEST_F(ComputedTorqueControllerTest, RejectsGoalsWithUnknownJoints)
{
  configureAndActivate(test::homeConfiguration());
  auto client_node = std::make_shared<rclcpp::Node>("ctc_test_client_bad");
  auto client = rclcpp_action::create_client<FollowJointTrajectory>(
    client_node, "/ctc_controller/follow_joint_trajectory");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client_node);
  executor.add_node(controller_->get_node()->get_node_base_interface());
  std::thread spinner([&executor]() { executor.spin(); });
  ASSERT_TRUE(client->wait_for_action_server(5s));

  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = {"some_other_joint"};
  trajectory_msgs::msg::JointTrajectoryPoint p;
  p.positions = {0.0};
  p.time_from_start = rclcpp::Duration(1s);
  goal.trajectory.points.push_back(p);
  auto goal_future = client->async_send_goal(goal);
  ASSERT_EQ(goal_future.wait_for(5s), std::future_status::ready);
  EXPECT_FALSE(goal_future.get());  // rejected

  executor.cancel();
  spinner.join();
}

TEST_F(ComputedTorqueControllerTest, PdGravityModeIsSelectable)
{
  controller_ = std::make_shared<ComputedTorqueController>();
  test::initController(*controller_, "ctc_controller", {rclcpp::Parameter("mode", "pd_gravity")});
  const arm_control_core::Vector q = test::homeConfiguration();
  configureAndActivate(q);
  controller_->update(rclcpp::Time(1, 0), rclcpp::Duration(1ms));
  // At rest on the setpoint PD + gravity also outputs the gravity torque.
  arm_control_core::RobotModel model(test::urdf(), test::joints(), "wrist_3_joint");
  EXPECT_TRUE(arm_.efforts().isApprox(model.gravity(q), 1e-6));
}

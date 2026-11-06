#include <chrono>
#include <memory>

#include <gtest/gtest.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "arm_control_core/control_laws.hpp"
#include "arm_controllers/cartesian_impedance_controller.hpp"
#include "controller_test_utils.hpp"

using arm_controllers::CartesianImpedanceController;
namespace test = arm_controllers::test;
using namespace std::chrono_literals;

class CartesianImpedanceControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    controller_ = std::make_shared<CartesianImpedanceController>();
    test::initController(
      *controller_, "impedance_controller",
      {rclcpp::Parameter("end_effector_frame", "tool0"),
       rclcpp::Parameter("stiffness", std::vector<double>{500, 500, 500, 30, 30, 30})});
    ASSERT_EQ(
      controller_->get_node()->configure().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    q_ = test::homeConfiguration();
    arm_.setState(q_, arm_control_core::Vector::Zero(6));
    arm_.assignTo(*controller_);
    ASSERT_EQ(
      controller_->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  std::shared_ptr<CartesianImpedanceController> controller_;
  test::FakeArm arm_;
  arm_control_core::Vector q_;
};

TEST_F(CartesianImpedanceControllerTest, HoldsThePoseAtActivation)
{
  controller_->update(rclcpp::Time(1, 0), rclcpp::Duration(1ms));
  // Target = current pose, at rest: only gravity is compensated.
  arm_control_core::RobotModel model(test::urdf(), test::joints(), "tool0");
  EXPECT_TRUE(arm_.efforts().isApprox(model.gravity(q_), 1e-6));
}

TEST_F(CartesianImpedanceControllerTest, NewTargetProducesTheSpringTorque)
{
  arm_control_core::RobotModel model(test::urdf(), test::joints(), "tool0");
  Eigen::Isometry3d target = model.endEffectorPose(q_);
  target.translation().x() += 0.02;

  auto node = std::make_shared<rclcpp::Node>("impedance_target_publisher");
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/impedance_controller/target_pose", rclcpp::SystemDefaultsQoS());
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = "world";
  msg.pose.position.x = target.translation().x();
  msg.pose.position.y = target.translation().y();
  msg.pose.position.z = target.translation().z();
  const Eigen::Quaterniond orientation(target.linear());
  msg.pose.orientation.w = orientation.w();
  msg.pose.orientation.x = orientation.x();
  msg.pose.orientation.y = orientation.y();
  msg.pose.orientation.z = orientation.z();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(controller_->get_node()->get_node_base_interface());
  // Publish until the controller has received the target.
  const arm_control_core::Vector gravity = model.gravity(q_);
  for (int i = 0; i < 50; ++i) {
    pub->publish(msg);
    executor.spin_some(20ms);
    controller_->update(rclcpp::Time(1, 0), rclcpp::Duration(1ms));
    if (!arm_.efforts().isApprox(gravity, 1e-6)) {
      break;
    }
  }

  // Same law, computed independently.
  arm_control_core::CartesianImpedanceLaw::Parameters p;
  p.stiffness << 500, 500, 500, 30, 30, 30;
  p.damping_ratio = 1.0;
  p.nullspace_stiffness = 5.0;
  p.nullspace_damping = 0.5;
  p.max_position_error = 0.05;
  p.max_orientation_error = 0.3;
  p.posture = q_;
  const arm_control_core::CartesianImpedanceLaw law(model, p);
  const arm_control_core::Vector expected =
    law.compute(q_, arm_control_core::Vector::Zero(6), target);
  EXPECT_TRUE(arm_.efforts().isApprox(expected, 1e-6));
  EXPECT_FALSE(arm_.efforts().isApprox(gravity, 1e-3));
}

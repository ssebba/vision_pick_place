#include "arm_controllers/computed_torque_controller.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <string>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

namespace arm_controllers
{

using controller_interface::CallbackReturn;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;

CallbackReturn ComputedTorqueController::on_init()
{
  try {
    param_listener_ = std::make_shared<computed_torque_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter initialization failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration ComputedTorqueController::command_interface_configuration() const
{
  InterfaceConfiguration config{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & joint : params_.joints) {
    config.names.push_back(joint + "/effort");
  }
  return config;
}

InterfaceConfiguration ComputedTorqueController::state_interface_configuration() const
{
  // Ordered: all positions, then all velocities.
  InterfaceConfiguration config{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & joint : params_.joints) {
    config.names.push_back(joint + "/position");
  }
  for (const auto & joint : params_.joints) {
    config.names.push_back(joint + "/velocity");
  }
  return config;
}

void ComputedTorqueController::rebuildLaw()
{
  const auto gains = arm_control_core::ComputedTorqueLaw::fromBandwidth(
    model_->dof(), params_.natural_frequency, params_.damping_ratio);
  if (params_.mode == "pd_gravity") {
    // Same nominal bandwidth: gains scaled by the joint inertia at the hold configuration.
    arm_control_core::Vector inertia = arm_control_core::Vector::Ones(model_->dof());
    if (hold_.position.size() == model_->dof()) {
      inertia = model_->massMatrix(hold_.position).diagonal();
    }
    law_.emplace<arm_control_core::PdGravityLaw>(
      *model_, arm_control_core::ComputedTorqueLaw::Gains{
                 gains.kp.cwiseProduct(inertia), gains.kd.cwiseProduct(inertia)});
  } else {
    law_.emplace<arm_control_core::ComputedTorqueLaw>(*model_, gains);
  }
}

CallbackReturn ComputedTorqueController::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  params_ = param_listener_->get_params();
  try {
    // The model ends at the last controlled joint: the end-effector frame is irrelevant here.
    model_ = std::make_unique<arm_control_core::RobotModel>(
      get_robot_description(), params_.joints, params_.joints.back());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot build the robot model: %s", e.what());
    return CallbackReturn::ERROR;
  }
  effort_limits_ = model_->effortLimits() * params_.effort_limit_scale;
  rebuildLaw();

  action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
    get_node(), "~/follow_joint_trajectory",
    [this](const auto & uuid, const auto & goal) { return handleGoal(uuid, goal); },
    [this](const auto & goal_handle) { return handleCancel(goal_handle); },
    [this](const auto & goal_handle) { handleAccepted(goal_handle); });

  state_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<ControllerState>>(
    get_node(), "~/controller_state", rclcpp::SystemDefaultsQoS());
  state_msg_.joint_names = params_.joints;

  RCLCPP_INFO(
    get_node()->get_logger(), "Configured: %d joints, mode '%s', w = %.1f rad/s", model_->dof(),
    params_.mode.c_str(), params_.natural_frequency);
  return CallbackReturn::SUCCESS;
}

bool ComputedTorqueController::readState(
  arm_control_core::Vector & q, arm_control_core::Vector & v) const
{
  const auto n = params_.joints.size();
  for (std::size_t i = 0; i < n; ++i) {
    const auto position = state_interfaces_[i].get_optional<double>();
    const auto velocity = state_interfaces_[n + i].get_optional<double>();
    if (!position || !velocity) {
      return false;
    }
    q(static_cast<Eigen::Index>(i)) = *position;
    v(static_cast<Eigen::Index>(i)) = *velocity;
  }
  return true;
}

void ComputedTorqueController::holdPosition(const arm_control_core::Vector & q)
{
  hold_.position = q;
  hold_.velocity = arm_control_core::Vector::Zero(q.size());
  hold_.acceleration = arm_control_core::Vector::Zero(q.size());
}

CallbackReturn ComputedTorqueController::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  const auto n = static_cast<Eigen::Index>(params_.joints.size());
  arm_control_core::Vector q(n);
  arm_control_core::Vector v(n);
  if (!readState(q, v)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot read the joint state");
    return CallbackReturn::ERROR;
  }
  holdPosition(q);
  rebuildLaw();
  active_goal_.set(nullptr);
  current_goal_.reset();
  current_finished_ = false;
  measured_state_.set(arm_control_core::JointState{q, v, arm_control_core::Vector::Zero(n)});
  // Goal results are sent from a non real-time timer.
  goal_handle_timer_ = get_node()->create_wall_timer(std::chrono::milliseconds(50), [this]() {
    std::shared_ptr<ActiveGoal> goal;
    active_goal_.get([&goal](const std::shared_ptr<ActiveGoal> & g) { goal = g; });
    if (goal && goal->goal_handle) {
      goal->goal_handle->runNonRealtime();
    }
  });
  return CallbackReturn::SUCCESS;
}

CallbackReturn ComputedTorqueController::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  std::shared_ptr<ActiveGoal> goal;
  active_goal_.get([&goal](const std::shared_ptr<ActiveGoal> & g) { goal = g; });
  if (goal && goal->goal_handle) {
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_string = "controller deactivated";
    goal->goal_handle->setAborted(result);
    goal->goal_handle->runNonRealtime();
  }
  active_goal_.set(nullptr);
  goal_handle_timer_.reset();
  for (auto & command : command_interfaces_) {
    (void)command.set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

std::vector<std::size_t> ComputedTorqueController::jointMapping(
  const std::vector<std::string> & goal_joints) const
{
  std::vector<std::size_t> mapping;
  for (const auto & joint : params_.joints) {
    const auto it = std::find(goal_joints.begin(), goal_joints.end(), joint);
    if (it == goal_joints.end()) {
      return {};
    }
    mapping.push_back(static_cast<std::size_t>(it - goal_joints.begin()));
  }
  return mapping;
}

rclcpp_action::GoalResponse ComputedTorqueController::handleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  const std::shared_ptr<const FollowJointTrajectory::Goal> & goal)
{
  if (get_lifecycle_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(get_node()->get_logger(), "Goal rejected: controller not active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->trajectory.points.empty() || jointMapping(goal->trajectory.joint_names).empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "Goal rejected: empty trajectory or missing joints");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ComputedTorqueController::handleCancel(
  const std::shared_ptr<GoalHandle> & goal_handle)
{
  std::shared_ptr<ActiveGoal> goal;
  active_goal_.get([&goal](const std::shared_ptr<ActiveGoal> & g) { goal = g; });
  if (goal && goal->goal_handle && goal->goal_handle->gh_ == goal_handle) {
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    goal->goal_handle->setCanceled(result);
    // Stop where we are: the real-time loop sees no trajectory and holds the measured state.
    active_goal_.set(nullptr);
    goal->goal_handle->runNonRealtime();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ComputedTorqueController::handleAccepted(const std::shared_ptr<GoalHandle> & goal_handle)
{
  const auto & msg = goal_handle->get_goal()->trajectory;
  const std::vector<std::size_t> mapping = jointMapping(msg.joint_names);
  const auto n = static_cast<Eigen::Index>(params_.joints.size());

  std::vector<arm_control_core::TrajectoryPoint> points;
  points.reserve(msg.points.size());
  for (const auto & p : msg.points) {
    arm_control_core::TrajectoryPoint point;
    point.time = rclcpp::Duration(p.time_from_start).seconds();
    point.position.resize(n);
    const bool has_vel = p.velocities.size() == msg.joint_names.size();
    const bool has_acc = p.accelerations.size() == msg.joint_names.size();
    if (has_vel) {
      point.velocity.resize(n);
    }
    if (has_acc) {
      point.acceleration.resize(n);
    }
    for (Eigen::Index i = 0; i < n; ++i) {
      const std::size_t j = mapping[static_cast<std::size_t>(i)];
      point.position(i) = p.positions[j];
      if (has_vel) {
        point.velocity(i) = p.velocities[j];
      }
      if (has_acc) {
        point.acceleration(i) = p.accelerations[j];
      }
    }
    points.push_back(std::move(point));
  }

  auto active = std::make_shared<ActiveGoal>();
  std::shared_ptr<GoalHandle> handle = goal_handle;  // RealtimeServerGoalHandle takes a mutable ref
  auto rt_goal = std::make_shared<RealtimeGoalHandle>(handle);
  // Answers the goal at once, without the real-time loop (a goal can only end once executing).
  const auto finish = [&rt_goal](int32_t code, const std::string & message) {
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = code;
    result->error_string = message;
    rt_goal->execute();
    if (code == FollowJointTrajectory::Result::SUCCESSFUL) {
      rt_goal->setSucceeded(result);
    } else {
      rt_goal->setAborted(result);
    }
    rt_goal->runNonRealtime();
  };
  arm_control_core::JointState start;
  measured_state_.get([&start](const arm_control_core::JointState & s) { start = s; });
  try {
    active->trajectory = arm_control_core::JointTrajectory(start, points);
  } catch (const std::invalid_argument & e) {
    finish(FollowJointTrajectory::Result::INVALID_GOAL, e.what());
    return;
  }
  active->start_time = get_node()->now();
  active->goal_handle = rt_goal;

  // Preempt the previous goal, if any.
  std::shared_ptr<ActiveGoal> previous;
  active_goal_.get([&previous](const std::shared_ptr<ActiveGoal> & g) { previous = g; });
  if (previous && previous->goal_handle) {
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_string = "preempted by a new goal";
    previous->goal_handle->setAborted(result);
    previous->goal_handle->runNonRealtime();
  }

  // Every point at t = 0 (MoveIt sends this when the arm is already at the goal): nothing to
  // track. Succeed if the arm is there, refuse a jump otherwise; either way hold where we are.
  if (active->trajectory.empty()) {
    active_goal_.set(nullptr);
    const double error = (points.back().position - start.position).cwiseAbs().maxCoeff();
    if (error <= params_.goal_tolerance) {
      finish(FollowJointTrajectory::Result::SUCCESSFUL, "");
    } else {
      finish(FollowJointTrajectory::Result::INVALID_GOAL, "no duration, away from the arm");
    }
    return;
  }
  rt_goal->execute();
  // Smart pointers can only be set through a function in RealtimeThreadSafeBox.
  active_goal_.set([&active](std::shared_ptr<ActiveGoal> & g) { g = active; });
}

return_type ComputedTorqueController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Runtime parameter changes (gains, mode) without blocking.
  if (param_listener_->try_get_params(params_)) {
    rebuildLaw();
  }

  const auto n = static_cast<Eigen::Index>(params_.joints.size());
  arm_control_core::Vector q(n);
  arm_control_core::Vector v(n);
  if (!readState(q, v)) {
    return return_type::OK;  // keep the previous command for one cycle
  }
  // Non-blocking: skipped if the action server is reading it right now.
  measured_state_.try_set([&](arm_control_core::JointState & s) {
    s.position = q;
    s.velocity = v;
  });

  // A new goal (or a cancel) from the action server?
  // Non-blocking read: if the action server holds the lock, keep the current goal this cycle.
  std::shared_ptr<ActiveGoal> latest = current_goal_;
  active_goal_.try_get([&latest](const std::shared_ptr<ActiveGoal> & g) { latest = g; });
  if (latest != current_goal_) {
    if (!latest && current_goal_ && !current_finished_) {
      holdPosition(q);  // canceled: stop where we are
    }
    current_goal_ = latest;
    current_finished_ = false;
  }

  arm_control_core::JointState desired = hold_;
  if (current_goal_ && !current_finished_ && !current_goal_->trajectory.empty()) {
    const double t = (time - current_goal_->start_time).seconds();
    desired = current_goal_->trajectory.sample(t);
    const double duration = current_goal_->trajectory.duration();
    if (t >= duration) {
      const double error = (desired.position - q).cwiseAbs().maxCoeff();
      const bool reached = error <= params_.goal_tolerance;
      if (reached || t > duration + params_.goal_time) {
        auto & goal_handle = *current_goal_->goal_handle;
        auto result = goal_handle.preallocated_result_;
        if (reached) {
          result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
          result->error_string.clear();
          goal_handle.setSucceeded(result);
        } else {
          result->error_code = FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
          result->error_string = "goal tolerance violated";
          goal_handle.setAborted(result);
        }
        holdPosition(desired.position);
        current_finished_ = true;
      }
    }
  }

  arm_control_core::Vector tau = std::visit(
    [&](const auto & law) -> arm_control_core::Vector {
      if constexpr (std::is_same_v<std::decay_t<decltype(law)>, std::monostate>) {
        return arm_control_core::Vector::Zero(n);
      } else {
        return law.compute(q, v, desired);
      }
    },
    law_);
  tau = arm_control_core::saturate(tau, effort_limits_);
  for (Eigen::Index i = 0; i < n; ++i) {
    (void)command_interfaces_[static_cast<std::size_t>(i)].set_value(tau(i));
  }

  // Controller state at 100 Hz (every 5th cycle at 500 Hz) for monitoring and benchmarks.
  if (state_publisher_ && ++publish_counter_ >= 5) {
    publish_counter_ = 0;
    auto & msg = state_msg_;
    msg.header.stamp = time;
    msg.reference.positions.assign(desired.position.data(), desired.position.data() + n);
    msg.reference.velocities.assign(desired.velocity.data(), desired.velocity.data() + n);
    msg.feedback.positions.assign(q.data(), q.data() + n);
    msg.feedback.velocities.assign(v.data(), v.data() + n);
    msg.error.positions.resize(static_cast<std::size_t>(n));
    for (Eigen::Index i = 0; i < n; ++i) {
      msg.error.positions[static_cast<std::size_t>(i)] = desired.position(i) - q(i);
    }
    msg.output.effort.assign(tau.data(), tau.data() + n);
    (void)state_publisher_->try_publish(msg);
  }
  return return_type::OK;
}

}  // namespace arm_controllers

PLUGINLIB_EXPORT_CLASS(
  arm_controllers::ComputedTorqueController, controller_interface::ControllerInterface)

#include "arm_controllers/cartesian_impedance_controller.hpp"

#include <string>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

namespace arm_controllers
{

using controller_interface::CallbackReturn;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;

CallbackReturn CartesianImpedanceController::on_init()
{
  try {
    param_listener_ = std::make_shared<cartesian_impedance_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter initialization failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration CartesianImpedanceController::command_interface_configuration() const
{
  InterfaceConfiguration config{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & joint : params_.joints) {
    config.names.push_back(joint + "/effort");
  }
  return config;
}

InterfaceConfiguration CartesianImpedanceController::state_interface_configuration() const
{
  InterfaceConfiguration config{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & joint : params_.joints) {
    config.names.push_back(joint + "/position");
  }
  for (const auto & joint : params_.joints) {
    config.names.push_back(joint + "/velocity");
  }
  return config;
}

arm_control_core::CartesianImpedanceLaw::Parameters CartesianImpedanceController::lawParameters(
  const arm_control_core::Vector & posture) const
{
  arm_control_core::CartesianImpedanceLaw::Parameters p;
  for (int i = 0; i < 6; ++i) {
    p.stiffness(i) = params_.stiffness[static_cast<std::size_t>(i)];
  }
  p.damping_ratio = params_.damping_ratio;
  p.nullspace_stiffness = params_.nullspace_stiffness;
  p.nullspace_damping = params_.nullspace_damping;
  p.max_position_error = params_.max_position_error;
  p.max_orientation_error = params_.max_orientation_error;
  p.posture = posture;
  return p;
}

CallbackReturn CartesianImpedanceController::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  params_ = param_listener_->get_params();
  try {
    model_ = std::make_unique<arm_control_core::RobotModel>(
      get_robot_description(), params_.joints, params_.end_effector_frame);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot build the robot model: %s", e.what());
    return CallbackReturn::ERROR;
  }
  effort_limits_ = model_->effortLimits() * params_.effort_limit_scale;

  target_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/target_pose", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::PoseStamped & msg) {
      Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
      target.translation() =
        Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
      target.linear() = Eigen::Quaterniond(
                          msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
                          msg.pose.orientation.z)
                          .normalized()
                          .toRotationMatrix();
      target_.set(target);
    });
  pose_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
      get_node(), "~/current_pose", rclcpp::SystemDefaultsQoS());
  // Poses are expressed in the root frame of the URDF (the work cell's "world" link).
  pose_msg_.header.frame_id = "world";
  RCLCPP_INFO(
    get_node()->get_logger(), "Configured: %d joints, end effector '%s'", model_->dof(),
    params_.end_effector_frame.c_str());
  return CallbackReturn::SUCCESS;
}

bool CartesianImpedanceController::readState(
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

CallbackReturn CartesianImpedanceController::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  const auto n = static_cast<Eigen::Index>(params_.joints.size());
  arm_control_core::Vector q(n);
  arm_control_core::Vector v(n);
  if (!readState(q, v)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot read the joint state");
    return CallbackReturn::ERROR;
  }
  // Hold the current pose; the current configuration is the null-space posture.
  posture_ = q;
  law_.emplace(*model_, lawParameters(posture_));
  current_target_ = model_->endEffectorPose(q);
  target_.set(current_target_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  for (auto & command : command_interfaces_) {
    (void)command.set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

return_type CartesianImpedanceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (!law_) {
    return return_type::ERROR;  // not configured
  }
  auto & law = *law_;
  if (param_listener_->try_get_params(params_)) {
    law.setParameters(lawParameters(posture_));
  }
  const auto n = static_cast<Eigen::Index>(params_.joints.size());
  arm_control_core::Vector q(n);
  arm_control_core::Vector v(n);
  if (!readState(q, v)) {
    return return_type::OK;
  }
  // Non-blocking: keep the previous target if the subscription is writing it right now.
  if (const auto target = target_.try_get()) {
    current_target_ = *target;
  }
  const arm_control_core::Vector tau =
    arm_control_core::saturate(law.compute(q, v, current_target_), effort_limits_);
  for (Eigen::Index i = 0; i < n; ++i) {
    (void)command_interfaces_[static_cast<std::size_t>(i)].set_value(tau(i));
  }

  if (pose_publisher_ && ++publish_counter_ >= 5) {
    publish_counter_ = 0;
    const Eigen::Isometry3d pose = model_->endEffectorPose(q);
    const Eigen::Quaterniond orientation(pose.linear());
    pose_msg_.header.stamp = time;
    pose_msg_.pose.position.x = pose.translation().x();
    pose_msg_.pose.position.y = pose.translation().y();
    pose_msg_.pose.position.z = pose.translation().z();
    pose_msg_.pose.orientation.w = orientation.w();
    pose_msg_.pose.orientation.x = orientation.x();
    pose_msg_.pose.orientation.y = orientation.y();
    pose_msg_.pose.orientation.z = orientation.z();
    (void)pose_publisher_->try_publish(pose_msg_);
  }
  return return_type::OK;
}

}  // namespace arm_controllers

PLUGINLIB_EXPORT_CLASS(
  arm_controllers::CartesianImpedanceController, controller_interface::ControllerInterface)

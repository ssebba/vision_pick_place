#ifndef ARM_CONTROLLERS__CARTESIAN_IMPEDANCE_CONTROLLER_HPP_
#define ARM_CONTROLLERS__CARTESIAN_IMPEDANCE_CONTROLLER_HPP_

#include <memory>
#include <optional>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

#include "arm_control_core/control_laws.hpp"
#include "arm_control_core/robot_model.hpp"
#include "arm_controllers/cartesian_impedance_controller_parameters.hpp"

namespace arm_controllers
{

/**
 * @brief Cartesian impedance control of the end effector, effort interfaces.
 *
 * The end effector behaves like a 6-DoF spring-damper attached to a target pose received on
 * `~/target_pose` (geometry_msgs/PoseStamped in the URDF root frame); on activation the target
 * is the current pose. Gravity and Coriolis torques are compensated with the Pinocchio model,
 * the damping follows the task-space inertia and the spring error is saturated, so a far
 * target produces a bounded force. Used for compliant placing and for disturbance-rejection
 * experiments; stiffness and damping can be changed at runtime.
 */
class CartesianImpedanceController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool readState(arm_control_core::Vector & q, arm_control_core::Vector & v) const;
  arm_control_core::CartesianImpedanceLaw::Parameters lawParameters(
    const arm_control_core::Vector & posture) const;

  std::shared_ptr<cartesian_impedance_controller::ParamListener> param_listener_;
  cartesian_impedance_controller::Params params_;
  std::unique_ptr<arm_control_core::RobotModel> model_;
  std::optional<arm_control_core::CartesianImpedanceLaw> law_;
  arm_control_core::Vector effort_limits_;
  arm_control_core::Vector posture_;

  // Non real-time (subscription) -> real-time: target pose.
  realtime_tools::RealtimeThreadSafeBox<Eigen::Isometry3d> target_;
  Eigen::Isometry3d current_target_{Eigen::Isometry3d::Identity()};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>
    pose_publisher_;
  geometry_msgs::msg::PoseStamped pose_msg_;
  int publish_counter_{0};
};

}  // namespace arm_controllers

#endif  // ARM_CONTROLLERS__CARTESIAN_IMPEDANCE_CONTROLLER_HPP_

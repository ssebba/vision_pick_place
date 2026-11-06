#ifndef ARM_CONTROLLERS__COMPUTED_TORQUE_CONTROLLER_HPP_
#define ARM_CONTROLLERS__COMPUTED_TORQUE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_server_goal_handle.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

#include "arm_control_core/control_laws.hpp"
#include "arm_control_core/joint_trajectory.hpp"
#include "arm_control_core/robot_model.hpp"
#include "arm_controllers/computed_torque_controller_parameters.hpp"

namespace arm_controllers
{

/**
 * @brief Joint trajectory tracking by computed torque (inverse dynamics), effort interfaces.
 *
 * Drop-in alternative to joint_trajectory_controller: same FollowJointTrajectory action, so
 * MoveIt can execute through it, but the joints are driven with torques computed from the
 * Pinocchio model of the arm (built from the robot_description):
 *
 *   tau = M(q) (qdd_d + Kp e + Kd edot) + C(q, qd) qd + g(q)
 *
 * `mode: pd_gravity` switches to PD + gravity compensation with the same nominal bandwidth,
 * the baseline used in the benchmark. Trajectories are handed to the real-time loop through a
 * realtime buffer and goal results are reported via RealtimeServerGoalHandle, so update()
 * never blocks on the non real-time action server.
 */
class ComputedTorqueController : public controller_interface::ControllerInterface
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<FollowJointTrajectory>;
  using ControllerState = control_msgs::msg::JointTrajectoryControllerState;

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
  // Trajectory being executed, owned by the real-time loop once published.
  struct ActiveGoal
  {
    arm_control_core::JointTrajectory trajectory;
    rclcpp::Time start_time;
    std::shared_ptr<RealtimeGoalHandle> goal_handle;
  };

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const FollowJointTrajectory::Goal> & goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> & goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandle> & goal_handle);

  // Maps the goal joints onto the controlled joints; empty if a joint is missing.
  std::vector<std::size_t> jointMapping(const std::vector<std::string> & goal_joints) const;
  bool readState(arm_control_core::Vector & q, arm_control_core::Vector & v) const;
  void holdPosition(const arm_control_core::Vector & q);
  void rebuildLaw();

  std::shared_ptr<computed_torque_controller::ParamListener> param_listener_;
  computed_torque_controller::Params params_;
  std::unique_ptr<arm_control_core::RobotModel> model_;
  std::variant<std::monostate, arm_control_core::ComputedTorqueLaw, arm_control_core::PdGravityLaw>
    law_;
  arm_control_core::Vector effort_limits_;

  // Non real-time -> real-time: the goal to execute (nullptr after a cancel). Only the action
  // server writes it; the real-time loop tracks what it has already completed itself.
  realtime_tools::RealtimeThreadSafeBox<std::shared_ptr<ActiveGoal>> active_goal_;
  // Real-time -> non real-time: last measured state, used as start of new trajectories.
  realtime_tools::RealtimeThreadSafeBox<arm_control_core::JointState> measured_state_;

  // Real-time loop state.
  std::shared_ptr<ActiveGoal> current_goal_;
  bool current_finished_{false};
  arm_control_core::JointState hold_;
  int publish_counter_{0};

  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;
  std::shared_ptr<realtime_tools::RealtimePublisher<ControllerState>> state_publisher_;
  ControllerState state_msg_;
};

}  // namespace arm_controllers

#endif  // ARM_CONTROLLERS__COMPUTED_TORQUE_CONTROLLER_HPP_

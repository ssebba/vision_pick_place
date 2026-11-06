#ifndef PP_TASK__TASK_CONTEXT_HPP_
#define PP_TASK__TASK_CONTEXT_HPP_

#include <atomic>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit_msgs/action/execute_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

#include "perception_core/grasp_planner.hpp"
#include "pp_task/grasp_attacher.hpp"

namespace pp_task
{

// Outcome of one pick-and-place attempt, compared with the simulation ground truth.
struct PickRecord
{
  std::string model;  // ground-truth model name (empty if unknown)
  std::string shape;
  double detection_error{std::numeric_limits<double>::quiet_NaN()};  // [m]
  bool grasped{false};  // the object really left the table
  bool placed{false};   // the object ended on the place pose
  double place_error{std::numeric_limits<double>::quiet_NaN()};  // [m], horizontal
  double cycle_time{0.0};     // [s] simulated, from target selection to release
  double planning_time{0.0};  // [s] wall clock, spent in MoveIt planning
};

/**
 * @brief State shared by the behavior tree nodes (stored on the blackboard as "context").
 * The BT nodes are plugins: everything they need from ROS comes through this object.
 */
struct TaskContext
{
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper;
  // move_group's trajectory execution action, used instead of MoveGroupInterface::execute
  // (which waits for the result without a timeout and blocks a Ctrl+C mid-motion).
  rclcpp_action::Client<moveit_msgs::action::ExecuteTrajectory>::SharedPtr execute_client;
  std::shared_ptr<GraspAttacher> attacher;
  perception_core::GraspParams grasp_params;
  double motion_velocity_scaling{0.4};

  // Place area: object i is released at place_origin + i * place_step.
  Eigen::Vector3d place_origin{0.10, -0.40, 0.0};
  Eigen::Vector3d place_step{0.10, 0.0, 0.0};
  int placed_count{0};

  // Latest detections (written by the subscription, read by DetectObjects).
  std::mutex detections_mutex;
  vision_msgs::msg::Detection3DArray::SharedPtr detections;

  // Objects that could not be picked: not selected again.
  std::vector<Eigen::Vector3d> failed_targets;

  // Set to stop the task (deactivation, destruction): the long waits give up.
  std::atomic<bool> stop_requested{false};
  // True when the task must stop: requested, or the process is shutting down (Ctrl+C).
  bool stopRequested() const { return stop_requested || !rclcpp::ok(); }

  // Benchmark bookkeeping.
  std::vector<PickRecord> records;
  std::optional<PickRecord> current;
  rclcpp::Time cycle_start{0, 0, RCL_ROS_TIME};

  // Closes the current record (if any), its cycle ending at `now`, and stores it.
  void closeRecord(const rclcpp::Time & now);
};

// Pure helpers (unit tested).
std::vector<perception_core::DetectedObject> fromDetections(
  const vision_msgs::msg::Detection3DArray & msg);
geometry_msgs::msg::Pose toPoseMsg(const Eigen::Isometry3d & pose);
Eigen::Isometry3d fromPoseMsg(const geometry_msgs::msg::Pose & pose);
// First object not close to a failed one (objects are sorted by the detector).
std::optional<perception_core::DetectedObject> selectTarget(
  const std::vector<perception_core::DetectedObject> & objects,
  const std::vector<Eigen::Vector3d> & failed, double min_distance);

}  // namespace pp_task

#endif  // PP_TASK__TASK_CONTEXT_HPP_

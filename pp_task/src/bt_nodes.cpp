#include "pp_task/bt_nodes.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <thread>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "moveit/robot_state/robot_state.hpp"
#include "moveit/robot_trajectory/robot_trajectory.hpp"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"

namespace pp_task
{

namespace
{

using moveit::core::MoveItErrorCode;
using moveit::planning_interface::MoveGroupInterface;

constexpr double kMaxHalfOpening = 0.0425;  // [m], gripper joint limit

// Waits for a future until the task is stopped or the deadline (simulated time) passes.
template <typename FutureT>
bool waitFor(const TaskContext & ctx, FutureT & future, const rclcpp::Time & deadline)
{
  while (future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
    if (ctx.stopRequested() || ctx.node->now() > deadline) {
      return false;
    }
  }
  return true;
}

// Executes a trajectory through move_group. The motion is cancelled if the task is stopped, or
// if no result comes within twice the trajectory duration plus a margin: a controller that
// never answers must not block the task.
bool executeTrajectory(TaskContext & ctx, const moveit_msgs::msg::RobotTrajectory & trajectory)
{
  // A goal sent before the server is discovered would be lost (immediate once it is known).
  if (!ctx.execute_client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_WARN(ctx.node->get_logger(), "move_group trajectory execution not available");
    return false;
  }
  const auto & points = trajectory.joint_trajectory.points;
  const double timeout =
    5.0 + (points.empty() ? 0.0 : 2.0 * rclcpp::Duration(points.back().time_from_start).seconds());
  const rclcpp::Time deadline = ctx.node->now() + rclcpp::Duration::from_seconds(timeout);
  moveit_msgs::action::ExecuteTrajectory::Goal goal;
  goal.trajectory = trajectory;
  auto goal_future = ctx.execute_client->async_send_goal(goal);
  if (!waitFor(ctx, goal_future, deadline)) {
    return false;
  }
  const auto goal_handle = goal_future.get();
  if (!goal_handle) {
    RCLCPP_WARN(ctx.node->get_logger(), "Trajectory execution rejected");
    return false;
  }
  auto result_future = ctx.execute_client->async_get_result(goal_handle);
  if (!waitFor(ctx, result_future, deadline)) {
    if (!ctx.stopRequested()) {
      RCLCPP_WARN(
        ctx.node->get_logger(), "No execution result %.1f s after the start: motion cancelled",
        timeout);
    }
    if (rclcpp::ok()) {
      ctx.execute_client->async_cancel_goal(goal_handle);
    }
    return false;
  }
  const auto result = result_future.get();
  return result.code == rclcpp_action::ResultCode::SUCCEEDED &&
         result.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
}

bool planAndExecute(TaskContext & ctx, MoveGroupInterface & group)
{
  MoveGroupInterface::Plan plan;
  if (group.plan(plan) != MoveItErrorCode::SUCCESS) {
    return false;
  }
  if (ctx.current) {
    ctx.current->planning_time += plan.planning_time;
  }
  return executeTrajectory(ctx, plan.trajectory);
}

template <typename T>
T input(const BT::TreeNode & node, const std::string & port)
{
  auto value = node.getInput<T>(port);
  if (!value) {
    throw BT::RuntimeError(node.name(), ": missing input [", port, "]: ", value.error());
  }
  return value.value();
}

}  // namespace

TaskContext & ContextNode::context()
{
  return *config().blackboard->get<std::shared_ptr<TaskContext>>("context");
}

BT::NodeStatus MoveToNamed::tick()
{
  auto & ctx = context();
  ctx.arm->setMaxVelocityScalingFactor(ctx.motion_velocity_scaling);
  ctx.arm->setMaxAccelerationScalingFactor(ctx.motion_velocity_scaling);
  if (!ctx.arm->setNamedTarget(input<std::string>(*this, "target"))) {
    return BT::NodeStatus::FAILURE;
  }
  return planAndExecute(ctx, *ctx.arm) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveToPose::tick()
{
  auto & ctx = context();
  ctx.arm->setMaxVelocityScalingFactor(ctx.motion_velocity_scaling);
  ctx.arm->setMaxAccelerationScalingFactor(ctx.motion_velocity_scaling);
  ctx.arm->setPoseTarget(input<geometry_msgs::msg::Pose>(*this, "pose"));
  return planAndExecute(ctx, *ctx.arm) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveLinear::tick()
{
  auto & ctx = context();
  const auto target = input<geometry_msgs::msg::Pose>(*this, "pose");
  const auto scaling = input<double>(*this, "velocity_scaling");

  moveit_msgs::msg::RobotTrajectory trajectory;
  const auto t0 = std::chrono::steady_clock::now();
  const double fraction = ctx.arm->computeCartesianPath({target}, 0.005, trajectory);
  if (ctx.current) {
    ctx.current->planning_time +=
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  }
  if (fraction < 0.99) {
    RCLCPP_WARN(ctx.node->get_logger(), "Cartesian path only %.0f%% feasible", 100.0 * fraction);
    return BT::NodeStatus::FAILURE;
  }
  // Slow, smooth approach: re-time the straight line with the requested scaling.
  robot_trajectory::RobotTrajectory timed(ctx.arm->getRobotModel(), ctx.arm->getName());
  timed.setRobotTrajectoryMsg(*ctx.arm->getCurrentState(), trajectory);
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(timed, scaling, scaling)) {
    return BT::NodeStatus::FAILURE;
  }
  timed.getRobotTrajectoryMsg(trajectory);
  return executeTrajectory(ctx, trajectory) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus SetGripper::tick()
{
  auto & ctx = context();
  const double opening = input<double>(*this, "opening") - input<double>(*this, "squeeze");
  const double half = std::clamp(0.5 * opening, 0.0, kMaxHalfOpening);
  ctx.gripper->setMaxVelocityScalingFactor(1.0);
  ctx.gripper->setMaxAccelerationScalingFactor(1.0);
  ctx.gripper->setJointValueTarget(std::vector<double>{half, half});
  // A finger stopped by the object still counts as success (see the controller tolerances).
  return planAndExecute(ctx, *ctx.gripper) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus DetectObjects::tick()
{
  auto & ctx = context();
  // Only a cloud captured after the arm stopped is meaningful.
  const rclcpp::Time requested = ctx.node->now() + rclcpp::Duration::from_seconds(0.3);
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(input<double>(*this, "timeout"));
  while (std::chrono::steady_clock::now() < deadline && !ctx.stopRequested()) {
    {
      std::scoped_lock lock(ctx.detections_mutex);
      if (ctx.detections && rclcpp::Time(ctx.detections->header.stamp) > requested) {
        setOutput("objects", fromDetections(*ctx.detections));
        RCLCPP_INFO(
          ctx.node->get_logger(), "%zu objects detected", ctx.detections->detections.size());
        return BT::NodeStatus::SUCCESS;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  RCLCPP_WARN(ctx.node->get_logger(), "No fresh detection received");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus SelectTarget::tick()
{
  auto & ctx = context();
  ctx.closeRecord(ctx.node->now());  // a previous attempt that never reached the release
  const auto objects = input<std::vector<perception_core::DetectedObject>>(*this, "objects");
  const auto target = selectTarget(objects, ctx.failed_targets, 0.05);
  if (!target) {
    return BT::NodeStatus::FAILURE;  // nothing left: ends the loop
  }
  PickRecord record;
  record.shape = perception_core::toString(target->shape);
  if (const auto model = ctx.attacher->closestModel(target->center)) {
    record.model = *model;
    if (const auto truth = ctx.attacher->groundTruth(*model)) {
      record.detection_error = (*truth - target->center).norm();
    }
  }
  ctx.current = record;
  ctx.cycle_start = ctx.node->now();
  setOutput("target", *target);
  RCLCPP_INFO(
    ctx.node->get_logger(), "Target: %s at (%.3f, %.3f), %.0f x %.0f mm", record.shape.c_str(),
    target->center.x(), target->center.y(), 1000.0 * target->size.x(), 1000.0 * target->size.y());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus PlanGrasp::tick()
{
  auto & ctx = context();
  const auto target = input<perception_core::DetectedObject>(*this, "target");
  const auto candidates = perception_core::planTopDownGrasps(target, ctx.grasp_params);
  const auto state = ctx.arm->getCurrentState(2.0);
  if (!state) {
    return BT::NodeStatus::FAILURE;
  }
  const auto * group = state->getJointModelGroup(ctx.arm->getName());
  for (const auto & grasp : candidates) {
    moveit::core::RobotState probe(*state);
    if (
      probe.setFromIK(group, toPoseMsg(grasp.pregrasp), 0.05) &&
      probe.setFromIK(group, toPoseMsg(grasp.pose), 0.05)) {
      setOutput("grasp", toPoseMsg(grasp.pose));
      setOutput("pregrasp", toPoseMsg(grasp.pregrasp));
      setOutput("width", grasp.width);
      return BT::NodeStatus::SUCCESS;
    }
  }
  RCLCPP_WARN(ctx.node->get_logger(), "No reachable grasp among %zu candidates", candidates.size());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus AttachObject::tick()
{
  auto & ctx = context();
  const auto tcp = ctx.arm->getCurrentPose().pose.position;
  return ctx.attacher->attach(Eigen::Vector3d(tcp.x, tcp.y, tcp.z)) ? BT::NodeStatus::SUCCESS
                                                                    : BT::NodeStatus::FAILURE;
}

BT::NodeStatus DetachObject::tick()
{
  context().attacher->detach();
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus VerifyLift::tick()
{
  auto & ctx = context();
  if (!ctx.current || ctx.current->model.empty()) {
    return BT::NodeStatus::SUCCESS;  // no ground truth: cannot check
  }
  const auto truth = ctx.attacher->groundTruth(ctx.current->model);
  ctx.current->grasped = truth && truth->z() > 0.06;  // well above its resting height
  return ctx.current->grasped ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus NextPlacePose::tick()
{
  auto & ctx = context();
  const auto grasp = input<geometry_msgs::msg::Pose>(*this, "grasp");
  const Eigen::Vector3d slot =
    ctx.place_origin + static_cast<double>(ctx.placed_count) * ctx.place_step;
  geometry_msgs::msg::Pose place = grasp;
  place.position.x = slot.x();
  place.position.y = slot.y();
  place.position.z = grasp.position.z + 0.01;  // release just above the table
  geometry_msgs::msg::Pose preplace = place;
  preplace.position.z += ctx.grasp_params.approach_distance;
  ++ctx.placed_count;
  setOutput("place", place);
  setOutput("preplace", preplace);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus VerifyPlace::tick()
{
  auto & ctx = context();
  if (ctx.current && !ctx.current->model.empty()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));  // let the object settle
    const auto place = input<geometry_msgs::msg::Pose>(*this, "place");
    if (const auto truth = ctx.attacher->groundTruth(ctx.current->model)) {
      ctx.current->place_error =
        std::hypot(truth->x() - place.position.x, truth->y() - place.position.y);
      ctx.current->placed = ctx.current->place_error < 0.03 && truth->z() < 0.1;
    }
  }
  ctx.closeRecord(ctx.node->now());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MarkFailed::tick()
{
  auto & ctx = context();
  ctx.failed_targets.push_back(input<perception_core::DetectedObject>(*this, "target").center);
  ctx.closeRecord(ctx.node->now());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace pp_task

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pp_task::MoveToNamed>("MoveToNamed");
  factory.registerNodeType<pp_task::MoveToPose>("MoveToPose");
  factory.registerNodeType<pp_task::MoveLinear>("MoveLinear");
  factory.registerNodeType<pp_task::SetGripper>("SetGripper");
  factory.registerNodeType<pp_task::DetectObjects>("DetectObjects");
  factory.registerNodeType<pp_task::SelectTarget>("SelectTarget");
  factory.registerNodeType<pp_task::PlanGrasp>("PlanGrasp");
  factory.registerNodeType<pp_task::AttachObject>("AttachObject");
  factory.registerNodeType<pp_task::DetachObject>("DetachObject");
  factory.registerNodeType<pp_task::VerifyLift>("VerifyLift");
  factory.registerNodeType<pp_task::NextPlacePose>("NextPlacePose");
  factory.registerNodeType<pp_task::VerifyPlace>("VerifyPlace");
  factory.registerNodeType<pp_task::MarkFailed>("MarkFailed");
}

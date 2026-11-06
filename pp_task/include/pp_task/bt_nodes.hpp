#ifndef PP_TASK__BT_NODES_HPP_
#define PP_TASK__BT_NODES_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose.hpp"

#include "perception_core/object_detector.hpp"
#include "pp_task/task_context.hpp"

namespace pp_task
{

/**
 * Behavior tree nodes of the pick-and-place task, built as a BT.CPP plugin library
 * (BT_REGISTER_NODES) and loaded by the TaskManager. All of them are synchronous: MoveIt
 * calls block the tree thread, which is dedicated to the task.
 */
class ContextNode : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;

protected:
  TaskContext & context();
};

// Moves the arm to a named pose of the SRDF (home, observe).
class MoveToNamed : public ContextNode
{
public:
  using ContextNode::ContextNode;
  static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("target")}; }
  BT::NodeStatus tick() override;
};

// Collision-free motion (OMPL) to a TCP pose.
class MoveToPose : public ContextNode
{
public:
  using ContextNode::ContextNode;
  static BT::PortsList providedPorts() { return {BT::InputPort<geometry_msgs::msg::Pose>("pose")}; }
  BT::NodeStatus tick() override;
};

// Straight-line TCP motion (Cartesian path, retimed with TOTG at `velocity_scaling`).
class MoveLinear : public ContextNode
{
public:
  using ContextNode::ContextNode;
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::Pose>("pose"),
      BT::InputPort<double>("velocity_scaling", 0.1, "fraction of the joint limits")};
  }
  BT::NodeStatus tick() override;
};

// Sets the finger opening [m]: `opening - squeeze` (squeeze presses the fingers on an object).
class SetGripper : public ContextNode
{
public:
  using ContextNode::ContextNode;
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("opening"),
      BT::InputPort<double>("squeeze", 0.0, "subtracted from the opening when closing")};
  }
  BT::NodeStatus tick() override;
};

// Waits for a detection newer than the call and outputs the objects.
class DetectObjects : public ContextNode
{
public:
  using ContextNode::ContextNode;
  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<std::vector<perception_core::DetectedObject>>("objects"),
      BT::InputPort<double>("timeout", 10.0, "[s]")};
  }
  BT::NodeStatus tick() override;
};

// Chooses the next object (fails when none is left) and opens a benchmark record.
class SelectTarget : public ContextNode
{
public:
  using ContextNode::ContextNode;
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<perception_core::DetectedObject>>("objects"),
      BT::OutputPort<perception_core::DetectedObject>("target")};
  }
  BT::NodeStatus tick() override;
};

// Top-down grasp candidates (perception_core), first one with IK solutions for both poses.
class PlanGrasp : public ContextNode
{
public:
  using ContextNode::ContextNode;
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<perception_core::DetectedObject>("target"),
      BT::OutputPort<geometry_msgs::msg::Pose>("grasp"),
      BT::OutputPort<geometry_msgs::msg::Pose>("pregrasp"), BT::OutputPort<double>("width")};
  }
  BT::NodeStatus tick() override;
};

class AttachObject : public ContextNode
{
public:
  using ContextNode::ContextNode;
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

class DetachObject : public ContextNode
{
public:
  using ContextNode::ContextNode;
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

// Success when the grasped object left the table (ground truth), records the grasp.
class VerifyLift : public ContextNode
{
public:
  using ContextNode::ContextNode;
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

// Place and pre-place TCP poses of the next free slot, same height and yaw as the grasp.
class NextPlacePose : public ContextNode
{
public:
  using ContextNode::ContextNode;
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::Pose>("grasp"),
      BT::OutputPort<geometry_msgs::msg::Pose>("place"),
      BT::OutputPort<geometry_msgs::msg::Pose>("preplace")};
  }
  BT::NodeStatus tick() override;
};

// Checks where the released object landed and closes the benchmark record.
class VerifyPlace : public ContextNode
{
public:
  using ContextNode::ContextNode;
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::Pose>("place")};
  }
  BT::NodeStatus tick() override;
};

// Marks the target as not pickable (never selected again) and closes its record.
class MarkFailed : public ContextNode
{
public:
  using ContextNode::ContextNode;
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<perception_core::DetectedObject>("target")};
  }
  BT::NodeStatus tick() override;
};

}  // namespace pp_task

#endif  // PP_TASK__BT_NODES_HPP_

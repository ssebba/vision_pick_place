#ifndef PP_TASK__GRASP_ATTACHER_HPP_
#define PP_TASK__GRASP_ATTACHER_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace pp_task
{

/**
 * @brief Makes a grasped object follow the gripper.
 *
 * On a real robot the friction of the closed fingers does this and the implementation is
 * empty; in Gazebo friction-only grasps slip, so the simulation attaches the object to the
 * wrist with a detachable fixed joint (see pp_description). The behavior tree only sees this
 * interface, not the simulation trick.
 */
class GraspAttacher
{
public:
  virtual ~GraspAttacher() = default;
  // Attaches the object closest to `tcp` (world frame); returns its name, if any.
  virtual std::optional<std::string> attach(const Eigen::Vector3d & tcp) = 0;
  virtual void detach() = 0;
  // Ground-truth position of a model, when available (benchmark metrics).
  virtual std::optional<Eigen::Vector3d> groundTruth(const std::string & name) const = 0;
  // Ground-truth model closest to `point`, when available.
  virtual std::optional<std::string> closestModel(const Eigen::Vector3d & point) const = 0;
};

// Name of the model closest to `point` among `models` within `max_distance`.
std::optional<std::string> closestModelName(
  const std::map<std::string, Eigen::Vector3d> & models, const Eigen::Vector3d & point,
  double max_distance);

/**
 * @brief Gazebo implementation: model poses from `/sim/model_poses` (Pose_V bridged to
 * tf2_msgs/TFMessage), commands on `/<model>/attach` and `/<model>/detach`.
 */
class SimGraspAttacher : public GraspAttacher
{
public:
  SimGraspAttacher(rclcpp::Node::SharedPtr node, std::string model_prefix, double max_distance);

  std::optional<std::string> attach(const Eigen::Vector3d & tcp) override;
  void detach() override;
  std::optional<Eigen::Vector3d> groundTruth(const std::string & name) const override;
  std::optional<std::string> closestModel(const Eigen::Vector3d & point) const override;

private:
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher(const std::string & topic);

  rclcpp::Node::SharedPtr node_;
  std::string model_prefix_;
  double max_distance_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr poses_sub_;
  mutable std::mutex mutex_;
  std::map<std::string, Eigen::Vector3d> models_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> publishers_;
  std::optional<std::string> attached_;
};

}  // namespace pp_task

#endif  // PP_TASK__GRASP_ATTACHER_HPP_

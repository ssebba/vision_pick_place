#include "pp_task/grasp_attacher.hpp"

#include <limits>
#include <utility>

namespace pp_task
{

std::optional<std::string> closestModelName(
  const std::map<std::string, Eigen::Vector3d> & models, const Eigen::Vector3d & point,
  double max_distance)
{
  std::optional<std::string> best;
  double best_distance = max_distance;
  for (const auto & [name, position] : models) {
    const double d = (position - point).norm();
    if (d < best_distance) {
      best_distance = d;
      best = name;
    }
  }
  return best;
}

SimGraspAttacher::SimGraspAttacher(
  rclcpp::Node::SharedPtr node, std::string model_prefix, double max_distance)
: node_(std::move(node)), model_prefix_(std::move(model_prefix)), max_distance_(max_distance)
{
  poses_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
    "/sim/model_poses", rclcpp::SensorDataQoS(), [this](const tf2_msgs::msg::TFMessage & msg) {
      std::scoped_lock lock(mutex_);
      for (const auto & tf : msg.transforms) {
        // Entities are named "<model>" (or "<model>::<link>" for links): keep the objects.
        const std::string name = tf.child_frame_id.substr(0, tf.child_frame_id.find("::"));
        if (name.starts_with(model_prefix_) && name == tf.child_frame_id) {
          models_[name] = Eigen::Vector3d(
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
        }
      }
    });
}

rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr SimGraspAttacher::publisher(
  const std::string & topic)
{
  auto & pub = publishers_[topic];
  if (!pub) {
    pub = node_->create_publisher<std_msgs::msg::Empty>(topic, 10);
  }
  return pub;
}

std::optional<std::string> SimGraspAttacher::attach(const Eigen::Vector3d & tcp)
{
  auto name = closestModel(tcp);
  if (!name) {
    RCLCPP_WARN(node_->get_logger(), "No object near the gripper to attach");
    return std::nullopt;
  }
  publisher("/" + *name + "/attach")->publish(std_msgs::msg::Empty());
  attached_ = name;
  RCLCPP_INFO(node_->get_logger(), "Attached %s", name->c_str());
  return name;
}

void SimGraspAttacher::detach()
{
  if (!attached_) {
    return;
  }
  const std::string name = *attached_;
  attached_.reset();
  publisher("/" + name + "/detach")->publish(std_msgs::msg::Empty());
  RCLCPP_INFO(node_->get_logger(), "Detached %s", name.c_str());
}

std::optional<Eigen::Vector3d> SimGraspAttacher::groundTruth(const std::string & name) const
{
  std::scoped_lock lock(mutex_);
  const auto it = models_.find(name);
  if (it == models_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::optional<std::string> SimGraspAttacher::closestModel(const Eigen::Vector3d & point) const
{
  std::scoped_lock lock(mutex_);
  return closestModelName(models_, point, max_distance_);
}

}  // namespace pp_task

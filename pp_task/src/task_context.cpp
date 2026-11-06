#include "pp_task/task_context.hpp"

#include <algorithm>
#include <cmath>

namespace pp_task
{

void TaskContext::closeRecord(const rclcpp::Time & now)
{
  if (current) {
    current->cycle_time = (now - cycle_start).seconds();
    records.push_back(*current);
    current.reset();
  }
}

std::vector<perception_core::DetectedObject> fromDetections(
  const vision_msgs::msg::Detection3DArray & msg)
{
  std::vector<perception_core::DetectedObject> objects;
  objects.reserve(msg.detections.size());
  for (const auto & d : msg.detections) {
    perception_core::DetectedObject o;
    const auto & c = d.bbox.center;
    o.center = Eigen::Vector3d(c.position.x, c.position.y, c.position.z);
    o.yaw = 2.0 * std::atan2(c.orientation.z, c.orientation.w);
    o.size = Eigen::Vector3d(d.bbox.size.x, d.bbox.size.y, d.bbox.size.z);
    o.shape = (!d.results.empty() && d.results.front().hypothesis.class_id == "cylinder")
                ? perception_core::Shape::kCylinder
                : perception_core::Shape::kBox;
    objects.push_back(o);
  }
  return objects;
}

geometry_msgs::msg::Pose toPoseMsg(const Eigen::Isometry3d & pose)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.translation().x();
  msg.position.y = pose.translation().y();
  msg.position.z = pose.translation().z();
  const Eigen::Quaterniond q(pose.linear());
  msg.orientation.w = q.w();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  return msg;
}

Eigen::Isometry3d fromPoseMsg(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  out.linear() = Eigen::Quaterniond(
                   pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
                   .normalized()
                   .toRotationMatrix();
  return out;
}

std::optional<perception_core::DetectedObject> selectTarget(
  const std::vector<perception_core::DetectedObject> & objects,
  const std::vector<Eigen::Vector3d> & failed, double min_distance)
{
  for (const auto & o : objects) {
    const bool already_failed =
      std::any_of(failed.begin(), failed.end(), [&](const Eigen::Vector3d & f) {
        return (f.head<2>() - o.center.head<2>()).norm() < min_distance;
      });
    if (!already_failed) {
      return o;
    }
  }
  return std::nullopt;
}

}  // namespace pp_task

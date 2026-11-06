#include "pp_perception/object_detector_node.hpp"

#include <chrono>
#include <cmath>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace pp_perception
{

namespace
{

geometry_msgs::msg::Pose toPose(const perception_core::DetectedObject & o)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = o.center.x();
  pose.position.y = o.center.y();
  pose.position.z = o.center.z();
  pose.orientation.z = std::sin(0.5 * o.yaw);
  pose.orientation.w = std::cos(0.5 * o.yaw);
  return pose;
}

}  // namespace

vision_msgs::msg::Detection3DArray toDetectionArray(
  const perception_core::DetectionResult & result, const std_msgs::msg::Header & header)
{
  vision_msgs::msg::Detection3DArray array;
  array.header = header;
  for (std::size_t i = 0; i < result.objects.size(); ++i) {
    const auto & o = result.objects[i];
    vision_msgs::msg::Detection3D d;
    d.header = header;
    d.id = std::to_string(i);
    d.bbox.center = toPose(o);
    d.bbox.size.x = o.size.x();
    d.bbox.size.y = o.size.y();
    d.bbox.size.z = o.size.z();
    vision_msgs::msg::ObjectHypothesisWithPose h;
    h.hypothesis.class_id = perception_core::toString(o.shape);
    h.hypothesis.score = 1.0;
    h.pose.pose = d.bbox.center;
    d.results.push_back(h);
    array.detections.push_back(d);
  }
  return array;
}

visualization_msgs::msg::MarkerArray toMarkers(
  const perception_core::DetectionResult & result, const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.header = header;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  for (std::size_t i = 0; i < result.objects.size(); ++i) {
    const auto & o = result.objects[i];
    visualization_msgs::msg::Marker m;
    m.header = header;
    m.ns = "detections";
    m.id = static_cast<int>(i);
    m.type = o.shape == perception_core::Shape::kBox ? visualization_msgs::msg::Marker::CUBE
                                                     : visualization_msgs::msg::Marker::CYLINDER;
    m.pose = toPose(o);
    m.scale.x = o.size.x();
    m.scale.y = o.size.y();
    m.scale.z = o.size.z();
    m.color.g = 0.9F;
    m.color.b = 0.2F;
    m.color.a = 0.45F;
    markers.markers.push_back(m);
  }
  return markers;
}

ObjectDetectorNode::ObjectDetectorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("object_detector", options)
{
}

perception_core::DetectorParams ObjectDetectorNode::detectorParams() const
{
  perception_core::DetectorParams p;
  p.crop_min = Eigen::Vector3f(
    static_cast<float>(params_.crop_min[0]), static_cast<float>(params_.crop_min[1]),
    static_cast<float>(params_.crop_min[2]));
  p.crop_max = Eigen::Vector3f(
    static_cast<float>(params_.crop_max[0]), static_cast<float>(params_.crop_max[1]),
    static_cast<float>(params_.crop_max[2]));
  p.voxel_size = static_cast<float>(params_.voxel_size);
  p.plane_distance = static_cast<float>(params_.plane_distance);
  p.min_height = static_cast<float>(params_.min_height);
  p.cluster_tolerance = static_cast<float>(params_.cluster_tolerance);
  p.min_cluster_size = static_cast<int>(params_.min_cluster_size);
  p.max_cluster_size = static_cast<int>(params_.max_cluster_size);
  return p;
}

ObjectDetectorNode::CallbackReturn ObjectDetectorNode::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  param_listener_ =
    std::make_shared<object_detector_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();
  detector_.emplace(detectorParams());
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  detections_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>("~/detections", 10);
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", 10);
  return CallbackReturn::SUCCESS;
}

ObjectDetectorNode::CallbackReturn ObjectDetectorNode::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  detections_pub_->on_activate();
  markers_pub_->on_activate();
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    params_.input_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) { cloudCallback(msg); });
  return CallbackReturn::SUCCESS;
}

void ObjectDetectorNode::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    detector_.emplace(detectorParams());
  }
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  if (
    last_processed_.nanoseconds() > 0 && (stamp - last_processed_).seconds() < params_.min_period) {
    return;
  }

  Eigen::Isometry3d to_target;
  try {
    to_target = tf2::transformToEigen(tf_buffer_->lookupTransform(
      params_.target_frame, msg->header.frame_id, msg->header.stamp,
      rclcpp::Duration::from_seconds(0.1)));
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No transform: %s", e.what());
    return;
  }
  last_processed_ = stamp;

  // PointCloud2 -> PCL in the target frame, dropping invalid points.
  perception_core::Cloud cloud_target;
  cloud_target.reserve(static_cast<std::size_t>(msg->width) * msg->height);
  const Eigen::Affine3f transform(to_target.matrix().cast<float>());
  sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    if (std::isfinite(*it_x) && std::isfinite(*it_y) && std::isfinite(*it_z)) {
      const Eigen::Vector3f q = transform * Eigen::Vector3f(*it_x, *it_y, *it_z);
      cloud_target.push_back(pcl::PointXYZ(q.x(), q.y(), q.z()));
    }
  }

  if (!detector_) {
    return;  // cleaned up meanwhile: no detector to run
  }
  const auto t0 = std::chrono::steady_clock::now();
  const perception_core::DetectionResult result = detector_->detect(cloud_target);
  const double elapsed_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  RCLCPP_DEBUG(
    get_logger(), "%zu objects in %.1f ms (%zu points)", result.objects.size(), elapsed_ms,
    cloud_target.size());

  std_msgs::msg::Header header;
  header.stamp = msg->header.stamp;
  header.frame_id = params_.target_frame;
  detections_pub_->publish(toDetectionArray(result, header));
  if (markers_pub_->get_subscription_count() > 0) {
    markers_pub_->publish(toMarkers(result, header));
  }
}

ObjectDetectorNode::CallbackReturn ObjectDetectorNode::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  cloud_sub_.reset();
  detections_pub_->on_deactivate();
  markers_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

ObjectDetectorNode::CallbackReturn ObjectDetectorNode::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  tf_listener_.reset();
  tf_buffer_.reset();
  detections_pub_.reset();
  markers_pub_.reset();
  detector_.reset();
  return CallbackReturn::SUCCESS;
}

}  // namespace pp_perception

RCLCPP_COMPONENTS_REGISTER_NODE(pp_perception::ObjectDetectorNode)

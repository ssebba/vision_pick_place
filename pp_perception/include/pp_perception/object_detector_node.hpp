#ifndef PP_PERCEPTION__OBJECT_DETECTOR_NODE_HPP_
#define PP_PERCEPTION__OBJECT_DETECTOR_NODE_HPP_

#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "perception_core/object_detector.hpp"
#include "pp_perception/object_detector_parameters.hpp"

namespace pp_perception
{

// ROS message of a detection result (bounding boxes in `header.frame_id`).
vision_msgs::msg::Detection3DArray toDetectionArray(
  const perception_core::DetectionResult & result, const std_msgs::msg::Header & header);

visualization_msgs::msg::MarkerArray toMarkers(
  const perception_core::DetectionResult & result, const std_msgs::msg::Header & header);

/**
 * @brief Lifecycle component: point cloud -> objects on the table.
 *
 * Transforms each cloud into `target_frame` (z up), runs perception_core::ObjectDetector and
 * publishes `~/detections` (vision_msgs/Detection3DArray: oriented boxes, class "box" or
 * "cylinder") and `~/markers` for RViz. Parameters can be tuned at runtime.
 */
class ObjectDetectorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit ObjectDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  perception_core::DetectorParams detectorParams() const;

  std::shared_ptr<object_detector_node::ParamListener> param_listener_;
  object_detector_node::Params params_;
  std::optional<perception_core::ObjectDetector> detector_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection3DArray>::SharedPtr
    detections_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    markers_pub_;
  rclcpp::Time last_processed_{0, 0, RCL_ROS_TIME};
};

}  // namespace pp_perception

#endif  // PP_PERCEPTION__OBJECT_DETECTOR_NODE_HPP_

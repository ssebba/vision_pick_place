// The node end to end: synthetic cloud in the camera frame + TF -> detections in "world".

#include <chrono>
#include <cmath>
#include <memory>
#include <numbers>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

#include "pp_perception/object_detector_node.hpp"

using namespace std::chrono_literals;

namespace
{

// Camera of the work cell: 0.95 m above the table looking down (pitch +90 deg).
Eigen::Isometry3d cameraPose()
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(0.5, 0.0, 0.95);
  pose.linear() =
    Eigen::AngleAxisd(std::numbers::pi / 2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  return pose;
}

// Table (z = 0) and a 5 x 7 x 5 cm box at (0.45, 0.1), yaw 0.3, as seen from above.
sensor_msgs::msg::PointCloud2 syntheticCloud()
{
  std::mt19937 rng(3);
  std::normal_distribution<double> noise(0.0, 0.001);
  const Eigen::Isometry3d world_to_camera = cameraPose().inverse();
  std::vector<Eigen::Vector3f> points;
  const auto add = [&](double x, double y, double z) {
    const Eigen::Vector3d p =
      world_to_camera * Eigen::Vector3d(x + noise(rng), y + noise(rng), z + noise(rng));
    points.push_back(p.cast<float>());
  };
  for (double x = 0.25; x <= 0.85; x += 0.004) {
    for (double y = -0.35; y <= 0.35; y += 0.004) {
      add(x, y, 0.0);
    }
  }
  const double c = std::cos(0.3);
  const double s = std::sin(0.3);
  for (double u = -0.025; u <= 0.025; u += 0.004) {
    for (double v = -0.035; v <= 0.035; v += 0.004) {
      add(0.45 + c * u - s * v, 0.1 + s * u + c * v, 0.05);
    }
  }
  sensor_msgs::msg::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");
  for (const auto & p : points) {
    *it_x = p.x();
    *it_y = p.y();
    *it_z = p.z();
    ++it_x;
    ++it_y;
    ++it_z;
  }
  msg.header.frame_id = "camera_link";
  return msg;
}

}  // namespace

TEST(ObjectDetectorNode, DetectsTheBoxInTheWorldFrame)
{
  rclcpp::init(0, nullptr);
  {
    auto detector = std::make_shared<pp_perception::ObjectDetectorNode>();
    ASSERT_EQ(detector->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(detector->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    auto helper = std::make_shared<rclcpp::Node>("detector_test_helper");
    tf2_ros::StaticTransformBroadcaster tf_broadcaster(*helper);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "world";
    tf.child_frame_id = "camera_link";
    const Eigen::Isometry3d cam = cameraPose();
    tf.transform.translation.x = cam.translation().x();
    tf.transform.translation.y = cam.translation().y();
    tf.transform.translation.z = cam.translation().z();
    const Eigen::Quaterniond q(cam.linear());
    tf.transform.rotation.w = q.w();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf_broadcaster.sendTransform(tf);

    vision_msgs::msg::Detection3DArray::SharedPtr received;
    auto sub = helper->create_subscription<vision_msgs::msg::Detection3DArray>(
      "/object_detector/detections", 10,
      [&received](vision_msgs::msg::Detection3DArray::SharedPtr msg) { received = msg; });
    auto pub = helper->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/camera/points", rclcpp::SensorDataQoS());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(detector->get_node_base_interface());
    executor.add_node(helper);
    const auto deadline = std::chrono::steady_clock::now() + 20s;
    while (!received && std::chrono::steady_clock::now() < deadline) {
      auto cloud = syntheticCloud();
      cloud.header.stamp = helper->now();
      pub->publish(cloud);
      executor.spin_some(200ms);
    }
    ASSERT_TRUE(received) << "no detections received";
    EXPECT_EQ(received->header.frame_id, "world");
    ASSERT_EQ(received->detections.size(), 1u);
    const auto & d = received->detections.front();
    EXPECT_EQ(d.results.front().hypothesis.class_id, "box");
    EXPECT_NEAR(d.bbox.center.position.x, 0.45, 0.008);
    EXPECT_NEAR(d.bbox.center.position.y, 0.10, 0.008);
    EXPECT_NEAR(d.bbox.center.position.z, 0.025, 0.008);
    EXPECT_NEAR(d.bbox.size.x, 0.05, 0.01);
    EXPECT_NEAR(d.bbox.size.y, 0.07, 0.01);
  }
  rclcpp::shutdown();
}

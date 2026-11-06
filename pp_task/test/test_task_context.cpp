#include <cmath>
#include <map>
#include <string>

#include <gtest/gtest.h>

#include "pp_task/grasp_attacher.hpp"
#include "pp_task/task_context.hpp"

using pp_task::closestModelName;
using pp_task::fromDetections;
using pp_task::fromPoseMsg;
using pp_task::selectTarget;
using pp_task::toPoseMsg;

TEST(TaskContext, DetectionsAreConvertedBack)
{
  vision_msgs::msg::Detection3DArray msg;
  vision_msgs::msg::Detection3D d;
  d.bbox.center.position.x = 0.5;
  d.bbox.center.position.y = -0.1;
  d.bbox.center.position.z = 0.03;
  d.bbox.center.orientation.z = std::sin(0.2);
  d.bbox.center.orientation.w = std::cos(0.2);
  d.bbox.size.x = 0.04;
  d.bbox.size.y = 0.06;
  d.bbox.size.z = 0.05;
  vision_msgs::msg::ObjectHypothesisWithPose h;
  h.hypothesis.class_id = "cylinder";
  d.results.push_back(h);
  msg.detections.push_back(d);
  const auto objects = fromDetections(msg);
  ASSERT_EQ(objects.size(), 1u);
  EXPECT_NEAR(objects[0].yaw, 0.4, 1e-12);
  EXPECT_EQ(objects[0].shape, perception_core::Shape::kCylinder);
  EXPECT_NEAR(objects[0].size.y(), 0.06, 1e-12);
}

TEST(TaskContext, PoseRoundTrip)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(0.1, 0.2, 0.3);
  pose.linear() = Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
  EXPECT_TRUE(fromPoseMsg(toPoseMsg(pose)).isApprox(pose, 1e-12));
}

TEST(TaskContext, SelectTargetSkipsFailedObjects)
{
  perception_core::DetectedObject a;
  a.center = Eigen::Vector3d(0.4, 0.0, 0.02);
  perception_core::DetectedObject b;
  b.center = Eigen::Vector3d(0.5, 0.1, 0.02);
  EXPECT_NEAR(selectTarget({a, b}, {}, 0.05)->center.x(), 0.4, 1e-12);
  EXPECT_NEAR(
    selectTarget({a, b}, {Eigen::Vector3d(0.41, 0.0, 0.0)}, 0.05)->center.x(), 0.5, 1e-12);
  EXPECT_FALSE(selectTarget({a}, {a.center}, 0.05).has_value());
  EXPECT_FALSE(selectTarget({}, {}, 0.05).has_value());
}

TEST(TaskContext, RecordsAreClosedOnce)
{
  pp_task::TaskContext ctx;
  ctx.closeRecord(rclcpp::Time(1, 0, RCL_ROS_TIME));
  EXPECT_TRUE(ctx.records.empty());
  ctx.current = pp_task::PickRecord{};
  ctx.cycle_start = rclcpp::Time(10, 0, RCL_ROS_TIME);
  ctx.closeRecord(rclcpp::Time(12, 500000000, RCL_ROS_TIME));
  ctx.closeRecord(rclcpp::Time(13, 0, RCL_ROS_TIME));
  ASSERT_EQ(ctx.records.size(), 1u);
  EXPECT_DOUBLE_EQ(ctx.records.front().cycle_time, 2.5);
}

TEST(GraspAttacher, ClosestModelWithinRange)
{
  const std::map<std::string, Eigen::Vector3d> models{
    {"object_0", {0.4, 0.0, 0.02}}, {"object_1", {0.5, 0.1, 0.03}}};
  EXPECT_EQ(closestModelName(models, {0.49, 0.1, 0.05}, 0.1).value(), "object_1");
  EXPECT_FALSE(closestModelName(models, {0.0, 0.0, 0.5}, 0.1).has_value());
}

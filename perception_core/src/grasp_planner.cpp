#include "perception_core/grasp_planner.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace perception_core
{

namespace
{

double angleDistance(double a, double b)
{
  return std::abs(std::remainder(a - b, 2.0 * std::numbers::pi));
}

}  // namespace

Eigen::Matrix3d topDownOrientation(double closing_yaw)
{
  const Eigen::Vector3d y(std::cos(closing_yaw), std::sin(closing_yaw), 0.0);
  const Eigen::Vector3d z(0.0, 0.0, -1.0);
  Eigen::Matrix3d r;
  r.col(0) = y.cross(z);
  r.col(1) = y;
  r.col(2) = z;
  return r;
}

std::vector<Grasp> planTopDownGrasps(const DetectedObject & object, const GraspParams & params)
{
  std::vector<Grasp> grasps;
  const double width = object.size.x();  // narrow side (size.x <= size.y)
  if (width + 2.0 * params.finger_clearance > params.max_opening) {
    return grasps;
  }
  // Fingers close along the box x axis; for a cylinder every direction is equivalent.
  std::vector<double> yaws{object.yaw, object.yaw + std::numbers::pi};
  if (object.shape == Shape::kCylinder) {
    yaws = {0.0, std::numbers::pi / 2.0, std::numbers::pi, -std::numbers::pi / 2.0};
  }
  const double top = object.center.z() + 0.5 * object.size.z();
  const double depth = std::min(params.grasp_depth_ratio * object.size.z(), params.finger_length);

  for (const double yaw : yaws) {
    Grasp g;
    g.pose = Eigen::Isometry3d::Identity();
    g.pose.linear() = topDownOrientation(yaw);
    g.pose.translation() = Eigen::Vector3d(object.center.x(), object.center.y(), top - depth);
    g.pregrasp = g.pose;
    g.pregrasp.translation().z() += params.approach_distance;
    g.width = width;
    // Prefer the wrist rotation closest to the preferred yaw (less joint travel), then
    // narrower grasps (more margin in the fingers).
    g.score = -angleDistance(yaw, params.preferred_yaw) - width;
    grasps.push_back(g);
  }
  std::sort(grasps.begin(), grasps.end(), [](const Grasp & a, const Grasp & b) {
    return a.score > b.score;
  });
  return grasps;
}

}  // namespace perception_core

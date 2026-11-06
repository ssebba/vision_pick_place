#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "perception_core/grasp_planner.hpp"

using perception_core::DetectedObject;
using perception_core::GraspParams;
using perception_core::planTopDownGrasps;
using perception_core::Shape;
using perception_core::topDownOrientation;

namespace
{

DetectedObject makeBox(double yaw, double sx, double sy, double sz)
{
  DetectedObject o;
  o.center = Eigen::Vector3d(0.5, 0.1, 0.02 + sz / 2);
  o.yaw = yaw;
  o.size = Eigen::Vector3d(sx, sy, sz);
  o.shape = Shape::kBox;
  return o;
}

}  // namespace

TEST(GraspPlanner, OrientationPointsDownAndIsAProperRotation)
{
  const Eigen::Matrix3d r = topDownOrientation(0.7);
  EXPECT_NEAR(r.determinant(), 1.0, 1e-12);
  EXPECT_TRUE(r.col(2).isApprox(Eigen::Vector3d(0, 0, -1)));
  EXPECT_NEAR(std::atan2(r(1, 1), r(0, 1)), 0.7, 1e-12);
}

TEST(GraspPlanner, BoxIsGraspedAcrossItsNarrowSide)
{
  const auto grasps = planTopDownGrasps(makeBox(0.4, 0.05, 0.08, 0.06), GraspParams{});
  ASSERT_EQ(grasps.size(), 2u);
  for (const auto & g : grasps) {
    EXPECT_NEAR(g.width, 0.05, 1e-12);
    // Closing direction (tool y) along the box x axis, i.e. the narrow side.
    const Eigen::Vector3d closing = g.pose.linear().col(1);
    EXPECT_NEAR(std::abs(closing.dot(Eigen::Vector3d(std::cos(0.4), std::sin(0.4), 0))), 1.0, 1e-9);
    // Fingers 60% into the object from the top.
    EXPECT_NEAR(g.pose.translation().z(), 0.02 + 0.06 - 0.6 * 0.06, 1e-9);
    EXPECT_NEAR(g.pregrasp.translation().z() - g.pose.translation().z(), 0.12, 1e-9);
  }
  EXPECT_GE(grasps[0].score, grasps[1].score);
}

TEST(GraspPlanner, PrefersTheWristRotationClosestToThePreferredYaw)
{
  GraspParams params;
  params.preferred_yaw = std::numbers::pi;
  const auto grasps = planTopDownGrasps(makeBox(0.1, 0.05, 0.08, 0.06), params);
  ASSERT_FALSE(grasps.empty());
  const Eigen::Vector3d closing = grasps.front().pose.linear().col(1);
  EXPECT_LT(closing.x(), 0.0);  // yaw 0.1 + pi
}

TEST(GraspPlanner, TooWideObjectsAreRejected)
{
  EXPECT_TRUE(planTopDownGrasps(makeBox(0.0, 0.09, 0.1, 0.05), GraspParams{}).empty());
}

TEST(GraspPlanner, CylinderHasSeveralCandidates)
{
  auto cyl = makeBox(0.0, 0.06, 0.06, 0.1);
  cyl.shape = Shape::kCylinder;
  const auto grasps = planTopDownGrasps(cyl, GraspParams{});
  EXPECT_EQ(grasps.size(), 4u);
  // Long objects: the fingers do not go deeper than their length.
  EXPECT_NEAR(grasps.front().pose.translation().z(), 0.02 + 0.1 - 0.04, 1e-9);
}

#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "perception_core/object_detector.hpp"
#include "synthetic_scene.hpp"

using perception_core::DetectorParams;
using perception_core::ObjectDetector;
using perception_core::Shape;
using perception_core::test::SceneBuilder;

namespace
{

// Yaw of a box is defined up to a half turn.
double yawError(double a, double b) { return std::abs(std::remainder(a - b, std::numbers::pi)); }

}  // namespace

TEST(ObjectDetector, FindsTheTableAndNothingElse)
{
  const auto scene = SceneBuilder().table(-0.3, 0.9, -0.6, 0.6, 0.02);
  const auto result = ObjectDetector().detect(scene.cloud());
  EXPECT_TRUE(result.table_found);
  EXPECT_NEAR(result.table_height, 0.02, 0.003);
  EXPECT_TRUE(result.objects.empty());
}

TEST(ObjectDetector, BoxPoseSizeAndShape)
{
  // 5 x 8 x 6 cm box rotated by 30 deg; the narrow side (x) is 5 cm.
  const double yaw = 30.0 * std::numbers::pi / 180.0;
  const auto scene =
    SceneBuilder().table(-0.3, 0.9, -0.6, 0.6).box(0.4, 0.1, yaw, 0.05, 0.08, 0.06);
  const auto result = ObjectDetector().detect(scene.cloud());
  ASSERT_EQ(result.objects.size(), 1u);
  const auto & box = result.objects.front();
  EXPECT_EQ(box.shape, Shape::kBox);
  EXPECT_NEAR(box.center.x(), 0.4, 0.006);
  EXPECT_NEAR(box.center.y(), 0.1, 0.006);
  EXPECT_NEAR(box.center.z(), 0.03, 0.006);
  EXPECT_NEAR(box.size.x(), 0.05, 0.008);
  EXPECT_NEAR(box.size.y(), 0.08, 0.008);
  EXPECT_NEAR(box.size.z(), 0.06, 0.006);
  EXPECT_LT(yawError(box.yaw, yaw), 5.0 * std::numbers::pi / 180.0);
}

TEST(ObjectDetector, SquareBoxOrientationIsStillFound)
{
  // PCA has no preferred direction on a square: the minimum-area rectangle does.
  const double yaw = 20.0 * std::numbers::pi / 180.0;
  const auto scene =
    SceneBuilder().table(-0.3, 0.9, -0.6, 0.6).box(0.3, -0.2, yaw, 0.06, 0.06, 0.05);
  const auto result = ObjectDetector().detect(scene.cloud());
  ASSERT_EQ(result.objects.size(), 1u);
  EXPECT_EQ(result.objects.front().shape, Shape::kBox);
  // Square: yaw defined up to a quarter turn.
  const double e = std::abs(std::remainder(result.objects.front().yaw - yaw, std::numbers::pi / 2));
  EXPECT_LT(e, 5.0 * std::numbers::pi / 180.0);
}

TEST(ObjectDetector, CylinderIsClassified)
{
  const auto scene = SceneBuilder().table(-0.3, 0.9, -0.6, 0.6).cylinder(0.5, -0.2, 0.03, 0.10);
  const auto result = ObjectDetector().detect(scene.cloud());
  ASSERT_EQ(result.objects.size(), 1u);
  const auto & cyl = result.objects.front();
  EXPECT_EQ(cyl.shape, Shape::kCylinder);
  EXPECT_NEAR(cyl.center.x(), 0.5, 0.006);
  EXPECT_NEAR(cyl.center.y(), -0.2, 0.006);
  EXPECT_NEAR(cyl.size.x(), 0.06, 0.008);
  EXPECT_NEAR(cyl.size.z(), 0.10, 0.006);
}

// Depth images have stray points around the silhouettes, also at the height of the top face:
// they must not change the footprint, the height or the shape of the objects.
TEST(ObjectDetector, StrayPointsAroundTheObjectsAreIgnored)
{
  const double yaw = 50.0 * std::numbers::pi / 180.0;
  const auto scene = [yaw](bool strays) {
    SceneBuilder builder;
    builder.table(-0.3, 0.9, -0.6, 0.6)
      .box(0.4, 0.15, yaw, 0.045, 0.07, 0.05)
      .cylinder(0.45, -0.1, 0.025, 0.08);
    if (strays) {  // after the objects: same noise on their points
      builder.boxStrays(0.4, 0.15, yaw, 0.045, 0.07, 0.006, 0.056, 25)
        .cylinderStrays(0.45, -0.1, 0.025, 0.006, 0.086, 25);
    }
    return builder;
  };
  const auto clean = ObjectDetector().detect(scene(false).cloud());
  const auto noisy = ObjectDetector().detect(scene(true).cloud());
  ASSERT_EQ(clean.objects.size(), 2u);
  ASSERT_EQ(noisy.objects.size(), 2u);
  EXPECT_EQ(clean.objects[0].shape, Shape::kBox);  // sorted by distance: box first
  EXPECT_EQ(clean.objects[1].shape, Shape::kCylinder);
  for (std::size_t i = 0; i < 2; ++i) {
    const auto & a = clean.objects[i];
    const auto & b = noisy.objects[i];
    EXPECT_EQ(b.shape, a.shape);
    EXPECT_LT((b.size - a.size).cwiseAbs().maxCoeff(), 0.002) << "object " << i;
    EXPECT_LT((b.center - a.center).norm(), 0.002) << "object " << i;
  }
}

TEST(ObjectDetector, SeparatesSeveralObjectsSortedByDistance)
{
  const auto scene = SceneBuilder()
                       .table(-0.3, 0.9, -0.6, 0.6)
                       .box(0.6, 0.3, 0.0, 0.04, 0.06, 0.05)
                       .cylinder(0.3, -0.1, 0.025, 0.08)
                       .box(0.7, -0.3, 1.0, 0.05, 0.05, 0.05);
  const auto result = ObjectDetector().detect(scene.cloud());
  ASSERT_EQ(result.objects.size(), 3u);
  EXPECT_EQ(result.objects[0].shape, Shape::kCylinder);  // the closest to the origin
  for (std::size_t i = 1; i < result.objects.size(); ++i) {
    EXPECT_GE(
      result.objects[i].center.head<2>().norm(), result.objects[i - 1].center.head<2>().norm());
  }
}

TEST(ObjectDetector, IgnoresPointsOutsideTheWorkspace)
{
  DetectorParams params;
  params.crop_min = Eigen::Vector3f(-0.3F, -0.6F, -0.05F);
  params.crop_max = Eigen::Vector3f(0.9F, 0.0F, 0.4F);  // only y <= 0
  const auto scene = SceneBuilder()
                       .table(-0.3, 0.9, -0.6, 0.6)
                       .box(0.4, 0.3, 0.0, 0.05, 0.05, 0.05)
                       .box(0.4, -0.3, 0.0, 0.05, 0.05, 0.05);
  const auto result = ObjectDetector(params).detect(scene.cloud());
  ASSERT_EQ(result.objects.size(), 1u);
  EXPECT_LT(result.objects.front().center.y(), 0.0);
}

TEST(ObjectDetector, EmptyCloud)
{
  const auto result = ObjectDetector().detect(perception_core::Cloud());
  EXPECT_FALSE(result.table_found);
  EXPECT_TRUE(result.objects.empty());
}

#include <cmath>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>

#include "perception_core/object_detector.hpp"

using perception_core::convexHull;
using perception_core::minAreaRectangle;
using perception_core::Point2;
using perception_core::polygonArea;
using perception_core::trimmedRectangle;

namespace
{

// Points 2 mm apart filling a sx x sy rectangle centered at c, rotated by a.
std::vector<Point2> filledRectangle(const Point2 & c, double a, double sx, double sy)
{
  const Point2 u(std::cos(a), std::sin(a));
  const Point2 v(-u.y(), u.x());
  std::vector<Point2> points;
  for (double x = -sx / 2; x <= sx / 2 + 1e-9; x += 0.002) {
    for (double y = -sy / 2; y <= sy / 2 + 1e-9; y += 0.002) {
      points.push_back(c + x * u + y * v);
    }
  }
  return points;
}

}  // namespace

TEST(Geometry, ConvexHullDropsInteriorAndCollinearPoints)
{
  const auto hull =
    convexHull({{0, 0}, {1, 0}, {2, 0}, {2, 1}, {2, 2}, {0, 2}, {1, 1}, {0.5, 1.5}});
  EXPECT_EQ(hull.size(), 4u);
  EXPECT_NEAR(polygonArea(hull), 4.0, 1e-12);
}

TEST(Geometry, MinAreaRectangleOfARotatedRectangle)
{
  const double a = 0.6;
  const Point2 u(std::cos(a), std::sin(a));
  const Point2 v(-u.y(), u.x());
  const Point2 c(1.0, -2.0);
  const auto hull = convexHull(
    {c + 0.5 * u + 0.2 * v, c - 0.5 * u + 0.2 * v, c - 0.5 * u - 0.2 * v, c + 0.5 * u - 0.2 * v});
  const auto rect = minAreaRectangle(hull);
  EXPECT_NEAR(rect.width * rect.height, 0.4, 1e-9);
  EXPECT_TRUE(rect.center.isApprox(c, 1e-9));
  EXPECT_NEAR(std::max(rect.width, rect.height), 1.0, 1e-9);
  // The long side is parallel to u (angle defined up to a quarter turn).
  const double side = rect.width > rect.height ? rect.angle : rect.angle + std::numbers::pi / 2;
  EXPECT_NEAR(std::abs(std::sin(side - a)), 0.0, 1e-9);
}

TEST(Geometry, TrimmedRectangleIsTheCentralPartOfThePoints)
{
  const double a = 0.6;
  const Point2 c(0.4, -0.2);
  const auto rect = trimmedRectangle(filledRectangle(c, a, 0.08, 0.05), 0.02);
  // 2 % of the points left out beyond each side of a uniform distribution: 96 % of the extent.
  EXPECT_NEAR(std::max(rect.width, rect.height), 0.96 * 0.08, 0.002);
  EXPECT_NEAR(std::min(rect.width, rect.height), 0.96 * 0.05, 0.002);
  EXPECT_LT((rect.center - c).norm(), 0.001);
  const double side = rect.width > rect.height ? rect.angle : rect.angle + std::numbers::pi / 2;
  EXPECT_LT(std::abs(std::sin(side - a)), std::sin(5.0 * std::numbers::pi / 180.0));
}

TEST(Geometry, StrayPointsDoNotMoveTheTrimmedRectangle)
{
  const Point2 c(0.4, -0.2);
  std::vector<Point2> points = filledRectangle(c, 0.3, 0.05, 0.05);
  const auto clean = trimmedRectangle(points, 0.02);
  // 1 % of stray points up to 8 mm outside one side: the convex hull grows by 8 mm.
  const std::size_t strays = points.size() / 100;
  const Point2 u(std::cos(0.3), std::sin(0.3));
  const Point2 v(-u.y(), u.x());
  for (std::size_t i = 0; i < strays; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(strays);
    points.push_back(c + (0.025 + 0.008 * t) * u + (-0.025 + 0.05 * t) * v);
  }
  const auto rect = trimmedRectangle(points, 0.02);
  EXPECT_NEAR(rect.width, clean.width, 0.002);
  EXPECT_NEAR(rect.height, clean.height, 0.002);
  EXPECT_LT((rect.center - clean.center).norm(), 0.001);
}

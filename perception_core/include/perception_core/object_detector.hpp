#ifndef PERCEPTION_CORE__OBJECT_DETECTOR_HPP_
#define PERCEPTION_CORE__OBJECT_DETECTOR_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

namespace perception_core
{

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

enum class Shape
{
  kBox,
  kCylinder,
};

std::string toString(Shape shape);

struct DetectorParams
{
  // Workspace box in the cloud frame (the table and what lies on it).
  Eigen::Vector3f crop_min{-1.0F, -1.0F, -0.1F};
  Eigen::Vector3f crop_max{1.0F, 1.0F, 0.5F};
  float voxel_size{0.004F};
  // RANSAC on a plane perpendicular to +z (the table).
  float plane_distance{0.008F};
  float plane_max_angle{0.15F};  // [rad] from horizontal
  int plane_max_iterations{300};
  // Points closer than this to the table plane belong to it.
  float min_height{0.008F};
  float cluster_tolerance{0.015F};
  int min_cluster_size{40};
  int max_cluster_size{50000};
  // Top face of an object: its points within this distance of the top height.
  float top_band{0.01F};
  // Stray points of a depth image: top-face points farther than stray_margin outside the
  // central part of the top view (trimmed of footprint_trim of the points on each side, see
  // trimmedRectangle) are dropped before computing the footprint.
  float footprint_trim{0.02F};
  float stray_margin{0.003F};
  // Area of the hull of the top-face points / footprint area below this (and aspect ~1):
  // cylinder (pi/4), otherwise box (1).
  float cylinder_fill_ratio{0.88F};
  float cylinder_max_aspect{1.2F};
};

// An object resting on the table: oriented box in the cloud frame (z = table normal).
struct DetectedObject
{
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};  // center of the bounding box
  double yaw{0.0};  // rotation of the box x axis around z, in (-pi/2, pi/2]
  // Extents along the box x, y and z axes (size.x <= size.y).
  Eigen::Vector3d size{Eigen::Vector3d::Zero()};
  Shape shape{Shape::kBox};
  std::size_t num_points{0};
};

struct DetectionResult
{
  bool table_found{false};
  double table_height{0.0};
  std::vector<DetectedObject> objects;
};

/**
 * @brief Geometric object detection for tabletop scenes (PCL).
 *
 * crop box -> voxel grid -> RANSAC table plane -> points above the plane -> Euclidean
 * clustering -> per cluster: top face (points near the top height), stray points dropped
 * (outside the central part of the top view), footprint as the minimum-area rectangle of the
 * rest (rotating calipers on the convex hull, also defined for square objects where PCA is
 * not), height from the table, box/cylinder classification from the hull fill ratio.
 * The cloud must be expressed in a frame whose z axis is (approximately) vertical.
 */
class ObjectDetector
{
public:
  explicit ObjectDetector(DetectorParams params = {});

  DetectionResult detect(const Cloud & cloud) const;

  const DetectorParams & params() const { return params_; }

private:
  DetectorParams params_;
};

// 2D helpers (exposed for testing).
using Point2 = Eigen::Vector2d;

// Convex hull (Andrew's monotone chain), counter-clockwise, no collinear points.
std::vector<Point2> convexHull(std::vector<Point2> points);

struct Rectangle2
{
  Point2 center;
  double angle{0.0};  // direction of the first side
  double width{0.0};  // along `angle`
  double height{0.0};
};

// Minimum-area enclosing rectangle of a convex polygon (rotating calipers).
Rectangle2 minAreaRectangle(const std::vector<Point2> & hull);

// Central part of a set of points: the smallest rectangle, over orientations 1 deg apart, that
// leaves out a fraction `trim` of the points beyond each of its sides. A few stray points do
// not move it, unlike the convex hull.
Rectangle2 trimmedRectangle(const std::vector<Point2> & points, double trim);

double polygonArea(const std::vector<Point2> & polygon);

}  // namespace perception_core

#endif  // PERCEPTION_CORE__OBJECT_DETECTOR_HPP_

#include "perception_core/object_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <utility>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace perception_core
{

std::string toString(Shape shape) { return shape == Shape::kBox ? "box" : "cylinder"; }

namespace
{

double cross(const Point2 & o, const Point2 & a, const Point2 & b)
{
  return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
}

// Value at fraction q in [0, 1] of the values, which are partially reordered.
double quantile(std::vector<double> & values, double q)
{
  const auto k = static_cast<std::size_t>(std::lround(q * static_cast<double>(values.size() - 1)));
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(k), values.end());
  return values[k];
}

// Wraps an angle to (-pi/2, pi/2]: a rectangle is symmetric under a half turn.
double wrapHalfPi(double a)
{
  constexpr double kPi = std::numbers::pi;
  while (a > kPi / 2.0) {
    a -= kPi;
  }
  while (a <= -kPi / 2.0) {
    a += kPi;
  }
  return a;
}

}  // namespace

std::vector<Point2> convexHull(std::vector<Point2> points)
{
  if (points.size() < 3) {
    return points;
  }
  std::sort(points.begin(), points.end(), [](const Point2 & a, const Point2 & b) {
    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
  });
  std::vector<Point2> hull(2 * points.size());
  std::size_t k = 0;
  for (const auto & p : points) {  // lower hull
    while (k >= 2 && cross(hull[k - 2], hull[k - 1], p) <= 0.0) {
      --k;
    }
    hull[k++] = p;
  }
  for (std::size_t i = points.size() - 1, lower = k + 1; i > 0; --i) {  // upper hull
    const Point2 & p = points[i - 1];
    while (k >= lower && cross(hull[k - 2], hull[k - 1], p) <= 0.0) {
      --k;
    }
    hull[k++] = p;
  }
  hull.resize(k - 1);
  return hull;
}

double polygonArea(const std::vector<Point2> & polygon)
{
  double area = 0.0;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const Point2 & a = polygon[i];
    const Point2 & b = polygon[(i + 1) % polygon.size()];
    area += a.x() * b.y() - b.x() * a.y();
  }
  return 0.5 * std::abs(area);
}

Rectangle2 minAreaRectangle(const std::vector<Point2> & hull)
{
  Rectangle2 best;
  if (hull.empty()) {
    return best;
  }
  double best_area = std::numeric_limits<double>::infinity();
  // The optimal rectangle has a side collinear with an edge of the hull.
  for (std::size_t i = 0; i < hull.size(); ++i) {
    const Point2 edge = hull[(i + 1) % hull.size()] - hull[i];
    if (edge.norm() < 1e-12) {
      continue;
    }
    const Point2 u = edge.normalized();
    const Point2 v(-u.y(), u.x());
    double min_u = std::numeric_limits<double>::infinity();
    double max_u = -min_u;
    double min_v = min_u;
    double max_v = -min_u;
    for (const auto & p : hull) {
      min_u = std::min(min_u, p.dot(u));
      max_u = std::max(max_u, p.dot(u));
      min_v = std::min(min_v, p.dot(v));
      max_v = std::max(max_v, p.dot(v));
    }
    const double area = (max_u - min_u) * (max_v - min_v);
    if (area < best_area) {
      best_area = area;
      best.angle = std::atan2(u.y(), u.x());
      best.width = max_u - min_u;
      best.height = max_v - min_v;
      best.center = 0.5 * (min_u + max_u) * u + 0.5 * (min_v + max_v) * v;
    }
  }
  if (hull.size() < 3) {
    best.center = hull.front();
  }
  return best;
}

Rectangle2 trimmedRectangle(const std::vector<Point2> & points, double trim)
{
  Rectangle2 best;
  if (points.empty()) {
    return best;
  }
  double best_area = std::numeric_limits<double>::infinity();
  std::vector<double> along(points.size());
  std::vector<double> across(points.size());
  // A rectangle repeats every quarter turn.
  constexpr int kOrientations = 90;
  for (int i = 0; i < kOrientations; ++i) {
    const double angle = 0.5 * std::numbers::pi * i / kOrientations;
    const Point2 u(std::cos(angle), std::sin(angle));
    const Point2 v(-u.y(), u.x());
    for (std::size_t k = 0; k < points.size(); ++k) {
      along[k] = points[k].dot(u);
      across[k] = points[k].dot(v);
    }
    const double min_u = quantile(along, trim);
    const double max_u = quantile(along, 1.0 - trim);
    const double min_v = quantile(across, trim);
    const double max_v = quantile(across, 1.0 - trim);
    const double area = (max_u - min_u) * (max_v - min_v);
    if (area < best_area) {
      best_area = area;
      best.angle = angle;
      best.width = max_u - min_u;
      best.height = max_v - min_v;
      best.center = 0.5 * (min_u + max_u) * u + 0.5 * (min_v + max_v) * v;
    }
  }
  return best;
}

namespace
{

// Oriented box of one cluster, from its top face. Depth images have stray points (depth
// interpolated across the silhouette) around the outline at any height: the height is a high
// quantile, and the footprint ignores the points outside the central part of the top view.
std::optional<DetectedObject> objectFromCluster(
  const Cloud & points, const std::vector<int> & indices, const Eigen::Vector4d & plane,
  const DetectorParams & p)
{
  std::vector<double> heights;
  heights.reserve(indices.size());
  for (const int idx : indices) {
    const auto & pt = points[static_cast<std::size_t>(idx)];
    heights.push_back(plane.head<3>().dot(Eigen::Vector3d(pt.x, pt.y, pt.z)) + plane.w());
  }
  std::vector<double> sorted = heights;
  const double upper = quantile(sorted, 0.95);
  std::vector<double> near_top;
  for (const double h : heights) {
    if (std::abs(h - upper) <= p.top_band) {
      near_top.push_back(h);
    }
  }
  const double top = quantile(near_top, 0.5);  // not empty: `upper` is one of the heights
  std::vector<Point2> top_view;
  for (std::size_t k = 0; k < heights.size(); ++k) {
    if (heights[k] >= top - p.top_band) {
      const auto & pt = points[static_cast<std::size_t>(indices[k])];
      top_view.emplace_back(pt.x, pt.y);
    }
  }
  const Rectangle2 core = trimmedRectangle(top_view, p.footprint_trim);
  const Point2 u(std::cos(core.angle), std::sin(core.angle));
  const Point2 v(-u.y(), u.x());
  std::vector<Point2> inliers;
  for (const auto & q : top_view) {
    const Point2 d = q - core.center;
    if (
      std::abs(d.dot(u)) <= 0.5 * core.width + p.stray_margin &&
      std::abs(d.dot(v)) <= 0.5 * core.height + p.stray_margin) {
      inliers.push_back(q);
    }
  }
  const std::vector<Point2> hull = convexHull(inliers);
  if (hull.size() < 3) {
    return std::nullopt;
  }
  Rectangle2 rect = minAreaRectangle(hull);
  // Convention: size.x <= size.y (x is the narrow side, the natural closing direction).
  if (rect.width > rect.height) {
    std::swap(rect.width, rect.height);
    rect.angle += std::numbers::pi / 2.0;
  }
  DetectedObject obj;
  obj.yaw = wrapHalfPi(rect.angle);
  obj.size = Eigen::Vector3d(rect.width, rect.height, top);
  const double base_z =
    (-plane.w() - plane.x() * rect.center.x() - plane.y() * rect.center.y()) / plane.z();
  obj.center = Eigen::Vector3d(rect.center.x(), rect.center.y(), base_z + 0.5 * top);
  obj.num_points = indices.size();
  const double fill = polygonArea(hull) / std::max(rect.width * rect.height, 1e-12);
  const double aspect = rect.height / std::max(rect.width, 1e-12);
  obj.shape = (fill < p.cylinder_fill_ratio && aspect < p.cylinder_max_aspect) ? Shape::kCylinder
                                                                               : Shape::kBox;
  return obj;
}

}  // namespace

ObjectDetector::ObjectDetector(DetectorParams params) : params_(std::move(params)) {}

DetectionResult ObjectDetector::detect(const Cloud & cloud) const
{
  DetectionResult result;
  const auto & p = params_;

  // 1. Workspace crop and downsampling.
  Cloud::Ptr cropped(new Cloud);
  pcl::CropBox<pcl::PointXYZ> crop;
  crop.setInputCloud(cloud.makeShared());
  crop.setMin(Eigen::Vector4f(p.crop_min.x(), p.crop_min.y(), p.crop_min.z(), 1.0F));
  crop.setMax(Eigen::Vector4f(p.crop_max.x(), p.crop_max.y(), p.crop_max.z(), 1.0F));
  crop.filter(*cropped);
  Cloud::Ptr filtered(new Cloud);
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cropped);
  voxel.setLeafSize(p.voxel_size, p.voxel_size, p.voxel_size);
  voxel.filter(*filtered);
  if (filtered->size() < 50) {
    return result;
  }

  // 2. Table: dominant plane perpendicular to z.
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setAxis(Eigen::Vector3f::UnitZ());
  seg.setEpsAngle(p.plane_max_angle);
  seg.setDistanceThreshold(p.plane_distance);
  seg.setMaxIterations(p.plane_max_iterations);
  seg.setInputCloud(filtered);
  pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
  pcl::ModelCoefficients coefficients;
  seg.segment(*plane_inliers, coefficients);
  if (plane_inliers->indices.empty() || coefficients.values.size() != 4) {
    return result;
  }
  Eigen::Vector4d plane(
    coefficients.values[0], coefficients.values[1], coefficients.values[2], coefficients.values[3]);
  if (plane.z() < 0.0) {
    plane = -plane;  // normal pointing up
  }
  plane /= plane.head<3>().norm();
  result.table_found = true;
  // Table height below the workspace center.
  const Eigen::Vector3d mid = (0.5F * (p.crop_min + p.crop_max)).cast<double>();
  result.table_height = (-plane.w() - plane.x() * mid.x() - plane.y() * mid.y()) / plane.z();

  // 3. Points above the table.
  Cloud::Ptr above(new Cloud);
  for (const auto & pt : filtered->points) {
    const double h = plane.head<3>().dot(Eigen::Vector3d(pt.x, pt.y, pt.z)) + plane.w();
    if (h > p.min_height) {
      above->push_back(pt);
    }
  }
  if (above->empty()) {
    return result;
  }

  // 4. Euclidean clustering.
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(above);
  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(p.cluster_tolerance);
  ec.setMinClusterSize(p.min_cluster_size);
  ec.setMaxClusterSize(p.max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(above);
  ec.extract(clusters);

  // 5. One oriented box per cluster.
  for (const auto & cluster : clusters) {
    if (const auto object = objectFromCluster(*above, cluster.indices, plane, p)) {
      result.objects.push_back(*object);
    }
  }
  // Deterministic order: closest to the origin first.
  std::sort(
    result.objects.begin(), result.objects.end(),
    [](const DetectedObject & a, const DetectedObject & b) {
      return a.center.head<2>().norm() < b.center.head<2>().norm();
    });
  return result;
}

}  // namespace perception_core

#ifndef PERCEPTION_CORE__TEST__SYNTHETIC_SCENE_HPP_
#define PERCEPTION_CORE__TEST__SYNTHETIC_SCENE_HPP_

#include <cmath>
#include <numbers>
#include <random>

#include "perception_core/object_detector.hpp"

namespace perception_core::test
{

// Synthetic tabletop clouds as seen from above, with Gaussian sensor noise.
class SceneBuilder
{
public:
  explicit SceneBuilder(double noise = 0.001, unsigned seed = 7) : rng_(seed), noise_(0.0, noise) {}

  // Table surface z = height over [x0, x1] x [y0, y1].
  SceneBuilder & table(double x0, double x1, double y0, double y1, double height = 0.0)
  {
    for (double x = x0; x <= x1; x += kStep) {
      for (double y = y0; y <= y1; y += kStep) {
        add(x, y, height);
      }
    }
    return *this;
  }

  // Box standing on the table: top face and the four sides.
  SceneBuilder & box(
    double cx, double cy, double yaw, double sx, double sy, double sz, double base = 0.0)
  {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const auto put = [&](double u, double v, double z) {
      add(cx + c * u - s * v, cy + s * u + c * v, base + z);
    };
    for (double u = -sx / 2; u <= sx / 2 + 1e-9; u += kStep) {
      for (double v = -sy / 2; v <= sy / 2 + 1e-9; v += kStep) {
        put(u, v, sz);
      }
    }
    for (double z = kStep; z < sz; z += kStep) {
      for (double u = -sx / 2; u <= sx / 2 + 1e-9; u += kStep) {
        put(u, -sy / 2, z);
        put(u, sy / 2, z);
      }
      for (double v = -sy / 2; v <= sy / 2 + 1e-9; v += kStep) {
        put(-sx / 2, v, z);
        put(sx / 2, v, z);
      }
    }
    return *this;
  }

  // Upright cylinder: top disk and lateral surface.
  SceneBuilder & cylinder(double cx, double cy, double radius, double height, double base = 0.0)
  {
    for (double r = 0.0; r <= radius + 1e-9; r += kStep) {
      const int n = std::max(1, static_cast<int>(2.0 * std::numbers::pi * r / kStep));
      for (int i = 0; i < n; ++i) {
        const double a = 2.0 * std::numbers::pi * i / n;
        add(cx + r * std::cos(a), cy + r * std::sin(a), base + height);
      }
    }
    const int n = static_cast<int>(2.0 * std::numbers::pi * radius / kStep);
    for (double z = kStep; z < height; z += kStep) {
      for (int i = 0; i < n; ++i) {
        const double a = 2.0 * std::numbers::pi * i / n;
        add(cx + radius * std::cos(a), cy + radius * std::sin(a), base + z);
      }
    }
    return *this;
  }

  // Stray points of a depth image along the outline of a box seen from above: `count` points
  // up to `margin` outside its sides, at any height between 1 cm and `height`.
  SceneBuilder & boxStrays(
    double cx, double cy, double yaw, double sx, double sy, double margin, double height, int count)
  {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    for (int i = 0; i < count; ++i) {
      const double t = 2.0 * (sx + sy) * unit_(rng_);  // position along the perimeter
      const double out = margin * unit_(rng_);
      double u = -sx / 2 - out;
      double v = sy / 2 - (t - 2.0 * sx - sy);
      if (t < sx) {
        u = t - sx / 2;
        v = -sy / 2 - out;
      } else if (t < sx + sy) {
        u = sx / 2 + out;
        v = t - sx - sy / 2;
      } else if (t < 2.0 * sx + sy) {
        u = sx / 2 - (t - sx - sy);
        v = sy / 2 + out;
      }
      add(cx + c * u - s * v, cy + s * u + c * v, strayHeight(height));
    }
    return *this;
  }

  // Same around an upright cylinder.
  SceneBuilder & cylinderStrays(
    double cx, double cy, double radius, double margin, double height, int count)
  {
    for (int i = 0; i < count; ++i) {
      const double a = 2.0 * std::numbers::pi * unit_(rng_);
      const double r = radius + margin * unit_(rng_);
      add(cx + r * std::cos(a), cy + r * std::sin(a), strayHeight(height));
    }
    return *this;
  }

  const Cloud & cloud() const { return cloud_; }

private:
  static constexpr double kStep = 0.004;

  double strayHeight(double height) { return 0.01 + (height - 0.01) * unit_(rng_); }

  void add(double x, double y, double z)
  {
    cloud_.push_back(
      pcl::PointXYZ(
        static_cast<float>(x + noise_(rng_)), static_cast<float>(y + noise_(rng_)),
        static_cast<float>(z + noise_(rng_))));
  }

  std::mt19937 rng_;
  std::normal_distribution<double> noise_;
  std::uniform_real_distribution<double> unit_{0.0, 1.0};
  Cloud cloud_;
};

}  // namespace perception_core::test

#endif  // PERCEPTION_CORE__TEST__SYNTHETIC_SCENE_HPP_

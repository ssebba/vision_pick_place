#include "arm_control_core/joint_trajectory.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>

namespace arm_control_core
{

namespace
{

bool hasVelocity(const TrajectoryPoint & p) { return p.velocity.size() == p.position.size(); }
bool hasAcceleration(const TrajectoryPoint & p)
{
  return hasVelocity(p) && p.acceleration.size() == p.position.size();
}

// Quintic polynomial matching position, velocity and acceleration at both ends; h is the
// segment duration, t the time since its start.
JointState quintic(const TrajectoryPoint & a, const TrajectoryPoint & b, double t)
{
  const double h = b.time - a.time;
  const double h2 = h * h;
  const double h3 = h2 * h;
  const double h4 = h3 * h;
  const double h5 = h4 * h;
  const Vector & q0 = a.position;
  const Vector & v0 = a.velocity;
  const Vector & a0 = a.acceleration;
  const Vector & q1 = b.position;
  const Vector & v1 = b.velocity;
  const Vector & a1 = b.acceleration;
  const Vector c3 =
    (20.0 * (q1 - q0) - (8.0 * v1 + 12.0 * v0) * h - (3.0 * a0 - a1) * h2) / (2.0 * h3);
  const Vector c4 =
    (30.0 * (q0 - q1) + (14.0 * v1 + 16.0 * v0) * h + (3.0 * a0 - 2.0 * a1) * h2) / (2.0 * h4);
  const Vector c5 = (12.0 * (q1 - q0) - 6.0 * (v1 + v0) * h - (a0 - a1) * h2) / (2.0 * h5);
  JointState out;
  out.position =
    q0 + v0 * t + 0.5 * a0 * t * t + c3 * t * t * t + c4 * t * t * t * t + c5 * t * t * t * t * t;
  out.velocity = v0 + a0 * t + 3.0 * c3 * t * t + 4.0 * c4 * t * t * t + 5.0 * c5 * t * t * t * t;
  out.acceleration = a0 + 6.0 * c3 * t + 12.0 * c4 * t * t + 20.0 * c5 * t * t * t;
  return out;
}

// Cubic Hermite polynomial matching position and velocity at both ends.
JointState cubic(const TrajectoryPoint & a, const TrajectoryPoint & b, double t)
{
  const double h = b.time - a.time;
  const Vector & q0 = a.position;
  const Vector & v0 = a.velocity;
  const Vector & q1 = b.position;
  const Vector & v1 = b.velocity;
  const Vector c2 = (3.0 * (q1 - q0) - (2.0 * v0 + v1) * h) / (h * h);
  const Vector c3 = (2.0 * (q0 - q1) + (v0 + v1) * h) / (h * h * h);
  JointState out;
  out.position = q0 + v0 * t + c2 * t * t + c3 * t * t * t;
  out.velocity = v0 + 2.0 * c2 * t + 3.0 * c3 * t * t;
  out.acceleration = 2.0 * c2 + 6.0 * c3 * t;
  return out;
}

JointState linear(const TrajectoryPoint & a, const TrajectoryPoint & b, double t)
{
  const double h = b.time - a.time;
  JointState out;
  out.velocity = (b.position - a.position) / h;
  out.position = a.position + out.velocity * t;
  out.acceleration = Vector::Zero(a.position.size());
  return out;
}

}  // namespace

JointTrajectory::JointTrajectory(const JointState & start, std::vector<TrajectoryPoint> points)
{
  if (points.empty()) {
    return;
  }
  const auto n = start.position.size();
  TrajectoryPoint first;
  first.time = 0.0;
  first.position = start.position;
  first.velocity = start.velocity.size() == n ? start.velocity : Vector::Zero(n);
  first.acceleration = start.acceleration.size() == n ? start.acceleration : Vector::Zero(n);
  points_.reserve(points.size() + 1);
  points_.push_back(std::move(first));
  for (auto & p : points) {
    if (p.position.size() != n) {
      throw std::invalid_argument("JointTrajectory: inconsistent number of joints");
    }
    if (p.time <= points_.back().time) {
      // A point at t = 0 replaces the measured start state (MoveIt does this).
      if (p.time <= 0.0 && points_.size() == 1) {
        continue;
      }
      throw std::invalid_argument(
        "JointTrajectory: time_from_start must be strictly increasing (" + std::to_string(p.time) +
        " s)");
    }
    points_.push_back(std::move(p));
  }
  if (points_.size() == 1) {
    points_.clear();
  }
}

JointState JointTrajectory::sample(double t) const
{
  if (points_.empty()) {
    throw std::logic_error("JointTrajectory: sample() on an empty trajectory");
  }
  const auto n = points_.front().position.size();
  if (t >= points_.back().time) {
    return {points_.back().position, Vector::Zero(n), Vector::Zero(n)};
  }
  t = std::max(t, 0.0);
  // Segment containing t (points are sorted by time).
  const auto it = std::upper_bound(
    points_.begin(), points_.end(), t,
    [](double value, const TrajectoryPoint & p) { return value < p.time; });
  const TrajectoryPoint & b = *it;
  const TrajectoryPoint & a = *(it - 1);
  const double local = t - a.time;
  if (hasAcceleration(a) && hasAcceleration(b)) {
    return quintic(a, b, local);
  }
  if (hasVelocity(a) && hasVelocity(b)) {
    return cubic(a, b, local);
  }
  return linear(a, b, local);
}

}  // namespace arm_control_core

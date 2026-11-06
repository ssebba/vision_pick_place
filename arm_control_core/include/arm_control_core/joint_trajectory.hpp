#ifndef ARM_CONTROL_CORE__JOINT_TRAJECTORY_HPP_
#define ARM_CONTROL_CORE__JOINT_TRAJECTORY_HPP_

#include <vector>

#include "arm_control_core/robot_model.hpp"

namespace arm_control_core
{

struct TrajectoryPoint
{
  double time{0.0};  // [s] from the start of the trajectory
  Vector position;
  Vector velocity;      // optional (empty)
  Vector acceleration;  // optional (empty)
};

struct JointState
{
  Vector position;
  Vector velocity;
  Vector acceleration;
};

/**
 * @brief Time-parametrized joint trajectory, interpolated like ros2_controllers' JTC.
 *
 * Between two points the interpolation is quintic when both carry accelerations, cubic
 * (Hermite) when both carry velocities and linear otherwise. Before the first point the
 * trajectory starts from `start` (the measured state when it was received), after the last
 * one it holds the final position with zero velocity.
 */
class JointTrajectory
{
public:
  JointTrajectory() = default;
  JointTrajectory(const JointState & start, std::vector<TrajectoryPoint> points);

  bool empty() const { return points_.empty(); }
  double duration() const { return points_.empty() ? 0.0 : points_.back().time; }
  const std::vector<TrajectoryPoint> & points() const { return points_; }

  // Desired state at time t (from the start of the trajectory).
  JointState sample(double t) const;

private:
  std::vector<TrajectoryPoint> points_;  // points_[0] is the start state at t = 0
};

}  // namespace arm_control_core

#endif  // ARM_CONTROL_CORE__JOINT_TRAJECTORY_HPP_

#include <stdexcept>

#include <gtest/gtest.h>

#include "arm_control_core/joint_trajectory.hpp"

using arm_control_core::JointState;
using arm_control_core::JointTrajectory;
using arm_control_core::TrajectoryPoint;
using arm_control_core::Vector;

namespace
{

Vector vec(double a, double b) { return (Vector(2) << a, b).finished(); }

JointState rest(double a, double b) { return {vec(a, b), vec(0, 0), vec(0, 0)}; }

}  // namespace

TEST(JointTrajectory, QuinticMatchesBoundaryConditions)
{
  TrajectoryPoint p;
  p.time = 2.0;
  p.position = vec(1.0, -0.5);
  p.velocity = vec(0.2, 0.0);
  p.acceleration = vec(0.0, 0.1);
  const JointTrajectory traj(rest(0.0, 0.0), {p});
  const JointState start = traj.sample(0.0);
  EXPECT_TRUE(start.position.isApprox(vec(0, 0)));
  EXPECT_TRUE(start.velocity.isZero(1e-12));
  EXPECT_TRUE(start.acceleration.isZero(1e-12));
  const JointState end = traj.sample(2.0 - 1e-9);
  EXPECT_TRUE(end.position.isApprox(p.position, 1e-6));
  EXPECT_TRUE(end.velocity.isApprox(p.velocity, 1e-6));
  EXPECT_TRUE(end.acceleration.isApprox(p.acceleration, 1e-6));
}

TEST(JointTrajectory, CubicWhenOnlyVelocitiesAreGiven)
{
  TrajectoryPoint p;
  p.time = 1.0;
  p.position = vec(1.0, 1.0);
  p.velocity = vec(0.0, 0.0);
  const JointTrajectory traj(rest(0.0, 0.0), {p});
  // Symmetric cubic from rest to rest: half way at t = T/2 with peak velocity 1.5 h / T.
  const JointState mid = traj.sample(0.5);
  EXPECT_NEAR(mid.position(0), 0.5, 1e-12);
  EXPECT_NEAR(mid.velocity(0), 1.5, 1e-12);
}

TEST(JointTrajectory, LinearWithoutDerivatives)
{
  TrajectoryPoint p;
  p.time = 2.0;
  p.position = vec(2.0, -2.0);
  JointState start = rest(0.0, 0.0);
  start.velocity.resize(0);
  TrajectoryPoint q = p;
  const JointTrajectory traj(start, {q});
  // The start state has zero velocity, the point none: linear.
  const JointState s = traj.sample(1.0);
  EXPECT_TRUE(s.position.isApprox(vec(1.0, -1.0)));
  EXPECT_TRUE(s.velocity.isApprox(vec(1.0, -1.0)));
}

TEST(JointTrajectory, HoldsTheLastPointAfterTheEnd)
{
  TrajectoryPoint p;
  p.time = 1.0;
  p.position = vec(0.3, 0.4);
  p.velocity = vec(0.0, 0.0);
  const JointTrajectory traj(rest(0.0, 0.0), {p});
  const JointState after = traj.sample(5.0);
  EXPECT_TRUE(after.position.isApprox(p.position));
  EXPECT_TRUE(after.velocity.isZero());
  EXPECT_DOUBLE_EQ(traj.duration(), 1.0);
}

TEST(JointTrajectory, PointAtTimeZeroReplacesTheStartState)
{
  TrajectoryPoint p0;
  p0.time = 0.0;
  p0.position = vec(0.0, 0.0);
  TrajectoryPoint p1;
  p1.time = 1.0;
  p1.position = vec(1.0, 1.0);
  const JointTrajectory traj(rest(0.0, 0.0), {p0, p1});
  EXPECT_EQ(traj.points().size(), 2u);
}

TEST(JointTrajectory, RejectsNonIncreasingTimes)
{
  TrajectoryPoint p1;
  p1.time = 1.0;
  p1.position = vec(1.0, 1.0);
  TrajectoryPoint p2 = p1;
  p2.time = 0.5;
  EXPECT_THROW(JointTrajectory(rest(0, 0), {p1, p2}), std::invalid_argument);
  TrajectoryPoint wrong;
  wrong.time = 1.0;
  wrong.position = Vector::Zero(3);
  EXPECT_THROW(JointTrajectory(rest(0, 0), {wrong}), std::invalid_argument);
  EXPECT_THROW(JointTrajectory().sample(0.0), std::logic_error);
}

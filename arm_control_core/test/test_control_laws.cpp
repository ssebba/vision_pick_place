// Closed-loop tests: the arm is simulated by integrating the Pinocchio forward dynamics of
// the UR5e at 1 kHz, the same rate as the ros2_control loop in Gazebo.

#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "arm_control_core/control_laws.hpp"
#include "test_utils.hpp"

using namespace arm_control_core;  // NOLINT(google-build-using-namespace)

namespace
{

constexpr double kDt = 1e-3;

// Smooth reference: each joint oscillates around the test configuration.
JointState sinusoid(double t)
{
  const Vector q0 = test::testConfiguration();
  Vector amp(6);
  amp << 0.4, 0.3, 0.3, 0.5, 0.5, 0.6;
  const double w = 2.0 * std::numbers::pi * 0.5;  // 0.5 Hz
  JointState s;
  s.position = q0 + amp * std::sin(w * t);
  s.velocity = amp * w * std::cos(w * t);
  s.acceleration = -amp * w * w * std::sin(w * t);
  return s;
}

double maxTrackingError(const test::SimulationResult & r, double from_time)
{
  double worst = 0.0;
  for (std::size_t i = 0; i < r.q.size(); ++i) {
    const double t = static_cast<double>(i + 1) * kDt;
    if (t >= from_time) {
      const double e = (r.q[i] - sinusoid(t).position).cwiseAbs().maxCoeff();
      // NaN (divergence) must not be swallowed by std::max.
      worst = std::isnan(e) ? e : std::max(worst, e);
      if (std::isnan(worst)) {
        return worst;
      }
    }
  }
  return worst;
}

}  // namespace

TEST(ControlLaws, SaturateClampsEachJoint)
{
  Vector tau(3);
  tau << 200.0, -200.0, 5.0;
  Vector lim(3);
  lim << 150.0, 150.0, 0.0;
  const Vector s = saturate(tau, lim);
  EXPECT_DOUBLE_EQ(s(0), 150.0);
  EXPECT_DOUBLE_EQ(s(1), -150.0);
  EXPECT_DOUBLE_EQ(s(2), 5.0);  // limit 0 = unlimited
}

TEST(ComputedTorque, TracksASinusoidWithAPerfectModel)
{
  RobotModel controller_model = test::makeUr5e();
  RobotModel plant = test::makeUr5e();
  const ComputedTorqueLaw law(controller_model, ComputedTorqueLaw::fromBandwidth(6, 30.0, 1.0));
  const auto result = test::simulate(
    plant, sinusoid(0.0).position, sinusoid(0.0).velocity, 4.0, kDt,
    [&](double t, const Vector & q, const Vector & v) { return law.compute(q, v, sinusoid(t)); });
  EXPECT_LT(maxTrackingError(result, 0.5), 2e-3);
}

namespace
{

// Max tracking error of a law on the UR5e whose wrist payload is `payload` x the model one.
template <typename Law>
double trackingError(const Law & law, double payload)
{
  RobotModel plant = test::makeUr5e();
  plant.scalePayload(payload);
  return maxTrackingError(
    test::simulate(
      plant, sinusoid(0.0).position, sinusoid(0.0).velocity, 4.0, kDt,
      [&](double t, const Vector & q, const Vector & v) { return law.compute(q, v, sinusoid(t)); }),
    0.5);
}

}  // namespace

// The point of model-based control: with the same nominal bandwidth on every joint, the
// computed-torque law cancels the coupled, configuration-dependent dynamics that a
// PD + gravity controller treats as disturbances.
TEST(ComputedTorque, OutperformsPdGravity)
{
  RobotModel model = test::makeUr5e();
  const auto gains = ComputedTorqueLaw::fromBandwidth(6, 30.0, 1.0);
  // Fair PD gains: same bandwidth, i.e. scaled by the joint inertia (diagonal of M).
  const Vector inertia = model.massMatrix(test::testConfiguration()).diagonal();
  const PdGravityLaw pd(model, {gains.kp.cwiseProduct(inertia), gains.kd.cwiseProduct(inertia)});
  const ComputedTorqueLaw ctc(model, gains);

  const double ctc_error = trackingError(ctc, 1.0);
  const double pd_error = trackingError(pd, 1.0);
  ASSERT_FALSE(std::isnan(pd_error));
  EXPECT_LT(ctc_error, 0.1 * pd_error) << "ctc " << ctc_error << " pd " << pd_error;

  // With a wrist payload 3x heavier than modeled the advantage shrinks but remains.
  const double ctc_mismatch = trackingError(ctc, 3.0);
  const double pd_mismatch = trackingError(pd, 3.0);
  ASSERT_FALSE(std::isnan(ctc_mismatch));
  EXPECT_LT(ctc_mismatch, 0.08);
  EXPECT_LT(ctc_mismatch, pd_mismatch) << "ctc " << ctc_mismatch << " pd " << pd_mismatch;
}

TEST(CartesianImpedance, HoldsTheTargetWithoutExternalForces)
{
  RobotModel controller_model = test::makeUr5e();
  RobotModel plant = test::makeUr5e();
  Eigen::Matrix<double, 6, 1> k;
  k << 800.0, 800.0, 800.0, 60.0, 60.0, 60.0;
  auto params = CartesianImpedanceLaw::Parameters::critical(k);
  params.posture = test::testConfiguration();
  const CartesianImpedanceLaw law(controller_model, params);
  const Vector q0 = test::testConfiguration();
  const Eigen::Isometry3d target = controller_model.endEffectorPose(q0);
  const auto result = test::simulate(
    plant, q0, Vector::Zero(6), 2.0, kDt,
    [&](double, const Vector & q, const Vector & v) { return law.compute(q, v, target); });
  const Eigen::Isometry3d final_pose = plant.endEffectorPose(result.q.back());
  EXPECT_LT((final_pose.translation() - target.translation()).norm(), 1e-3);
}

TEST(CartesianImpedance, SaturatedErrorBoundsTheForce)
{
  RobotModel model = test::makeUr5e();
  Eigen::Matrix<double, 6, 1> k;
  k << 500.0, 500.0, 500.0, 50.0, 50.0, 50.0;
  auto params = CartesianImpedanceLaw::Parameters::critical(k);
  params.max_position_error = 0.05;
  const CartesianImpedanceLaw law(model, params);
  const Vector q = test::testConfiguration();
  Eigen::Isometry3d far = model.endEffectorPose(q);
  far.translation().x() += 1.0;  // 1 m away: the spring must act as if 5 cm away
  Eigen::Isometry3d near = model.endEffectorPose(q);
  near.translation().x() += 0.05;
  const Vector zero = Vector::Zero(6);
  EXPECT_TRUE(law.compute(q, zero, far).isApprox(law.compute(q, zero, near), 1e-9));
}

TEST(CartesianImpedance, ComplianceMatchesTheStiffness)
{
  RobotModel controller_model = test::makeUr5e();
  RobotModel plant = test::makeUr5e();
  Eigen::Matrix<double, 6, 1> k;
  k << 500.0, 500.0, 500.0, 50.0, 50.0, 50.0;
  auto params = CartesianImpedanceLaw::Parameters::critical(k);
  params.posture = test::testConfiguration();
  const CartesianImpedanceLaw law(controller_model, params);
  const Vector q0 = test::testConfiguration();
  const Eigen::Isometry3d target = controller_model.endEffectorPose(q0);

  // 10 N push along +x at the end effector: tau_ext = J^T F.
  Eigen::Matrix<double, 6, 1> push = Eigen::Matrix<double, 6, 1>::Zero();
  push(0) = 10.0;
  const auto result = test::simulate(
    plant, q0, Vector::Zero(6), 3.0, kDt,
    [&](double, const Vector & q, const Vector & v) { return law.compute(q, v, target); },
    [&](const Vector & q) -> Vector { return plant.endEffectorJacobian(q).transpose() * push; });
  const Eigen::Vector3d displacement =
    plant.endEffectorPose(result.q.back()).translation() - target.translation();
  // Expected static deflection F / K = 2 cm along x, nothing in the other directions.
  EXPECT_NEAR(displacement.x(), 10.0 / 500.0, 0.003);
  EXPECT_NEAR(displacement.y(), 0.0, 0.003);
  EXPECT_NEAR(displacement.z(), 0.0, 0.003);
}

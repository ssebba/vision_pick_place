#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>
#include <Eigen/Eigenvalues>

#include "arm_control_core/control_laws.hpp"
#include "test_utils.hpp"

using namespace arm_control_core;  // NOLINT(google-build-using-namespace)

TEST(RobotModel, LoadsTheUr5e)
{
  RobotModel model = test::makeUr5e();
  EXPECT_EQ(model.dof(), 6);
  EXPECT_EQ(model.jointNames().front(), "shoulder_pan_joint");
  // UR5e datasheet: 150 Nm on the first three joints, 28 Nm on the wrist.
  EXPECT_NEAR(model.effortLimits()(0), 150.0, 1e-9);
  EXPECT_NEAR(model.effortLimits()(5), 28.0, 1e-9);
}

TEST(RobotModel, RejectsUnknownJointsAndWrongOrder)
{
  const std::string urdf = test::readUr5eUrdf();
  EXPECT_THROW(RobotModel(urdf, {"no_such_joint"}, "tool0"), std::invalid_argument);
  auto reversed = test::ur5eJoints();
  std::swap(reversed[0], reversed[1]);
  EXPECT_THROW(RobotModel(urdf, reversed, "tool0"), std::invalid_argument);
  EXPECT_THROW(RobotModel(urdf, test::ur5eJoints(), "no_such_frame"), std::invalid_argument);
}

TEST(RobotModel, MassMatrixIsSymmetricPositiveDefinite)
{
  RobotModel model = test::makeUr5e();
  const Matrix m = model.massMatrix(test::testConfiguration());
  EXPECT_TRUE(m.isApprox(m.transpose(), 1e-12));
  const Eigen::SelfAdjointEigenSolver<Matrix> eig(m);
  EXPECT_GT(eig.eigenvalues().minCoeff(), 0.0);
}

TEST(RobotModel, InverseAndForwardDynamicsAreConsistent)
{
  RobotModel model = test::makeUr5e();
  const Vector q = test::testConfiguration();
  Vector v(6);
  v << 0.5, -0.3, 0.2, 0.1, -0.4, 0.6;
  Vector a(6);
  a << 1.0, -2.0, 0.5, 3.0, -1.0, 0.2;
  const Vector tau = model.inverseDynamics(q, v, a);
  EXPECT_TRUE(model.forwardDynamics(q, v, tau).isApprox(a, 1e-9));
  // tau = M a + C v + g
  EXPECT_TRUE(tau.isApprox(model.massMatrix(q) * a + model.nonLinearEffects(q, v), 1e-9));
  EXPECT_TRUE(model.nonLinearEffects(q, Vector::Zero(6)).isApprox(model.gravity(q), 1e-12));
}

TEST(RobotModel, JacobianMatchesFiniteDifferencesOfTheKinematics)
{
  RobotModel model = test::makeUr5e();
  const Vector q = test::testConfiguration();
  const Jacobian j = model.endEffectorJacobian(q);
  const Eigen::Isometry3d x0 = model.endEffectorPose(q);
  const double eps = 1e-7;
  for (int i = 0; i < 6; ++i) {
    Vector dq = Vector::Zero(6);
    dq(i) = eps;
    const Eigen::Isometry3d x1 = model.endEffectorPose(q + dq);
    // poseError(x1, x0) ~ J dq for small dq.
    const Eigen::Matrix<double, 6, 1> numeric = poseError(x1, x0) / eps;
    EXPECT_TRUE(numeric.isApprox(j.col(i), 1e-5)) << "column " << i;
  }
}

TEST(RobotModel, PayloadScalingChangesGravityOnly)
{
  RobotModel model = test::makeUr5e();
  const Vector q = test::testConfiguration();
  const Vector g_nominal = model.gravity(q);
  model.scalePayload(3.0);
  const Vector g_heavy = model.gravity(q);
  EXPECT_GT((g_heavy - g_nominal).norm(), 1e-3);
  EXPECT_THROW(model.gravity(Vector::Zero(5)), std::invalid_argument);
}

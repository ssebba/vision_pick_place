#include "arm_control_core/control_laws.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <Eigen/Cholesky>
#include <Eigen/LU>

namespace arm_control_core
{

Vector saturate(const Vector & tau, const Vector & limits)
{
  Vector out = tau;
  for (Eigen::Index i = 0; i < out.size() && i < limits.size(); ++i) {
    if (limits(i) > 0.0) {
      out(i) = std::clamp(out(i), -limits(i), limits(i));
    }
  }
  return out;
}

ComputedTorqueLaw::ComputedTorqueLaw(RobotModel & model, Gains gains) : model_(&model)
{
  setGains(std::move(gains));
}

ComputedTorqueLaw::Gains ComputedTorqueLaw::fromBandwidth(
  int dof, double natural_frequency, double damping_ratio)
{
  return {
    Vector::Constant(dof, natural_frequency * natural_frequency),
    Vector::Constant(dof, 2.0 * damping_ratio * natural_frequency)};
}

void ComputedTorqueLaw::setGains(Gains gains)
{
  if (gains.kp.size() != model_->dof() || gains.kd.size() != model_->dof()) {
    throw std::invalid_argument("ComputedTorqueLaw: gains do not match the number of joints");
  }
  gains_ = std::move(gains);
}

Vector ComputedTorqueLaw::compute(
  const Vector & q, const Vector & v, const JointState & desired) const
{
  const Vector a_cmd = desired.acceleration + gains_.kp.cwiseProduct(desired.position - q) +
                       gains_.kd.cwiseProduct(desired.velocity - v);
  return model_->inverseDynamics(q, v, a_cmd);
}

PdGravityLaw::PdGravityLaw(RobotModel & model, ComputedTorqueLaw::Gains gains)
: model_(&model), gains_(std::move(gains))
{
}

Vector PdGravityLaw::compute(const Vector & q, const Vector & v, const JointState & desired) const
{
  return gains_.kp.cwiseProduct(desired.position - q) +
         gains_.kd.cwiseProduct(desired.velocity - v) + model_->gravity(q);
}

Eigen::Matrix<double, 6, 1> poseError(
  const Eigen::Isometry3d & target, const Eigen::Isometry3d & current)
{
  Eigen::Matrix<double, 6, 1> e;
  e.head<3>() = target.translation() - current.translation();
  const Eigen::AngleAxisd aa(target.linear() * current.linear().transpose());
  e.tail<3>() = aa.angle() * aa.axis();
  return e;
}

CartesianImpedanceLaw::Parameters CartesianImpedanceLaw::Parameters::critical(
  const Eigen::Matrix<double, 6, 1> & stiffness)
{
  Parameters p;
  p.stiffness = stiffness;
  p.damping_ratio = 1.0;
  return p;
}

CartesianImpedanceLaw::CartesianImpedanceLaw(RobotModel & model, Parameters params) : model_(&model)
{
  setParameters(std::move(params));
}

void CartesianImpedanceLaw::setParameters(Parameters params)
{
  if (params.posture.size() != 0 && params.posture.size() != model_->dof()) {
    throw std::invalid_argument("CartesianImpedanceLaw: posture does not match the joints");
  }
  params_ = std::move(params);
}

Vector CartesianImpedanceLaw::compute(
  const Vector & q, const Vector & v, const Eigen::Isometry3d & target) const
{
  const Eigen::Isometry3d current = model_->endEffectorPose(q);
  const Jacobian j = model_->endEffectorJacobian(q);
  const Eigen::Matrix<double, 6, 1> twist = j * v;

  // Task-space inertia Lambda = (J M^-1 J^T)^-1 (damped inverse near singularities).
  const Matrix m_inv_jt = model_->massMatrix(q).ldlt().solve(j.transpose());
  const Eigen::Matrix<double, 6, 6> lambda_inv =
    j * m_inv_jt + 1e-6 * Eigen::Matrix<double, 6, 6>::Identity();
  const Eigen::Matrix<double, 6, 1> lambda_diag =
    lambda_inv.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity()).diagonal();
  const Eigen::Matrix<double, 6, 1> damping =
    2.0 * params_.damping_ratio *
    (params_.stiffness.cwiseProduct(lambda_diag.cwiseMax(0.0))).cwiseSqrt();

  Eigen::Matrix<double, 6, 1> error = poseError(target, current);
  const auto clamp_norm = [](auto segment, double max_norm) {
    const double n = segment.norm();
    if (max_norm > 0.0 && n > max_norm) {
      segment *= max_norm / n;
    }
  };
  clamp_norm(error.head<3>(), params_.max_position_error);
  clamp_norm(error.tail<3>(), params_.max_orientation_error);

  const Eigen::Matrix<double, 6, 1> wrench =
    params_.stiffness.cwiseProduct(error) - damping.cwiseProduct(twist);

  Vector tau = j.transpose() * wrench + model_->nonLinearEffects(q, v);

  if (params_.posture.size() == model_->dof()) {
    // Damped least-squares pseudo-inverse J^+ = J^T (J J^T + l^2 I)^-1.
    const double l2 = params_.pseudo_inverse_damping * params_.pseudo_inverse_damping;
    const Eigen::Matrix<double, 6, 6> jjt =
      j * j.transpose() + l2 * Eigen::Matrix<double, 6, 6>::Identity();
    const Matrix j_pinv = j.transpose() * jjt.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());
    const Matrix nullspace = Matrix::Identity(model_->dof(), model_->dof()) - j_pinv * j;
    const Vector posture_tau =
      params_.nullspace_stiffness * (params_.posture - q) - params_.nullspace_damping * v;
    tau += nullspace.transpose() * posture_tau;
  }
  return tau;
}

}  // namespace arm_control_core

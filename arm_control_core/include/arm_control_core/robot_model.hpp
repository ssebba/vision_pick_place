#ifndef ARM_CONTROL_CORE__ROBOT_MODEL_HPP_
#define ARM_CONTROL_CORE__ROBOT_MODEL_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace arm_control_core
{

using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;
using Jacobian = Eigen::Matrix<double, 6, Eigen::Dynamic>;

/**
 * @brief Rigid-body model of a serial arm (Pinocchio behind a pimpl).
 *
 * Built from a URDF string. Only the `joints` listed are degrees of freedom, in that order;
 * every other joint of the URDF (e.g. gripper fingers) is locked at its neutral position,
 * so its links still contribute to the dynamics as rigid payload.
 *
 * Not thread-safe: the algorithms reuse an internal workspace (no allocations in the
 * control loop). Use one instance per thread.
 */
class RobotModel
{
public:
  RobotModel(
    const std::string & urdf_xml, const std::vector<std::string> & joints,
    const std::string & end_effector_frame);
  ~RobotModel();
  RobotModel(RobotModel &&) noexcept;
  RobotModel & operator=(RobotModel &&) noexcept;
  RobotModel(const RobotModel &) = delete;
  RobotModel & operator=(const RobotModel &) = delete;

  int dof() const;
  const std::vector<std::string> & jointNames() const;

  // Limits read from the URDF.
  const Vector & lowerPositionLimits() const;
  const Vector & upperPositionLimits() const;
  const Vector & velocityLimits() const;
  const Vector & effortLimits() const;

  // tau = M(q) a + C(q, v) v + g(q)   (recursive Newton-Euler)
  Vector inverseDynamics(const Vector & q, const Vector & v, const Vector & a);
  // a = M(q)^-1 (tau - C(q, v) v - g(q))   (articulated body algorithm)
  Vector forwardDynamics(const Vector & q, const Vector & v, const Vector & tau);
  Vector gravity(const Vector & q);
  // C(q, v) v + g(q)
  Vector nonLinearEffects(const Vector & q, const Vector & v);
  Matrix massMatrix(const Vector & q);

  // End-effector pose in the model root frame.
  Eigen::Isometry3d endEffectorPose(const Vector & q);
  // Geometric Jacobian of the end effector: [v; omega] = J(q) qdot, both expressed in the
  // root frame at the end-effector origin (Pinocchio LOCAL_WORLD_ALIGNED).
  Jacobian endEffectorJacobian(const Vector & q);

  // Scales the mass and inertia of the end-effector link (model-mismatch experiments).
  void scalePayload(double factor);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace arm_control_core

#endif  // ARM_CONTROL_CORE__ROBOT_MODEL_HPP_

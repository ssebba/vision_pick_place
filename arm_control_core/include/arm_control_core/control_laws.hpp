#ifndef ARM_CONTROL_CORE__CONTROL_LAWS_HPP_
#define ARM_CONTROL_CORE__CONTROL_LAWS_HPP_

#include <Eigen/Geometry>

#include "arm_control_core/joint_trajectory.hpp"
#include "arm_control_core/robot_model.hpp"

namespace arm_control_core
{

// Saturates each torque to +-limit (limits <= 0 are ignored).
Vector saturate(const Vector & tau, const Vector & limits);

/**
 * @brief Computed-torque (inverse dynamics) joint trajectory tracking.
 *
 *   tau = M(q) (qdd_d + Kp e + Kd edot) + C(q, qd) qd + g(q),   e = q_d - q
 *
 * evaluated in one recursive Newton-Euler pass. With a perfect model the closed loop is
 * a set of decoupled linear systems edd + Kd edot + Kp e = 0: Kp = w^2, Kd = 2 zeta w.
 */
class ComputedTorqueLaw
{
public:
  struct Gains
  {
    Vector kp;
    Vector kd;
  };

  ComputedTorqueLaw(RobotModel & model, Gains gains);

  // Gains for a closed-loop natural frequency w [rad/s] and damping ratio zeta on all joints.
  static Gains fromBandwidth(int dof, double natural_frequency, double damping_ratio);

  Vector compute(const Vector & q, const Vector & v, const JointState & desired) const;

  void setGains(Gains gains);
  const Gains & gains() const { return gains_; }

private:
  RobotModel * model_;
  Gains gains_;
};

/**
 * @brief Joint PD with gravity compensation: tau = Kp e + Kd edot + g(q).
 * The classic baseline the computed-torque law is compared against.
 */
class PdGravityLaw
{
public:
  PdGravityLaw(RobotModel & model, ComputedTorqueLaw::Gains gains);
  Vector compute(const Vector & q, const Vector & v, const JointState & desired) const;

private:
  RobotModel * model_;
  ComputedTorqueLaw::Gains gains_;
};

// Pose error [position; orientation] of `target` w.r.t. `current`, both in the base frame;
// the orientation part is the rotation vector (log map) of R_target R_current^T.
Eigen::Matrix<double, 6, 1> poseError(
  const Eigen::Isometry3d & target, const Eigen::Isometry3d & current);

/**
 * @brief Cartesian impedance control with gravity and Coriolis compensation.
 *
 *   F   = K e - D J qd                                    (spring-damper at the flange)
 *   tau = J^T F + C(q, qd) qd + g(q) + N^T (Kn (q_n - q) - Dn qd)
 *
 * where N = I - J^+ J projects the posture task into the null space of the Cartesian
 * task (J^+ damped least-squares pseudo-inverse). Without external forces the end
 * effector settles on the target; an external wrench F_ext moves it by about K^-1 F_ext.
 *
 * The damping is recomputed every cycle from the task-space inertia
 * Lambda = (J M^-1 J^T)^-1 as D = 2 zeta sqrt(K Lambda_ii): a constant D = 2 sqrt(K) only
 * fits a unit mass and makes the light wrist joints numerically unstable at 1 kHz.
 */
class CartesianImpedanceLaw
{
public:
  struct Parameters
  {
    Eigen::Matrix<double, 6, 1> stiffness;  // [N/m x3, Nm/rad x3]
    double damping_ratio{1.0};              // zeta of each Cartesian direction
    double nullspace_stiffness{10.0};       // [Nm/rad]
    double nullspace_damping{0.5};          // [Nms/rad]
    double pseudo_inverse_damping{1e-2};
    // The spring error is saturated (bounded force for distant targets); <= 0 disables.
    double max_position_error{0.0};     // [m]
    double max_orientation_error{0.0};  // [rad]
    Vector posture;                     // q_n; empty = posture task disabled

    static Parameters critical(const Eigen::Matrix<double, 6, 1> & stiffness);
  };

  CartesianImpedanceLaw(RobotModel & model, Parameters params);

  Vector compute(const Vector & q, const Vector & v, const Eigen::Isometry3d & target) const;

  void setParameters(Parameters params);
  const Parameters & parameters() const { return params_; }

private:
  RobotModel * model_;
  Parameters params_;
};

}  // namespace arm_control_core

#endif  // ARM_CONTROL_CORE__CONTROL_LAWS_HPP_

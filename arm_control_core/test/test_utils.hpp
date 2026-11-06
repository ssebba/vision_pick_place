#ifndef ARM_CONTROL_CORE__TEST__TEST_UTILS_HPP_
#define ARM_CONTROL_CORE__TEST__TEST_UTILS_HPP_

#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include "arm_control_core/robot_model.hpp"

namespace arm_control_core::test
{

inline const std::vector<std::string> & ur5eJoints()
{
  static const std::vector<std::string> joints{"shoulder_pan_joint", "shoulder_lift_joint",
                                               "elbow_joint",        "wrist_1_joint",
                                               "wrist_2_joint",      "wrist_3_joint"};
  return joints;
}

inline std::string readUr5eUrdf()
{
  std::ifstream file(UR5E_URDF);
  std::stringstream ss;
  ss << file.rdbuf();
  return ss.str();
}

inline RobotModel makeUr5e() { return {readUr5eUrdf(), ur5eJoints(), "tool0"}; }

// A configuration away from singularities.
inline Vector testConfiguration()
{
  Vector q(6);
  q << 0.3, -1.2, 1.4, -1.8, -1.5, 0.4;
  return q;
}

/**
 * Closed-loop simulation of the arm: semi-implicit Euler integration of the forward dynamics
 * of `plant` at `dt`, with `controller(t, q, v)` returning the joint torques and
 * `external(q)` an optional external joint torque (e.g. J^T F).
 */
struct SimulationResult
{
  std::vector<Vector> q;
  std::vector<Vector> v;
};

inline SimulationResult simulate(
  RobotModel & plant, Vector q, Vector v, double duration, double dt,
  const std::function<Vector(double, const Vector &, const Vector &)> & controller,
  const std::function<Vector(const Vector &)> & external = nullptr)
{
  SimulationResult out;
  for (double t = 0.0; t < duration; t += dt) {
    Vector tau = controller(t, q, v);
    if (external) {
      tau += external(q);
    }
    const Vector a = plant.forwardDynamics(q, v, tau);
    v += dt * a;
    q += dt * v;
    out.q.push_back(q);
    out.v.push_back(v);
  }
  return out;
}

}  // namespace arm_control_core::test

#endif  // ARM_CONTROL_CORE__TEST__TEST_UTILS_HPP_

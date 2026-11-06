#include "arm_control_core/robot_model.hpp"

// Pinocchio is confined to this translation unit: its headers are heavy and need specific
// Boost preprocessor settings, which stay private to this library.
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace arm_control_core
{

struct RobotModel::Impl
{
  pinocchio::Model model;
  pinocchio::Data data;
  pinocchio::FrameIndex ee_frame{0};
  std::vector<std::string> joints;
  Vector lower;
  Vector upper;
  Vector velocity;
  Vector effort;

  void check(const Vector & v, const char * what) const
  {
    if (v.size() != model.nv) {
      throw std::invalid_argument(
        std::string("RobotModel: ") + what + " has size " + std::to_string(v.size()) +
        ", expected " + std::to_string(model.nv));
    }
  }
};

RobotModel::RobotModel(
  const std::string & urdf_xml, const std::vector<std::string> & joints,
  const std::string & end_effector_frame)
: impl_(std::make_unique<Impl>())
{
  pinocchio::Model full;
  pinocchio::urdf::buildModelFromXML(urdf_xml, full);

  // Lock every joint that is not controlled, in its neutral configuration.
  std::vector<pinocchio::JointIndex> locked;
  for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(full.njoints); ++j) {
    if (std::find(joints.begin(), joints.end(), full.names[j]) == joints.end()) {
      locked.push_back(j);
    }
  }
  for (const auto & name : joints) {
    if (!full.existJointName(name)) {
      throw std::invalid_argument("RobotModel: joint '" + name + "' not found in the URDF");
    }
    if (full.joints[full.getJointId(name)].nv() != 1) {
      throw std::invalid_argument("RobotModel: joint '" + name + "' is not a 1-DoF joint");
    }
  }
  impl_->model =
    locked.empty() ? full : pinocchio::buildReducedModel(full, locked, pinocchio::neutral(full));
  // The reduced model keeps the URDF order: check that it matches the requested order.
  for (std::size_t i = 0; i < joints.size(); ++i) {
    if (impl_->model.names[i + 1] != joints[i]) {
      throw std::invalid_argument(
        "RobotModel: joints must be listed in kinematic order (expected '" +
        impl_->model.names[i + 1] + "' at position " + std::to_string(i) + ")");
    }
  }
  if (!impl_->model.existFrame(end_effector_frame)) {
    throw std::invalid_argument("RobotModel: frame '" + end_effector_frame + "' not found");
  }
  impl_->ee_frame = impl_->model.getFrameId(end_effector_frame);
  impl_->data = pinocchio::Data(impl_->model);
  impl_->joints = joints;
  impl_->lower = impl_->model.lowerPositionLimit;
  impl_->upper = impl_->model.upperPositionLimit;
  impl_->velocity = impl_->model.velocityLimit;
  impl_->effort = impl_->model.effortLimit;
}

RobotModel::~RobotModel() = default;
RobotModel::RobotModel(RobotModel &&) noexcept = default;
RobotModel & RobotModel::operator=(RobotModel &&) noexcept = default;

int RobotModel::dof() const { return impl_->model.nv; }
const std::vector<std::string> & RobotModel::jointNames() const { return impl_->joints; }
const Vector & RobotModel::lowerPositionLimits() const { return impl_->lower; }
const Vector & RobotModel::upperPositionLimits() const { return impl_->upper; }
const Vector & RobotModel::velocityLimits() const { return impl_->velocity; }
const Vector & RobotModel::effortLimits() const { return impl_->effort; }

Vector RobotModel::inverseDynamics(const Vector & q, const Vector & v, const Vector & a)
{
  impl_->check(q, "q");
  impl_->check(v, "v");
  impl_->check(a, "a");
  return pinocchio::rnea(impl_->model, impl_->data, q, v, a);
}

Vector RobotModel::forwardDynamics(const Vector & q, const Vector & v, const Vector & tau)
{
  impl_->check(q, "q");
  impl_->check(v, "v");
  impl_->check(tau, "tau");
  return pinocchio::aba(impl_->model, impl_->data, q, v, tau, pinocchio::Convention::WORLD);
}

Vector RobotModel::gravity(const Vector & q)
{
  impl_->check(q, "q");
  return pinocchio::computeGeneralizedGravity(impl_->model, impl_->data, q);
}

Vector RobotModel::nonLinearEffects(const Vector & q, const Vector & v)
{
  impl_->check(q, "q");
  impl_->check(v, "v");
  return pinocchio::nonLinearEffects(impl_->model, impl_->data, q, v);
}

Matrix RobotModel::massMatrix(const Vector & q)
{
  impl_->check(q, "q");
  pinocchio::crba(impl_->model, impl_->data, q, pinocchio::Convention::WORLD);
  // CRBA fills the upper triangle only.
  Matrix m = impl_->data.M;
  m.triangularView<Eigen::StrictlyLower>() = m.transpose().triangularView<Eigen::StrictlyLower>();
  return m;
}

Eigen::Isometry3d RobotModel::endEffectorPose(const Vector & q)
{
  impl_->check(q, "q");
  pinocchio::framesForwardKinematics(impl_->model, impl_->data, q);
  const auto & world_to_ee = impl_->data.oMf[impl_->ee_frame];
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = world_to_ee.rotation();
  pose.translation() = world_to_ee.translation();
  return pose;
}

Jacobian RobotModel::endEffectorJacobian(const Vector & q)
{
  impl_->check(q, "q");
  Jacobian j = Jacobian::Zero(6, impl_->model.nv);
  pinocchio::computeFrameJacobian(
    impl_->model, impl_->data, q, impl_->ee_frame, pinocchio::LOCAL_WORLD_ALIGNED, j);
  return j;
}

void RobotModel::scalePayload(double factor)
{
  const auto joint = impl_->model.frames[impl_->ee_frame].parentJoint;
  auto & inertia = impl_->model.inertias[joint];
  inertia = pinocchio::Inertia(
    inertia.mass() * factor, inertia.lever(), inertia.inertia().matrix() * factor);
}

}  // namespace arm_control_core

#ifndef ARM_CONTROLLERS__TEST__CONTROLLER_TEST_UTILS_HPP_
#define ARM_CONTROLLERS__TEST__CONTROLLER_TEST_UTILS_HPP_

#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

#include "arm_control_core/robot_model.hpp"

namespace arm_controllers::test
{

inline const std::vector<std::string> & joints()
{
  static const std::vector<std::string> names{"shoulder_pan_joint", "shoulder_lift_joint",
                                              "elbow_joint",        "wrist_1_joint",
                                              "wrist_2_joint",      "wrist_3_joint"};
  return names;
}

inline std::string urdf()
{
  std::ifstream file(UR5E_URDF);
  std::stringstream ss;
  ss << file.rdbuf();
  return ss.str();
}

inline arm_control_core::Vector homeConfiguration()
{
  arm_control_core::Vector q(6);
  q << 0.3, -1.2, 1.4, -1.8, -1.5, 0.4;
  return q;
}

/**
 * Stand-in for the hardware handled by the controller manager: one position and velocity
 * state interface and one effort command interface per joint, in the order requested by
 * the controllers (all positions, then all velocities).
 */
class FakeArm
{
public:
  FakeArm()
  {
    for (const auto & j : joints()) {
      positions_.push_back(
        std::make_shared<hardware_interface::StateInterface>(j, "position", "double", "0.0"));
      velocities_.push_back(
        std::make_shared<hardware_interface::StateInterface>(j, "velocity", "double", "0.0"));
      efforts_.push_back(
        std::make_shared<hardware_interface::CommandInterface>(j, "effort", "double", "0.0"));
    }
  }

  void assignTo(controller_interface::ControllerInterface & controller)
  {
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    std::vector<hardware_interface::LoanedStateInterface> states;
    for (auto & e : efforts_) {
      commands.emplace_back(e);
    }
    for (auto & p : positions_) {
      states.emplace_back(p);
    }
    for (auto & v : velocities_) {
      states.emplace_back(v);
    }
    controller.assign_interfaces(std::move(commands), std::move(states));
  }

  void setState(const arm_control_core::Vector & q, const arm_control_core::Vector & v)
  {
    for (std::size_t i = 0; i < positions_.size(); ++i) {
      (void)positions_[i]->set_value(q(static_cast<Eigen::Index>(i)), true);
      (void)velocities_[i]->set_value(v(static_cast<Eigen::Index>(i)), true);
    }
  }

  arm_control_core::Vector efforts() const
  {
    arm_control_core::Vector tau(static_cast<Eigen::Index>(efforts_.size()));
    for (std::size_t i = 0; i < efforts_.size(); ++i) {
      tau(static_cast<Eigen::Index>(i)) = efforts_[i]->get_optional<double>().value_or(0.0);
    }
    return tau;
  }

private:
  std::vector<hardware_interface::StateInterface::SharedPtr> positions_;
  std::vector<hardware_interface::StateInterface::SharedPtr> velocities_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> efforts_;
};

// Initializes a controller like the controller manager does (robot_description + parameters).
inline void initController(
  controller_interface::ControllerInterface & controller, const std::string & name,
  std::vector<rclcpp::Parameter> parameters)
{
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = name;
  params.robot_description = urdf();
  params.update_rate = 1000;
  params.controller_manager_update_rate = 1000;
  parameters.emplace_back("joints", joints());
  params.node_options.parameter_overrides(parameters);
  if (controller.init(params) != controller_interface::return_type::OK) {
    throw std::runtime_error("controller init failed");
  }
}

}  // namespace arm_controllers::test

#endif  // ARM_CONTROLLERS__TEST__CONTROLLER_TEST_UTILS_HPP_

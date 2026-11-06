#ifndef PERCEPTION_CORE__GRASP_PLANNER_HPP_
#define PERCEPTION_CORE__GRASP_PLANNER_HPP_

#include <vector>

#include <Eigen/Geometry>

#include "perception_core/object_detector.hpp"

namespace perception_core
{

struct GraspParams
{
  double max_opening{0.085};       // [m] distance between the open fingers
  double finger_clearance{0.01};   // [m] free space required on each side when approaching
  double finger_length{0.04};      // [m] usable depth of the fingers
  double grasp_depth_ratio{0.6};   // fraction of the object height inside the fingers
  double approach_distance{0.12};  // [m] pre-grasp height above the grasp pose
  double preferred_yaw{0.0};       // [rad] tool yaw preferred when several are possible
};

/**
 * @brief Top-down parallel-jaw grasp candidate.
 *
 * Tool frame convention (TCP between the fingertips): z = approach direction (pointing
 * down at the object), y = finger closing direction, x = y cross z.
 */
struct Grasp
{
  Eigen::Isometry3d pose;      // TCP at the grasp
  Eigen::Isometry3d pregrasp;  // same orientation, approach_distance above
  double width{0.0};           // object width between the fingers [m]
  double score{0.0};           // higher is better
};

// Tool orientation for a top-down grasp whose fingers close along `closing_yaw`.
Eigen::Matrix3d topDownOrientation(double closing_yaw);

/**
 * Candidates sorted by score. Boxes are grasped across their narrow side (two candidates
 * related by a half turn of the wrist), cylinders across their diameter at a few yaws.
 * Objects wider than the gripper opening yield no candidate.
 */
std::vector<Grasp> planTopDownGrasps(const DetectedObject & object, const GraspParams & params);

}  // namespace perception_core

#endif  // PERCEPTION_CORE__GRASP_PLANNER_HPP_

// m3_grasp.cpp — Milestone M3, first pass: pad-centre-corrected grasp +
//                grasp-success verification. NOT the whole M3 milestone.
//
// EVIDENCE THIS PRODUCES
//   Approach to a pad-centre-corrected grasp target (reusing M2's proven
//   two-stage joint+Cartesian pattern) and a bounded gripper close-and-hold —
//   NOT trusting the ROS action's own stalled:true result for the achieved
//   position (see gripper_close_and_hold). See docs/HANDOFF_M3.md,
//   "Pad-centre correction and grasp-success verification: design, not yet
//   implemented" (now implemented, this file) and the section above it,
//   "box-settle false-quiescence", for why stalled:true alone is not
//   sufficient evidence of a real grasp. The grasp VERDICT itself —
//   whether the object was actually carried — is not decided here at all;
//   see "GRASP-SUCCESS VERIFICATION, NOT THE ACTION RESULT" below.
//
// WHAT THIS DELIBERATELY DOES NOT DO YET
//   No lift, no retreat, no attachObject, no post-lift slip check. Those are
//   still-open design items (see HANDOFF_M3.md's "Open design item, not yet
//   resolved" under the box-settle section — the slip baseline needs a
//   windowed object-settle wait before capture, not yet built) and are
//   deliberately out of scope for this pass: this node proves the corrected
//   grasp target and the verification logic work, before anything is built
//   on top of them.
//
// PAD-CENTRE CORRECTION
//   M2 targets tool0 such that grasp_tcp (tool0 + tcp_offset along local Z)
//   lands at grasp_frame — correct for M2's own purpose (open-loop TCP
//   accuracy in free space, which is literally how tcp_offset was
//   measured). This node targets tool0 + (tcp_offset + pad_centre_offset)
//   instead: Blocker 2 proved geometrically that the true pad contact
//   surface sits pad_centre_offset FARTHER from tool0 than tcp_offset alone
//   implies (the object seats DOWN from the fingertip-link-origin height
//   onto the true pad centre by that much). NOT YET LIVE-VERIFIED: the
//   direction (add, not subtract) is reasoned from geometry, not confirmed
//   by comparing achieved contact geometry against the uncorrected
//   baseline — this project has been burned twice already by sign/frame
//   mixups of exactly this shape (tcp_offset's original anchor-frame
//   confusion; the shortfall-vs-achieved-angle units error). The
//   grasp-success check below is the first live check of whether this
//   correction actually improves anything.
//
// GRASP-SUCCESS VERIFICATION IS NOT ANGLE-BASED
//   After gripper_close_and_hold-equivalent logic settles on an achieved
//   joint angle, that angle is compared against config/grasp_table.yaml's
//   grip_angle_rad (interpolated for object.size[grasp_width_axis]) within
//   grasp.grasp_tolerance_rad and logged — INFORMATIONAL ONLY. It used to
//   abort the cycle (GRIPPER_GOAL_REJECTED) outside tolerance; that was
//   removed. Reasoning: against a rigid object the joint physically cannot
//   advance past geometric touch (DART will not allow real interpenetration),
//   so squeeze is a force, not an angle, and no angle band — wide or narrow
//   — can serve as a grasp verdict. A wide band (the one this file shipped
//   with first) passes a bare-touch non-grasp as WITHIN TOLERANCE; a narrow
//   one written to demand real squeeze (grasp.squeeze) would fail every
//   real grasp too, including good ones, for the same underlying reason.
//   Transport is attempted on every cycle that produced a real close sample
//   regardless of this check (see attempted_transport below); the actual
//   verdict — whether the object was carried — comes from Gazebo's own pose
//   ground truth via scripts/lib/slip.py, same separation of concerns
//   transport.hpp documents for attachObject.
//
// NOTHING IS HARDCODED
//   Every threshold, offset and table value arrives as a ROS parameter from
//   config/scene.yaml / config/grasp_table.yaml via the launch file. The
//   only compile-time constants are the wrist_3_link -> flange -> tool0
//   fixed rotations, same as M2, for the same reason (properties of the
//   URDF, not tunable scene parameters).

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/gripper_command.hpp>

#include <array>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "ur5e_pick_place/failure.hpp"
#include "ur5e_pick_place/gz_topic_utils.hpp"
#include "ur5e_pick_place/mask_orientation.hpp"
#include "ur5e_pick_place/moveit_compat.hpp"
#include <moveit/robot_state/robot_state.hpp>
#include "ur5e_pick_place/planning_scene_manager.hpp"
#include "ur5e_pick_place/transport.hpp"

using ur5e_pick_place::Result;
using ur5e_pick_place::to_string;
using ur5e_pick_place::run_command;
using ur5e_pick_place::parse_joint_position;
using namespace std::chrono_literals;
using GripperCommand = control_msgs::action::GripperCommand;

namespace
{
constexpr char kPlanningGroup[] = "arm";

// wrist_3_link -> flange -> tool0, confirmed by recon: fixed joints, zero
// translation both. Same constants as m2_cartesian_approach.cpp and
// scripts/05_measure_gripper_geometry.sh.
tf2::Matrix3x3 R_from_rpy(double roll, double pitch, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return tf2::Matrix3x3(q);
}

const tf2::Matrix3x3 kR_wrist3_to_flange = R_from_rpy(0.0, -M_PI_2, -M_PI_2);
const tf2::Matrix3x3 kR_flange_to_tool0 = R_from_rpy(M_PI_2, 0.0, M_PI_2);

struct PregraspCandidate {
  std::vector<double> joint_positions;
  double d_descent;
  double d_transit;
  double cartesian_fraction;
  size_t waypoint_count;
};

static double wrap_angle(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}

// Stage-2C intentionally supports yaw changes only for the existing vertical
// pick geometry. General tilted-approach composition is a separate problem:
// rejecting it here keeps the validated local-Z/downward contract explicit.
constexpr double kYawGeometryToleranceRad = 1.0e-4;
constexpr double kYawQuaternionNormTolerance = 1.0e-3;

bool perceived_yaw_configuration_valid(bool use_perceived_yaw, bool use_perceived_position)
{
  return !use_perceived_yaw || use_perceived_position;
}

Result perceived_yaw_sample_result(bool have_sample)
{
  return have_sample ? Result::SUCCESS : Result::PERCEPTION_TIMEOUT;
}

std::optional<double> planar_yaw_from_valid_quaternion(
  const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & q_msg = pose.pose.orientation;
  if (!std::isfinite(q_msg.x) || !std::isfinite(q_msg.y) ||
    !std::isfinite(q_msg.z) || !std::isfinite(q_msg.w))
  {
    return std::nullopt;
  }
  const double norm_sq = q_msg.x * q_msg.x + q_msg.y * q_msg.y +
    q_msg.z * q_msg.z + q_msg.w * q_msg.w;
  if (!std::isfinite(norm_sq) || std::abs(norm_sq - 1.0) > kYawQuaternionNormTolerance) {
    return std::nullopt;
  }
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  q.normalize();
  const tf2::Matrix3x3 basis(q);
  const double yaw = std::atan2(basis[1][0], basis[0][0]);
  return std::isfinite(yaw) ? std::optional<double>(yaw) : std::nullopt;
}

// This is telemetry-only when perceived yaw is disabled: failure to obtain a
// configured yaw must not alter the established configured-yaw manipulation
// path. Perceived-yaw targeting performs the additional approach validation
// below and turns invalid geometry into CONFIG_ERROR.
std::optional<double> configured_object_planar_yaw(const tf2::Transform & T_world_object)
{
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  T_world_object.getBasis().getRPY(roll, pitch, yaw);
  if (!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw) ||
    std::abs(roll) > kYawGeometryToleranceRad ||
    std::abs(pitch) > kYawGeometryToleranceRad)
  {
    return std::nullopt;
  }
  return yaw;
}

bool is_fresh_world_pose(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & world_frame,
  const rclcpp::Time & m1_stationary_stamp)
{
  return pose.header.frame_id == world_frame &&
    rclcpp::Time(pose.header.stamp) > m1_stationary_stamp;
}

bool configured_yaw_reference_supported(
  const tf2::Transform & T_world_object,
  const tf2::Transform & T_world_grasp,
  double & configured_yaw,
  std::string & error)
{
  const auto yaw = configured_object_planar_yaw(T_world_object);
  if (!yaw) {
    error = "configured object frame is not level (roll/pitch must be approximately zero)";
    return false;
  }

  const tf2::Vector3 local_z_in_world =
    T_world_grasp.getBasis() * tf2::Vector3(0.0, 0.0, 1.0);
  const tf2::Vector3 expected_world_down(0.0, 0.0, -1.0);
  if (!std::isfinite(local_z_in_world.x()) || !std::isfinite(local_z_in_world.y()) ||
    !std::isfinite(local_z_in_world.z()) ||
    (local_z_in_world - expected_world_down).length() > kYawGeometryToleranceRad)
  {
    error = "grasp approach local +Z is not approximately world -Z";
    return false;
  }
  configured_yaw = *yaw;
  return true;
}

tf2::Matrix3x3 grasp_basis_with_perceived_yaw(
  const tf2::Matrix3x3 & existing_basis,
  bool use_perceived_yaw,
  double perceived_yaw,
  double configured_yaw)
{
  if (!use_perceived_yaw) {
    return existing_basis;
  }
  const double delta = ur5e_pick_place::axial_difference(perceived_yaw, configured_yaw);
  return R_from_rpy(0.0, 0.0, delta) * existing_basis;
}

static bool select_deterministic_pregrasp_branch(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const geometry_msgs::msg::Pose & pregrasp_pose,
  const geometry_msgs::msg::Pose & grasp_pose,
  const std::vector<std::string> & arm_joint_names,
  const std::string & tool0_frame,
  double eef_step,
  double cartesian_fraction_min,
  const rclcpp::Logger & logger,
  std::vector<double> & out_selected_joints)
{
  const auto robot_model = move_group.getRobotModel();
  if (!robot_model) {
    RCLCPP_ERROR(logger, "DETERMINISTIC_SELECTOR_ERROR: RobotModel is null.");
    return false;
  }
  const auto * jmg = robot_model->getJointModelGroup(kPlanningGroup);
  if (!jmg) {
    RCLCPP_ERROR(logger, "DETERMINISTIC_SELECTOR_ERROR: JointModelGroup 'arm' is null.");
    return false;
  }

  // Get current state (M1) as reference for transit distance
  const auto current_state = move_group.getCurrentState(1.0);
  if (!current_state) {
    RCLCPP_ERROR(logger, "DETERMINISTIC_SELECTOR_ERROR: Current RobotState is null.");
    return false;
  }
  std::vector<double> q_m1;
  current_state->copyJointGroupPositions(jmg, q_m1);

  // Deterministic numerical KDL seeds
  const std::vector<std::vector<double>> fixed_seeds = {
    {-0.571183951, -0.907168565, 1.521906541, 0.956058351, 1.570796327, 0.999612376}, // Seed B (Branch B prototype)
    {-0.571183951, -1.268026458, 2.127412001, -2.430181870, -1.570796327, -2.141980278}, // Seed A (Branch A prototype)
    q_m1, // M1 start state
    {-0.571183951, -2.0, -1.5, 0.0, 1.570796327, 0.0},
    {-0.571183951, -2.0, -1.5, 0.0, -1.570796327, 0.0}
  };

  std::vector<std::vector<double>> valid_unique_candidates;
  size_t seeds_tested = fixed_seeds.size();
  size_t ik_converged = 0;
  size_t fk_passed = 0;
  size_t bounds_passed = 0;

  for (const auto & seed : fixed_seeds) {
    auto state = std::make_shared<moveit::core::RobotState>(robot_model);
    state->setToDefaultValues();
    state->setJointGroupPositions(jmg, seed);
    state->update();

    if (!state->setFromIK(jmg, pregrasp_pose, tool0_frame, 0.010)) {
      continue;
    }
    ik_converged++;

    std::vector<double> q_cand;
    state->copyJointGroupPositions(jmg, q_cand);

    // 1. Verify FK tolerance (< 1.0 mm position, < 0.01 rad orientation)
    const auto & tf_cand = state->getGlobalLinkTransform(tool0_frame);
    const auto & p_cand = tf_cand.translation();
    double pos_err = std::sqrt(
      std::pow(p_cand.x() - pregrasp_pose.position.x, 2) +
      std::pow(p_cand.y() - pregrasp_pose.position.y, 2) +
      std::pow(p_cand.z() - pregrasp_pose.position.z, 2));

    Eigen::Quaterniond q_actual(tf_cand.rotation());
    tf2::Quaternion q_target(pregrasp_pose.orientation.x, pregrasp_pose.orientation.y, pregrasp_pose.orientation.z, pregrasp_pose.orientation.w);
    double dot = std::abs(q_actual.x() * q_target.x() + q_actual.y() * q_target.y() + q_actual.z() * q_target.z() + q_actual.w() * q_target.w());
    double rot_err = 2.0 * std::acos(std::min(1.0, dot));

    if (pos_err > 0.001 || rot_err > 0.01) {
      continue;
    }
    fk_passed++;

    // 2. Validate joint limits with safety margin 0.10 rad
    if (!state->satisfiesBounds(jmg, 0.10)) {
      continue;
    }
    bounds_passed++;

    // 3. Deduplicate using wrapped max-joint distance < 0.01 rad
    bool is_dup = false;
    for (const auto & existing : valid_unique_candidates) {
      double max_d = 0.0;
      for (size_t i = 0; i < q_cand.size(); ++i) {
        max_d = std::max(max_d, std::abs(wrap_angle(q_cand[i] - existing[i])));
      }
      if (max_d < 0.01) {
        is_dup = true;
        break;
      }
    }
    if (is_dup) {
      continue;
    }

    valid_unique_candidates.push_back(q_cand);
  }

  RCLCPP_INFO(
    logger,
    "DETERMINISTIC_SELECTOR_CANDIDATES: seeds_tested=%zu ik_converged=%zu fk_passed=%zu bounds_passed=%zu unique_candidates=%zu",
    seeds_tested, ik_converged, fk_passed, bounds_passed, valid_unique_candidates.size());

  if (valid_unique_candidates.empty()) {
    RCLCPP_ERROR(logger, "DETERMINISTIC_SELECTOR_FAIL: No valid unique IK candidates generated from seeds.");
    return false;
  }

  // 4. Cartesian dry-run evaluation & ranking for each candidate
  std::vector<PregraspCandidate> evaluated_candidates;

  for (const auto & q_cand : valid_unique_candidates) {
    auto cand_state = std::make_shared<moveit::core::RobotState>(robot_model);
    cand_state->setToDefaultValues();
    cand_state->setJointGroupPositions(jmg, q_cand);
    cand_state->update();

    move_group.setStartState(*cand_state);
    moveit_msgs::msg::RobotTrajectory traj;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    double fraction = move_group.computeCartesianPath({grasp_pose}, eef_step, traj, true, &error_code);
    move_group.setStartStateToCurrentState();

    if (fraction < cartesian_fraction_min) {
      RCLCPP_INFO(logger, "DETERMINISTIC_SELECTOR: candidate rejected, Cartesian fraction %.4f < %.4f", fraction, cartesian_fraction_min);
      continue;
    }

    const auto & pts = traj.joint_trajectory.points;
    if (pts.size() < 2) {
      RCLCPP_INFO(logger, "DETERMINISTIC_SELECTOR: candidate rejected, Cartesian path has < 2 points.");
      continue;
    }

    // Explicit continuity check: reject any consecutive wrapped joint step > 0.15 rad
    bool continuous = true;
    for (size_t k = 0; k + 1 < pts.size(); ++k) {
      for (size_t i = 0; i < 6; ++i) {
        double step_delta = std::abs(wrap_angle(pts[k+1].positions[i] - pts[k].positions[i]));
        if (step_delta > 0.15) {
          continuous = false;
          RCLCPP_INFO(
            logger,
            "DETERMINISTIC_SELECTOR: candidate rejected due to joint jump: step %zu joint %zu delta=%.4f > 0.15 rad",
            k, i, step_delta);
          break;
        }
      }
      if (!continuous) break;
    }
    if (!continuous) continue;

    // Compute FULL trajectory cumulative travel per joint
    std::vector<double> travel(6, 0.0);
    for (size_t k = 0; k + 1 < pts.size(); ++k) {
      for (size_t i = 0; i < 6; ++i) {
        travel[i] += std::abs(wrap_angle(pts[k+1].positions[i] - pts[k].positions[i]));
      }
    }
    double sum_travel = 0.0;
    for (size_t i = 0; i < 6; ++i) {
      sum_travel += travel[i];
    }
    // shoulder_lift_joint is index 1 in m1_joint_names
    double travel_shoulder_lift = travel[1];
    double D_descent = sum_travel + 2.0 * travel_shoulder_lift;

    // Compute transit distance from M1
    double D_transit_sq = 0.0;
    for (size_t i = 0; i < 6; ++i) {
      double diff = wrap_angle(q_cand[i] - q_m1[i]);
      D_transit_sq += diff * diff;
    }
    double D_transit = std::sqrt(D_transit_sq);

    evaluated_candidates.push_back({q_cand, D_descent, D_transit, fraction, pts.size()});
  }

  if (evaluated_candidates.empty()) {
    RCLCPP_ERROR(logger, "DETERMINISTIC_SELECTOR_FAIL: All IK candidates failed Cartesian dry-run or continuity check.");
    return false;
  }

  // Sort candidates by ranking:
  // Primary: minimum cumulative D_descent (within 0.05 rad treated as tied)
  // Tie-breaker: minimum wrapped M1 -> pregrasp L2 distance (D_transit)
  std::sort(
    evaluated_candidates.begin(), evaluated_candidates.end(),
    [](const PregraspCandidate & a, const PregraspCandidate & b) {
      if (std::abs(a.d_descent - b.d_descent) > 0.05) {
        return a.d_descent < b.d_descent;
      }
      return a.d_transit < b.d_transit;
    });

  const auto & winner = evaluated_candidates.front();
  out_selected_joints = winner.joint_positions;

  RCLCPP_INFO(
    logger,
    "DETERMINISTIC_PREGRASP_SELECTED: q=[%.6f %.6f %.6f %.6f %.6f %.6f] D_descent=%.4f D_transit=%.4f fraction=%.4f waypoints=%zu",
    winner.joint_positions[0], winner.joint_positions[1], winner.joint_positions[2],
    winner.joint_positions[3], winner.joint_positions[4], winner.joint_positions[5],
    winner.d_descent, winner.d_transit, winner.cartesian_fraction, winner.waypoint_count);

  return true;
}

// Same parse as m2_cartesian_approach.cpp — see that file's header for why
// this replaced a ros_gz_bridge TFMessage approach.
std::optional<double> extract_header_stamp(const std::string & dump)
{
  auto h_pos = dump.find("header {");
  if (h_pos == std::string::npos) {
    return std::nullopt;
  }
  auto sec_pos = dump.find("sec:", h_pos);
  auto nsec_pos = dump.find("nsec:", h_pos);
  if (sec_pos == std::string::npos || nsec_pos == std::string::npos) {
    return std::nullopt;
  }
  long sec = std::strtol(dump.c_str() + sec_pos + 4, nullptr, 10);
  long nsec = std::strtol(dump.c_str() + nsec_pos + 5, nullptr, 10);
  return static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
}

std::optional<tf2::Transform> parse_link_pose(const std::string & dump, const std::string & link_name)
{
  const std::string name_needle = "name: \"" + link_name + "\"";

  std::size_t pos = 0;
  while (pos < dump.size()) {
    auto block_start = dump.find("pose {", pos);
    if (block_start == std::string::npos) {
      break;
    }
    auto block_end = dump.find("\n}", block_start);
    if (block_end == std::string::npos) {
      block_end = dump.size();
    } else {
      block_end += 2;
    }
    std::string block = dump.substr(block_start, block_end - block_start);
    pos = block_end;

    if (block.find(name_needle) == std::string::npos) {
      continue;
    }

    auto extract_field = [&](const std::string & b, char field) -> double {
        const std::string needle = std::string(1, field) + ":";
        auto p = b.find(needle);
        if (p == std::string::npos) {
          return 0.0;
        }
        return std::strtod(b.c_str() + p + needle.size(), nullptr);
      };

    auto pos_start = block.find("position {");
    auto pos_end = (pos_start != std::string::npos) ? block.find('}', pos_start) : std::string::npos;
    auto ori_start = block.find("orientation {");
    auto ori_end = (ori_start != std::string::npos) ? block.find('}', ori_start) : std::string::npos;

    std::string pos_block = (pos_start != std::string::npos && pos_end != std::string::npos)
      ? block.substr(pos_start, pos_end - pos_start) : "";
    std::string ori_block = (ori_start != std::string::npos && ori_end != std::string::npos)
      ? block.substr(ori_start, ori_end - ori_start) : "";

    tf2::Vector3 origin(
      extract_field(pos_block, 'x'), extract_field(pos_block, 'y'), extract_field(pos_block, 'z'));
    const bool has_x = ori_block.find("x:") != std::string::npos;
    const bool has_y = ori_block.find("y:") != std::string::npos;
    const bool has_z = ori_block.find("z:") != std::string::npos;
    const bool has_w = ori_block.find("w:") != std::string::npos;

    const double qx = has_x ? extract_field(ori_block, 'x') : 0.0;
    const double qy = has_y ? extract_field(ori_block, 'y') : 0.0;
    const double qz = has_z ? extract_field(ori_block, 'z') : 0.0;
    const double qw = has_w ? extract_field(ori_block, 'w') : ((has_x || has_y || has_z) ? 0.0 : 1.0);

    tf2::Quaternion q(qx, qy, qz, qw);
    return tf2::Transform(q, origin);
  }

  return std::nullopt;
}

// run_command() and parse_joint_position() now live in gz_topic_utils.hpp,
// shared with transport.cpp's own grasp-loss check (Stage 3, lift_transport_
// place) rather than duplicated — used below (TIMED_OUT_HELD fallback) the
// same way they always were.

// Linear interpolation of grasp_table.yaml's (width_m, grip_angle_rad)
// rows for `width`. Returns nullopt if width falls outside the measured
// range — extrapolating past what was actually swept is not supported;
// see config/grasp_table.yaml's header for why.
std::optional<double> interpolate_grip_angle(
  const std::vector<double> & widths, const std::vector<double> & angles, double width)
{
  if (widths.size() != angles.size() || widths.empty()) {
    return std::nullopt;
  }
  if (width < widths.front() || width > widths.back()) {
    return std::nullopt;
  }
  for (std::size_t i = 0; i + 1 < widths.size(); ++i) {
    if (width >= widths[i] && width <= widths[i + 1]) {
      const double span = widths[i + 1] - widths[i];
      if (span < 1e-12) {
        return angles[i];
      }
      const double t = (width - widths[i]) / span;
      return angles[i] + t * (angles[i + 1] - angles[i]);
    }
  }
  // width == widths.back() falls through the loop above (loop condition
  // needs a following point); handle the exact-last-row case explicitly.
  return angles.back();
}

// gripper_close_and_hold, ported from scripts/lib/gz_settle.sh. Sends a
// GripperCommand goal toward `target`, bounded by `cmd_timeout`, then
// unconditionally re-commands a hold goal at wherever the joint actually
// ended up. See that shell function's own header comment (still the fuller
// explanation) for why: check_for_success's stalled branch never calls
// set_hold_position(), so the ORIGINAL target keeps being written every
// control cycle after the action reports done, and a stalled grasp left
// alone can eject the object it just successfully grasped minutes later.
struct GripperCloseResult
{
  enum class Kind { REACHED_GOAL, STALLED, TIMED_OUT_HELD, UNKNOWN_NO_SAMPLE } kind =
    Kind::UNKNOWN_NO_SAMPLE;
  double achieved_position = 0.0;
  bool have_hold_result = false;
};

const char * to_string(GripperCloseResult::Kind k)
{
  switch (k) {
    case GripperCloseResult::Kind::REACHED_GOAL: return "REACHED_GOAL";
    case GripperCloseResult::Kind::STALLED: return "STALLED";
    case GripperCloseResult::Kind::TIMED_OUT_HELD: return "TIMED_OUT_HELD";
    case GripperCloseResult::Kind::UNKNOWN_NO_SAMPLE: return "UNKNOWN_NO_SAMPLE";
  }
  return "UNHANDLED";
}

GripperCloseResult gripper_close_and_hold(
  rclcpp_action::Client<GripperCommand>::SharedPtr client,
  double target, double max_effort, double cmd_timeout_s,
  const std::string & gz_js_topic, const std::string & master_joint,
  rclcpp::Logger logger)
{
  GripperCloseResult out;

  auto send_and_wait = [&](double position) -> std::optional<GripperCommand::Result> {
      GripperCommand::Goal goal;
      goal.command.position = position;
      goal.command.max_effort = max_effort;

      auto send_goal_future = client->async_send_goal(goal);
      if (send_goal_future.wait_for(std::chrono::duration<double>(cmd_timeout_s)) !=
        std::future_status::ready)
      {
        return std::nullopt;
      }
      auto goal_handle = send_goal_future.get();
      if (!goal_handle) {
        RCLCPP_ERROR(logger, "GripperCommand goal rejected by the action server");
        return std::nullopt;
      }
      auto result_future = client->async_get_result(goal_handle);
      if (result_future.wait_for(std::chrono::duration<double>(cmd_timeout_s)) !=
        std::future_status::ready)
      {
        return std::nullopt;
      }
      auto wrapped = result_future.get();
      if (!wrapped.result) {
        return std::nullopt;
      }
      return *wrapped.result;
    };

  auto closing_result = send_and_wait(target);
  if (closing_result) {
    out.achieved_position = closing_result->position;
    out.kind = closing_result->stalled
      ? GripperCloseResult::Kind::STALLED
      : GripperCloseResult::Kind::REACHED_GOAL;
  } else {
    // No Result within the bound. Unlike the bash version's `timeout`
    // (which SIGTERMs the CLI client but leaves the goal running
    // server-side, confirmed live 2026-08-06), this node's own goal handle
    // is still tracked — but the safest, already-validated behavior is to
    // treat this identically: sample ground truth directly rather than
    // assume anything about the outstanding goal's eventual result.
    const std::string dump = run_command("gz topic -e -t " + gz_js_topic + " -n 1 2>/dev/null");
    auto sampled = parse_joint_position(dump, master_joint);
    if (!sampled) {
      out.kind = GripperCloseResult::Kind::UNKNOWN_NO_SAMPLE;
      return out;
    }
    out.achieved_position = *sampled;
    out.kind = GripperCloseResult::Kind::TIMED_OUT_HELD;
  }

  // Unconditional hold, re-commanding `target` (not `out.achieved_position`)
  // — changed 2026-08-11. Re-commanding wherever the joint landed was the
  // original fix (validated 2026-08-06, see the header comment above) for a
  // real bug: the ROS action's own internal loop kept driving toward the
  // ORIGINAL target forever after reporting done, with nothing capping it,
  // and a stalled grasp left alone could eject the object it just grasped
  // minutes later. That fix predates this file's lift/transport pipeline;
  // nobody had reason to check whether "hold at achieved position" still
  // held anything once real load (gravity, lift acceleration) was applied
  // against a bounded, application-level hold rather than an uncapped
  // low-level one.
  //
  // It does not. gz_ros2_control's POSITION command branch converts a
  // position command into a velocity command every control cycle
  // (gz_system.cpp, GazeboSimSystem::write, same mechanism
  // stall_monitor.py's header documents): commanding the position the
  // joint is ALREADY at drives that P-loop's error, and with it the
  // commanded velocity/force, to ~0 — DART then applies no sustained
  // constraint force to maintain closure. The grip goes slack the instant
  // this hold is issued, which is exactly when the subsequent lift needs
  // it not to. Re-commanding `target` (the original close goal, e.g. the
  // joint's max bound) keeps the error — and the commanded velocity/effort
  // — railed into the contact for as long as the object blocks it, the
  // same way a real gripper keeps commanding closed rather than "stay
  // exactly here." Trade-off: since `target` remains physically
  // unreachable once contact is made, this call will generally also run
  // out its own `cmd_timeout_s` bound before returning (same ~5s the
  // initial close already spent), roughly doubling this function's own
  // worst-case duration versus commanding the already-reached
  // `achieved_position`. Accepted deliberately: a slower, real hold beats
  // a fast, slack one.
  auto hold_result = send_and_wait(target);
  out.have_hold_result = hold_result.has_value();
  if (!out.have_hold_result) {
    RCLCPP_WARN(
      logger, "hold command at %.4f rad did not return within %.1fs — the gripper may "
      "still be settling; proceeding with the achieved position already sampled",
      out.achieved_position, cmd_timeout_s);
  }
  return out;
}

}  // namespace

#ifndef UR5E_PICK_PLACE_M3_GRASP_UNIT_TEST
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  Result result = Result::SUCCESS;
  {
    auto node = std::make_shared<rclcpp::Node>(
      "m3_grasp",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    auto logger = node->get_logger();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

  // ---------------------------------------------------------------------
  // Parameters. All originate in config/scene.yaml / config/grasp_table.yaml
  // via the launch file.
  // ---------------------------------------------------------------------
  std::string world_frame = "world";
  std::string grasp_frame_name = "grasp_frame";
  std::string place_frame_name = "place_frame";
  std::string tool0_frame = "tool0";
  double standoff = 0.0;
  double retreat = 0.0;
  double slip_sample_dwell_s = 2.0;
  double release_position_rad = 0.0;
  double tcp_offset = 0.0;
  double pad_centre_offset = 0.0;
  double tf_lookup_timeout_s = 2.0;
  double cartesian_fraction_min = 0.95;
  double grasp_pose_error_max_m = 0.005;
  double planning_time_s = 5.0;
  int plan_attempts = 10;
  double vel_scale = 0.1;
  double acc_scale = 0.1;
  double eef_step = 0.01;
  std::string csv_path = "m3_grasp.csv";
  std::string grasp_mode = "friction";
  std::string gt_wrist3_link_name = "wrist_3_link";
  std::string gz_world = "empty";
  std::string gt_base_link_name = "base_link";
  std::vector<double> expected_base_xyz;
  std::vector<double> expected_base_rpy;
  double base_pose_tol_m = 0.005;
  double base_pose_tol_rad = 0.01;

  std::string gripper_ctrl = "gripper_controller";
  std::string actuated_joint = "robotiq_85_left_knuckle_joint";
  double gripper_command_timeout_s = 5.0;
  double gripper_max_effort = 50.0;
  double object_width_m = 0.0;
  std::vector<double> grasp_table_widths_m;
  std::vector<double> grasp_table_grip_angles_rad;
  double grasp_tolerance_rad = 0.0235;
  double preclose_margin_rad = 0.05;
  double grasp_loss_threshold_rad = 0.01;

  // --- gripper_model dispatch (2026-08-25) ---------------------------------
  // "robotiq_linkage" (default) is the vendor path above, byte-identical to
  // before this parameter existed. "parallel_jaw" is the opt-in
  // docs/GRIPPER_REDESIGN_DESIGN.md path: metres/newtons, no grasp_table.yaml,
  // no theta anywhere. Every pj_* value below is METRES or NEWTONS, computed
  // launch-side from scripts/lib/parallel_jaw_geometry.py (the single source
  // for these formulas — not reimplemented here) and passed in already
  // resolved for the current object_width_m; this file does no parallel-jaw
  // geometry arithmetic of its own, only dispatch.
  std::string gripper_model = "robotiq_linkage";
  // Pre-close aperture's q (wider than the object; free-air partial close,
  // same ROLE as preclose_target below, native units metres).
  double pj_q_preclose = 0.0;
  // q_for_width(object_width_m) -- the aperture that exactly matches the
  // object. INFORMATIONAL verification target only (mirrors expected_grip_angle
  // below), NOT what gets commanded to the action -- see pj_q_close_commanded.
  double pj_q_final_expected = 0.0;
  // The joint's own upper bound (0.085 m, parallel_jaw_gripper.urdf.xacro's
  // Q_MAX_M) -- what actually gets COMMANDED for the final close, so contact
  // stops it rather than a possibly-off q_final_expected. Same "command past
  // what the object permits" pattern as the vendor path's
  // getVariableBounds(actuated_joint).max_position_ lookup, computed here
  // via parallel_jaw_geometry.py instead of a MoveIt robot-model query --
  // see this file's own dispatch block below for why that query is not used
  // for parallel_jaw.
  double pj_q_close_commanded = 0.0;
  // grasp_centre_offset_m(q_preclose), metres, toward the fixed jaw --
  // applied along tool0's local X (== grasp_frame's local X, identity
  // rotation between them) in the SAME transform that carries the Z depth
  // below. See this file's corrected_offset dispatch block.
  double pj_preclose_offset_x_m = 0.0;
  // tool0 -> gripper_base_link, Z only (parallel_jaw_geometry.TCP_OFFSET_Z_M).
  // Replaces vendor's (tcp_offset + pad_centre_offset) for this model --
  // see that constant's own header comment for why no separate
  // pad_centre_offset term is needed here.
  double pj_tcp_offset_z_m = 0.0;
  // Explicitly-derived LINEAR tolerance for the informational grasp-success
  // check, metres. NOT grasp_tolerance_rad reinterpreted -- a separate
  // parameter, so a radian value can never silently reach this comparison.
  double pj_grasp_tolerance_m = 0.001;
  // Empty (default) disables it. See TransportParams::marker_file_prefix
  // for why this exists -- 2026-08-12, a robust alternative to
  // scripts/11_m3_cycles.sh's watcher parsing live stdout for stage markers.
  std::string marker_file_prefix;
  // close_and_hold_only, 2026-08-21: measurement-mode switch for the M6
  // width investigation's "what is the gripper's actual physical pose at
  // the stall" question, which needs the close/stall to happen genuinely
  // but must not proceed into lift/transport/place/release/retreat.
  // Default false preserves this file's only previous behavior exactly.
  bool close_and_hold_only = false;
  bool use_perceived_position = false;
  std::string perceived_position_topic = "object_detector/position_world";
  bool use_perceived_yaw = false;
  std::string perceived_pose_topic = "object_detector/pose_world";
  double perceived_position_timeout_s = 2.0;
  double object_height_m = 0.0;
  std::string object_frame_name = "object_frame";

  // --- Milestone F1 (evidence-grade perception mode) ------------------------
  // require_perception turns the production-convenience fallback OFF. With it
  // set, a perception timeout is a typed failure that stops the run before any
  // target is composed, rather than a WARN that quietly continues from
  // scene.yaml. That distinction is the whole point: the classical pipeline
  // succeeds perfectly from the configured position, so a silent fallback
  // produces a clean, successful, evidence-backed run that is indistinguishable
  // from a genuine perception success.
  bool require_perception = false;
  // pregrasp_only stops the lifecycle immediately after the pre-grasp pose has
  // been executed and verified. Deliberately NOT close_and_hold_only, which
  // stops after contact -- far too late for a milestone whose acceptance
  // criteria include "the gripper does not close" and "the object does not
  // move".
  bool pregrasp_only = false;
  // descent_only stops after Stage-2 Cartesian descent and pose verification.
  // No gripper closure, no lift, no transport.
  bool descent_only = false;
  // Milestone F2: run the existing approach/contact/close-and-hold path, then
  // stop before lift_transport_place(). This is intentionally distinct from
  // pregrasp_only and from the older measurement-oriented close_and_hold_only.
  bool grasp_only = false;
  // Milestone F3: use the normal Stage-3 implementation, including its
  // post-lift loss check and full dwell, then stop before TRANSPORT_BEGIN.
  bool lift_only = false;
  bool transport_only = false;
  // Milestone F3 (controlled P12.5 lift), 2026-08-23: pre-lift barrier. Empty
  // (default) disables it entirely and this file behaves exactly as before.
  //
  // Why it exists: in lift_only the close and LIFT_BEGIN are adjacent -- the
  // original F3 Scene-A trial logged the close result and LIFT_BEGIN 21
  // MICROseconds apart -- so an external evaluator has no window in which to
  // establish a loaded reference, switch the master effort interface to an
  // evaluation controller, and hold. That is also why that trial's G0 window
  // fell inside the close's own settling transient and was ruled unreliable.
  // See docs/F3_P12_5_LIFT_PLAN.md §7.1.
  //
  // This is a synchronisation point ONLY. It changes no controller, gain,
  // physics, perception, grasp, geometry, transport, place or release
  // behaviour, and the lift it gates is the unmodified existing lift_only
  // path. It deliberately reuses the marker-file idiom already used by
  // marker_file_prefix rather than introducing a new IPC mechanism.
  std::string pre_lift_barrier_file;
  double pre_lift_barrier_timeout_s = 300.0;
  // M1 observation pose. Milestones C/D/E were all validated with the arm here;
  // perceiving from wherever the arm happens to be is outside that envelope.
  // The values come from scene.yaml (milestones.m1.goal_positions) via the
  // launch file -- they are NOT redefined here.
  std::vector<std::string> m1_joint_names;
  std::vector<double> m1_goal_positions;
  // Experiment-only override. Empty preserves the normal pose-goal IK path.
  // When populated, Stage 1 plans to this explicit vector (in m1_joint_names
  // order), letting a controlled run hold the pre-grasp IK branch fixed.
  std::vector<double> pregrasp_joint_target;
  // Empty disables capture. When set, the exact post-scaling Stage-2
  // RobotTrajectory passed to MoveIt execution is written for this run.
  std::string experiment_cartesian_fjt_path;
  double stationary_velocity_eps = 1.0e-3;
  int stationary_consecutive_samples = 6;
  double stationary_timeout_s = 25.0;
  double startup_m1_tolerance_rad = 0.01;
  std::string joint_states_topic = "/joint_states";
  // Pre-grasp is a free-air pose 0.1 m above the object, so it does not carry
  // the grasp target's tight tolerance; this is the F1 acceptance bound.
  double pregrasp_pose_error_max_m = 0.010;
  // Which object position the run actually used. Written to the CSV verbatim
  // so the evidence states the source instead of leaving it to be inferred
  // from commanded coordinates after the fact.
  std::string position_source = "configured";
  std::string yaw_source = "configured";
  double configured_object_yaw_deg = std::numeric_limits<double>::quiet_NaN();
  double perceived_object_yaw_deg = std::numeric_limits<double>::quiet_NaN();
  double yaw_delta_deg = std::numeric_limits<double>::quiet_NaN();
  double commanded_grasp_yaw_deg = std::numeric_limits<double>::quiet_NaN();

  node->get_parameter_or("world_frame", world_frame, world_frame);
  node->get_parameter_or("object_frame_name", object_frame_name, object_frame_name);
  node->get_parameter_or("grasp_frame_name", grasp_frame_name, grasp_frame_name);
  node->get_parameter_or("place_frame_name", place_frame_name, place_frame_name);
  node->get_parameter_or("tool0_frame", tool0_frame, tool0_frame);
  node->get_parameter_or("standoff", standoff, standoff);
  node->get_parameter_or("retreat", retreat, retreat);
  node->get_parameter_or("slip_sample_dwell_s", slip_sample_dwell_s, slip_sample_dwell_s);
  node->get_parameter_or("marker_file_prefix", marker_file_prefix, marker_file_prefix);
  node->get_parameter_or("release_position_rad", release_position_rad, release_position_rad);
  node->get_parameter_or("tcp_offset", tcp_offset, tcp_offset);
  node->get_parameter_or("pad_centre_offset", pad_centre_offset, pad_centre_offset);
  node->get_parameter_or("tf_lookup_timeout_s", tf_lookup_timeout_s, tf_lookup_timeout_s);
  node->get_parameter_or("cartesian_fraction_min", cartesian_fraction_min, cartesian_fraction_min);
  node->get_parameter_or("grasp_pose_error_max_m", grasp_pose_error_max_m, grasp_pose_error_max_m);
  node->get_parameter_or("planning_time_s", planning_time_s, planning_time_s);
  node->get_parameter_or("plan_attempts", plan_attempts, plan_attempts);
  node->get_parameter_or("velocity_scaling", vel_scale, vel_scale);
  node->get_parameter_or("acceleration_scaling", acc_scale, acc_scale);
  node->get_parameter_or("eef_step", eef_step, eef_step);
  node->get_parameter_or("csv_path", csv_path, csv_path);
  node->get_parameter_or("grasp_mode", grasp_mode, grasp_mode);
  node->get_parameter_or("gt_wrist3_link_name", gt_wrist3_link_name, gt_wrist3_link_name);
  node->get_parameter_or("gz_world", gz_world, gz_world);
  node->get_parameter_or("gt_base_link_name", gt_base_link_name, gt_base_link_name);
  node->get_parameter_or("expected_base_xyz", expected_base_xyz, expected_base_xyz);
  node->get_parameter_or("expected_base_rpy", expected_base_rpy, expected_base_rpy);
  node->get_parameter_or("base_pose_tol_m", base_pose_tol_m, base_pose_tol_m);
  node->get_parameter_or("base_pose_tol_rad", base_pose_tol_rad, base_pose_tol_rad);

  node->get_parameter_or("gripper_ctrl", gripper_ctrl, gripper_ctrl);
  node->get_parameter_or("actuated_joint", actuated_joint, actuated_joint);
  node->get_parameter_or("gripper_command_timeout_s", gripper_command_timeout_s, gripper_command_timeout_s);
  node->get_parameter_or("gripper_max_effort", gripper_max_effort, gripper_max_effort);
  node->get_parameter_or("object_width_m", object_width_m, object_width_m);
  node->get_parameter_or("gripper_model", gripper_model, gripper_model);
  node->get_parameter_or("parallel_jaw_q_preclose", pj_q_preclose, pj_q_preclose);
  node->get_parameter_or("parallel_jaw_q_final_expected", pj_q_final_expected, pj_q_final_expected);
  node->get_parameter_or("parallel_jaw_q_close_commanded", pj_q_close_commanded, pj_q_close_commanded);
  node->get_parameter_or(
    "parallel_jaw_preclose_offset_x_m", pj_preclose_offset_x_m, pj_preclose_offset_x_m);
  node->get_parameter_or("parallel_jaw_tcp_offset_z_m", pj_tcp_offset_z_m, pj_tcp_offset_z_m);
  node->get_parameter_or("parallel_jaw_grasp_tolerance_m", pj_grasp_tolerance_m, pj_grasp_tolerance_m);
  node->get_parameter_or("grasp_table_widths_m", grasp_table_widths_m, grasp_table_widths_m);
  node->get_parameter_or(
    "grasp_table_grip_angles_rad", grasp_table_grip_angles_rad, grasp_table_grip_angles_rad);
  node->get_parameter_or("grasp_tolerance_rad", grasp_tolerance_rad, grasp_tolerance_rad);
  node->get_parameter_or("preclose_margin_rad", preclose_margin_rad, preclose_margin_rad);
  node->get_parameter_or(
    "grasp_loss_threshold_rad", grasp_loss_threshold_rad, grasp_loss_threshold_rad);
  node->get_parameter_or("close_and_hold_only", close_and_hold_only, close_and_hold_only);
  node->get_parameter_or(
    "use_perceived_position", use_perceived_position, use_perceived_position);
  node->get_parameter_or(
    "perceived_position_topic", perceived_position_topic, perceived_position_topic);
  node->get_parameter_or("use_perceived_yaw", use_perceived_yaw, use_perceived_yaw);
  node->get_parameter_or("perceived_pose_topic", perceived_pose_topic, perceived_pose_topic);
  node->get_parameter_or(
    "perceived_position_timeout_s", perceived_position_timeout_s, perceived_position_timeout_s);
  node->get_parameter_or("object_height_m", object_height_m, object_height_m);
  node->get_parameter_or("require_perception", require_perception, require_perception);
  node->get_parameter_or("pregrasp_only", pregrasp_only, pregrasp_only);
  node->get_parameter_or("descent_only", descent_only, descent_only);
  node->get_parameter_or("grasp_only", grasp_only, grasp_only);
  node->get_parameter_or("lift_only", lift_only, lift_only);
  node->get_parameter_or("transport_only", transport_only, transport_only);
  node->get_parameter_or(
    "pre_lift_barrier_file", pre_lift_barrier_file, pre_lift_barrier_file);
  node->get_parameter_or(
    "pre_lift_barrier_timeout_s", pre_lift_barrier_timeout_s, pre_lift_barrier_timeout_s);
  node->get_parameter_or("m1_joint_names", m1_joint_names, m1_joint_names);
  node->get_parameter_or("m1_goal_positions", m1_goal_positions, m1_goal_positions);
  node->get_parameter_or(
    "pregrasp_joint_target", pregrasp_joint_target, pregrasp_joint_target);
  node->get_parameter_or(
    "experiment_cartesian_fjt_path", experiment_cartesian_fjt_path,
    experiment_cartesian_fjt_path);
  node->get_parameter_or(
    "stationary_velocity_eps", stationary_velocity_eps, stationary_velocity_eps);
  node->get_parameter_or(
    "stationary_consecutive_samples", stationary_consecutive_samples,
    stationary_consecutive_samples);
  node->get_parameter_or("stationary_timeout_s", stationary_timeout_s, stationary_timeout_s);
  node->get_parameter_or("startup_m1_tolerance_rad", startup_m1_tolerance_rad, startup_m1_tolerance_rad);
  node->get_parameter_or("joint_states_topic", joint_states_topic, joint_states_topic);
  node->get_parameter_or(
    "pregrasp_pose_error_max_m", pregrasp_pose_error_max_m, pregrasp_pose_error_max_m);

  RCLCPP_INFO(
    logger, "GRASP MODE: %s",
    grasp_mode == "friction" ? "friction (physics)"
                             : "contact-triggered attach (fallback)");

  if (standoff <= 0.0) {
    RCLCPP_ERROR(
      logger,
      "CONFIG_ERROR: grasp.standoff must be > 0, got %.4f. Approach with zero "
      "standoff is not an approach.",
      standoff);
    result = Result::CONFIG_ERROR;
  }
  if (ur5e_pick_place::ok(result) && retreat <= 0.0) {
    RCLCPP_ERROR(
      logger,
      "CONFIG_ERROR: grasp.retreat must be > 0, got %.4f. A zero-distance "
      "lift leaves the object at grasp height with nowhere to clear.",
      retreat);
    result = Result::CONFIG_ERROR;
  }
  // gripper_model dispatch flag, computed once and used everywhere below.
  // Validated FIRST so every subsequent branch can trust its value.
  const bool is_parallel_jaw = (gripper_model == "parallel_jaw");
  if (ur5e_pick_place::ok(result) && gripper_model != "robotiq_linkage" && !is_parallel_jaw) {
    RCLCPP_ERROR(
      logger,
      "CONFIG_ERROR: gripper_model='%s' is not recognised. Must be "
      "'robotiq_linkage' (default) or 'parallel_jaw'.", gripper_model.c_str());
    result = Result::CONFIG_ERROR;
  }
  // grasp_table.yaml is vendor-only (theta_for_width semantics) -- parallel_jaw
  // never reads it, so an empty table is expected and not an error for that
  // model. Vendor behavior (empty table = CONFIG_ERROR) is unchanged.
  if (ur5e_pick_place::ok(result) && !is_parallel_jaw && grasp_table_widths_m.empty()) {
    RCLCPP_ERROR(
      logger,
      "CONFIG_ERROR: grasp_table_widths_m is empty. Did the launch file load "
      "config/grasp_table.yaml?");
    result = Result::CONFIG_ERROR;
  }
  // parallel_jaw's own geometry parameters, sanity-checked against the
  // closed-form identity aperture(q)=0.085-q rather than trusted blind: a
  // wider aperture must correspond to a SMALLER q, so q_preclose must be
  // strictly less than q_final_expected (e.g. 30mm object: q_preclose=0.051 <
  // q_final_expected=0.055). Catches a launch-file wiring mistake before any
  // motion, rather than producing a pre-close that is already past the
  // final target.
  if (ur5e_pick_place::ok(result) && is_parallel_jaw) {
    if (pj_q_preclose <= 0.0 || pj_q_final_expected <= 0.0 || pj_q_close_commanded <= 0.0) {
      RCLCPP_ERROR(
        logger,
        "CONFIG_ERROR: parallel_jaw_q_preclose (%.6f), parallel_jaw_q_final_expected "
        "(%.6f) and parallel_jaw_q_close_commanded (%.6f) must all be > 0. Did the "
        "launch file compute them from scripts/lib/parallel_jaw_geometry.py?",
        pj_q_preclose, pj_q_final_expected, pj_q_close_commanded);
      result = Result::CONFIG_ERROR;
    } else if (!(pj_q_preclose < pj_q_final_expected &&
        pj_q_final_expected <= pj_q_close_commanded))
    {
      RCLCPP_ERROR(
        logger,
        "CONFIG_ERROR: expected pj_q_preclose (%.6f) < pj_q_final_expected (%.6f) "
        "<= pj_q_close_commanded (%.6f) -- aperture(q)=0.085-q means a WIDER "
        "pre-close aperture must be a SMALLER q than the final target.",
        pj_q_preclose, pj_q_final_expected, pj_q_close_commanded);
      result = Result::CONFIG_ERROR;
    }
  }
  if (ur5e_pick_place::ok(result) && use_perceived_position && object_height_m <= 0.0) {
    RCLCPP_ERROR(
      logger,
      "CONFIG_ERROR: object_height_m must be > 0 when use_perceived_position is true, got %.6f",
      object_height_m);
    result = Result::CONFIG_ERROR;
  }
  if (ur5e_pick_place::ok(result) &&
    !perceived_yaw_configuration_valid(use_perceived_yaw, use_perceived_position))
  {
    RCLCPP_ERROR(
      logger,
      "CONFIG_ERROR: use_perceived_yaw is true but use_perceived_position is false. "
      "Perceived yaw is accepted only at the validated M1 perception boundary.");
    result = Result::CONFIG_ERROR;
  }
  if (ur5e_pick_place::ok(result) && require_perception && !use_perceived_position) {
    RCLCPP_ERROR(
      logger,
      "CONFIG_ERROR: require_perception is true but use_perceived_position is false. "
      "Strict perception mode with the perception source disabled is not a meaningful "
      "configuration; it would demand a sample nothing is allowed to use.");
    result = Result::CONFIG_ERROR;
  }
  const int boundary_mode_count = static_cast<int>(pregrasp_only) +
    static_cast<int>(grasp_only) + static_cast<int>(close_and_hold_only) +
    static_cast<int>(lift_only) + static_cast<int>(descent_only) + static_cast<int>(transport_only);
  if (ur5e_pick_place::ok(result) && boundary_mode_count > 1) {
    RCLCPP_ERROR(
      logger,
      "CONFIG_ERROR: pregrasp_only, grasp_only, close_and_hold_only, lift_only, descent_only, and transport_only "
      "are mutually exclusive boundary modes; got %d enabled and will not choose "
      "one silently.", boundary_mode_count);
    result = Result::CONFIG_ERROR;
  }
  if (ur5e_pick_place::ok(result) && use_perceived_position &&
    (m1_joint_names.empty() || m1_joint_names.size() != m1_goal_positions.size()))
  {
    RCLCPP_ERROR(
      logger,
      "CONFIG_ERROR: m1_joint_names (%zu) and m1_goal_positions (%zu) must be non-empty and "
      "the same length when use_perceived_position is true. Perception must be taken from the "
      "validated M1 observation pose, not from wherever the arm happens to be.",
      m1_joint_names.size(), m1_goal_positions.size());
    result = Result::CONFIG_ERROR;
  }
  if (ur5e_pick_place::ok(result) && !pregrasp_joint_target.empty() &&
    pregrasp_joint_target.size() != m1_joint_names.size())
  {
    RCLCPP_ERROR(
      logger,
      "CONFIG_ERROR: pregrasp_joint_target has %zu values, but arm joint order has %zu. "
      "Refusing an ambiguous experiment target.",
      pregrasp_joint_target.size(), m1_joint_names.size());
    result = Result::CONFIG_ERROR;
  }
  if (ur5e_pick_place::ok(result) && use_perceived_position && perceived_position_timeout_s <= 0.0) {
    RCLCPP_ERROR(
      logger,
      "CONFIG_ERROR: perceived_position_timeout_s must be > 0 when perception is enabled, got %.3f",
      perceived_position_timeout_s);
    result = Result::CONFIG_ERROR;
  }

  // Grasp-table lookup moved here (was previously computed only after the
  // final close call) so PRE-CLOSE, below, has a data-driven target angle
  // to aim for before the Cartesian descent even starts — not just the
  // grasp-success check afterward. Same interpolation, same table, used
  // twice: once as a target, once as a verification threshold.
  //
  // VENDOR-ONLY (is_parallel_jaw stays false here): grasp_table.yaml is a
  // width->RADIANS table (theta_for_width semantics baked into measured
  // rows). parallel_jaw's equivalent target (pj_q_final_expected) is
  // launch-computed metres, already loaded above -- not looked up here.
  double expected_grip_angle = 0.0;
  bool have_expected_grip_angle = false;
  if (ur5e_pick_place::ok(result) && !is_parallel_jaw) {
    auto expected = interpolate_grip_angle(
      grasp_table_widths_m, grasp_table_grip_angles_rad, object_width_m);
    if (!expected) {
      RCLCPP_ERROR(
        logger,
        "CONFIG_ERROR: object_width_m=%.4f falls outside grasp_table.yaml's "
        "measured range [%.4f, %.4f] — refusing to extrapolate a "
        "grasp-success tolerance past what was actually swept.",
        object_width_m, grasp_table_widths_m.front(), grasp_table_widths_m.back());
      result = Result::CONFIG_ERROR;
    } else {
      expected_grip_angle = *expected;
      have_expected_grip_angle = true;
    }
  }

  // --- grasp_tcp_offset_vec: "tool0 -> reference point" (the point that gets
  // aligned to the target/grasp_frame), used at every downstream site
  // (target composition and both ground-truth reconstructions) so they
  // cannot disagree with each other. Same ROLE as the vendor's own
  // Vector3(0,0,corrected_offset) (see the two ground-truth reconstruction
  // sites below, unchanged in form): tool0 -> grasp_tcp = (0,0,+corrected_offset).
  //
  // VENDOR (robotiq_linkage): unchanged. corrected_offset = tcp_offset +
  // pad_centre_offset, Z only, exactly as before this dispatch existed.
  //
  // PARALLEL_JAW: Z is pj_tcp_offset_z_m (parallel_jaw_geometry.TCP_OFFSET_Z_M,
  // a plain constant -- no pad_centre_offset term needed, see that constant's
  // own header). X is the pad-midpoint's (at q_preclose) SIGNED local-X
  // coordinate relative to gripper_base_link, which shares tool0's X,Y
  // exactly (only Z differs, at gripper_rotation=0):
  //   grasp_centre_offset_m(q) returns a POSITIVE MAGNITUDE meaning "toward
  //   the fixed jaw", and the fixed jaw sits at local -X (parallel_jaw_
  //   geometry.py's own docstring), so the pad-midpoint's actual SIGNED
  //   X-coordinate is the NEGATION of that magnitude:
  //     midpoint_x = -grasp_centre_offset_m(q_preclose) = -pj_preclose_offset_x_m
  //   "tool0 -> pad_midpoint" X = midpoint_x - tool0_x = midpoint_x (since
  //   tool0 IS the origin of this comparison) = -pj_preclose_offset_x_m.
  //   VERIFIED with concrete world coordinates (not just role-matching) in
  //   this integration's own report, 2026-08-25, Section 3 -- an earlier
  //   draft of this file had the opposite sign here and was caught by that
  //   re-derivation before any Gazebo run, not after one.
  const double corrected_offset = tcp_offset + pad_centre_offset;
  const double pj_offset_x = -pj_preclose_offset_x_m;
  const tf2::Vector3 grasp_tcp_offset_vec = is_parallel_jaw
    ? tf2::Vector3(pj_offset_x, 0.0, pj_tcp_offset_z_m)
    : tf2::Vector3(0.0, 0.0, corrected_offset);
  if (is_parallel_jaw) {
    RCLCPP_INFO(
      logger,
      "gripper_model=parallel_jaw: grasp_tcp_offset_vec = [x=%.6f y=0.000000 z=%.6f] m "
      "(pre-close-centred X toward fixed jaw + constant Z depth, see file header)",
      pj_offset_x, pj_tcp_offset_z_m);
  } else {
    RCLCPP_INFO(
      logger,
      "gripper_model=robotiq_linkage: pad-centre-corrected offset: tcp_offset=%.6f + "
      "pad_centre_offset=%.6f = %.6f m (NOT YET LIVE-VERIFIED direction -- see file header)",
      tcp_offset, pad_centre_offset, corrected_offset);
  }

  // Ground-truth query, identical to m2_cartesian_approach.cpp.
  const std::string gz_pose_topic = "/world/" + gz_world + "/pose/info";
  const std::string gz_js_topic = "/world/" + gz_world + "/model/ur5e_robotiq/joint_state";
  auto ground_truth_tool0 = [gz_pose_topic, gt_wrist3_link_name]() -> std::optional<tf2::Transform> {
      const std::string dump = run_command(
        "gz topic -e -t " + gz_pose_topic + " -n 1 2>/dev/null");
      auto w = parse_link_pose(dump, gt_wrist3_link_name);
      if (!w) {
        return std::nullopt;
      }
      tf2::Matrix3x3 R_world_tool0 = w->getBasis() * kR_wrist3_to_flange * kR_flange_to_tool0;
      tf2::Quaternion q;
      R_world_tool0.getRotation(q);
      return tf2::Transform(q, w->getOrigin());
    };

  // Base-pose guard, identical to m2_cartesian_approach.cpp — see that
  // file's comment for why this exists (a silently stale base pose between
  // MoveIt's model and Gazebo's actual sim was a real, previously-hit bug).
  if (ur5e_pick_place::ok(result) && expected_base_xyz.size() == 3 && expected_base_rpy.size() == 3) {
    const std::string dump = run_command("gz topic -e -t " + gz_pose_topic + " -n 1 2>/dev/null");
    auto base_gt = parse_link_pose(dump, gt_base_link_name);
    if (!base_gt) {
      RCLCPP_ERROR(
        logger,
        "CONFIG_ERROR: could not read '%s' pose via `gz topic -e -t %s` to "
        "verify it matches scene.yaml's robot.base_pose. Is the sim running?",
        gt_base_link_name.c_str(), gz_pose_topic.c_str());
      result = Result::CONFIG_ERROR;
    } else {
      tf2::Vector3 expected_origin(expected_base_xyz[0], expected_base_xyz[1], expected_base_xyz[2]);
      tf2::Quaternion expected_q;
      expected_q.setRPY(expected_base_rpy[0], expected_base_rpy[1], expected_base_rpy[2]);

      const double pos_err = base_gt->getOrigin().distance(expected_origin);
      tf2::Quaternion actual_q;
      base_gt->getBasis().getRotation(actual_q);
      const double dot = std::abs(actual_q.dot(expected_q));
      const double angle_err = 2.0 * std::acos(std::min(1.0, dot));

      if (pos_err > base_pose_tol_m || angle_err > base_pose_tol_rad) {
        RCLCPP_ERROR(
          logger,
          "CONFIG_ERROR: Gazebo's actual '%s' pose does not match "
          "scene.yaml's robot.base_pose (pos error %.4f m, angle error "
          "%.4f rad; tolerances %.4f m / %.4f rad).",
          gt_base_link_name.c_str(), pos_err, angle_err, base_pose_tol_m, base_pose_tol_rad);
        result = Result::CONFIG_ERROR;
      } else {
        RCLCPP_INFO(
          logger, "base-pose guard OK: '%s' matches scene.yaml within tolerance "
          "(pos error %.5f m, angle error %.5f rad)",
          gt_base_link_name.c_str(), pos_err, angle_err);
      }
    }
  }

  bool got_grasp_tf = false;
  tf2::Transform T_world_grasp;
  double configured_object_yaw = std::numeric_limits<double>::quiet_NaN();

  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  if (ur5e_pick_place::ok(result)) {
    tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_buffer->setUsingDedicatedThread(true);
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Polling canTransform() instead of lookupTransform's own timeout
    // overload — see m2_cartesian_approach.cpp's comment for the measured
    // reason this is more reliable.
    auto t0 = std::chrono::steady_clock::now();
    const auto deadline = t0 + std::chrono::duration<double>(tf_lookup_timeout_s);
    std::string tf_error;
    while (std::chrono::steady_clock::now() < deadline) {
      if (tf_buffer->canTransform(world_frame, grasp_frame_name, tf2::TimePointZero, &tf_error)) {
        auto stamped = tf_buffer->lookupTransform(world_frame, grasp_frame_name, tf2::TimePointZero);
        tf2::fromMsg(stamped.transform, T_world_grasp);
        got_grasp_tf = true;
        break;
      }
      rclcpp::sleep_for(20ms);
    }
    if (!got_grasp_tf) {
      RCLCPP_ERROR(
        logger,
        "TF_LOOKUP_TIMEOUT: could not resolve %s -> %s within %.1fs: %s",
        world_frame.c_str(), grasp_frame_name.c_str(), tf_lookup_timeout_s,
        tf_error.c_str());
      result = Result::TF_LOOKUP_TIMEOUT;
    }

    // Record configured object yaw whenever the static reference is already
    // available, independently of whether this run consumes perceived yaw.
    // This lookup is deliberately best-effort for default-off behavior: a
    // missing/invalid telemetry reference must not create a new failure path.
    bool got_object_tf = false;
    tf2::Transform T_world_object;
    if (tf_buffer->canTransform(world_frame, object_frame_name, tf2::TimePointZero, &tf_error)) {
      auto stamped = tf_buffer->lookupTransform(world_frame, object_frame_name, tf2::TimePointZero);
      tf2::fromMsg(stamped.transform, T_world_object);
      got_object_tf = true;
      const auto telemetry_yaw = configured_object_planar_yaw(T_world_object);
      if (telemetry_yaw) {
        configured_object_yaw_deg = *telemetry_yaw * 180.0 / M_PI;
      }
    }

    if (ur5e_pick_place::ok(result) && use_perceived_yaw) {
      const auto object_deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(tf_lookup_timeout_s);
      while (!got_object_tf && std::chrono::steady_clock::now() < object_deadline) {
        if (tf_buffer->canTransform(world_frame, object_frame_name, tf2::TimePointZero, &tf_error)) {
          auto stamped = tf_buffer->lookupTransform(world_frame, object_frame_name, tf2::TimePointZero);
          tf2::fromMsg(stamped.transform, T_world_object);
          got_object_tf = true;
          break;
        }
        rclcpp::sleep_for(20ms);
      }
      if (!got_object_tf) {
        RCLCPP_ERROR(
          logger,
          "TF_LOOKUP_TIMEOUT: could not resolve %s -> %s within %.1fs: %s",
          world_frame.c_str(), object_frame_name.c_str(), tf_lookup_timeout_s,
          tf_error.c_str());
        result = Result::TF_LOOKUP_TIMEOUT;
      } else {
        std::string geometry_error;
        if (!configured_yaw_reference_supported(
            T_world_object, T_world_grasp, configured_object_yaw, geometry_error))
        {
          RCLCPP_ERROR(
            logger, "CONFIG_ERROR: perceived-yaw targeting requires %s.", geometry_error.c_str());
          result = Result::CONFIG_ERROR;
        } else {
          configured_object_yaw_deg = configured_object_yaw * 180.0 / M_PI;
        }
      }
    }
    if (got_grasp_tf) {
      const auto & basis = T_world_grasp.getBasis();
      commanded_grasp_yaw_deg = std::atan2(basis[1][0], basis[0][0]) * 180.0 / M_PI;
    }
  }

  // Milestone F1: the perception substitution that used to sit here has MOVED
  // down into the MoveGroupInterface scope below.  It has to run after the arm
  // has been driven to the M1 observation pose and confirmed stationary, and
  // that motion needs the move_group this scope does not yet have.  The
  // substitution logic itself -- top-surface to object-centre conversion,
  // translation-only replacement, configured rotation retained -- is unchanged
  // from the reviewed implementation.

  double achieved_fraction = 0.0;
  bool executed = false;
  double commanded_tcp[3] = {0, 0, 0};
  double achieved_tcp[3] = {0, 0, 0};
  double tcp_error_m = -1.0;
  bool have_ground_truth = false;

  GripperCloseResult grip;
  bool have_grip_result = false;
  // Commanded final-close target, native units (rad vendor / m parallel_jaw),
  // hoisted from the close block below so the CSV section can log it without
  // re-deriving it. -1.0 means "never attempted".
  double final_close_target_commanded = -1.0;
  GripperCloseResult preclose_result;
  bool have_preclose_result = false;
  bool within_tolerance = false;
  bool attempted_transport = false;
  bool pregrasp_attempted = false;
  bool pregrasp_succeeded = false;
  bool descent_attempted = false;
  bool gripper_close_attempted = false;
  bool f2_stop_reached = false;
  bool lift_attempted = false;
  bool transport_attempted = false;
  bool place_release_attempted = false;
  bool lift_only_stop_reached = false;
  Result transport_result = Result::SUCCESS;

  if (ur5e_pick_place::ok(result) && got_grasp_tf) {
    moveit::planning_interface::MoveGroupInterface move_group(node, kPlanningGroup);
    move_group.setPlanningTime(planning_time_s);
    move_group.setNumPlanningAttempts(plan_attempts);
    move_group.setMaxVelocityScalingFactor(vel_scale);
    move_group.setMaxAccelerationScalingFactor(acc_scale);

    // =====================================================================
    // PLANNING SCENE: Initialize PlanningSceneManager & Table CollisionObject
    // =====================================================================
    auto psm = std::make_shared<ur5e_pick_place::PlanningSceneManager>(node, world_frame);
    if (ur5e_pick_place::ok(result)) {
      std::string scene_err;
      if (!psm->initializeTable(scene_err)) {
        RCLCPP_ERROR(logger, "SCENE_INIT_FAILURE: table initialization failed: %s", scene_err.c_str());
        result = Result::SCENE_INIT_FAILURE;
      } else if (!psm->verifyExpectedScene(scene_err)) {
        RCLCPP_ERROR(logger, "SCENE_INIT_FAILURE: startup table verification failed: %s", scene_err.c_str());
        result = Result::SCENE_INIT_FAILURE;
      } else {
        RCLCPP_INFO(
          logger, "SCENE_INIT_SUCCESS: table initialized and verified with fingerprint %s",
          psm->fingerprint().c_str());
      }
    }

    // =====================================================================
    // MILESTONE F1 / Stage-2C: M1 observation pose -> stationarity -> fresh
    // position and (when enabled) independently subscribed yaw samples.
    // =====================================================================
    // Order matters and is not negotiable.  Milestones C, D and E were every
    // one of them validated with the arm at M1; a sample taken from any other
    // pose is outside the envelope those PASSes cover.  The stationarity
    // boundary that this establishes is then used to reject any observation
    // that could have been exposed while the arm was still moving.
    rclcpp::Time m1_stationary_stamp(0, 0, RCL_ROS_TIME);
    if (ur5e_pick_place::ok(result) && use_perceived_position) {
      // Startup policy A: the simulation is spawned at M1.  Never command an
      // unknown-target home->M1 path as a fallback.
      const auto current = move_group.getCurrentState(2.0);
      double max_error = std::numeric_limits<double>::infinity();
      if (current && m1_joint_names.size() == m1_goal_positions.size()) {
        max_error = 0.0;
        for (std::size_t i = 0; i < m1_joint_names.size(); ++i) {
          const auto & variable_names = current->getRobotModel()->getVariableNames();
          if (std::find(variable_names.begin(), variable_names.end(), m1_joint_names[i]) ==
              variable_names.end()) {
            max_error = std::numeric_limits<double>::infinity(); break;
          }
          max_error = std::max(
            max_error, std::abs(current->getVariablePosition(m1_joint_names[i]) - m1_goal_positions[i]));
        }
      }
      if (!(max_error <= startup_m1_tolerance_rad)) {
        RCLCPP_ERROR(logger, "STARTUP_NOT_AT_M1: max joint error %.6f rad exceeds %.6f rad; refusing any arm trajectory before perception.", max_error, startup_m1_tolerance_rad);
        result = Result::STARTUP_NOT_AT_M1;
      } else {
        RCLCPP_INFO(logger, "STARTUP_M1_VERIFIED: max joint error %.6f rad (tolerance %.6f rad).", max_error, startup_m1_tolerance_rad);
      }
    }

    if (ur5e_pick_place::ok(result) && use_perceived_position) {
      // Stationarity from /joint_states velocities.  A single below-threshold
      // sample is not evidence of rest -- the same lesson gz_settle.py records
      // -- so N consecutive samples are required.
      struct JointWatch
      {
        std::mutex mutex;
        sensor_msgs::msg::JointState::ConstSharedPtr last;
      };
      auto watch = std::make_shared<JointWatch>();
      auto js_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic, 10,
        [watch](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(watch->mutex);
          watch->last = std::move(msg);
        });

      int consecutive = 0;
      double last_vmax = -1.0;
      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(stationary_timeout_s);
      while (std::chrono::steady_clock::now() < deadline &&
        consecutive < stationary_consecutive_samples)
      {
        rclcpp::sleep_for(50ms);
        sensor_msgs::msg::JointState::ConstSharedPtr snap;
        {
          std::lock_guard<std::mutex> lock(watch->mutex);
          snap = watch->last;
        }
        if (!snap || snap->velocity.empty()) {
          continue;
        }
        double vmax = 0.0;
        bool complete = true;
        for (const auto & joint_name : m1_joint_names) {
          const auto it = std::find(snap->name.begin(), snap->name.end(), joint_name);
          if (it == snap->name.end()) {
            complete = false;
            break;
          }
          const std::size_t idx =
            static_cast<std::size_t>(std::distance(snap->name.begin(), it));
          if (idx >= snap->velocity.size()) {
            complete = false;
            break;
          }
          vmax = std::max(vmax, std::abs(snap->velocity[idx]));
        }
        if (!complete) {
          continue;
        }
        last_vmax = vmax;
        consecutive = (vmax < stationary_velocity_eps) ? consecutive + 1 : 0;
      }

      if (consecutive < stationary_consecutive_samples) {
        RCLCPP_ERROR(
          logger,
          "EXECUTE_FAILURE: the six arm joints never held below %.2e rad/s for %d "
          "consecutive samples within %.1fs at M1 (last max |velocity| = %.3e). "
          "Refusing to perceive from a moving arm.",
          stationary_velocity_eps, stationary_consecutive_samples, stationary_timeout_s,
          last_vmax);
        result = Result::EXECUTE_FAILURE;
      } else {
        m1_stationary_stamp = node->now();
        RCLCPP_INFO(
          logger,
          "F1: M1_STATIONARY max|velocity|=%.3e rad/s over %d consecutive samples; "
          "freshness boundary = %.9f (sim time). Observations at or before this "
          "stamp are rejected.",
          last_vmax, stationary_consecutive_samples, m1_stationary_stamp.seconds());
      }
    }


    // ---------------------------------------------------------------------
    // MILESTONE F: perceived object position substitution.
    //
    // The conversion and substitution below are the independently reviewed
    // implementation, unchanged: the published point is the visible
    // TOP-SURFACE centre, grasp_frame is anchored at the object CENTRE
    // (static_scene_tf.cpp publishes object_frame -> grasp_frame with zero
    // translation), hence the explicit half-height subtraction.  Only the
    // translation is replaced. With the default yaw mode disabled, the
    // rotation stays exactly as orientation_from_approach_axis derived it
    // from approach_axis and gripper_roll. Stage-2C's separate yaw block
    // below may pre-multiply only an axial world-Z delta. object_height_m
    // comes from the configured object size -- nothing here is hardcoded,
    // and no empirical correction is applied.
    //
    // MILESTONE F1 additions: the accepted sample must be stamped strictly
    // after the M1 stationarity boundary, and require_perception removes the
    // fallback.
    // ---------------------------------------------------------------------
    if (ur5e_pick_place::ok(result) && use_perceived_position) {
      geometry_msgs::msg::PointStamped::ConstSharedPtr perceived;
      std::mutex perceived_mutex;
      std::size_t rejected_stale = 0;
      std::size_t rejected_invalid = 0;
      auto subscription = node->create_subscription<geometry_msgs::msg::PointStamped>(
        perceived_position_topic, 10,
        [&perceived, &perceived_mutex, &rejected_stale, &rejected_invalid, &world_frame,
        &m1_stationary_stamp](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
          if (msg->header.frame_id != world_frame || !std::isfinite(msg->point.x) ||
            !std::isfinite(msg->point.y) || !std::isfinite(msg->point.z))
          {
            std::lock_guard<std::mutex> lock(perceived_mutex);
            ++rejected_invalid;
            return;
          }
          // Freshness gate.  An observation exposed while the arm was still
          // settling is not evidence about the stationary scene, and the
          // detector publishes continuously at 5 Hz, so stale samples are
          // guaranteed to be sitting in the queue.
          if (rclcpp::Time(msg->header.stamp) <= m1_stationary_stamp) {
            std::lock_guard<std::mutex> lock(perceived_mutex);
            ++rejected_stale;
            return;
          }
          std::lock_guard<std::mutex> lock(perceived_mutex);
          if (!perceived) {
            perceived = std::move(msg);
          }
        });

      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(perceived_position_timeout_s);
      while (std::chrono::steady_clock::now() < deadline) {
        {
          std::lock_guard<std::mutex> lock(perceived_mutex);
          if (perceived) {
            break;
          }
        }
        rclcpp::sleep_for(20ms);
      }

      geometry_msgs::msg::PointStamped::ConstSharedPtr sample;
      std::size_t stale_count = 0;
      std::size_t invalid_count = 0;
      {
        std::lock_guard<std::mutex> lock(perceived_mutex);
        sample = perceived;
        stale_count = rejected_stale;
        invalid_count = rejected_invalid;
      }

      if (sample) {
        const tf2::Vector3 configured = T_world_grasp.getOrigin();
        const tf2::Vector3 perceived_centre(
          sample->point.x, sample->point.y, sample->point.z - object_height_m / 2.0);
        T_world_grasp.setOrigin(perceived_centre);
        position_source = "perceived";
        RCLCPP_INFO(
          logger,
          "PERCEPTION_POSITION_USED: stamp=%.9f (boundary %.9f) top_surface=[%.6f %.6f %.6f] "
          "object_centre=[%.6f %.6f %.6f] configured_centre=[%.6f %.6f %.6f] "
          "delta=[%.6f %.6f %.6f]; rejected_stale=%zu rejected_invalid=%zu; "
          "configured grasp rotation retained",
          rclcpp::Time(sample->header.stamp).seconds(), m1_stationary_stamp.seconds(),
          sample->point.x, sample->point.y, sample->point.z,
          perceived_centre.x(), perceived_centre.y(), perceived_centre.z(),
          configured.x(), configured.y(), configured.z(),
          perceived_centre.x() - configured.x(), perceived_centre.y() - configured.y(),
          perceived_centre.z() - configured.z(), stale_count, invalid_count);
        // Frozen from here on.  The subscription goes out of scope at the end
        // of this block, so nothing can move the target while MoveIt plans.
      } else if (require_perception) {
        position_source = "perception_timeout";
        RCLCPP_ERROR(
          logger,
          "PERCEPTION_TIMEOUT: no fresh valid %s PointStamped in frame '%s' stamped after "
          "the M1 stationarity boundary %.9f within %.2fs (rejected_stale=%zu "
          "rejected_invalid=%zu). require_perception is set: NOT falling back to the "
          "configured position, NOT planning, NOT executing.",
          perceived_position_topic.c_str(), world_frame.c_str(),
          m1_stationary_stamp.seconds(), perceived_position_timeout_s,
          stale_count, invalid_count);
        result = Result::PERCEPTION_TIMEOUT;
      } else {
        position_source = "fallback_configured";
        RCLCPP_WARN(
          logger,
          "PERCEPTION_FALLBACK: no fresh valid %s PointStamped in frame '%s' within %.2fs "
          "(rejected_stale=%zu rejected_invalid=%zu); using configured grasp position "
          "[%.6f %.6f %.6f]. This run is NOT perception-derived.",
          perceived_position_topic.c_str(), world_frame.c_str(), perceived_position_timeout_s,
          stale_count, invalid_count,
          T_world_grasp.getOrigin().x(), T_world_grasp.getOrigin().y(),
          T_world_grasp.getOrigin().z());
      }
    }

    // Stage-2C yaw is deliberately a separate subscription from the position
    // path above. A valid position never authorises a configured-yaw fallback:
    // if no fresh valid pose arrives, this run stops before target composition.
    if (ur5e_pick_place::ok(result) && use_perceived_yaw) {
      geometry_msgs::msg::PoseStamped::ConstSharedPtr perceived_pose;
      std::mutex perceived_pose_mutex;
      std::size_t rejected_stale = 0;
      std::size_t rejected_invalid = 0;
      auto pose_subscription = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        perceived_pose_topic, 10,
        [&perceived_pose, &perceived_pose_mutex, &rejected_stale, &rejected_invalid, &world_frame,
        &m1_stationary_stamp](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          if (msg->header.frame_id != world_frame || !planar_yaw_from_valid_quaternion(*msg)) {
            std::lock_guard<std::mutex> lock(perceived_pose_mutex);
            ++rejected_invalid;
            return;
          }
          if (!is_fresh_world_pose(*msg, world_frame, m1_stationary_stamp)) {
            std::lock_guard<std::mutex> lock(perceived_pose_mutex);
            ++rejected_stale;
            return;
          }
          std::lock_guard<std::mutex> lock(perceived_pose_mutex);
          if (!perceived_pose) {
            perceived_pose = std::move(msg);
          }
        });

      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(perceived_position_timeout_s);
      while (std::chrono::steady_clock::now() < deadline) {
        {
          std::lock_guard<std::mutex> lock(perceived_pose_mutex);
          if (perceived_pose) {
            break;
          }
        }
        rclcpp::sleep_for(20ms);
      }

      geometry_msgs::msg::PoseStamped::ConstSharedPtr sample;
      std::size_t stale_count = 0;
      std::size_t invalid_count = 0;
      {
        std::lock_guard<std::mutex> lock(perceived_pose_mutex);
        sample = perceived_pose;
        stale_count = rejected_stale;
        invalid_count = rejected_invalid;
      }
      if (sample) {
        const double perceived_yaw = *planar_yaw_from_valid_quaternion(*sample);
        const double delta = ur5e_pick_place::axial_difference(
          perceived_yaw, configured_object_yaw);
        T_world_grasp.setBasis(grasp_basis_with_perceived_yaw(
          T_world_grasp.getBasis(), true, perceived_yaw, configured_object_yaw));
        perceived_object_yaw_deg = perceived_yaw * 180.0 / M_PI;
        yaw_delta_deg = delta * 180.0 / M_PI;
        const auto & basis = T_world_grasp.getBasis();
        commanded_grasp_yaw_deg = std::atan2(basis[1][0], basis[0][0]) * 180.0 / M_PI;
        yaw_source = "perceived";
        RCLCPP_INFO(
          logger,
          "PERCEPTION_YAW_USED: stamp=%.9f (boundary %.9f) configured_object_yaw_deg=%.6f "
          "perceived_object_yaw_deg=%.6f axial_delta_deg=%.6f "
          "commanded_grasp_yaw_deg=%.6f; rejected_stale=%zu rejected_invalid=%zu",
          rclcpp::Time(sample->header.stamp).seconds(), m1_stationary_stamp.seconds(),
          configured_object_yaw_deg, perceived_object_yaw_deg, yaw_delta_deg,
          commanded_grasp_yaw_deg, stale_count, invalid_count);
      } else {
        yaw_source = "perception_timeout";
        RCLCPP_ERROR(
          logger,
          "PERCEPTION_TIMEOUT: no fresh valid %s PoseStamped in frame '%s' stamped after "
          "the M1 stationarity boundary %.9f within %.2fs (rejected_stale=%zu "
          "rejected_invalid=%zu). NOT falling back to configured yaw, NOT planning, "
          "NOT executing.",
          perceived_pose_topic.c_str(), world_frame.c_str(), m1_stationary_stamp.seconds(),
          perceived_position_timeout_s, stale_count, invalid_count);
        result = perceived_yaw_sample_result(false);
      }
    }

    // Same construction as m2_cartesian_approach.cpp, generalised to X,Y,Z via
    // grasp_tcp_offset_vec (defined above): tool0 -> grasp_tcp is
    // Translation(grasp_tcp_offset_vec); its inverse (grasp target -> tool0
    // target) is Translation(-grasp_tcp_offset_vec). For robotiq_linkage this
    // vector is (0,0,corrected_offset), so this line is unchanged in effect
    // from before the dispatch existed.
    tf2::Transform T_tcp_tool0(tf2::Quaternion(0, 0, 0, 1), -grasp_tcp_offset_vec);

    tf2::Transform T_grasp_pregrasp(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, -standoff));
    tf2::Transform T_world_pregrasp = T_world_grasp * T_grasp_pregrasp;
    tf2::Transform T_world_pregrasp_tool0 = T_world_pregrasp * T_tcp_tool0;

    tf2::Transform T_world_grasp_tool0 = T_world_grasp * T_tcp_tool0;

    geometry_msgs::msg::Pose pregrasp_pose;
    tf2::toMsg(T_world_pregrasp_tool0, pregrasp_pose);
    geometry_msgs::msg::Pose grasp_pose;
    tf2::toMsg(T_world_grasp_tool0, grasp_pose);

    // NO MOTION ON A FAILED RESULT.  Stage 1 historically needed no ok(result)
    // guard because nothing could fail between the TF lookup and here -- the
    // whole scope was already gated.  PERCEPTION_TIMEOUT is the first failure
    // that can be raised INSIDE this scope, and without this guard a strict-mode
    // perception failure still planned and executed a move to the CONFIGURED
    // pre-grasp: measured, 2026-08-23, execute accepted 0.03 s after the
    // timeout was logged.  That is precisely the "NO PERCEPTION = NO MOTION"
    // rule being violated, so the guard is here rather than in the caller.
    const bool may_move = ur5e_pick_place::ok(result);
    if (!may_move) {
      RCLCPP_ERROR(
        logger,
        "NO_MOTION: result is already %s before the pre-grasp stage. Not composing a "
        "target, not planning, not executing. The arm stays where it is.",
        to_string(result));
    } else {
      RCLCPP_INFO(
        logger,
        "pre-grasp tool0 target (world): [%.4f %.4f %.4f]  grasp tool0 target: "
        "[%.4f %.4f %.4f]  (gripper_tcp_offset_vec=[%.6f %.6f %.6f])",
        pregrasp_pose.position.x, pregrasp_pose.position.y, pregrasp_pose.position.z,
        grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z,
        grasp_tcp_offset_vec.x(), grasp_tcp_offset_vec.y(), grasp_tcp_offset_vec.z());
      if (position_source == "perceived") {
        const tf2::Vector3 perceived_top(
          T_world_grasp.getOrigin().x(), T_world_grasp.getOrigin().y(),
          T_world_grasp.getOrigin().z() + object_height_m / 2.0);
        RCLCPP_INFO(
          logger,
          "F2 TARGETS FROZEN: PERCEIVED_TOP_WORLD=[%.6f %.6f %.6f] "
          "OBJECT_REFERENCE_WORLD=[%.6f %.6f %.6f] "
          "PERCEIVED_PREGRASP=[%.6f %.6f %.6f] "
          "PERCEIVED_GRASP_TARGET=[%.6f %.6f %.6f]. Truth has not been queried for "
          "target evaluation and cannot alter these targets.",
          perceived_top.x(), perceived_top.y(), perceived_top.z(),
          T_world_grasp.getOrigin().x(), T_world_grasp.getOrigin().y(),
          T_world_grasp.getOrigin().z(), T_world_pregrasp.getOrigin().x(),
          T_world_pregrasp.getOrigin().y(), T_world_pregrasp.getOrigin().z(),
          T_world_grasp.getOrigin().x(), T_world_grasp.getOrigin().y(),
          T_world_grasp.getOrigin().z());
      }

      // Insert perceived target into planning scene as WORLD CollisionObject
      if (ur5e_pick_place::ok(result)) {
        geometry_msgs::msg::Pose perceived_target_pose;
        perceived_target_pose.position.x = T_world_grasp.getOrigin().x();
        perceived_target_pose.position.y = T_world_grasp.getOrigin().y();
        perceived_target_pose.position.z = T_world_grasp.getOrigin().z();
        tf2::Quaternion q_target;
        T_world_grasp.getBasis().getRotation(q_target);
        perceived_target_pose.orientation = tf2::toMsg(q_target);

        std::string scene_err;
        if (!psm->addWorldTarget(perceived_target_pose, scene_err)) {
          RCLCPP_ERROR(
            logger, "TARGET_INSERTION_FAILURE: failed to insert world target: %s",
            scene_err.c_str());
          result = Result::TARGET_INSERTION_FAILURE;
        } else if (!psm->verifyExpectedScene(scene_err)) {
          RCLCPP_ERROR(
            logger, "SCENE_STALE_OR_CORRUPT: target scene verification failed: %s",
            scene_err.c_str());
          result = Result::SCENE_STALE_OR_CORRUPT;
        } else {
          RCLCPP_INFO(
            logger,
            "SCENE_TARGET_INSERTED: pick_target added to world at [%.6f %.6f %.6f] "
            "with fingerprint %s",
            perceived_target_pose.position.x, perceived_target_pose.position.y,
            perceived_target_pose.position.z, psm->fingerprint().c_str());
        }
      }
    }

    // Stage 1: joint-space plan+execute to pre-grasp. Same reasoning as M2.
    moveit::planning_interface::MoveGroupInterface::Plan pregrasp_plan;
    bool pregrasp_planned = false;
    if (may_move && ur5e_pick_place::ok(result)) {
      pregrasp_attempted = true;
      move_group.setStartStateToCurrentState();
      if (pregrasp_joint_target.empty()) {
        std::vector<double> selected_joints;
        if (!select_deterministic_pregrasp_branch(
              move_group, pregrasp_pose, grasp_pose, m1_joint_names,
              tool0_frame, eef_step, cartesian_fraction_min, logger,
              selected_joints))
        {
          RCLCPP_ERROR(
            logger,
            "PLAN_FAILURE: deterministic pre-grasp branch selector found no valid solution.");
          pregrasp_planned = false;
        } else {
          std::map<std::string, double> explicit_target;
          for (std::size_t i = 0; i < m1_joint_names.size(); ++i) {
            explicit_target[m1_joint_names[i]] = selected_joints[i];
          }
          move_group.setJointValueTarget(explicit_target);
          pregrasp_planned =
            (move_group.plan(pregrasp_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }
      } else {
        std::map<std::string, double> explicit_target;
        for (std::size_t i = 0; i < m1_joint_names.size(); ++i) {
          explicit_target[m1_joint_names[i]] = pregrasp_joint_target[i];
        }
        move_group.setJointValueTarget(explicit_target);
        RCLCPP_INFO(
          logger,
          "EXPERIMENT_EXPLICIT_PREGRASP_TARGET (joint order follows m1_joint_names): "
          "[%.9f %.9f %.9f %.9f %.9f %.9f]",
          pregrasp_joint_target[0], pregrasp_joint_target[1], pregrasp_joint_target[2],
          pregrasp_joint_target[3], pregrasp_joint_target[4], pregrasp_joint_target[5]);
        pregrasp_planned =
          (move_group.plan(pregrasp_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      }
    }
    if (!may_move) {
      // already reported above; fall through to the CSV with result intact
    } else if (!pregrasp_planned) {
      RCLCPP_ERROR(logger, "PLAN_FAILURE: could not plan to the pre-grasp pose.");
      result = Result::PLAN_FAILURE;
    } else {
      const auto pregrasp_exec = move_group.execute(pregrasp_plan);
      if (pregrasp_exec != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(
          logger, "EXECUTE_FAILURE: pre-grasp plan found but execution returned code %d.",
          pregrasp_exec.val);
        result = Result::EXECUTE_FAILURE;
      } else {
        pregrasp_succeeded = true;
        if (!pregrasp_joint_target.empty()) {
          const auto state = move_group.getCurrentState(1.0);
          if (!state) {
            RCLCPP_ERROR(
              logger,
              "EXPERIMENT_PREGRASP_STATE_UNAVAILABLE: controller reported success, but "
              "MoveIt supplied no current state for the controlled branch.");
            result = Result::EXECUTE_FAILURE;
          } else {
            std::vector<double> actual;
            const auto * joint_group = state->getJointModelGroup(kPlanningGroup);
            if (!joint_group) {
              RCLCPP_ERROR(
                logger, "EXPERIMENT_PREGRASP_STATE_UNAVAILABLE: arm joint model group is absent.");
              result = Result::EXECUTE_FAILURE;
            } else {
              state->copyJointGroupPositions(joint_group, actual);
            const auto & tf = state->getGlobalLinkTransform(tool0_frame);
            const auto & p = tf.translation();
            const double max_q_error = [&]() {
                double value = 0.0;
                for (std::size_t i = 0; i < actual.size(); ++i) {
                  value = std::max(value, std::abs(actual[i] - pregrasp_joint_target[i]));
                }
                return value;
              }();
            const double fk_error = std::sqrt(
              std::pow(p.x() - pregrasp_pose.position.x, 2) +
              std::pow(p.y() - pregrasp_pose.position.y, 2) +
              std::pow(p.z() - pregrasp_pose.position.z, 2));
            RCLCPP_INFO(
              logger,
              "EXPERIMENT_PREGRASP_ACTUAL_JOINTS=[%.9f %.9f %.9f %.9f %.9f %.9f] "
              "max_joint_error=%.9f tool0_fk=[%.9f %.9f %.9f] "
              "tool0_target=[%.9f %.9f %.9f] fk_position_error_m=%.9g",
              actual[0], actual[1], actual[2], actual[3], actual[4], actual[5], max_q_error,
              p.x(), p.y(), p.z(), pregrasp_pose.position.x, pregrasp_pose.position.y,
              pregrasp_pose.position.z, fk_error);
            }
          }
        }
      }
    }

    // =====================================================================
    // MILESTONE F1 BOUNDARY: stop here, at the pre-grasp pose.
    // =====================================================================
    // Verification uses the SAME ground-truth mechanism as the Stage 2 check
    // below -- Gazebo's own pose, not TF, because execution reporting SUCCESS
    // means the controller finished its trajectory, not that the arm is where
    // it was told.  Note the comparison target: the commanded PRE-GRASP pose,
    // which in a perceived run came from the sensor.  Measuring achieved pose
    // against ground truth does not let truth influence motion -- the motion
    // was already commanded and executed before this runs.
    if (pregrasp_only) {
      if (ur5e_pick_place::ok(result)) {
        commanded_tcp[0] = T_world_pregrasp.getOrigin().x();
        commanded_tcp[1] = T_world_pregrasp.getOrigin().y();
        commanded_tcp[2] = T_world_pregrasp.getOrigin().z();
        rclcpp::sleep_for(1000ms);
        if (auto tool0_gt = ground_truth_tool0()) {
          const tf2::Vector3 tcp_gt = tool0_gt->getOrigin() +
            tool0_gt->getBasis() * grasp_tcp_offset_vec;
          achieved_tcp[0] = tcp_gt.x();
          achieved_tcp[1] = tcp_gt.y();
          achieved_tcp[2] = tcp_gt.z();
          tcp_error_m = std::sqrt(
            std::pow(achieved_tcp[0] - commanded_tcp[0], 2) +
            std::pow(achieved_tcp[1] - commanded_tcp[1], 2) +
            std::pow(achieved_tcp[2] - commanded_tcp[2], 2));
          have_ground_truth = true;
          RCLCPP_INFO(
            logger,
            "F1 pre-grasp verification: commanded grasp_tcp = [%.6f %.6f %.6f]  "
            "achieved (Gazebo, not TF) = [%.6f %.6f %.6f]  tcp_error_m=%.6f (max %.6f)",
            commanded_tcp[0], commanded_tcp[1], commanded_tcp[2],
            achieved_tcp[0], achieved_tcp[1], achieved_tcp[2],
            tcp_error_m, pregrasp_pose_error_max_m);
          if (tcp_error_m > pregrasp_pose_error_max_m) {
            RCLCPP_ERROR(
              logger,
              "POSE_VERIFY_FAILURE: pre-grasp execution reported SUCCESS but ground truth "
              "puts the TCP %.6f m from the commanded pre-grasp pose (max %.6f).",
              tcp_error_m, pregrasp_pose_error_max_m);
            result = Result::POSE_VERIFY_FAILURE;
          }
        } else {
          RCLCPP_ERROR(
            logger,
            "POSE_VERIFY_FAILURE: no ground-truth pose for '%s' via `gz topic -e -t %s`; "
            "cannot verify the pre-grasp landed where it was commanded.",
            gt_wrist3_link_name.c_str(), gz_pose_topic.c_str());
          result = Result::POSE_VERIFY_FAILURE;
        }
      }
      RCLCPP_INFO(
        logger,
        "F1 STOP: pregrasp_only is set. No descent, no pre-close, no gripper command, "
        "no lift, no transport, no place, no release will be attempted.");
    }

    // Stage 1.5: pre-close at pre-grasp height, in free air.
    // Pre-descent gripper guard: vendor needs a resolved expected_grip_angle;
    // parallel_jaw's pre-close target was already validated above.
    const bool have_preclose_target = have_expected_grip_angle || is_parallel_jaw;
    auto gripper_client = rclcpp_action::create_client<GripperCommand>(
      node, "/" + gripper_ctrl + "/gripper_cmd");
    if (ur5e_pick_place::ok(result) && have_preclose_target && !pregrasp_only) {
      if (!gripper_client->wait_for_action_server(
            std::chrono::duration<double>(gripper_command_timeout_s)))
      {
        RCLCPP_ERROR(
          logger, "GRIPPER_GOAL_REJECTED: action server /%s/gripper_cmd not available "
          "within %.1fs (pre-close)", gripper_ctrl.c_str(), gripper_command_timeout_s);
        if (ur5e_pick_place::ok(result)) { result = Result::GRIPPER_GOAL_REJECTED; }
      } else {
        // VENDOR: expected_grip_angle - preclose_margin_rad (radians).
        // PARALLEL_JAW: launch-resolved pre-close joint target (metres).
        const double preclose_target = is_parallel_jaw
          ? pj_q_preclose
          : std::max(0.0, expected_grip_angle - preclose_margin_rad);
        preclose_result = gripper_close_and_hold(
          gripper_client, preclose_target, gripper_max_effort, gripper_command_timeout_s,
          gz_js_topic, actuated_joint, logger);
        have_preclose_result = true;
        RCLCPP_INFO(
          logger, "pre-close: %s in achieved=%.4f %s (target was %.4f, free air)",
          to_string(preclose_result.kind), preclose_result.achieved_position,
          is_parallel_jaw ? "m" : "rad", preclose_target);
        if (preclose_result.kind != GripperCloseResult::Kind::REACHED_GOAL) {
          // Anything other than a clean REACHED_GOAL means something
          // stopped the gripper before it reached its free-air target —
          // by construction there should be nothing there to stop it.
          // Don't proceed into the descent on an unexplained surprise.
          RCLCPP_ERROR(
            logger,
            "GRIPPER_GOAL_REJECTED: pre-close reported %s instead of REACHED_GOAL in "
            "what should be free air (no object in reach at pre-grasp height) — "
            "refusing to descend on an unexplained result.",
            to_string(preclose_result.kind));
          if (ur5e_pick_place::ok(result)) { result = Result::GRIPPER_GOAL_REJECTED; }
        }
      }
    }

    // Stage 2: short vertical Cartesian descent to the corrected grasp
    // target. Same 0.95-fraction discipline as M2.
    if (ur5e_pick_place::ok(result) && !pregrasp_only) {
      std::string scene_err;
      if (!psm->verifyExpectedScene(scene_err)) {
        RCLCPP_ERROR(
          logger, "SCENE_STALE_OR_CORRUPT: scene invalid before descent: %s",
          scene_err.c_str());
        result = Result::SCENE_STALE_OR_CORRUPT;
      }
    }

    if (ur5e_pick_place::ok(result) && !pregrasp_only) {
      descent_attempted = true;
      commanded_tcp[0] = T_world_grasp.getOrigin().x();
      commanded_tcp[1] = T_world_grasp.getOrigin().y();
      commanded_tcp[2] = T_world_grasp.getOrigin().z();

      moveit_msgs::msg::RobotTrajectory trajectory;
      achieved_fraction = move_group.computeCartesianPath(
        {grasp_pose}, eef_step, trajectory, true, nullptr);

      RCLCPP_INFO(
        logger, "computeCartesianPath fraction achieved: %.4f (need > %.4f)",
        achieved_fraction, cartesian_fraction_min);

      if (achieved_fraction < cartesian_fraction_min) {
        RCLCPP_ERROR(
          logger,
          "CARTESIAN_FRACTION_LOW: achieved %.4f, threshold is %.4f. Not "
          "executing a partial Cartesian path.",
          achieved_fraction, cartesian_fraction_min);
        result = Result::CARTESIAN_FRACTION_LOW;
      } else {
        // Deterministic 2.0x Cartesian trajectory time scaling to prevent
        // multi-joint tracking lag during coordinated descent.
        for (auto & pt : trajectory.joint_trajectory.points) {
          int64_t ns = rclcpp::Duration(pt.time_from_start).nanoseconds() * 2;
          pt.time_from_start = rclcpp::Duration::from_nanoseconds(ns);
          for (auto & v : pt.velocities) { v *= 0.5; }
          for (auto & a : pt.accelerations) { a *= 0.25; }
        }

        if (!experiment_cartesian_fjt_path.empty()) {
          std::ofstream capture(experiment_cartesian_fjt_path);
          if (!capture) {
            RCLCPP_ERROR(
              logger, "EXPERIMENT_FJT_CAPTURE_FAILURE: could not open '%s'.",
              experiment_cartesian_fjt_path.c_str());
            result = Result::EXECUTE_FAILURE;
          } else {
            capture << std::setprecision(17);
            capture << "joint_names";
            for (const auto & name : trajectory.joint_trajectory.joint_names) {
              capture << " " << name;
            }
            capture << "\n";
            for (std::size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
              const auto & pt = trajectory.joint_trajectory.points[i];
              capture << "point " << i << " time_ns "
                      << rclcpp::Duration(pt.time_from_start).nanoseconds();
              auto write_values = [&capture](const char * label, const auto & values) {
                  capture << " " << label << " " << values.size();
                  for (const auto & value : values) { capture << " " << value; }
                };
              write_values("positions", pt.positions);
              write_values("velocities", pt.velocities);
              write_values("accelerations", pt.accelerations);
              write_values("effort", pt.effort);
              capture << "\n";
            }
            RCLCPP_INFO(
              logger,
              "EXPERIMENT_FJT_CAPTURE: wrote %zu post-scaling Cartesian points to '%s'.",
              trajectory.joint_trajectory.points.size(), experiment_cartesian_fjt_path.c_str());
          }
        }

        if (ur5e_pick_place::ok(result)) {
          moveit::planning_interface::MoveGroupInterface::Plan plan;
          ur5e_pick_place::set_trajectory(plan, trajectory);
          RCLCPP_INFO(logger, "executing Cartesian descent to grasp target (2.0x time scaled)...");
          const auto exec_code = move_group.execute(plan);
          executed = (exec_code == moveit::core::MoveItErrorCode::SUCCESS);
          if (executed) {
            RCLCPP_INFO(logger, "execution reported SUCCESS");

            // ---------------------------------------------------------------
            // Stage 2 verification. Execution reporting SUCCESS means the
            // controller finished its trajectory, not that the arm is where
            // it was told — M2 established these can disagree, which is why this
            // comes from Gazebo and not TF. Checked HERE, before the close: the
            // close sweeps the pads 7.8mm along the approach axis, so anything
            // measured after it is measuring a different geometry than the one
            // that was commanded, and this is the only moment the arm stands at
            // the commanded grasp pose with nothing having moved since.
            //
            // A hard abort, not a log line: if the descent lands somewhere
            // other than commanded and the node closes anyway, that cycle
            // enters the statistics as a grasp that failed on contact — and it
            // reads exactly like a friction problem instead of the positioning
            // problem it actually was.
            const rclcpp::Time t_descent_done = node->now();
            const double boundary_sec = t_descent_done.seconds();
            RCLCPP_INFO(
              logger, "stage 2 verification: establishing freshness boundary T_DESCENT_DONE = %.9f (sim time)",
              boundary_sec);

            std::vector<tf2::Transform> settled_samples;
            std::vector<double> settled_stamps;
            std::size_t stale_samples_rejected = 0;
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(5000);

            while (std::chrono::steady_clock::now() < deadline && settled_samples.size() < 3) {
              const std::string dump = run_command(
                "gz topic -e -t " + gz_pose_topic + " -n 1 2>/dev/null");
              auto stamp = extract_header_stamp(dump);
              if (!stamp || *stamp <= boundary_sec) {
                ++stale_samples_rejected;
                rclcpp::sleep_for(20ms);
                continue;
              }

              auto w = parse_link_pose(dump, gt_wrist3_link_name);
              if (!w) {
                rclcpp::sleep_for(20ms);
                continue;
              }

              if (!settled_samples.empty()) {
                const double delta = w->getOrigin().distance(settled_samples.back().getOrigin());
                if (delta > 0.0005) {
                  settled_samples.clear();
                  settled_stamps.clear();
                }
              }

              settled_samples.push_back(*w);
              settled_stamps.push_back(*stamp);
              rclcpp::sleep_for(20ms);
            }

            if (settled_samples.size() >= 3) {
              tf2::Vector3 avg_origin(0, 0, 0);
              for (const auto & s : settled_samples) {
                avg_origin += s.getOrigin();
              }
              avg_origin /= static_cast<double>(settled_samples.size());
              const auto & last_w = settled_samples.back();
              tf2::Matrix3x3 R_world_tool0 = last_w.getBasis() * kR_wrist3_to_flange * kR_flange_to_tool0;
              tf2::Quaternion q;
              R_world_tool0.getRotation(q);
              tf2::Transform tool0_gt(q, avg_origin);

              tf2::Vector3 tcp_gt = tool0_gt.getOrigin() +
                tool0_gt.getBasis() * grasp_tcp_offset_vec;
              achieved_tcp[0] = tcp_gt.x();
              achieved_tcp[1] = tcp_gt.y();
              achieved_tcp[2] = tcp_gt.z();
              tcp_error_m = std::sqrt(
                std::pow(achieved_tcp[0] - commanded_tcp[0], 2) +
                std::pow(achieved_tcp[1] - commanded_tcp[1], 2) +
                std::pow(achieved_tcp[2] - commanded_tcp[2], 2));
              have_ground_truth = true;
              RCLCPP_INFO(
                logger,
                "stage 2 ground truth: grasp_tcp (Gazebo settled %zu samples, rejected_stale=%zu, "
                "first_stamp=%.4f last_stamp=%.4f) = [%.4f %.4f %.4f]  tcp_error_m=%.4f (max %.4f)",
                settled_samples.size(), stale_samples_rejected, settled_stamps.front(), settled_stamps.back(),
                achieved_tcp[0], achieved_tcp[1], achieved_tcp[2], tcp_error_m,
                grasp_pose_error_max_m);

              if (tcp_error_m > grasp_pose_error_max_m) {
                RCLCPP_ERROR(
                  logger,
                  "POSE_VERIFY_FAILURE: descent reported SUCCESS but ground truth "
                  "puts the TCP %.4f m from the commanded grasp pose (max %.4f). "
                  "Closing here would produce a cycle that reads as a friction "
                  "failure. Aborting before the close.",
                  tcp_error_m, grasp_pose_error_max_m);
                result = Result::POSE_VERIFY_FAILURE;
              }
            } else {
              RCLCPP_ERROR(
                logger,
                "POSE_VERIFY_FAILURE: failed to obtain 3 fresh settled post-boundary (%.9f) "
                "Gazebo Pose_V samples within timeout (rejected_stale=%zu). Aborting.",
                boundary_sec, stale_samples_rejected);
              result = Result::POSE_VERIFY_FAILURE;
            }
          } else {
            RCLCPP_ERROR(
              logger,
              "EXECUTE_FAILURE: MoveIt returned error code %d after a valid "
              "Cartesian path was found.",
              exec_code.val);
            result = Result::EXECUTE_FAILURE;
          }
        }
      }
    }
#if 0  // superseded duplicate of the Stage-2 verification block above
          RCLCPP_INFO(logger, "execution reported SUCCESS");

          // ---------------------------------------------------------------
          // Stage 2 verification. Execution reporting SUCCESS means the
          // controller finished its trajectory, not that the arm is where it
          // was told — M2 established these can disagree, which is why this
          // comes from Gazebo and not TF. Checked HERE, before the close: the
          // close sweeps the pads 7.8mm along the approach axis, so anything
          // measured after it is measuring a different geometry than the one
          // that was commanded, and this is the only moment the arm stands at
          // the commanded grasp pose with nothing having moved since.
          //
          // A hard abort, not a log line: if the descent lands somewhere
          // other than commanded and the node closes anyway, that cycle
          // enters the statistics as a grasp that failed on contact — and it
          // reads exactly like a friction problem instead of the positioning
          // problem it actually was.
          const rclcpp::Time t_descent_done = node->now();
          const double boundary_sec = t_descent_done.seconds();
          RCLCPP_INFO(
            logger, "stage 2 verification: establishing freshness boundary T_DESCENT_DONE = %.9f (sim time)",
            boundary_sec);

          std::vector<tf2::Transform> settled_samples;
          std::vector<double> settled_stamps;
          std::size_t stale_samples_rejected = 0;
          const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(5000);

          while (std::chrono::steady_clock::now() < deadline && settled_samples.size() < 3) {
            const std::string dump = run_command(
              "gz topic -e -t " + gz_pose_topic + " -n 1 2>/dev/null");
            auto stamp = extract_header_stamp(dump);
            if (!stamp || *stamp <= boundary_sec) {
              ++stale_samples_rejected;
              rclcpp::sleep_for(20ms);
              continue;
            }

            auto w = parse_link_pose(dump, gt_wrist3_link_name);
            if (!w) {
              rclcpp::sleep_for(20ms);
              continue;
            }

            if (!settled_samples.empty()) {
              const double delta = w->getOrigin().distance(settled_samples.back().getOrigin());
              if (delta > 0.0005) {
                settled_samples.clear();
                settled_stamps.clear();
              }
            }

            settled_samples.push_back(*w);
            settled_stamps.push_back(*stamp);
            rclcpp::sleep_for(20ms);
          }

          if (settled_samples.size() >= 3) {
            tf2::Vector3 avg_origin(0, 0, 0);
            for (const auto & s : settled_samples) {
              avg_origin += s.getOrigin();
            }
            avg_origin /= static_cast<double>(settled_samples.size());
            const auto & last_w = settled_samples.back();
            tf2::Matrix3x3 R_world_tool0 = last_w.getBasis() * kR_wrist3_to_flange * kR_flange_to_tool0;
            tf2::Quaternion q;
            R_world_tool0.getRotation(q);
            tf2::Transform tool0_gt(q, avg_origin);

            tf2::Vector3 tcp_gt = tool0_gt.getOrigin() +
              tool0_gt.getBasis() * grasp_tcp_offset_vec;
            achieved_tcp[0] = tcp_gt.x();
            achieved_tcp[1] = tcp_gt.y();
            achieved_tcp[2] = tcp_gt.z();
            tcp_error_m = std::sqrt(
              std::pow(achieved_tcp[0] - commanded_tcp[0], 2) +
              std::pow(achieved_tcp[1] - commanded_tcp[1], 2) +
              std::pow(achieved_tcp[2] - commanded_tcp[2], 2));
            have_ground_truth = true;
            RCLCPP_INFO(
              logger,
              "stage 2 ground truth: grasp_tcp (Gazebo settled %zu samples, rejected_stale=%zu, "
              "first_stamp=%.4f last_stamp=%.4f) = [%.4f %.4f %.4f]  tcp_error_m=%.4f (max %.4f)",
              settled_samples.size(), stale_samples_rejected, settled_stamps.front(), settled_stamps.back(),
              achieved_tcp[0], achieved_tcp[1], achieved_tcp[2], tcp_error_m,
              grasp_pose_error_max_m);

            if (tcp_error_m > grasp_pose_error_max_m) {
              RCLCPP_ERROR(
                logger,
                "POSE_VERIFY_FAILURE: descent reported SUCCESS but ground truth "
                "puts the TCP %.4f m from the commanded grasp pose (max %.4f). "
                "Closing here would produce a cycle that reads as a friction "
                "failure. Aborting before the close.",
                tcp_error_m, grasp_pose_error_max_m);
              result = Result::POSE_VERIFY_FAILURE;
            }
          } else {
            RCLCPP_ERROR(
              logger,
              "POSE_VERIFY_FAILURE: failed to obtain 3 fresh settled post-boundary (%.9f) "
              "Gazebo Pose_V samples within timeout (rejected_stale=%zu). Aborting.",
              boundary_sec, stale_samples_rejected);
            result = Result::POSE_VERIFY_FAILURE;
          }
        } else {
          RCLCPP_ERROR(
            logger,
            "EXECUTE_FAILURE: MoveIt returned error code %d after a valid "
            "Cartesian path was found.",
            exec_code.val);
          result = Result::EXECUTE_FAILURE;
        }
      }
    }

#endif
    if (executed && ur5e_pick_place::ok(result) && descent_only) {
      RCLCPP_INFO(
        logger,
        "DESCENT_ONLY_STOP: descent completed successfully and ground-truth pose "
        "verification passed (tcp_error_m=%.6f m <= %.6f m). descent_only is set. "
        "No gripper closure command, no lift, no transport. Halting execution here.",
        tcp_error_m, grasp_pose_error_max_m);
    }

    // -----------------------------------------------------------------
    // Gripper close, hold, and verify — only once the arm has genuinely
    // reached the corrected grasp target AND that has been verified against
    // ground truth (Stage 2 verification above). Reuses gripper_client from
    // the pre-close stage above (same action server, no need to reconnect).
    // -----------------------------------------------------------------
    if (executed && ur5e_pick_place::ok(result) && !pregrasp_only && !descent_only) {
      gripper_close_attempted = true;
      // Command PAST what the object permits and let contact stop it —
      // same pattern as every measurement script in this project
      // (04/06/m0_verify.sh's C2), not a new heuristic.
      //
      // VENDOR: closed position resolved from the URDF's own joint bounds at
      // runtime via MoveIt's robot model, per scene.yaml's own documented
      // intent for gripper.closed_position, rather than hardcoding 0.8.
      // Unchanged.
      //
      // PARALLEL_JAW: does NOT query MoveIt's robot model here. MoveIt's
      // robot_description (built by m3_grasp.launch.py's MoveItConfigsBuilder)
      // is NOT gripper_model-aware -- it is always built from the vendor
      // xacro args, so model->getJointModel("gripper_jaw_joint") would
      // return nullptr, silently falling through to the 0.8 (RADIAN-scale)
      // default below -- exactly the "silently reinterpret radians as
      // metres" bug this dispatch must not introduce. pj_q_close_commanded
      // (parallel_jaw_geometry.Q_MAX_M, validated > 0 above) is used
      // directly instead, sidestepping that mismatched model entirely. See
      // this integration's own report for why the MoveIt config gap itself
      // is not fixed here.
      double target_position = 0.8;
      if (is_parallel_jaw) {
        target_position = pj_q_close_commanded;
      } else if (auto model = move_group.getRobotModel()) {
        if (auto joint = model->getJointModel(actuated_joint)) {
          target_position = joint->getVariableBounds(actuated_joint).max_position_;
        }
      }

      final_close_target_commanded = target_position;
      RCLCPP_INFO(
        logger, "RESOLVED FINAL CLOSE TARGET: target_position=%.6f %s (aperture=%.6f m)",
        target_position, is_parallel_jaw ? "m" : "rad",
        is_parallel_jaw ? (0.085 - target_position) : -1.0);

      // Enable C1/C2 closure contacts in ACM immediately before final gripper closure
      if (ur5e_pick_place::ok(result)) {
        std::string scene_err;
        if (!psm->enableClosureContacts(scene_err)) {
          RCLCPP_ERROR(
            logger, "CLOSURE_ACM_FAILURE: could not enable C1/C2 closure contacts: %s",
            scene_err.c_str());
          result = Result::CLOSURE_ACM_FAILURE;
        } else if (!psm->verifyExpectedScene(scene_err)) {
          RCLCPP_ERROR(
            logger, "CLOSURE_ACM_FAILURE: C1/C2 readback verification failed: %s",
            scene_err.c_str());
          result = Result::CLOSURE_ACM_FAILURE;
        } else {
          RCLCPP_INFO(
            logger, "CLOSURE_ACM_VERIFIED: C1/C2 enabled in ACM with fingerprint %s",
            psm->fingerprint().c_str());
        }
      }

      if (ur5e_pick_place::ok(result)) {
        grip = gripper_close_and_hold(
          gripper_client, target_position, gripper_max_effort, gripper_command_timeout_s,
          gz_js_topic, actuated_joint, logger);
        have_grip_result = true;
      }

      RCLCPP_INFO(
        logger, "gripper_close_and_hold: %s in achieved=%.4f %s (target was %.4f)",
        to_string(grip.kind), grip.achieved_position, is_parallel_jaw ? "m" : "rad",
        target_position);

      if (grip.kind == GripperCloseResult::Kind::UNKNOWN_NO_SAMPLE) {
        RCLCPP_ERROR(
          logger, "GRIPPER_GOAL_REJECTED: overclose call produced no result AND "
          "ground truth could not be sampled");
        if (ur5e_pick_place::ok(result)) { result = Result::GRIPPER_GOAL_REJECTED; }
      } else if (have_expected_grip_angle) {
        // within_tolerance is INFORMATIONAL ONLY, logged for the CSV/summary
        // and nothing else — it does not gate transport and does not set
        // `result`. Against a rigid object the joint physically cannot
        // advance past geometric touch (DART will not allow a mm of
        // interpenetration per side), so no achieved-angle band can ever
        // distinguish a real squeeze from the fingers merely resting at
        // first contact — a wide tolerance passes a gap as a grasp, a tight
        // one (e.g. gripper.squeeze) fails every real grasp including good
        // ones. Angle cannot be the verdict either way. Whether the object
        // was actually grasped is decided from Gazebo's own pose ground
        // truth, by scripts/lib/slip.py, same separation of concerns
        // transport.hpp already documents for attachObject.
        //
        // VENDOR ONLY (this branch requires have_expected_grip_angle, which
        // parallel_jaw never sets -- see the pj_within_tolerance branch
        // below for its own, separately-derived linear check).
        const double err = std::abs(grip.achieved_position - expected_grip_angle);
        within_tolerance = err <= grasp_tolerance_rad;
        RCLCPP_INFO(
          logger,
          "grasp-success check (informational only, does not gate the cycle): "
          "achieved=%.4f expected=%.4f (width=%.4fm) |err|=%.4f tolerance=%.4f -> %s",
          grip.achieved_position, expected_grip_angle, object_width_m, err,
          grasp_tolerance_rad, within_tolerance ? "WITHIN TOLERANCE" : "OUTSIDE TOLERANCE");
      } else if (is_parallel_jaw) {
        // PARALLEL_JAW's own informational check, metres, using
        // pj_grasp_tolerance_m -- an explicitly-derived linear value (see
        // this parameter's own header comment), never grasp_tolerance_rad
        // reinterpreted. Also does not gate the cycle, for the identical
        // reason given above: DART will not allow interpenetration, so no
        // achieved-aperture band can distinguish contact from squeeze
        // either.
        const double pj_err = std::abs(grip.achieved_position - pj_q_final_expected);
        within_tolerance = pj_err <= pj_grasp_tolerance_m;
        RCLCPP_INFO(
          logger,
          "grasp-success check (informational only, does not gate the cycle): "
          "achieved_q=%.6f expected_q=%.6f (width=%.4fm) |err|=%.6f tolerance=%.6f m -> %s",
          grip.achieved_position, pj_q_final_expected, object_width_m, pj_err,
          pj_grasp_tolerance_m, within_tolerance ? "WITHIN TOLERANCE" : "OUTSIDE TOLERANCE");
      }

      // Attach target to gripper only after physical grasp success verification
      if (ur5e_pick_place::ok(result) && have_grip_result &&
          grip.kind != GripperCloseResult::Kind::UNKNOWN_NO_SAMPLE)
      {
        std::string scene_err;
        if (!psm->attachTarget(scene_err)) {
          RCLCPP_ERROR(
            logger, "ATTACH_FAILURE: failed to attach target to gripper: %s",
            scene_err.c_str());
          result = Result::ATTACH_FAILURE;
        } else if (!psm->verifyExpectedScene(scene_err)) {
          RCLCPP_ERROR(
            logger, "ATTACH_FAILURE: attached target readback verification failed: %s",
            scene_err.c_str());
          result = Result::ATTACH_FAILURE;
        } else {
          RCLCPP_INFO(
            logger,
            "TARGET_ATTACHED_VERIFIED: pick_target attached to gripper_base_link with pad touch links; S enabled (fingerprint %s).",
            psm->fingerprint().c_str());
        }
      }
    }

    if (grasp_only && ur5e_pick_place::ok(result) && have_grip_result) {
      f2_stop_reached = true;
      RCLCPP_INFO(
        logger,
        "F2 STOP: grasp_only set; grasp established, no lift/transport/place will be "
        "attempted. Physical grasp acceptance remains based on the existing bilateral "
        "contact and Gazebo object-pose evidence captured by the validation harness.");
    }

    // -----------------------------------------------------------------
    // Lift, transport, place, release, retreat — the legs M3's criterion
    // names that this file did not have. Attempted whenever the close
    // produced a real sample (grip.kind != UNKNOWN_NO_SAMPLE, checked
    // above), regardless of within_tolerance — see the comment at that
    // check for why achieved angle cannot be the gate. Running transport
    // unconditionally lets Gazebo's own ground truth (slip.py) decide every
    // cycle uniformly, including the ones angle would have wrongly
    // dismissed or wrongly waved through.
    //
    // close_and_hold_only short-circuits this entire block: the close
    // above has already run and recorded its result unmodified, this just
    // skips everything after it. attempted_transport stays false and
    // transport_result stays at its SUCCESS default, both already handled
    // correctly by the existing "N/A" ternaries in the CSV/summary below.
    // -----------------------------------------------------------------
    if (ur5e_pick_place::ok(result) && !close_and_hold_only && !pregrasp_only && !grasp_only && !descent_only) {
      // Keep the legacy attempted_transport field unchanged for the classical
      // path. In lift_only the helper enters Stage 3 but returns before Stage 4.
      attempted_transport = !lift_only;
      transport_attempted = !lift_only;
      place_release_attempted = !lift_only && !transport_only;

      // place_frame is published by static_scene_tf the same way grasp_frame
      // is: world -> object_frame at object.place_pose, then the SAME
      // approach_axis/gripper_roll rotation grasp_frame uses. Looked up here
      // rather than recomputed, so the orientation math exists in exactly
      // one place in this project.
      // Stage 3 needs no place pose. Avoid making lift_only depend on a Stage-4
      // transform that it is explicitly forbidden to consume.
      bool got_place_tf = lift_only;
      tf2::Transform T_world_place;
      if (!lift_only) {
        auto t0 = std::chrono::steady_clock::now();
        const auto deadline = t0 + std::chrono::duration<double>(tf_lookup_timeout_s);
        std::string tf_error;
        while (std::chrono::steady_clock::now() < deadline) {
          if (tf_buffer->canTransform(world_frame, place_frame_name, tf2::TimePointZero, &tf_error)) {
            auto stamped = tf_buffer->lookupTransform(world_frame, place_frame_name, tf2::TimePointZero);
            tf2::fromMsg(stamped.transform, T_world_place);
            got_place_tf = true;
            break;
          }
          rclcpp::sleep_for(20ms);
        }
        if (!got_place_tf) {
          RCLCPP_ERROR(
            logger,
            "TF_LOOKUP_TIMEOUT: could not resolve %s -> %s within %.1fs: %s. "
            "static_scene_tf only publishes this if object.place_pose was "
            "passed to it — check the launch file.",
            world_frame.c_str(), place_frame_name.c_str(), tf_lookup_timeout_s,
            tf_error.c_str());
          transport_result = Result::TF_LOOKUP_TIMEOUT;
          if (ur5e_pick_place::ok(result)) { result = transport_result; }
        }
      }

      if (got_place_tf) {
        // Full gripper pose in the planning frame, composed the SAME way
        // grasp_pose was above (T_world_grasp * T_tcp_tool0) — orientation
        // included. A position-only place pose plans to an arbitrary wrist
        // orientation and drops the object sideways; this is the one place
        // flagged as likeliest to get wrong, so it is built identically to
        // the already-verified grasp_pose rather than freshly invented.
        geometry_msgs::msg::Pose place_pose_msg;
        if (!lift_only) {
          tf2::Transform T_world_place_tool0 = T_world_place * T_tcp_tool0;
          tf2::toMsg(T_world_place_tool0, place_pose_msg);
        }

        // approach_axis "already resolved from object_frame into the
        // planning frame", exactly as transport.hpp's contract requires:
        // grasp_frame's own local Z axis, expressed in world, IS that
        // resolved axis by construction (orientation_from_approach_axis in
        // static_scene_tf.cpp puts local Z along approach_axis). Reusing it
        // via the already-looked-up T_world_grasp avoids a second copy of
        // the approach_axis parameter and the rotation math that builds it.
        tf2::Vector3 world_z = T_world_grasp.getBasis() * tf2::Vector3(0, 0, 1);

        ur5e_pick_place::TransportParams tp;
        tp.approach_axis = {{world_z.x(), world_z.y(), world_z.z()}};
        tp.lift_distance = retreat;
        tp.standoff = standoff;
        tp.place_pose = place_pose_msg;
        tp.cartesian_fraction_min = cartesian_fraction_min;
        tp.eef_step = eef_step;
        tp.release_position_rad = release_position_rad;
        tp.slip_sample_dwell_s = slip_sample_dwell_s;
        tp.lift_only = lift_only;
        tp.transport_only = transport_only;
        tp.velocity_scaling = vel_scale;
        tp.acceleration_scaling = acc_scale;
        tp.cycle_index = 0;
        tp.sim_instance = 0;
        tp.marker_file_prefix = marker_file_prefix;
        tp.planning_scene_manager = psm;
        tp.pickup_clearance_m = 0.005;
        tp.terminal_stroke_m = 0.005;
        // Grasp-loss check (transport.hpp's Stage 3 note). Left at
        // TransportParams' own defaults (expected_grip_angle=0.0, disabling
        // the check) when the grasp table didn't resolve an expected angle
        // for this object width — same guard have_expected_grip_angle
        // already uses above for the informational within_tolerance check.
        //
        // DELIBERATE for gripper_model=parallel_jaw too: have_expected_grip_angle
        // is never set true for that model (see the vendor-only block above),
        // so this check stays disabled rather than being fed
        // grasp_loss_threshold_rad -- a RADIANS threshold -- against an
        // achieved value now in METRES, which would silently never trigger
        // (a millimetre-scale drift would never exceed a threshold sized
        // for a ~1.4rad joint range) and give false confidence that no grasp
        // was lost. transport.cpp/transport.hpp are NOT modified by this
        // integration; a linear grasp-loss threshold for parallel_jaw is
        // unimplemented scope, not silently approximated. Any run that
        // proceeds past the grasp stage for gripper_model=parallel_jaw must
        // stop before this check would have mattered (grasp_only or
        // earlier) until that is addressed.
        if (have_expected_grip_angle) {
          tp.expected_grip_angle = expected_grip_angle;
          tp.grasp_loss_threshold_rad = grasp_loss_threshold_rad;
          tp.actuated_joint = actuated_joint;
          tp.gz_js_topic = gz_js_topic;
        }

        // Reuses gripper_close_and_hold rather than a new send-only helper:
        // "send toward this position, then unconditionally hold wherever it
        // ended up" is exactly what a release needs too, not just a close.
        // Any outcome that produced an achieved position (REACHED_GOAL,
        // STALLED, TIMED_OUT_HELD) counts as a usable release; only
        // UNKNOWN_NO_SAMPLE — no result AND ground truth unreadable — is a
        // real failure, because it leaves the object's state unknown with
        // the retreat still to come.
        std::function<Result(double)> release_gripper =
          [&](double target_q) -> Result {
            auto rel = gripper_close_and_hold(
              gripper_client, target_q, gripper_max_effort, gripper_command_timeout_s,
              gz_js_topic, actuated_joint, logger);
            if (is_parallel_jaw) {
              constexpr double kParallelJawFullOpenM = 0.085;
              const double achieved_q = rel.achieved_position;
              const double achieved_aperture = kParallelJawFullOpenM - achieved_q;
              RCLCPP_INFO(
                logger,
                "release: action_result=%s target_q=%.4f m achieved_q=%.4f m achieved_aperture=%.4f m",
                to_string(rel.kind), target_q, achieved_q, achieved_aperture);
              // Ensure commanded opening actually clears the object (must not STALL and aperture must clear object)
              if (rel.kind == GripperCloseResult::Kind::STALLED || achieved_aperture < 0.050) {
                RCLCPP_ERROR(
                  logger,
                  "RELEASE_FAILURE: parallel_jaw failed to clear object (action=%s achieved_q=%.4f m achieved_aperture=%.4f m)",
                  to_string(rel.kind), achieved_q, achieved_aperture);
                return Result::GRIPPER_GOAL_REJECTED;
              }
            } else {
              RCLCPP_INFO(
                logger, "release: %s in achieved=%.4f rad (target was %.4f)",
                to_string(rel.kind), rel.achieved_position, target_q);
            }
            if (rel.kind == GripperCloseResult::Kind::UNKNOWN_NO_SAMPLE) {
              return Result::GRIPPER_GOAL_REJECTED;
            }
            return Result::SUCCESS;
          };

        // --- Pre-lift barrier (default-off) ------------------------------
        // Reached immediately before the lift is attempted, with the grasp
        // established and nothing having moved since. Enabled only when
        // pre_lift_barrier_file is non-empty AND lift_only is set: an empty
        // path (the default) leaves every existing path byte-for-byte
        // unchanged, and the barrier is deliberately not offered to the
        // classical transport path, which has no evaluator to wait for.
        //
        // The wait is on the steady clock, not the simulation clock, for the
        // same reason the place-TF lookup above is: a barrier whose timeout
        // depends on simulation time cannot fire if the simulator itself
        // stalls, which turns a fail-safe into a hang.
        bool barrier_released = true;
        if (!pre_lift_barrier_file.empty() && lift_only) {
          barrier_released = false;
          if (!marker_file_prefix.empty()) {
            std::ofstream(marker_file_prefix + ".pre_lift_ready", std::ios::trunc).close();
          }
          RCLCPP_INFO(
            logger,
            "M3 F3 PRE_LIFT_BARRIER_ARMED t=%.6f file=%s timeout_s=%.3f; the grasp is "
            "established and the lift will NOT begin until this file appears.",
            node->get_clock()->now().seconds(), pre_lift_barrier_file.c_str(),
            pre_lift_barrier_timeout_s);

          const auto barrier_start = std::chrono::steady_clock::now();
          const auto barrier_deadline =
            barrier_start + std::chrono::duration<double>(pre_lift_barrier_timeout_s);
          while (rclcpp::ok() && std::chrono::steady_clock::now() < barrier_deadline) {
            if (std::ifstream(pre_lift_barrier_file).good()) {
              barrier_released = true;
              break;
            }
            rclcpp::sleep_for(20ms);
          }

          const double waited_wall_s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - barrier_start).count();
          if (barrier_released) {
            RCLCPP_INFO(
              logger,
              "M3 F3 PRE_LIFT_BARRIER_RELEASED t=%.6f waited_wall_s=%.3f; proceeding into the "
              "unmodified lift_only path.",
              node->get_clock()->now().seconds(), waited_wall_s);
          } else {
            RCLCPP_ERROR(
              logger,
              "PRE_LIFT_BARRIER_TIMEOUT: no release at '%s' within %.3fs (waited %.3fs wall, "
              "t=%.6f). The lift is NOT attempted. The grasp was established and is left "
              "exactly as it was; no lift, transport, place or release occurs.",
              pre_lift_barrier_file.c_str(), pre_lift_barrier_timeout_s, waited_wall_s,
              node->get_clock()->now().seconds());
            transport_result = Result::PRE_LIFT_BARRIER_TIMEOUT;
            if (ur5e_pick_place::ok(result)) { result = transport_result; }
          }
        }

        if (barrier_released) {
          lift_attempted = true;
          transport_result = ur5e_pick_place::lift_transport_place(
            node, move_group, tp, release_gripper);
          if (ur5e_pick_place::ok(result)) { result = transport_result; }
          lift_only_stop_reached = lift_only && ur5e_pick_place::ok(transport_result);
        }
      }
    }
  }

  // Ground-truth TCP evidence (tcp_error_m, achieved_tcp, have_ground_truth)
  // is now captured earlier, at Stage 2 verification — right after the
  // descent and before the close, the only moment the arm stands at the
  // commanded grasp pose with nothing having moved since. A second
  // measurement here, after lift/transport/place, would be comparing the
  // arm's post-cycle position against the pre-cycle grasp target and calling
  // the (expected, large) difference an "error" — removed rather than kept
  // as a second, now-meaningless number.

  // ---------------------------------------------------------------------
  // CSV evidence, written regardless of outcome.
  // ---------------------------------------------------------------------
  {
    std::ofstream csv(csv_path);
    if (csv) {
      // position_source is deliberately FIRST after result: it is the field
      // that decides whether a row is perception evidence at all. Inferring it
      // afterwards from commanded coordinates is not acceptable -- a fallback
      // run and a perceived run differ by ~1 mm, which is unreadable without
      // already knowing the configured value.
      csv << "result,position_source,yaw_source,configured_object_yaw_deg,"
             "perceived_object_yaw_deg,yaw_delta_deg,commanded_grasp_yaw_deg,"
             "pregrasp_only,grasp_only,"
             "pregrasp_attempted,pregrasp_succeeded,descent_attempted,descent_succeeded,"
             "gripper_close_attempted,gripper_close_succeeded,f2_stop_reached,"
             "lift_attempted,transport_attempted,place_release_attempted,"
             "cartesian_fraction,executed,"
             "commanded_x,commanded_y,commanded_z,"
             "achieved_x,achieved_y,achieved_z,tcp_error_m,have_ground_truth,"
             "gripper_result_kind,achieved_grip_angle_rad,expected_grip_angle_rad,"
             "within_tolerance,preclose_result_kind,preclose_achieved_rad,"
             "attempted_transport,transport_result,lift_only,lift_only_stop_reached,"
             "lift_result,"
             // --- gripper_model dispatch evidence, 2026-08-25. Additive only --
             // every column above is unchanged in name, order and meaning.
             // command_units/commanded_q/achieved_q hold the SAME native value
             // as gripper_result_kind's achieved_grip_angle_rad /
             // preclose_achieved_rad columns above, just labelled with the
             // model that produced them instead of assuming radians.
             // commanded_aperture_m/achieved_aperture_m are populated (via
             // aperture(q)=0.085-q) ONLY for parallel_jaw; -1.0 for
             // robotiq_linkage, which has no equivalent linear aperture
             // formula available here.
             "gripper_model,command_units,commanded_q,commanded_aperture_m,"
             "achieved_q,achieved_aperture_m,max_effort_commanded\n";
      // 0.085 m: parallel_jaw_geometry.APERTURE_FULL_OPEN_M, the ONE constant
      // duplicated here (for CSV display only, not decision logic) because
      // C++ cannot import the Python module — see this file's own dispatch
      // comments for why every other parallel-jaw VALUE (q_preclose,
      // q_final_expected, q_close_commanded, offsets) arrives as a launch
      // parameter instead of being recomputed here.
      constexpr double kParallelJawApertureFullOpenM = 0.085;
      const double commanded_aperture_m = (is_parallel_jaw && final_close_target_commanded >= 0.0)
        ? (kParallelJawApertureFullOpenM - final_close_target_commanded) : -1.0;
      const double achieved_aperture_m = (is_parallel_jaw && have_grip_result)
        ? (kParallelJawApertureFullOpenM - grip.achieved_position) : -1.0;
      csv << to_string(result) << ',' << position_source << ',' << yaw_source
          << ',' << configured_object_yaw_deg << ',' << perceived_object_yaw_deg
          << ',' << yaw_delta_deg << ',' << commanded_grasp_yaw_deg
          << ',' << (pregrasp_only ? 1 : 0) << ',' << (grasp_only ? 1 : 0)
          << ',' << (pregrasp_attempted ? 1 : 0) << ',' << (pregrasp_succeeded ? 1 : 0)
          << ',' << (descent_attempted ? 1 : 0) << ',' << (executed ? 1 : 0)
          << ',' << (gripper_close_attempted ? 1 : 0)
          << ',' << (have_grip_result &&
            grip.kind != GripperCloseResult::Kind::UNKNOWN_NO_SAMPLE ? 1 : 0)
          << ',' << (f2_stop_reached ? 1 : 0)
          << ',' << (lift_attempted ? 1 : 0)
          << ',' << (transport_attempted ? 1 : 0)
          << ',' << (place_release_attempted ? 1 : 0)
          << ',' << achieved_fraction << ',' << (executed ? 1 : 0)
          << ',' << commanded_tcp[0] << ',' << commanded_tcp[1] << ',' << commanded_tcp[2]
          << ',' << achieved_tcp[0] << ',' << achieved_tcp[1] << ',' << achieved_tcp[2]
          << ',' << tcp_error_m << ',' << (have_ground_truth ? 1 : 0)
          << ',' << (have_grip_result ? to_string(grip.kind) : "N/A")
          << ',' << (have_grip_result ? grip.achieved_position : -1.0)
          << ',' << (have_expected_grip_angle ? expected_grip_angle : -1.0)
          << ',' << (within_tolerance ? 1 : 0)
          << ',' << (have_preclose_result ? to_string(preclose_result.kind) : "N/A")
          << ',' << (have_preclose_result ? preclose_result.achieved_position : -1.0)
          << ',' << (attempted_transport ? 1 : 0)
          << ',' << (attempted_transport ? to_string(transport_result) : "N/A")
          << ',' << (lift_only ? 1 : 0)
          << ',' << (lift_only_stop_reached ? 1 : 0)
          << ',' << (lift_attempted ? to_string(transport_result) : "N/A")
          << ',' << gripper_model << ',' << (is_parallel_jaw ? "m" : "rad")
          << ',' << final_close_target_commanded
          << ',' << commanded_aperture_m
          << ',' << (have_grip_result ? grip.achieved_position : -1.0)
          << ',' << achieved_aperture_m
          << ',' << gripper_max_effort << '\n';
      RCLCPP_INFO(logger, "wrote evidence to %s", csv_path.c_str());
    } else {
      RCLCPP_ERROR(logger, "could not open %s for writing", csv_path.c_str());
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("m3_grasp"),
    "RUN SUMMARY: milestone=M3 gripper_model=%s command_units=%s result=%s "
    "position_source=%s yaw_source=%s configured_object_yaw_deg=%.4f "
    "perceived_object_yaw_deg=%.4f yaw_delta_deg=%.4f commanded_grasp_yaw_deg=%.4f "
    "pregrasp_only=%s grasp_only=%s "
    "lift_only=%s lift_only_stop_reached=%s "
    "pregrasp_attempted=%s pregrasp_succeeded=%s descent_attempted=%s "
    "descent_succeeded=%s gripper_close_attempted=%s gripper_close_succeeded=%s "
    "f2_stop_reached=%s lift_attempted=%s transport_attempted=%s "
    "place_release_attempted=%s cartesian_fraction=%.4f executed=%s "
    "tcp_error_m=%.4f ground_truth=%s gripper_result=%s achieved_grip_angle=%.4f "
    "expected_grip_angle=%.4f within_tolerance=%s preclose_result=%s preclose_achieved=%.4f "
    "attempted_transport=%s transport_result=%s lift_result=%s",
    gripper_model.c_str(), is_parallel_jaw ? "m" : "rad",
    to_string(result), position_source.c_str(), yaw_source.c_str(), configured_object_yaw_deg,
    perceived_object_yaw_deg, yaw_delta_deg, commanded_grasp_yaw_deg,
    pregrasp_only ? "yes" : "no",
    grasp_only ? "yes" : "no", lift_only ? "yes" : "no",
    lift_only_stop_reached ? "yes" : "no", pregrasp_attempted ? "yes" : "no",
    pregrasp_succeeded ? "yes" : "no", descent_attempted ? "yes" : "no",
    executed ? "yes" : "no", gripper_close_attempted ? "yes" : "no",
    have_grip_result && grip.kind != GripperCloseResult::Kind::UNKNOWN_NO_SAMPLE ? "yes" : "no",
    f2_stop_reached ? "yes" : "no", lift_attempted ? "yes" : "no",
    transport_attempted ? "yes" : "no", place_release_attempted ? "yes" : "no",
    achieved_fraction, executed ? "yes" : "no",
    tcp_error_m,
    have_ground_truth ? "yes" : "no",
    have_grip_result ? to_string(grip.kind) : "N/A",
    have_grip_result ? grip.achieved_position : -1.0,
    have_expected_grip_angle ? expected_grip_angle : -1.0,
    within_tolerance ? "yes" : "no",
    have_preclose_result ? to_string(preclose_result.kind) : "N/A",
    have_preclose_result ? preclose_result.achieved_position : -1.0,
    attempted_transport ? "yes" : "no",
    attempted_transport ? to_string(transport_result) : "N/A",
    lift_attempted ? to_string(transport_result) : "N/A");

  // Marker-file signal, mirroring transport.cpp's LIFT_DONE/TRANSPORT_DONE
  // touches (TransportParams::marker_file_prefix). Written LAST, after
  // every path through this function -- success or any typed failure --
  // so scripts/11_m3_cycles.sh's poll-based watcher can break out promptly
  // on an aborted cycle (one that never reaches TRANSPORT_DONE) instead of
  // polling for the full MARKER_TIMEOUT, matching the old stdout-based
  // watcher's "RUN SUMMARY seen, stop waiting" behavior.
  if (!marker_file_prefix.empty()) {
    std::ofstream(marker_file_prefix + ".run_summary_ready", std::ios::trunc).close();
  }

    executor.cancel();
    if (spinner.joinable()) { spinner.join(); }
  }
  rclcpp::shutdown();

  return ur5e_pick_place::ok(result) ? 0 : 1;
}
#endif  // UR5E_PICK_PLACE_M3_GRASP_UNIT_TEST

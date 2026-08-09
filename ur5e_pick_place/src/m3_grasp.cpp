// m3_grasp.cpp — Milestone M3, first pass: pad-centre-corrected grasp +
//                grasp-success verification. NOT the whole M3 milestone.
//
// EVIDENCE THIS PRODUCES
//   Approach to a pad-centre-corrected grasp target (reusing M2's proven
//   two-stage joint+Cartesian pattern), a bounded gripper close-and-hold,
//   and a grasp-success verdict from comparing the achieved joint angle
//   against config/grasp_table.yaml — NOT from trusting the ROS action's
//   own stalled:true result. See docs/HANDOFF_M3.md, "Pad-centre correction
//   and grasp-success verification: design, not yet implemented" (now
//   implemented, this file) and the section above it, "box-settle
//   false-quiescence", for why stalled:true alone is not sufficient
//   evidence of a real grasp.
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
// GRASP-SUCCESS VERIFICATION, NOT THE ACTION RESULT
//   After gripper_close_and_hold-equivalent logic settles on an achieved
//   joint angle, that angle is compared against config/grasp_table.yaml's
//   grip_angle_rad (interpolated for object.size[grasp_width_axis]) within
//   grasp.grasp_tolerance_rad. Outside tolerance -> GRIPPER_GOAL_REJECTED,
//   the cycle aborts rather than proceeding as if a successful attach were
//   evidence of a successful grasp.
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
#include <control_msgs/action/gripper_command.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "ur5e_pick_place/failure.hpp"
#include "ur5e_pick_place/moveit_compat.hpp"

using ur5e_pick_place::Result;
using ur5e_pick_place::to_string;
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

std::string run_command(const std::string & cmd)
{
  std::array<char, 4096> buf;
  std::string out;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
  if (!pipe) {
    return out;
  }
  while (fgets(buf.data(), buf.size(), pipe.get()) != nullptr) {
    out += buf.data();
  }
  return out;
}

// Same parse as m2_cartesian_approach.cpp — see that file's header for why
// this replaced a ros_gz_bridge TFMessage approach.
std::optional<tf2::Transform> parse_link_pose(const std::string & dump, const std::string & link_name)
{
  const std::string name_needle = "name: \"" + link_name + "\"";
  auto name_pos = dump.find(name_needle);
  if (name_pos == std::string::npos) {
    return std::nullopt;
  }

  auto extract_field = [&](const std::string & block, char field) -> double {
      const std::string needle = std::string(1, field) + ":";
      auto p = block.find(needle);
      if (p == std::string::npos) {
        return 0.0;
      }
      return std::strtod(block.c_str() + p + needle.size(), nullptr);
    };

  auto pos_start = dump.find("position {", name_pos);
  auto pos_end = dump.find('}', pos_start);
  auto ori_start = dump.find("orientation {", name_pos);
  auto ori_end = dump.find('}', ori_start);
  if (pos_start == std::string::npos || pos_end == std::string::npos ||
    ori_start == std::string::npos || ori_end == std::string::npos)
  {
    return std::nullopt;
  }

  const std::string pos_block = dump.substr(pos_start, pos_end - pos_start);
  const std::string ori_block = dump.substr(ori_start, ori_end - ori_start);

  tf2::Vector3 origin(
    extract_field(pos_block, 'x'), extract_field(pos_block, 'y'), extract_field(pos_block, 'z'));
  auto oz = ori_block.find("z:");
  auto ow = ori_block.find("w:");
  tf2::Quaternion q(
    extract_field(ori_block, 'x'), extract_field(ori_block, 'y'),
    oz == std::string::npos ? 0.0 : extract_field(ori_block, 'z'),
    ow == std::string::npos ? 1.0 : extract_field(ori_block, 'w'));
  return tf2::Transform(q, origin);
}

// Ground-truth JOINT position (not a link pose) for a named joint, from a
// `gz topic -e` text dump of a gz.msgs.Model joint_state topic. Used only
// as the TIMED_OUT_HELD fallback below, same role as gz_settle.sh's
// gripper_close_and_hold(): the action client's own result future is the
// primary source, this is what covers "the command call itself never
// returned a Result before the bound expired."
std::optional<double> parse_joint_position(const std::string & dump, const std::string & joint_name)
{
  const std::string name_needle = "name: \"" + joint_name + "\"";
  auto name_pos = dump.find(name_needle);
  if (name_pos == std::string::npos) {
    return std::nullopt;
  }
  auto pos_key = dump.find("position:", name_pos);
  // Bound the search to this joint's own block: stop at the next "joint {"
  // or end of string, so a malformed/short dump can't accidentally read a
  // neighboring joint's position field.
  auto next_joint = dump.find("joint {", name_pos + name_needle.size());
  if (pos_key == std::string::npos || (next_joint != std::string::npos && pos_key > next_joint)) {
    return std::nullopt;
  }
  return std::strtod(dump.c_str() + pos_key + std::string("position:").size(), nullptr);
}

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

  // Unconditional hold at wherever the joint actually ended up — this is
  // the fix, not the fallback. Best-effort: a hold-call timeout here is
  // logged but does not change the outcome already determined above.
  auto hold_result = send_and_wait(out.achieved_position);
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "m3_grasp",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  Result result = Result::SUCCESS;

  // ---------------------------------------------------------------------
  // Parameters. All originate in config/scene.yaml / config/grasp_table.yaml
  // via the launch file.
  // ---------------------------------------------------------------------
  std::string world_frame = "world";
  std::string grasp_frame_name = "grasp_frame";
  std::string tool0_frame = "tool0";
  double standoff = 0.0;
  double tcp_offset = 0.0;
  double pad_centre_offset = 0.0;
  double tf_lookup_timeout_s = 2.0;
  double cartesian_fraction_min = 0.95;
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

  node->get_parameter_or("world_frame", world_frame, world_frame);
  node->get_parameter_or("grasp_frame_name", grasp_frame_name, grasp_frame_name);
  node->get_parameter_or("tool0_frame", tool0_frame, tool0_frame);
  node->get_parameter_or("standoff", standoff, standoff);
  node->get_parameter_or("tcp_offset", tcp_offset, tcp_offset);
  node->get_parameter_or("pad_centre_offset", pad_centre_offset, pad_centre_offset);
  node->get_parameter_or("tf_lookup_timeout_s", tf_lookup_timeout_s, tf_lookup_timeout_s);
  node->get_parameter_or("cartesian_fraction_min", cartesian_fraction_min, cartesian_fraction_min);
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
  node->get_parameter_or("grasp_table_widths_m", grasp_table_widths_m, grasp_table_widths_m);
  node->get_parameter_or(
    "grasp_table_grip_angles_rad", grasp_table_grip_angles_rad, grasp_table_grip_angles_rad);
  node->get_parameter_or("grasp_tolerance_rad", grasp_tolerance_rad, grasp_tolerance_rad);
  node->get_parameter_or("preclose_margin_rad", preclose_margin_rad, preclose_margin_rad);

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
  if (ur5e_pick_place::ok(result) && grasp_table_widths_m.empty()) {
    RCLCPP_ERROR(
      logger,
      "CONFIG_ERROR: grasp_table_widths_m is empty. Did the launch file load "
      "config/grasp_table.yaml?");
    result = Result::CONFIG_ERROR;
  }

  // Grasp-table lookup moved here (was previously computed only after the
  // final close call) so PRE-CLOSE, below, has a data-driven target angle
  // to aim for before the Cartesian descent even starts — not just the
  // grasp-success check afterward. Same interpolation, same table, used
  // twice: once as a target, once as a verification threshold.
  double expected_grip_angle = 0.0;
  bool have_expected_grip_angle = false;
  if (ur5e_pick_place::ok(result)) {
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

  const double corrected_offset = tcp_offset + pad_centre_offset;
  RCLCPP_INFO(
    logger,
    "pad-centre-corrected offset: tcp_offset=%.6f + pad_centre_offset=%.6f = %.6f m "
    "(NOT YET LIVE-VERIFIED direction -- see file header)",
    tcp_offset, pad_centre_offset, corrected_offset);

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
  }

  double achieved_fraction = 0.0;
  bool executed = false;
  double commanded_tcp[3] = {0, 0, 0};
  double achieved_tcp[3] = {0, 0, 0};
  double tcp_error_m = -1.0;
  bool have_ground_truth = false;

  GripperCloseResult grip;
  bool have_grip_result = false;
  GripperCloseResult preclose_result;
  bool have_preclose_result = false;
  bool within_tolerance = false;

  if (ur5e_pick_place::ok(result) && got_grasp_tf) {
    moveit::planning_interface::MoveGroupInterface move_group(node, kPlanningGroup);
    move_group.setPlanningTime(planning_time_s);
    move_group.setNumPlanningAttempts(plan_attempts);
    move_group.setMaxVelocityScalingFactor(vel_scale);
    move_group.setMaxAccelerationScalingFactor(acc_scale);

    // Same construction as m2_cartesian_approach.cpp, with the
    // pad-centre-CORRECTED offset in place of tcp_offset alone: tool0 ->
    // grasp_tcp is Translation(0,0,corrected_offset); its inverse (grasp
    // target -> tool0 target) is Translation(0,0,-corrected_offset).
    tf2::Transform T_tcp_tool0(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, -corrected_offset));

    tf2::Transform T_grasp_pregrasp(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, -standoff));
    tf2::Transform T_world_pregrasp = T_world_grasp * T_grasp_pregrasp;
    tf2::Transform T_world_pregrasp_tool0 = T_world_pregrasp * T_tcp_tool0;

    tf2::Transform T_world_grasp_tool0 = T_world_grasp * T_tcp_tool0;

    geometry_msgs::msg::Pose pregrasp_pose;
    tf2::toMsg(T_world_pregrasp_tool0, pregrasp_pose);
    geometry_msgs::msg::Pose grasp_pose;
    tf2::toMsg(T_world_grasp_tool0, grasp_pose);

    RCLCPP_INFO(
      logger,
      "pre-grasp tool0 target (world): [%.4f %.4f %.4f]  grasp tool0 target: "
      "[%.4f %.4f %.4f]  (corrected_offset=%.6f)",
      pregrasp_pose.position.x, pregrasp_pose.position.y, pregrasp_pose.position.z,
      grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z, corrected_offset);

    // Stage 1: joint-space plan+execute to pre-grasp. Same reasoning as M2.
    move_group.setPoseTarget(pregrasp_pose, tool0_frame);
    moveit::planning_interface::MoveGroupInterface::Plan pregrasp_plan;
    const bool pregrasp_planned =
      (move_group.plan(pregrasp_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!pregrasp_planned) {
      RCLCPP_ERROR(logger, "PLAN_FAILURE: could not plan to the pre-grasp pose.");
      result = Result::PLAN_FAILURE;
    } else {
      const auto pregrasp_exec = move_group.execute(pregrasp_plan);
      if (pregrasp_exec != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(
          logger, "EXECUTE_FAILURE: pre-grasp plan found but execution returned code %d.",
          pregrasp_exec.val);
        result = Result::EXECUTE_FAILURE;
      }
    }

    // Stage 1.5: pre-close the gripper toward (expected_grip_angle -
    // preclose_margin_rad) WHILE STILL AT PRE-GRASP HEIGHT — standoff
    // metres above the object, free air, nothing to contact yet. This is
    // the pre-close this project's own spec has required since the
    // ejection finding (scene.yaml's gripper.preclose_heuristic), but
    // m3_grasp.cpp never implemented it — confirmed by grep during this
    // session's "what touches first" investigation. Turns out to matter
    // for a second reason nobody had connected until the depth-tracking
    // and knuckle-clearance findings met: with the gripper fully open
    // through the ENTIRE descent (the previous behaviour), the ~8-9mm of
    // pad (and knuckle) descent that happens as the joint closes from 0
    // to grip_angle happens RIGHT NEXT TO the object, eating into the
    // already-thin clearance margin. Pre-closing here means that descent
    // happens in free air instead, and the final close-and-verify call
    // below only has a few hundredths of a radian left to travel, not the
    // full stroke. See docs/HANDOFF_M3.md, "positioned wrong or built
    // wrong" and the cube-height follow-up.
    //
    // Target is expected_grip_angle MINUS a margin, not the expected angle
    // itself: closing exactly to the expected contact angle in free air
    // (nothing to stop it) would just drive to that commanded position
    // with reached_goal:true, leaving nothing left to genuinely verify
    // during the final close — the margin preserves a real, detectable
    // contact event for the grasp-success check to act on.
    auto gripper_client = rclcpp_action::create_client<GripperCommand>(
      node, "/" + gripper_ctrl + "/gripper_cmd");
    if (ur5e_pick_place::ok(result) && have_expected_grip_angle) {
      if (!gripper_client->wait_for_action_server(
            std::chrono::duration<double>(gripper_command_timeout_s)))
      {
        RCLCPP_ERROR(
          logger, "GRIPPER_GOAL_REJECTED: action server /%s/gripper_cmd not available "
          "within %.1fs (pre-close)", gripper_ctrl.c_str(), gripper_command_timeout_s);
        if (ur5e_pick_place::ok(result)) { result = Result::GRIPPER_GOAL_REJECTED; }
      } else {
        const double preclose_target =
          std::max(0.0, expected_grip_angle - preclose_margin_rad);
        preclose_result = gripper_close_and_hold(
          gripper_client, preclose_target, gripper_max_effort, gripper_command_timeout_s,
          gz_js_topic, actuated_joint, logger);
        have_preclose_result = true;
        RCLCPP_INFO(
          logger, "pre-close: %s in achieved=%.4f rad (target was %.4f, free air)",
          to_string(preclose_result.kind), preclose_result.achieved_position, preclose_target);
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
    if (ur5e_pick_place::ok(result)) {
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
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        ur5e_pick_place::set_trajectory(plan, trajectory);
        RCLCPP_INFO(logger, "executing Cartesian descent to grasp target...");
        const auto exec_code = move_group.execute(plan);
        executed = (exec_code == moveit::core::MoveItErrorCode::SUCCESS);
        if (executed) {
          RCLCPP_INFO(logger, "execution reported SUCCESS");
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

    // -----------------------------------------------------------------
    // Gripper close, hold, and verify — only once the arm has genuinely
    // reached the corrected grasp target. Reuses gripper_client from the
    // pre-close stage above (same action server, no need to reconnect).
    // -----------------------------------------------------------------
    if (executed) {
      // Command PAST what the object permits and let contact stop it —
      // same pattern as every measurement script in this project
      // (04/06/m0_verify.sh's C2), not a new heuristic. Closed position
      // resolved from the URDF's own joint bounds at runtime, per
      // scene.yaml's own documented intent for gripper.closed_position,
      // rather than hardcoding 0.8. With pre-close already having done
      // most of the travel, this call's job is the small residual close
      // plus genuine contact detection, not the whole stroke.
      double target_position = 0.8;
      if (auto model = move_group.getRobotModel()) {
        if (auto joint = model->getJointModel(actuated_joint)) {
          target_position = joint->getVariableBounds(actuated_joint).max_position_;
        }
      }

      grip = gripper_close_and_hold(
        gripper_client, target_position, gripper_max_effort, gripper_command_timeout_s,
        gz_js_topic, actuated_joint, logger);
      have_grip_result = true;

      RCLCPP_INFO(
        logger, "gripper_close_and_hold: %s in achieved=%.4f rad (target was %.4f)",
        to_string(grip.kind), grip.achieved_position, target_position);

      if (grip.kind == GripperCloseResult::Kind::UNKNOWN_NO_SAMPLE) {
        RCLCPP_ERROR(
          logger, "GRIPPER_GOAL_REJECTED: overclose call produced no result AND "
          "ground truth could not be sampled");
        if (ur5e_pick_place::ok(result)) { result = Result::GRIPPER_GOAL_REJECTED; }
      } else if (have_expected_grip_angle) {
        // Grasp-success verification: compare against the table, not the
        // action's own stalled:true/reached_goal fields. See file header.
        const double err = std::abs(grip.achieved_position - expected_grip_angle);
        within_tolerance = err <= grasp_tolerance_rad;
        RCLCPP_INFO(
          logger,
          "grasp-success check: achieved=%.4f expected=%.4f (width=%.4fm) "
          "|err|=%.4f tolerance=%.4f -> %s",
          grip.achieved_position, expected_grip_angle, object_width_m, err,
          grasp_tolerance_rad, within_tolerance ? "WITHIN TOLERANCE" : "OUTSIDE TOLERANCE");
        if (!within_tolerance) {
          RCLCPP_ERROR(
            logger,
            "GRIPPER_GOAL_REJECTED: achieved angle %.4f rad is %.4f rad from the "
            "table's expected %.4f rad for a %.4fm object (tolerance %.4f) — "
            "not treating this as a successful grasp, aborting the cycle.",
            grip.achieved_position, err, expected_grip_angle, object_width_m,
            grasp_tolerance_rad);
          if (ur5e_pick_place::ok(result)) { result = Result::GRIPPER_GOAL_REJECTED; }
        }
      }
    }
  }

  // ---------------------------------------------------------------------
  // Ground-truth TCP evidence, identical timing/method to M2.
  // ---------------------------------------------------------------------
  if (executed) {
    rclcpp::sleep_for(500ms);
    if (auto tool0_gt = ground_truth_tool0()) {
      tf2::Vector3 tcp_gt = tool0_gt->getOrigin() +
        tool0_gt->getBasis() * tf2::Vector3(0, 0, corrected_offset);
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
        "ground-truth grasp_tcp (Gazebo, not TF): [%.4f %.4f %.4f]  "
        "error vs commanded: %.4f m",
        achieved_tcp[0], achieved_tcp[1], achieved_tcp[2], tcp_error_m);
    } else {
      RCLCPP_ERROR(
        logger,
        "no ground-truth pose found for '%s' via `gz topic -e -t %s` — is "
        "the sim running? Evidence is commanded-only, not verified against "
        "physical truth.",
        gt_wrist3_link_name.c_str(), gz_pose_topic.c_str());
    }
  }

  // ---------------------------------------------------------------------
  // CSV evidence, written regardless of outcome.
  // ---------------------------------------------------------------------
  {
    std::ofstream csv(csv_path);
    if (csv) {
      csv << "result,cartesian_fraction,executed,"
             "commanded_x,commanded_y,commanded_z,"
             "achieved_x,achieved_y,achieved_z,tcp_error_m,have_ground_truth,"
             "gripper_result_kind,achieved_grip_angle_rad,expected_grip_angle_rad,"
             "within_tolerance,preclose_result_kind,preclose_achieved_rad\n";
      csv << to_string(result) << ',' << achieved_fraction << ',' << (executed ? 1 : 0)
          << ',' << commanded_tcp[0] << ',' << commanded_tcp[1] << ',' << commanded_tcp[2]
          << ',' << achieved_tcp[0] << ',' << achieved_tcp[1] << ',' << achieved_tcp[2]
          << ',' << tcp_error_m << ',' << (have_ground_truth ? 1 : 0)
          << ',' << (have_grip_result ? to_string(grip.kind) : "N/A")
          << ',' << (have_grip_result ? grip.achieved_position : -1.0)
          << ',' << (have_expected_grip_angle ? expected_grip_angle : -1.0)
          << ',' << (within_tolerance ? 1 : 0)
          << ',' << (have_preclose_result ? to_string(preclose_result.kind) : "N/A")
          << ',' << (have_preclose_result ? preclose_result.achieved_position : -1.0) << '\n';
      RCLCPP_INFO(logger, "wrote evidence to %s", csv_path.c_str());
    } else {
      RCLCPP_ERROR(logger, "could not open %s for writing", csv_path.c_str());
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("m3_grasp"),
    "RUN SUMMARY: milestone=M3 result=%s cartesian_fraction=%.4f executed=%s "
    "tcp_error_m=%.4f ground_truth=%s gripper_result=%s achieved_grip_angle=%.4f "
    "expected_grip_angle=%.4f within_tolerance=%s preclose_result=%s preclose_achieved=%.4f",
    to_string(result), achieved_fraction, executed ? "yes" : "no", tcp_error_m,
    have_ground_truth ? "yes" : "no",
    have_grip_result ? to_string(grip.kind) : "N/A",
    have_grip_result ? grip.achieved_position : -1.0,
    have_expected_grip_angle ? expected_grip_angle : -1.0,
    within_tolerance ? "yes" : "no",
    have_preclose_result ? to_string(preclose_result.kind) : "N/A",
    have_preclose_result ? preclose_result.achieved_position : -1.0);

  executor.cancel();
  if (spinner.joinable()) { spinner.join(); }
  rclcpp::shutdown();

  return ur5e_pick_place::ok(result) ? 0 : 1;
}

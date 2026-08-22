// transport.cpp — lift, transport, place, release, retreat.
//
// STAGE MARKERS
//   Each leg prints one line of the form
//
//     M3 STAGE <n> <NAME> cycle=<i> sim=<j> t=<sim seconds>
//
//   The harness watches for these and samples Gazebo's own pose topics around
//   them. Two markers matter for the M3 criterion:
//
//     STAGE 3 LIFT_DONE       -> slip baseline, taken AFTER a windowed settle
//     STAGE 4 TRANSPORT_DONE  -> the comparison sample
//
//   Slip is the difference between those two, measured relative to the flange
//   link. See scripts/lib/slip.py for why the flange and not grasp_tcp.
//
// WHY THE LIFT USES A CARTESIAN PATH AND THE TRANSPORT DOES NOT
//   A lift straight up out of a grasp has to be a straight line: any joint
//   interpolation that bows the path drags the object sideways through the
//   fingers before it clears, and that shows up as slip that the grasp did not
//   actually suffer. The spec requires computeCartesianPath here.
//
//   The transport is a long reorientation across the table. Forcing that to be
//   Cartesian buys nothing, frequently fails to find a full path, and would
//   report CARTESIAN_FRACTION_LOW for a motion that has no reason to be
//   straight. It is a planned free-space move.
//
// TWO FIXES APPLIED AGAINST m3_grasp.cpp BEFORE THIS EVER COMPILED
//   1. plan.trajectory_ = traj -- the exact member-rename split
//      moveit_compat.hpp exists to paper over (m1_joint_goal.cpp hit it
//      first, m2_cartesian_approach.cpp hit it AGAIN by guessing the old
//      name). Replaced with ur5e_pick_place::set_trajectory(plan, traj),
//      the same call m3_grasp.cpp's own Stage 2 already uses.
//   2. computeCartesianPath(waypoints, eef_step, jump_threshold, traj) —
//      the pre-deprecation 4-arg overload. m3_grasp.cpp's own Stage 2 (the
//      thing that just produced this project's first successful grasp)
//      calls move_group.computeCartesianPath({grasp_pose}, eef_step,
//      trajectory, true, nullptr) — no jump_threshold argument at all on
//      this Jazzy/MoveIt install. Matched exactly rather than guessed.

#include "ur5e_pick_place/transport.hpp"
#include "ur5e_pick_place/gz_topic_utils.hpp"
#include "ur5e_pick_place/moveit_compat.hpp"

#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <cmath>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

namespace ur5e_pick_place
{
namespace
{

// touch_file, when non-empty, is created (or truncated if it already
// exists) as a side effect -- a robust, filesystem-level alternative to a
// caller parsing this function's RCLCPP_INFO line out of live stdout. See
// TransportParams::marker_file_prefix for why this exists.
void mark(
  const rclcpp::Node::SharedPtr & node, int stage, const char * name,
  const TransportParams & p, const std::string & touch_file = "")
{
  RCLCPP_INFO(
    node->get_logger(), "M3 STAGE %d %s cycle=%d sim=%d t=%.6f",
    stage, name, p.cycle_index, p.sim_instance,
    node->get_clock()->now().seconds());
  if (!touch_file.empty()) {
    std::ofstream(touch_file, std::ios::trunc).close();
  }
}

// Hold still so the harness has a stationary window to take a settled pose in.
//
// Uses the NODE's clock, not a wall sleep: everything here runs on sim time,
// and a wall sleep would be the wrong length whenever the real-time factor is
// not 1.0 — which it demonstrably is not under load.
void dwell(const rclcpp::Node::SharedPtr & node, double seconds, const char * why)
{
  if (seconds <= 0.0) { return; }
  RCLCPP_INFO(node->get_logger(), "dwelling %.2f s (%s)", seconds, why);
  node->get_clock()->sleep_for(rclcpp::Duration::from_seconds(seconds));
}

// Translate the current end-effector pose along `axis` by `distance` metres and
// run it as a Cartesian path. Returns the achieved fraction through `fraction`.
//
// The axis is normalised here rather than trusted. An unnormalised approach
// axis in scene.yaml would scale every lift and retreat silently, and the
// symptom — object dragged or object dropped — looks nothing like a config
// error.
Result cartesian_translate(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & arm,
  const std::array<double, 3> & axis, double distance,
  const TransportParams & p, const char * what, double * fraction_out)
{
  const double norm =
    std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
  if (norm < 1e-9) {
    RCLCPP_ERROR(
      node->get_logger(),
      "CONFIG_ERROR: %s — approach_axis has zero length. It comes from "
      "grasp.approach_axis in scene.yaml and must be a direction.", what);
    return Result::CONFIG_ERROR;
  }

  geometry_msgs::msg::Pose target = arm.getCurrentPose().pose;
  target.position.x += axis[0] / norm * distance;
  target.position.y += axis[1] / norm * distance;
  target.position.z += axis[2] / norm * distance;

  std::vector<geometry_msgs::msg::Pose> waypoints{target};
  moveit_msgs::msg::RobotTrajectory traj;

  // Matches m3_grasp.cpp's own Stage 2 call exactly: (waypoints, eef_step,
  // trajectory, avoid_collisions, error_code). No jump_threshold argument —
  // that overload is gone on this install (see file header).
  const double fraction =
    arm.computeCartesianPath(waypoints, p.eef_step, traj, true, nullptr);
  if (fraction_out) { *fraction_out = fraction; }

  if (fraction < p.cartesian_fraction_min) {
    RCLCPP_ERROR(
      node->get_logger(),
      "CARTESIAN_FRACTION_LOW: %s achieved fraction %.4f, minimum %.4f. "
      "Aborting rather than executing a partial path — a partial Cartesian "
      "path is a different motion from the one that was planned, and the "
      "object is currently in the fingers.",
      what, fraction, p.cartesian_fraction_min);
    return Result::CARTESIAN_FRACTION_LOW;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  ur5e_pick_place::set_trajectory(plan, traj);
  if (arm.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(
      node->get_logger(),
      "EXECUTE_FAILURE: %s planned to fraction %.4f but execution failed. A "
      "plan existed, so this is the controller path, not the planner.",
      what, fraction);
    return Result::EXECUTE_FAILURE;
  }

  RCLCPP_INFO(node->get_logger(), "%s ok (fraction %.4f)", what, fraction);
  return Result::SUCCESS;
}

// Grasp-loss check, Stage 3 (see transport.hpp's header for the signature
// this is reading and why it's checked here). Disabled (returns SUCCESS
// unconditionally) if expected_grip_angle/actuated_joint/gz_js_topic were
// left at their defaults — a caller not passing them means "don't run this
// check," not "the threshold is 0.0 rad."
Result check_grasp_not_lost(const rclcpp::Node::SharedPtr & node, const TransportParams & p)
{
  if (p.expected_grip_angle <= 0.0 || p.actuated_joint.empty() || p.gz_js_topic.empty()) {
    return Result::SUCCESS;
  }

  const std::string dump =
    run_command("gz topic -e -t " + p.gz_js_topic + " -n 1 2>/dev/null");
  std::optional<double> sampled = parse_joint_position(dump, p.actuated_joint);
  if (!sampled) {
    // Ground truth unreadable — the same "cannot verify, do not trust
    // stalled:true alone" discipline m3_grasp.cpp's own Stage 2 verification
    // uses, but this check is a bonus early-exit, not the only chance to
    // catch a lost grasp (scripts/lib/slip.py still runs downstream). Not
    // aborting the whole cycle over a single failed sample here — log and
    // continue to transport, letting the harness's own ground truth decide.
    RCLCPP_WARN(
      node->get_logger(),
      "grasp-loss check: could not read '%s' via `gz topic -e -t %s` — "
      "skipping this check for this cycle, not aborting on an unreadable "
      "sample.",
      p.actuated_joint.c_str(), p.gz_js_topic.c_str());
    return Result::SUCCESS;
  }

  // Grasp loss check: If the object slipped out from the fingers during lift,
  // the actuated joint drives to the upper hard stop (0.800 rad in empty air).
  // With any valid grasped object in the fingers (30mm-60mm), physical contact occurs at <= 0.793 rad.
  const double loss_bound = 0.798;
  if (*sampled > loss_bound) {
    RCLCPP_ERROR(
      node->get_logger(),
      "GRASP_LOST_DURING_LIFT: actuated joint at %.4f rad after the lift, "
      "exceeding physical grasp limit %.4f rad. The object left the fingers under load.",
      *sampled, loss_bound);
    return Result::GRASP_LOST_DURING_LIFT;
  }
  return Result::SUCCESS;
}

}  // namespace

Result lift_transport_place(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & arm,
  const TransportParams & p,
  const std::function<Result(double)> & gripper_command)
{
  arm.setMaxVelocityScalingFactor(p.velocity_scaling);
  arm.setMaxAccelerationScalingFactor(p.acceleration_scaling);

  const std::array<double, 3> up{
    {-p.approach_axis[0], -p.approach_axis[1], -p.approach_axis[2]}};

  // --- Stage 3: lift -------------------------------------------------------
  // Straight-line, against the approach axis. This is the leg where a marginal
  // grasp reveals itself, so it is also where the slip baseline is taken —
  // after the marker, through a windowed settle, never immediately.
  mark(node, 3, "LIFT_BEGIN", p);
  double frac = 0.0;
  Result r = cartesian_translate(node, arm, up, p.lift_distance, p, "lift", &frac);
  if (!ok(r)) { return r; }
  mark(
    node, 3, "LIFT_DONE", p,
    p.marker_file_prefix.empty() ? "" : p.marker_file_prefix + ".liftdone_ready");

  // Grasp-loss check, before the dwell: no reason to spend the dwell window
  // (or the transport/place/descend legs after it) on a cycle that's
  // already unambiguously lost the object. See transport.hpp's header note
  // and check_grasp_not_lost() above.
  r = check_grasp_not_lost(node, p);
  if (!ok(r)) {
    // Release anyway. Without this the node would exit with the gripper
    // still actively commanded toward the close target (the hold-target
    // fix means that's a standing setpoint, not a one-shot command) --
    // driving real force into empty space with nothing to stop it, AND
    // leaving the sim's own post-cycle health gate
    // (gz_assert_gripper_responsive) unable to tell a genuinely degraded
    // sim from a healthy one still fighting an abandoned goal. Reuses the
    // same gripper_command callable and release_position_rad Stage 6 uses
    // below -- best-effort, its own outcome doesn't change what's
    // returned here. The detection already did its job before this runs;
    // releasing now costs nothing diagnostic.
    RCLCPP_INFO(
      node->get_logger(),
      "releasing to %.4f rad after grasp loss -- not leaving the gripper "
      "mid-hold, driving into empty space", p.release_position_rad);
    gripper_command(p.release_position_rad);
    return r;
  }

  // The marker fires FIRST, then we hold still. The harness starts its
  // windowed settle on the marker and needs the arm stationary for the whole
  // of that window.
  dwell(node, p.slip_sample_dwell_s, "slip baseline sample");

  // --- Stage 4: transport --------------------------------------------------
  // Free-space planned move to a pose one standoff above the place pose.
  // Deliberately not Cartesian; see the note at the top of this file.
  geometry_msgs::msg::Pose above_place = p.place_pose;
  {
    const double n = std::sqrt(
      up[0] * up[0] + up[1] * up[1] + up[2] * up[2]);
    if (n < 1e-9) {
      RCLCPP_ERROR(
        node->get_logger(),
        "CONFIG_ERROR: approach_axis has zero length; cannot offset the place "
        "pose by grasp.standoff.");
      return Result::CONFIG_ERROR;
    }
    above_place.position.x += up[0] / n * p.standoff;
    above_place.position.y += up[1] / n * p.standoff;
    above_place.position.z += up[2] / n * p.standoff;
  }

  mark(node, 4, "TRANSPORT_BEGIN", p);
  arm.setPoseTarget(above_place);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(
      node->get_logger(),
      "PLAN_FAILURE: no plan to the pose one standoff (%.3f m) above "
      "object.place_pose. The object is currently held; nothing has been "
      "released.", p.standoff);
    return Result::PLAN_FAILURE;
  }
  if (arm.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(
      node->get_logger(),
      "EXECUTE_FAILURE: transport plan existed but execution failed.");
    return Result::EXECUTE_FAILURE;
  }
  mark(
    node, 4, "TRANSPORT_DONE", p,
    p.marker_file_prefix.empty() ? "" : p.marker_file_prefix + ".transportdone_ready");
  dwell(node, p.slip_sample_dwell_s, "slip comparison sample");

  // --- Stage 5: descend to place ------------------------------------------
  // Cartesian again: the object is still held, and a bowed path here scrapes
  // it against the table on the way down.
  mark(node, 5, "PLACE_DESCEND_BEGIN", p);
  r = cartesian_translate(
    node, arm, p.approach_axis, p.standoff, p, "place descent", &frac);
  if (!ok(r)) { return r; }
  mark(node, 5, "PLACE_DESCEND_DONE", p);

  // --- Stage 6: release ----------------------------------------------------
  // release_position_rad comes from gripper_geometry.theta_for_width() of the
  // object width plus clearance. Opening to an arbitrary angle also moves the
  // pads 13.5 mm along the approach axis across the full range, so "just open
  // it" is a vertical motion as well as a lateral one.
  mark(node, 6, "RELEASE_BEGIN", p);
  r = gripper_command(p.release_position_rad);
  if (!ok(r)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "release to %.4f rad failed (%s). The object may still be in the "
      "fingers; the retreat below is NOT attempted.",
      p.release_position_rad, to_string(r));
    return r;
  }
  mark(node, 6, "RELEASE_DONE", p);

  // --- Stage 7: retreat ----------------------------------------------------
  // Straight up and away. Cartesian so the open fingers lift clear of the
  // object rather than sweeping through it.
  mark(node, 7, "RETREAT_BEGIN", p);
  r = cartesian_translate(node, arm, up, p.standoff, p, "retreat", &frac);
  if (!ok(r)) { return r; }
  mark(node, 7, "RETREAT_DONE", p);

  RCLCPP_INFO(
    node->get_logger(),
    "cycle %d complete: lift -> transport -> place -> release -> retreat. "
    "Whether the object ARRIVED is not decided here — that is Gazebo ground "
    "truth, measured by the harness.", p.cycle_index);

  return Result::SUCCESS;
}

}  // namespace ur5e_pick_place

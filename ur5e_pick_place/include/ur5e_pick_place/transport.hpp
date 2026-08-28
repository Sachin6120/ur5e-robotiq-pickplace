// transport.hpp — the legs M3's criterion names that m3_grasp does not have:
// lift, transport, place, retreat.
//
// WHY A SEPARATE TRANSLATION UNIT
//   m3_grasp.cpp is verified and produced this project's first successful
//   grasp. Editing a working file blind is how a regression gets introduced
//   into the one thing that works. This declares a function m3_grasp calls
//   after its Stage 2, and needs to know nothing about m3_grasp's internals.
//
//   It also means M4 and M5 reuse this rather than growing a second copy.
//
// WIRING
//   After Stage 2 reports a successful close, m3_grasp calls:
//
//     ur5e_pick_place::TransportParams tp;   // filled from ROS parameters
//     auto r = ur5e_pick_place::lift_transport_place(
//         node, move_group, tp,
//         [&](double rad) { return gripper_command(rad); });
//
//   The gripper is passed as a callable so this file does not depend on
//   m3_grasp's gripper helper signature. Adapt it in the lambda.
//
// WHAT THIS DELIBERATELY DOES NOT DO
//   It does not decide whether the grasp is holding, in the sense of ground
//   truth. attachObject and detachObject are MoveIt collision bookkeeping and
//   nothing else — a successful attach is not evidence of a successful grasp
//   and is never treated as such here. Whether the object actually came
//   along is decided afterwards, from Gazebo's own pose ground truth, by
//   scripts/lib/slip.py.
//
//   It also does not sample poses itself. It emits stage markers on stdout
//   with timestamps, and the harness samples Gazebo around them. Keeping the
//   measurement outside the node under test is the same separation M0 through
//   M2 used.
//
// ONE EXCEPTION: the grasp-loss check at the end of Stage 3 (lift)
//   Added 2026-08-11. Right after the lift completes, the actuated joint's
//   own position is sampled (ground truth, the same `gz topic -e` mechanism
//   m3_grasp.cpp's gripper_close_and_hold fallback already uses, not
//   /joint_states) and compared against
//   `expected_grip_angle + grasp_loss_threshold_rad`. If it's past that, the
//   object left the fingers during the lift under load — instrumenting a
//   real signature this project measured live, not a guess: the joint
//   sailing well past the touch angle while actuated-joint effort collapses
//   toward zero (nothing left to push against) is unambiguous. This is
//   still not a ground-truth slip verdict — it is a cheap, in-node early
//   abort for the specific case where the signal is already unambiguous
//   from the gripper's own state, so a doomed cycle doesn't burn the
//   remaining ~90s of transport/place/release/retreat and doesn't enter the
//   slip statistics needing a post-hoc ~350mm number to explain what
//   happened. A cycle that passes this check still gets the full
//   scripts/lib/slip.py treatment; this only ever narrows the SUCCESS-looking
//   population, it never claims one on its own.
//
//   On the abort path this also fires the gripper's release
//   (release_position_rad, the same target and callable Stage 6 uses) before
//   returning — otherwise the node exits with the joint still actively
//   commanded toward the close target (a standing setpoint now, not a
//   one-shot command, see gripper_close_and_hold's hold-at-target fix),
//   driving real force into empty space, and the caller's own post-cycle
//   health gate (gz_assert_gripper_responsive) cannot tell that apart from
//   genuine sim degradation. Best-effort: the release's own outcome does not
//   change the Result already decided above it.

#ifndef UR5E_PICK_PLACE__TRANSPORT_HPP_
#define UR5E_PICK_PLACE__TRANSPORT_HPP_

#if __has_include(<moveit/move_group_interface/move_group_interface.hpp>)
#include <moveit/move_group_interface/move_group_interface.hpp>
#else
#include <moveit/move_group_interface/move_group_interface.h>
#endif

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <functional>
#include <memory>
#include <string>

#include "ur5e_pick_place/failure.hpp"

namespace ur5e_pick_place
{

// Every field arrives from config/scene.yaml through the launch file. There
// are no defaults worth trusting here: a silently-defaulted place pose would
// put the object somewhere nobody wrote down.
struct TransportParams
{
  // Direction the gripper approached along, in the PLANNING frame, already
  // resolved from object_frame by the caller. Lift and retreat travel along
  // its negation.
  std::array<double, 3> approach_axis{{0.0, 0.0, -1.0}};

  // grasp.retreat — how far to lift after the close, metres.
  double lift_distance = 0.0;

  // grasp.standoff — height above the place pose to fly to before descending.
  double standoff = 0.0;

  // object.place_pose, already composed into a gripper pose by the caller,
  // in the planning frame.
  geometry_msgs::msg::Pose place_pose;

  // thresholds.cartesian_fraction_min. Below this the leg aborts with
  // CARTESIAN_FRACTION_LOW and a logged reason. It is not retried at a lower
  // bar — a partial Cartesian path is a different motion from the one that
  // was planned, and executing it silently is the failure mode this whole
  // project is built to avoid.
  double cartesian_fraction_min = 0.95;
  double eef_step = 0.005;
  double jump_threshold = 0.0;

  // Aperture to open to for release, RADIANS of the actuated joint. Computed
  // by the launch file from config/gripper_geometry.py — theta_for_width() of
  // the object width plus a clearance — never typed in by hand. The pads
  // travel 13.5 mm along the approach axis across the aperture range, so an
  // arbitrary "open" value moves the fingers vertically as well as apart.
  double release_position_rad = 0.0;

  double velocity_scaling = 0.1;
  double acceleration_scaling = 0.1;

  // Held stationary after LIFT_DONE and TRANSPORT_DONE, on the node's own
  // clock (not a wall sleep — sim real-time factor is not reliably 1.0 under
  // load). Gives the harness's windowed-quiescence sampler
  // (scripts/lib/sample_pose.py) an actual stationary window to sample a
  // settled slip-baseline/comparison pose in; without it the node moves on to
  // planning the transport almost immediately after LIFT_DONE fires.
  double slip_sample_dwell_s = 2.0;

  // Opt-in F3 boundary: complete Stage 3, its grasp-loss check, and the full
  // post-LIFT_DONE dwell, then return SUCCESS before TRANSPORT_BEGIN.
  // false preserves the full lift/transport/place/release/retreat lifecycle.
  bool lift_only = false;

  // Opt-in Stage 4 boundary: complete Stage 4 transport and its post-TRANSPORT_DONE dwell,
  // then return SUCCESS before PLACE_DESCEND_BEGIN.
  bool transport_only = false;

  // Emitted verbatim in the stage markers so a CSV row can be tied back to a
  // specific cycle of a specific sim instance after the fact.
  int cycle_index = 0;
  int sim_instance = 0;

  // Grasp-loss check (Stage 3, see the header note above). All four required
  // together if the check is to run at all — expected_grip_angle <= 0.0
  // disables it (see lift_transport_place), since 0.0 is not a valid
  // touch angle for any real object and this project does not want a
  // silently-defaulted threshold pretending to be a real one.
  double expected_grip_angle = 0.0;
  double grasp_loss_threshold_rad = 0.0;
  std::string actuated_joint;
  std::string gz_js_topic;

  // Empty (default) disables this entirely -- normal single-cycle usage never
  // sets it. Added 2026-08-12: scripts/11_m3_cycles.sh's watcher parses
  // stdout for "M3 STAGE 3 LIFT_DONE"/"M3 STAGE 4 TRANSPORT_DONE" to know
  // when to sample Gazebo's pose topic. Confirmed live that this can fail
  // silently under the sweep harness's nested bash -c / backgrounded-tail
  // process tree -- three consecutive swept cycles all reported
  // result=SUCCESS while the watcher's own live tail saw nothing past the
  // point m3_grasp's launch even started, despite the same lines appearing
  // correctly in the completed log file afterward and an isolated
  // reproduction of the same nesting pattern working fine. Root cause not
  // pinned down further; a filesystem write is immune to the entire class of
  // pipe/stdout-buffering problem this sits in, so that is the fix rather
  // than continuing to chase the exact interaction. When non-empty, LIFT_DONE
  // and TRANSPORT_DONE additionally touch "<prefix>.liftdone_ready" /
  // "<prefix>.transportdone_ready" -- the harness polls for file existence
  // instead of parsing a live stream.
  std::string marker_file_prefix;
};

// Runs lift -> transport -> place -> release -> retreat.
//
// Returns SUCCESS only if every leg succeeded. Any failure returns the typed
// cause, having already logged at ERROR with a named reason, and does not
// attempt the remaining legs — continuing past a failed lift would carry a
// dropped object's cycle into the slip statistics as if it were a grasp.
Result lift_transport_place(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & arm,
  const TransportParams & params,
  const std::function<Result(double)> & gripper_command);

}  // namespace ur5e_pick_place

#endif  // UR5E_PICK_PLACE__TRANSPORT_HPP_

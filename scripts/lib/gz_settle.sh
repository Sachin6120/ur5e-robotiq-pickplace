#!/usr/bin/env bash
# gz_settle.sh — bash wrappers around gz_settle.py. Source this, then call
# gz_settle_joint / gz_settle_pose to WAIT before taking a ground-truth
# sample, instead of a fixed `sleep N` guessed to be "probably long enough".
# See gz_settle.py's header for why fixed sleeps are a measurement-integrity
# bug, not a style preference.
#
# Both functions return nonzero (and gz_settle.py prints a [STOP] line to
# stderr) if the poll times out without settling. Treat that as a hard stop
# for the calling script's current sample — do not follow either call with
# `|| true` and sample anyway.

GZ_SETTLE_PY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/gz_settle.py"

# gz_settle_joint <gz_joint_state_topic> <vel_eps_rad_s> <timeout_s> <poll_dt_s> <joint_name> [joint_name ...]
gz_settle_joint() {
  local topic="$1" eps="$2" timeout="$3" poll="$4"; shift 4
  python3 "$GZ_SETTLE_PY" joint --topic "$topic" --eps "$eps" \
    --timeout "$timeout" --poll "$poll" "$@"
}

# gz_settle_pose <gz_pose_info_topic> <pos_eps_m> <timeout_s> <poll_dt_s> <link_name> [link_name ...]
#
# Consecutive-poll settling only, i.e. the historical default (window_s ==
# poll_dt) -- fine for non-contact motion (arm links, fingertip pose during a
# simple open/close), proven fast and accurate for those. NOT safe for an
# object settling against contact/compliance after a grasp -- use
# gz_settle_pose_windowed for that. See gz_settle.py's settle_poses docstring.
gz_settle_pose() {
  local topic="$1" eps="$2" timeout="$3" poll="$4"; shift 4
  python3 "$GZ_SETTLE_PY" pose --topic "$topic" --eps "$eps" \
    --timeout "$timeout" --poll "$poll" "$@"
}

# gz_settle_pose_windowed <gz_pose_info_topic> <pos_eps_m> <timeout_s> <poll_dt_s> <window_s> <link_name> [link_name ...]
#
# Same as gz_settle_pose, but compares each sample against one taken
# <window_s> seconds earlier instead of just the previous poll. Required for
# any object settling against gripper contact/compliance: that process creeps
# slowly enough (confirmed live 2026-08-08, docs/HANDOFF_M3.md -- a box
# settling after gripper_close_and_hold was declared "settled" by the
# consecutive-poll check at 0.58s, then measured still sinking ~9.4mm over
# the next 30s and not leveling off until ~120s) that no number of
# consecutive short-interval polls can tell it apart from genuine rest --
# each individual poll-to-poll delta is small even while the object is still
# very much in motion on a longer timescale. <timeout_s> must comfortably
# exceed <window_s> (it needs at least one full window of history before it
# can declare anything settled at all).
gz_settle_pose_windowed() {
  local topic="$1" eps="$2" timeout="$3" poll="$4" window="$5"; shift 5
  python3 "$GZ_SETTLE_PY" pose --topic "$topic" --eps "$eps" \
    --timeout "$timeout" --poll "$poll" --window-s "$window" "$@"
}

# gz_assert_joint <gz_joint_state_topic> <joint_name> <expected_rad> <tol_rad> [label]
#
# PRECONDITION assertion, not a settle wait: reads the joint's CURRENT
# position once and fails loudly if it isn't where the caller assumed. Every
# probe script in this project stated "gripper OPEN" as a precondition in a
# comment and never checked it -- it was false on every fresh sim launch this
# session (confirmed 5/5, ~0.767rad not ~0rad; see docs/HANDOFF_M3.md). Call
# this before any script proceeds on an assumed starting joint position.
gz_assert_joint() {
  local topic="$1" name="$2" expected="$3" tol="$4" label="${5:-}"
  python3 "$GZ_SETTLE_PY" assert-joint --topic "$topic" \
    --expected "$expected" --tol "$tol" ${label:+--label "$label"} "$name"
}

# kill_sim [launch_pattern]
#
# Kill every process belonging to a sim launch, not just the `ros2 launch`
# parent. `ros2 launch` spawns robot_state_publisher, every controller
# spawner, and parameter_bridge as independent child processes (launch_ros
# `Node` actions) that do not reliably die when the parent is pkill'd from
# outside the launch framework's own shutdown path. Moved here (originally
# local to scripts/07_check_gripper_spawn_state.sh) because this is a
# property of `ros2 launch` itself, not of that one script -- any script that
# comes to own sim lifecycle will hit the same leak. Confirmed directly: a
# first version that only targeted the launch.py process and `gz sim` left
# 18 orphaned parameter_bridge processes running across one session (one per
# prior launch, each still bridging /clock), which reproduced a 15+ minute
# launch hang. See docs/HANDOFF_M3.md, "orphaned processes".
kill_sim() {
  local pattern="${1:-ur5e_robotiq_sim_control.launch.py}"
  pkill -9 -f "$pattern" 2>/dev/null
  pkill -9 -f "gz sim" 2>/dev/null
  pkill -9 -f "robot_state_publisher" 2>/dev/null
  pkill -9 -f "parameter_bridge" 2>/dev/null
  pkill -9 -f "controller_manager/spawner" 2>/dev/null
  sleep 2
  local leftover
  leftover=$(ps -eo pid,cmd | grep -E "gz sim|robot_state_publisher|parameter_bridge|controller_manager|spawner" | grep -v grep)
  if [[ -n "$leftover" ]]; then
    printf '  [WARN] kill_sim left processes running -- killing by PID directly:\n%s\n' "$leftover"
    printf '%s\n' "$leftover" | awk '{print $1}' | xargs -r kill -9 2>/dev/null
    sleep 1
  fi
}

# gz_assert_clean_slate
#
# STANDARD PREAMBLE, run before any script launches a sim it intends to
# treat as fresh. Counts stray parameter_bridge / robot_state_publisher /
# gz sim / spawner processes and ABORTS BY NAME if any exist, rather than
# launching on top of them and calling the result "fresh". This exists
# because "no prior sim instance running" was previously asserted by a
# check that only looked for the launch.py parent and gz sim itself --
# exactly the check that missed the orphan leak kill_sim() documents above.
# A script that wants to clean up and continue should call kill_sim first,
# then this, not skip straight past it.
gz_assert_clean_slate() {
  local leftover
  leftover=$(ps -eo pid,cmd | grep -E "gz sim|robot_state_publisher|parameter_bridge|controller_manager|spawner" | grep -v grep)
  if [[ -n "$leftover" ]]; then
    printf '  [STOP] gz_assert_clean_slate: stray process(es) present -- refusing to call this launch fresh:\n%s\n' "$leftover" >&2
    return 1
  fi
  return 0
}

# gz_wait_controller_active_bounded <controller_name> [bound_s]
#
# Poll `ros2 control list_controllers` for <controller_name> to report
# active, [STOP] if it takes longer than <bound_s> (default 20s). This is a
# diagnostic assertion, not a settle wait: controller-activation time is a
# known-good health signal for this stack -- 6.8-13.1s is the confirmed
# healthy range on a clean-slate launch, 40+s was observed once orphaned
# processes had accumulated (see docs/HANDOFF_M3.md, "orphaned processes").
# A slow activation is evidence the system under test is contaminated, not
# noise to average past -- treat it the same as any other measurement
# precondition failure: record it, do not retry silently, investigate before
# trusting anything sampled from that launch.
#
# Sets GZ_LAST_WAIT_S to the elapsed seconds (as a string) on every return,
# success or [STOP] -- callers that log a wait time per launch (e.g.
# scripts/07_check_gripper_spawn_state.sh's results table) read that instead
# of re-timing the call themselves.
gz_wait_controller_active_bounded() {
  local name="$1" bound="${2:-20}"
  local t0 t1
  t0=$(date +%s.%N)
  while ! ros2 control list_controllers 2>/dev/null | grep -qE "^${name}\b.*\bactive\b"; do
    t1=$(date +%s.%N)
    GZ_LAST_WAIT_S=$(python3 -c "print(f'{${t1} - ${t0}:.2f}')")
    if python3 -c "import sys; sys.exit(0 if float('$GZ_LAST_WAIT_S') > float('$bound') else 1)"; then
      printf '  [STOP] gz_wait_controller_active_bounded: %s not active after %ss (bound=%ss) -- system health suspect, not just slow\n' \
        "$name" "$GZ_LAST_WAIT_S" "$bound" >&2
      return 1
    fi
    sleep 0.2
  done
  t1=$(date +%s.%N)
  GZ_LAST_WAIT_S=$(python3 -c "print(f'{${t1} - ${t0}:.2f}')")
  printf '  controllers active after %ss\n' "$GZ_LAST_WAIT_S"
  return 0
}

# gripper_close_and_hold <gripper_ctrl> <master_joint> <target_pos> <max_effort> <cmd_timeout_s> <gz_joint_state_topic> <outfile>
#
# Sends a gripper_cmd goal toward <target_pos>, bounded by <cmd_timeout_s>,
# then IMMEDIATELY re-commands the gripper to hold at wherever it actually
# ended up, instead of leaving <target_pos> as the still-active background
# command. Exists because `GripperActionController::check_for_success`
# (ros2_controllers, gripper_action_controller_impl.hpp) calls
# `setSucceeded()` on both its reached-goal and stalled branches but never
# calls `set_hold_position()` on either — the position command interface
# keeps being written the ORIGINAL target every control cycle after the
# action reports done. For a stalled grasp (achieved position != target)
# that means sustained force continues toward full closure after "success",
# and confirmed live 2026-08-06 (docs/HANDOFF_M3.md, "06 retried post
# spawn-state fixes") this eventually ejects the object the action just
# reported successfully grasping — minutes later, no intervening commands.
#
# Also handles the case where the CLIENT itself is killed by the
# <cmd_timeout_s> bound before any Result arrives: `timeout` kills the
# `ros2 action send_goal` process, it does NOT cancel the goal on the
# action server, so the gripper may still be actively driving toward
# <target_pos> at that moment too. Falls back to a live ground-truth sample
# in that case rather than trusting nothing.
#
# Sets on return:
#   GRIPPER_HOLD_RESULT   — one of:
#     REACHED_GOAL      goal completed at (or within tolerance of) target
#     STALLED           goal completed short of target (stalled: true)
#     TIMED_OUT_HELD     no Result arrived within <cmd_timeout_s>; held at a
#                        live-sampled ground-truth position instead
#     UNKNOWN_NO_SAMPLE  no Result AND ground truth could not be sampled —
#                        caller must treat this as a hard stop, not a value
#   GRIPPER_HOLD_POSITION — the position now being held (radians, as text;
#                           empty on UNKNOWN_NO_SAMPLE)
#   GRIPPER_HOLD_ELAPSED_S — wall-clock seconds from goal send to the first
#                           Result (or to the <cmd_timeout_s> kill on
#                           TIMED_OUT_HELD). Exists because the whole point of
#                           the stall_velocity_threshold/stall_timeout tuning
#                           (docs/HANDOFF_M3.md) is how long stalled:true
#                           takes to arrive — that number was previously only
#                           ever eyeballed from log timestamps after the fact.
#
# Returns 0 except on UNKNOWN_NO_SAMPLE, where it returns 1 and does NOT
# send a hold command (nothing known to hold at).
#
# This does not fix the underlying report-vs-reality gap (that lives in
# ros2_controllers, not this repo) and it does not decide whether a
# TIMEOUT/STALL counts as a valid measurement — callers keep making that
# call themselves, per this project's no-auto-retry rule. It only stops the
# gripper from continuing to squeeze after the call is done deciding.
gripper_close_and_hold() {
  local gripper_ctrl="$1" master="$2" target="$3" max_effort="$4" \
        cmd_timeout="$5" gz_js_topic="$6" outfile="$7"

  local __t0 __t1
  __t0=$(date +%s.%N)
  timeout "$cmd_timeout" ros2 action send_goal \
    "/${gripper_ctrl}/gripper_cmd" control_msgs/action/GripperCommand \
    "{command: {position: ${target}, max_effort: ${max_effort}}}" \
    > "$outfile" 2>&1
  __t1=$(date +%s.%N)
  GRIPPER_HOLD_ELAPSED_S=$(python3 -c "print(f'{${__t1} - ${__t0}:.3f}')")

  local parsed achieved="" stalled=""
  parsed=$(python3 - "$outfile" <<'PY'
import sys, re
txt = open(sys.argv[1]).read()
# Anchored to the literal Result: ... Goal finished markers ros2 action
# send_goal prints -- NOT a bare "position:" grep, which would also match
# the echoed "Sending goal: command: position: <target>" line above it.
m = re.search(r'Result:\s*\n(.*?)\n\s*\nGoal finished', txt, re.S)
if m:
    block = m.group(1)
    pos = re.search(r'position:\s*(-?[\d.eE+-]+)', block)
    stalled = re.search(r'stalled:\s*(\w+)', block)
    print(f"{pos.group(1) if pos else ''}\t{stalled.group(1) if stalled else ''}")
PY
)
  if [[ -n "$parsed" ]]; then
    IFS=$'\t' read -r achieved stalled <<<"$parsed"
  fi

  if [[ -z "$achieved" ]]; then
    achieved=$(python3 - "$gz_js_topic" "$master" <<'PY'
import sys, subprocess, re
topic, master = sys.argv[1], sys.argv[2]
txt = subprocess.run(["gz", "topic", "-e", "-t", topic, "-n", "1"],
                      capture_output=True, text=True, timeout=5).stdout
for blk in re.findall(r'joint\s*\{(.*?)\n\}', txt, re.S):
    n = re.search(r'name:\s*"([^"]+)"', blk)
    p = re.search(r'position:\s*(-?[\d.eE+-]+)', blk)
    if n and n.group(1) == master and p:
        print(p.group(1))
        break
PY
)
    if [[ -z "$achieved" ]]; then
      GRIPPER_HOLD_RESULT="UNKNOWN_NO_SAMPLE"
      GRIPPER_HOLD_POSITION=""
      return 1
    fi
    GRIPPER_HOLD_RESULT="TIMED_OUT_HELD"
  elif [[ "${stalled,,}" == "true" ]]; then
    GRIPPER_HOLD_RESULT="STALLED"
  else
    GRIPPER_HOLD_RESULT="REACHED_GOAL"
  fi
  GRIPPER_HOLD_POSITION="$achieved"

  timeout "$cmd_timeout" ros2 action send_goal \
    "/${gripper_ctrl}/gripper_cmd" control_msgs/action/GripperCommand \
    "{command: {position: ${achieved}, max_effort: ${max_effort}}}" \
    >> "$outfile" 2>&1
  return 0
}

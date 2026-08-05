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
gz_settle_pose() {
  local topic="$1" eps="$2" timeout="$3" poll="$4"; shift 4
  python3 "$GZ_SETTLE_PY" pose --topic "$topic" --eps "$eps" \
    --timeout "$timeout" --poll "$poll" "$@"
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

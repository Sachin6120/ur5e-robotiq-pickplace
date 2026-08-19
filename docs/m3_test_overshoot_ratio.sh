#!/usr/bin/env bash
# m3_test_overshoot_ratio.sh — falsification test for the "mimic accumulates
# into the master joint's command, effective target ~= N x commanded" theory
# (N=5, the mimic-follower count) built 2026-08-10 to explain the pre-close
# overshoot found in docs/m3_trace_goal_traffic.py (target 0.1055, observed
# peak 0.521 and 0.5615 across two trials -- ratios 4.94x and 5.32x).
#
# Prediction under the hypothesis: peak/target stays near 5 regardless of
# target, until clamped by the joint's own upper=0.8 limit.
#   0.05 -> predict peak ~0.25
#   0.15 -> predict peak ~0.75 (just under the limit)
#   0.20 -> predict clamped at 0.8, no visible reversal
# If peak/target does NOT stay roughly constant across the first two, the
# hypothesis is dead -- that's an equally useful outcome, not a failure of
# the test.
#
# Gripper-only: no arm motion, no object, no move_group needed. Reset to 0
# and settle (gz_settle_joint, velocity-based) before each target so every
# trial starts from genuine rest, not mid-motion from the previous one.
cd "$(dirname "$0")/.." || exit 2

source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
source scripts/lib/gz_settle.sh
set -u

TS="$(date +%Y%m%d_%H%M%S)"
LOGDIR="docs"
SIM_LOG="${LOGDIR}/m3_ratio_sim_${TS}.log"
TRACE_OUT="${LOGDIR}/m3_ratio_trace_${TS}.txt"
MARKERS="${LOGDIR}/m3_ratio_markers_${TS}.txt"
GZ_JS="/world/empty/model/ur5e_robotiq/joint_state"
JOINT="robotiq_85_left_knuckle_joint"

hr()  { printf '\n%s\n' "════════════════════════════════════════════════════════════"; }
sec() { hr; printf '§ %s\n' "$1"; hr; }
die() { printf '  [STOP] %s\n' "$1"; exit 2; }

# ---------------------------------------------------------------------------
sec "0. Clean slate"
kill_sim
pkill -9 -f "move_group" 2>/dev/null
sleep 1
if ! gz_assert_clean_slate; then
  die "refusing to start on a contaminated system"
fi

# ---------------------------------------------------------------------------
sec "1. Launch sim only -- no move_group, no object, gripper commands don't need them"
nohup ros2 launch ur5e_robotiq_description ur5e_robotiq_sim_control.launch.py \
  > "$SIM_LOG" 2>&1 &
if ! gz_wait_controller_active_bounded "gripper_controller" 30; then
  die "gripper_controller never went active -- see $SIM_LOG"
fi

# ---------------------------------------------------------------------------
sec "2. Gate BEFORE"
if ! gz_assert_gripper_responsive; then
  die "sim degraded before the test even started"
fi

# ---------------------------------------------------------------------------
sec "3. Start tracer, run three targets, each preceded by a settled reset to 0"
TRACE_DURATION_S=60
python3 docs/m3_trace_goal_traffic.py "$TRACE_OUT" "$TRACE_DURATION_S" \
  gripper_controller "$GZ_JS" &
TRACER_PID=$!
sleep 1

: > "$MARKERS"
for TARGET in 0.05 0.15 0.20; do
  printf '  reset to 0, settle...\n'
  ros2 action send_goal "/gripper_controller/gripper_cmd" control_msgs/action/GripperCommand \
    "{command: {position: 0.0, max_effort: 50.0}}" > /dev/null 2>&1
  gz_settle_joint "$GZ_JS" 0.01 5.0 0.1 "$JOINT" || printf '  (settle timed out, proceeding anyway -- recorded)\n'

  T_SEND=$(date +%s.%N)
  printf 'TARGET %s t_send=%s\n' "$TARGET" "$T_SEND" | tee -a "$MARKERS"
  timeout 10 ros2 action send_goal "/gripper_controller/gripper_cmd" control_msgs/action/GripperCommand \
    "{command: {position: ${TARGET}, max_effort: 50.0}}" 2>&1 | sed 's/^/    /'
  T_DONE=$(date +%s.%N)
  printf 'TARGET %s t_done=%s\n' "$TARGET" "$T_DONE" | tee -a "$MARKERS"
  sleep 1.5   # clean separation in the trace before the next target
done

sleep 2
kill "$TRACER_PID" 2>/dev/null
wait "$TRACER_PID" 2>/dev/null
printf '  trace=%s markers=%s\n' "$TRACE_OUT" "$MARKERS"

# ---------------------------------------------------------------------------
sec "4. Gate AFTER"
if gz_assert_gripper_responsive; then
  printf '  GATE_AFTER=OK\n'
else
  printf '  GATE_AFTER=FAIL -- treat peak numbers as suspect, but the RATIO comparison across three targets in the same short window is far less exposed to slow degradation than any single absolute number\n'
fi

# ---------------------------------------------------------------------------
sec "5. Cleanup"
kill_sim
pkill -9 -f "move_group" 2>/dev/null

printf '\ndone: %s\n' "$(date -Is)"

#!/usr/bin/env bash
# m3_run_context_overshoot_test_v2.sh — same context-dependence test as
# m3_run_context_overshoot_test.sh, but closing the confound flagged in that
# run's own result: no move_group was running, and the arm motion was a raw
# FollowJointTrajectory goal, not a MoveIt-planned one. Both live overshoots
# happened with move_group active, planning and executing a real Cartesian
# approach -- genuine extra load the first context test didn't reproduce.
#
# This version reuses `m2_cartesian_approach` (the actual closed-milestone
# node m3_grasp.cpp's own Stage 1/2 pattern is copied from) as the arm-
# motion trigger, under a real move_group, and fires the gripper command the
# instant M2's own log reports "execution reported SUCCESS" -- polling the
# log rather than waiting for `ros2 launch` to return, because that launch
# never returns on its own (static_scene_tf is persistent, same issue as
# m3_grasp's own launch, confirmed earlier tonight).
cd "$(dirname "$0")/.." || exit 2

source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
source scripts/lib/gz_settle.sh
set -u

TS="$(date +%Y%m%d_%H%M%S)"
LOGDIR="docs"
SIM_LOG="${LOGDIR}/m3_ctx2_sim_${TS}.log"
MG_LOG="${LOGDIR}/m3_ctx2_movegroup_${TS}.log"
M2_LOG="${LOGDIR}/m3_ctx2_m2_${TS}.log"
TRACE_OUT="${LOGDIR}/m3_ctx2_trace_${TS}.txt"
MARKERS="${LOGDIR}/m3_ctx2_markers_${TS}.txt"
M2_CSV="m2_approach_ctx2_${TS}.csv"

hr()  { printf '\n%s\n' "════════════════════════════════════════════════════════════"; }
sec() { hr; printf '§ %s\n' "$1"; hr; }
die() { printf '  [STOP] %s\n' "$1"; exit 2; }

# ---------------------------------------------------------------------------
sec "0. Clean slate"
kill_sim
pkill -9 -f "move_group" 2>/dev/null
pkill -9 -f "m2_cartesian_approach" 2>/dev/null
sleep 1
if ! gz_assert_clean_slate; then
  die "refusing to start on a contaminated system"
fi

# ---------------------------------------------------------------------------
sec "1. Launch sim"
nohup ros2 launch ur5e_robotiq_description ur5e_robotiq_sim_control.launch.py \
  > "$SIM_LOG" 2>&1 &
if ! gz_wait_controller_active_bounded "gripper_controller" 30; then
  die "gripper_controller never went active -- see $SIM_LOG"
fi
if ! gz_wait_controller_active_bounded "arm_controller" 10; then
  die "arm_controller never went active -- see $SIM_LOG"
fi

# ---------------------------------------------------------------------------
sec "2. Launch move_group"
nohup ros2 launch ur5e_robotiq_moveit_config move_group.launch.py \
  > "$MG_LOG" 2>&1 &
MG_T0=$(date +%s.%N)
MG_ELAPSED=0
until ros2 node list 2>/dev/null | grep -q '^/move_group$'; do
  MG_ELAPSED=$(python3 -c "import time; print(f'{time.time()-${MG_T0}:.1f}')")
  if python3 -c "import sys; sys.exit(0 if float('$MG_ELAPSED') > 40 else 1)"; then
    die "move_group node never appeared after 40s -- see $MG_LOG"
  fi
  sleep 0.5
done
sleep 3
printf '  move_group node present after %ss (+3s buffer taken)\n' "$MG_ELAPSED"

# ---------------------------------------------------------------------------
sec "3. Gate BEFORE"
if ! gz_assert_gripper_responsive; then
  die "sim degraded before the test even started"
fi

# ---------------------------------------------------------------------------
sec "4. Start tracer"
TRACE_DURATION_S=90
python3 docs/m3_trace_goal_traffic.py "$TRACE_OUT" "$TRACE_DURATION_S" \
  gripper_controller "/world/empty/model/ur5e_robotiq/joint_state" &
TRACER_PID=$!
sleep 1
: > "$MARKERS"

# ---------------------------------------------------------------------------
sec "5. CONTROL: isolated gripper close, nothing else moving"
python3 docs/m3_send_gripper_once.py 0.1055 CONTROL "$MARKERS" --reset-first

# ---------------------------------------------------------------------------
sec "6. Reset gripper to 0 and settle before the arm motion"
python3 docs/m3_send_gripper_once.py 0.0 RESET_BEFORE_M2 "$MARKERS"
gz_settle_joint "/world/empty/model/ur5e_robotiq/joint_state" 0.01 5.0 0.1 \
  robotiq_85_left_knuckle_joint || true

# ---------------------------------------------------------------------------
sec "7. Launch m2_cartesian_approach (real MoveIt plan+execute), poll its log"
: > "$M2_LOG"
nohup ros2 launch ur5e_pick_place m2_cartesian_approach.launch.py \
  csv_path:="$M2_CSV" > "$M2_LOG" 2>&1 &
M2_LAUNCH_PID=$!

echo "M2_LAUNCH_send t=$(date +%s.%N)" | tee -a "$MARKERS"
M2_T0=$(date +%s.%N)
until grep -q "execution reported SUCCESS" "$M2_LOG" 2>/dev/null; do
  M2_ELAPSED=$(python3 -c "import time; print(f'{time.time()-${M2_T0}:.1f}')")
  if python3 -c "import sys; sys.exit(0 if float('$M2_ELAPSED') > 60 else 1)"; then
    die "m2_cartesian_approach did not report execution SUCCESS within 60s -- see $M2_LOG"
  fi
  sleep 0.02
done
echo "M2_EXECUTE_SUCCESS_detected t=$(date +%s.%N)" | tee -a "$MARKERS"

# ---------------------------------------------------------------------------
sec "8. TEST: gripper close fired the instant M2's arm motion finished"
python3 docs/m3_send_gripper_once.py 0.1055 TEST "$MARKERS"

# ---------------------------------------------------------------------------
sec "9. Cleanup M2's launch (static_scene_tf never exits on its own)"
kill -9 "$M2_LAUNCH_PID" 2>/dev/null
pkill -9 -f "m2_cartesian_approach" 2>/dev/null
pkill -9 -f "static_scene_tf" 2>/dev/null

# ---------------------------------------------------------------------------
sec "10. Let the tracer finish on its own"
wait "$TRACER_PID" 2>/dev/null
printf '  trace=%s markers=%s\n' "$TRACE_OUT" "$MARKERS"

# ---------------------------------------------------------------------------
sec "11. Gate AFTER"
if gz_assert_gripper_responsive; then
  printf '  GATE_AFTER=OK\n'
else
  printf '  GATE_AFTER=FAIL -- flag it, do not discard the trace\n'
fi

# ---------------------------------------------------------------------------
sec "12. Cleanup"
kill_sim
pkill -9 -f "move_group" 2>/dev/null

printf '\ndone: %s\n' "$(date -Is)"
printf 'artifacts:\n  sim_log=%s\n  movegroup_log=%s\n  m2_log=%s\n  trace=%s\n  markers=%s\n' \
  "$SIM_LOG" "$MG_LOG" "$M2_LOG" "$TRACE_OUT" "$MARKERS"

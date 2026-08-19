#!/usr/bin/env bash
# m3_run_yaw_check_trial.sh — bracketed trial to test the rotated-object
# hypothesis for the repeatable ~0.13rad stall (0.1301, 0.1309 across two
# trials -- too tight to be scatter, reads as a hard geometric stop at the
# wrong width: expected contact for a 45mm object is 0.4055rad, and the
# local slope from the 40mm->0.458rad datapoint puts an obstruction in the
# 65-70mm range. A 45mm cube yawed 45 deg about vertical presents 63.6mm --
# closest candidate, and this project's own Gazebo screenshots already
# showed fingertip contact displacing the cube during descent, which is a
# real mechanism for exactly this rotation.
#
# docs/m3_capture_object_yaw_at_final_close.py watches the gripper_cmd
# action status and, on the 3rd EXECUTING goal (confirmed against
# m3_grasp.cpp's gripper_close_and_hold() to be the MAIN close -- the first
# two are pre-close's close-then-hold pair), immediately queries
# pick_target's ground-truth pose and converts to yaw. That is "the moment
# before the final close fires."
#
# Lighter than m3_run_streamed_hold_gap_trial.sh: this run doesn't need the
# joint-cmd streamer, goal-traffic tracer, or stall monitor again -- already
# have that evidence from the previous trial. Just the yaw watcher.
cd "$(dirname "$0")/.." || exit 2

source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
source scripts/lib/gz_settle.sh
set -u

TS="$(date +%Y%m%d_%H%M%S)"
LOGDIR="docs"
SIM_LOG="${LOGDIR}/m3_yaw_sim_${TS}.log"
MG_LOG="${LOGDIR}/m3_yaw_movegroup_${TS}.log"
GRASP_LOG="${LOGDIR}/m3_yaw_grasp_${TS}.log"
YAW_LOG="${LOGDIR}/m3_yaw_watcher_${TS}.log"
CSV_PATH="m3_grasp_trace_yawcheck_${TS}.csv"

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
sec "1. Launch sim (ur5e_robotiq_sim_control.launch.py)"
nohup ros2 launch ur5e_robotiq_description ur5e_robotiq_sim_control.launch.py \
  > "$SIM_LOG" 2>&1 &
SIM_PID=$!
printf '  sim launch PID=%s, log=%s\n' "$SIM_PID" "$SIM_LOG"

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
MG_PID=$!
printf '  move_group PID=%s, log=%s\n' "$MG_PID" "$MG_LOG"

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
sec "3. Spawn pick object"
if ! bash scripts/08_spawn_pick_object.sh; then
  die "object spawn/settle failed -- see output above"
fi

# ---------------------------------------------------------------------------
sec "4. Gate BEFORE (gz_assert_gripper_responsive)"
if ! gz_assert_gripper_responsive; then
  die "sim is degraded before the trial even started -- restart and retry, do not proceed"
fi

# ---------------------------------------------------------------------------
sec "5. Start the final-close yaw watcher"
python3 docs/m3_capture_object_yaw_at_final_close.py \
  gripper_controller empty pick_target > "$YAW_LOG" 2>&1 &
WATCHER_PID=$!
printf '  yaw watcher PID=%s, log=%s\n' "$WATCHER_PID" "$YAW_LOG"
sleep 1

# ---------------------------------------------------------------------------
sec "6. Run m3_grasp (single trial, expected to exit after one cycle)"
timeout 90 ros2 launch ur5e_pick_place m3_grasp.launch.py csv_path:="$CSV_PATH" \
  > "$GRASP_LOG" 2>&1
RC=$?
if [[ $RC -eq 124 ]]; then
  printf '  m3_grasp launch hit the 90s bound, killing it now\n'
  pkill -9 -f "m3_grasp.launch.py" 2>/dev/null
else
  printf '  m3_grasp launch returned (rc=%s) before the 90s bound\n' "$RC"
fi

# ---------------------------------------------------------------------------
sec "7. Collect the watcher"
sleep 2
kill -TERM "$WATCHER_PID" 2>/dev/null
wait "$WATCHER_PID" 2>/dev/null
echo "--- $YAW_LOG ---"
cat "$YAW_LOG"

# ---------------------------------------------------------------------------
sec "8. Gate AFTER (gz_assert_gripper_responsive)"
if gz_assert_gripper_responsive; then
  printf '  GATE_AFTER=OK\n'
else
  printf '  GATE_AFTER=FAIL -- sim degraded during/after the trial\n'
fi

# ---------------------------------------------------------------------------
sec "9. Cleanup"
kill_sim
pkill -9 -f "move_group" 2>/dev/null

printf '\ndone: %s\n' "$(date -Is)"
printf 'artifacts:\n  sim_log=%s\n  movegroup_log=%s\n  grasp_log=%s\n  yaw_log=%s\n  grasp_csv=%s\n' \
  "$SIM_LOG" "$MG_LOG" "$GRASP_LOG" "$YAW_LOG" "$CSV_PATH"

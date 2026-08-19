#!/usr/bin/env bash
# m3_run_gui_full_cycle_trial.sh — the visual check on cycle 3's finding: the
# 3-cycle harness measured pick_target sitting byte-identical at its spawn
# pose between LIFT_DONE and TRANSPORT_DONE while wrist_3_link lifted and
# transported normally -- the grasp reported SUCCESS at essentially the same
# achieved_grip_angle as the two passing cycles, but the object was never
# actually carried. The pose data says WHAT happened; this is for WHY.
#
# Four screenshots via docs/m3_capture_full_cycle_screenshots.py:
#   BEFORE_CLOSE   -- fingers about to close, pre-contact
#   CLOSE_RESULT   -- right when the close goal reaches ANY terminal status
#   LIFT_DONE      -- during transport.cpp's post-lift dwell (arm stationary)
#   TRANSPORT_DONE -- during the post-transport dwell
#
# GUI required (gazebo_gui:=true) so /gui/screenshot has something to
# capture -- confirmed live earlier this session: the service exists, returns
# a real non-blank PNG, and /gui/move_to frames the camera on a named entity.
#
# Grasp log is teed live, same fix as m3_run_full_cycle_trial_live.sh and for
# the same reason: the watcher's log-tail thread needs to see "M3 STAGE 3
# LIFT_DONE" while transport.cpp's dwell window is still open, not after the
# whole cycle (plus cleanup) has already finished.
#
# NOTE: this is one run, not a repeat of the 3-cycle sweep. The ejection in
# cycle 3 was 1 of 3, not deterministic -- this run may show a clean grasp
# instead, which is itself informative (rules out "always broken this way"),
# or it may reproduce the ejection, which is what it's here to catch.
cd "$(dirname "$0")/.." || exit 2

source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
source scripts/lib/gz_settle.sh
set -u

TS="$(date +%Y%m%d_%H%M%S)"
LOGDIR="docs"
SIM_LOG="${LOGDIR}/m3_guicycle_sim_${TS}.log"
MG_LOG="${LOGDIR}/m3_guicycle_movegroup_${TS}.log"
GRASP_LOG="${LOGDIR}/m3_guicycle_grasp_${TS}.log"
WATCHER_LOG="${LOGDIR}/m3_guicycle_watcher_${TS}.log"
SHOT_DIR="${LOGDIR}/m3_guicycle_shots_${TS}"
CSV_PATH="m3_grasp_trace_guicycle_${TS}.csv"

mkdir -p "$SHOT_DIR"

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
sec "1. Launch sim WITH GUI"
nohup ros2 launch ur5e_robotiq_description ur5e_robotiq_sim_control.launch.py \
  gazebo_gui:=true > "$SIM_LOG" 2>&1 &
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
sec "5. Start the full-cycle screenshot watcher (frames camera on pick_target now)"
: > "$GRASP_LOG"   # created empty FIRST so the watcher's log-tail thread has
                   # something to open immediately, no race on file existence
python3 docs/m3_capture_full_cycle_screenshots.py \
  gripper_controller pick_target "$GRASP_LOG" "$SHOT_DIR" > "$WATCHER_LOG" 2>&1 &
WATCHER_PID=$!
printf '  watcher PID=%s, log=%s, shots=%s\n' "$WATCHER_PID" "$WATCHER_LOG" "$SHOT_DIR"
sleep 2   # let move_to actually reposition before anything interesting happens

# ---------------------------------------------------------------------------
sec "6. Run m3_grasp (streamed live -- see file header for why this matters)"
timeout 120 ros2 launch ur5e_pick_place m3_grasp.launch.py csv_path:="$CSV_PATH" \
  2>&1 | tee -a "$GRASP_LOG"
RC="${PIPESTATUS[0]}"
if [[ $RC -eq 124 ]]; then
  printf '  m3_grasp launch hit the 120s bound, killing it now\n'
  pkill -9 -f "m3_grasp.launch.py" 2>/dev/null
else
  printf '  m3_grasp launch returned (rc=%s) before the 120s bound\n' "$RC"
fi

# ---------------------------------------------------------------------------
sec "7. Collect the watcher"
sleep 2
kill -TERM "$WATCHER_PID" 2>/dev/null
wait "$WATCHER_PID" 2>/dev/null
echo "--- $WATCHER_LOG ---"
cat "$WATCHER_LOG"
echo "--- shots ---"
ls -la "$SHOT_DIR"

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
printf 'artifacts:\n  sim_log=%s\n  movegroup_log=%s\n  grasp_log=%s\n  watcher_log=%s\n  shots=%s\n  grasp_csv=%s\n' \
  "$SIM_LOG" "$MG_LOG" "$GRASP_LOG" "$WATCHER_LOG" "$SHOT_DIR" "$CSV_PATH"

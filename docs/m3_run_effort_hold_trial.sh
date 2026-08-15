#!/usr/bin/env bash
# m3_run_effort_hold_trial.sh -- discriminator run for the "was the grasp
# actually squeezing" question raised by the GUI cycle
# (m3_guicycle_grasp_20260811_051613.log: achieved=0.4031 vs touch=0.4029,
# ~6 microns of interference per side, within_tolerance=yes anyway because
# the +/-0.0235rad band spans +/-1.3mm of aperture -- it cannot distinguish
# a rest from a squeeze).
#
# achieved_grip_angle alone can't answer this. Actuated-joint effort during
# the post-close hold can: near zero means the fingers were never pressing
# (gap, not squeeze); substantial and sustained means they were (squeeze,
# and friction is then a live question). Same headless single-cycle
# structure as m3_run_full_cycle_trial_live.sh, with one addition:
# m3_capture_joint_effort.py runs the whole time, logging
# robotiq_85_left_knuckle_joint's position/velocity/effort from
# /joint_states so the CSV covers close, hold, lift, and transport in one
# continuous trace -- started before m3_grasp launches, killed after
# teardown, no separate before/after capture to drift across an
# interactive gap.
cd "$(dirname "$0")/.." || exit 2

source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
source scripts/lib/gz_settle.sh
set -u

# JointCmdProbe, 2026-08-12: wired into ur5e_robotiq.urdf.xacro after
# gz_ros2_control's own plugin block (see that file for why order matters).
# Without this on the plugin search path, gz-sim fails to find it at spawn.
export GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/docs/gz_joint_cmd_probe/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

TS="$(date +%Y%m%d_%H%M%S)"
LOGDIR="docs"
SIM_LOG="${LOGDIR}/m3_effort_sim_${TS}.log"
MG_LOG="${LOGDIR}/m3_effort_movegroup_${TS}.log"
GRASP_LOG="${LOGDIR}/m3_effort_grasp_${TS}.log"
WATCHER_LOG="${LOGDIR}/m3_effort_watcher_${TS}.log"
EFFORT_CSV="${LOGDIR}/m3_effort_joint_${TS}.csv"
RIGHT_KNUCKLE_CSV="${LOGDIR}/m3_effort_right_knuckle_${TS}.csv"
RIGHT_FINGER_TIP_CSV="${LOGDIR}/m3_effort_right_finger_tip_${TS}.csv"
LEFT_FINGER_TIP_CSV="${LOGDIR}/m3_effort_left_finger_tip_${TS}.csv"
LEFT_INNER_KNUCKLE_CSV="${LOGDIR}/m3_effort_left_inner_knuckle_${TS}.csv"
RIGHT_INNER_KNUCKLE_CSV="${LOGDIR}/m3_effort_right_inner_knuckle_${TS}.csv"
CSV_PATH="m3_grasp_trace_effort_${TS}.csv"
ACTUATED_JOINT="robotiq_85_left_knuckle_joint"

hr()  { printf '\n%s\n' "════════════════════════════════════════════════════════════"; }
sec() { hr; printf '§ %s\n' "$1"; hr; }
die() { printf '  [STOP] %s\n' "$1"; exit 2; }

# ---------------------------------------------------------------------------
sec "0. Clean slate"
kill_sim
pkill -9 -f "move_group" 2>/dev/null
pkill -9 -f "m3_capture_joint_effort.py" 2>/dev/null
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
sec "5. Start joint-effort loggers (master + right-side mimic followers)"
# right_knuckle_joint and right_finger_tip_joint added 2026-08-12: HANDOFF_M3.md
# records right_knuckle diverging ~0.027-0.033rad from its multiplier under load
# only (tracks exactly in free space) -- run through the mimic law
# (velocity_sp = -(pos_follower - pos_master*mult) * 500) and that divergence
# commands ~13.5 rad/s against a 0.1 rad/s rail, i.e. railed ~135x whenever
# loaded. Logging both right-side joints here checks it directly instead of
# re-deriving it from the master's trace alone.
python3 docs/m3_capture_joint_effort.py "$ACTUATED_JOINT" "$EFFORT_CSV" \
  > "$WATCHER_LOG" 2>&1 &
WATCHER_PID=$!
printf '  effort logger PID=%s, log=%s, csv=%s\n' "$WATCHER_PID" "$WATCHER_LOG" "$EFFORT_CSV"
python3 docs/m3_capture_joint_effort.py "robotiq_85_right_knuckle_joint" "$RIGHT_KNUCKLE_CSV" \
  > "${LOGDIR}/m3_effort_right_knuckle_watcher_${TS}.log" 2>&1 &
RIGHT_KNUCKLE_PID=$!
printf '  right_knuckle logger PID=%s, csv=%s\n' "$RIGHT_KNUCKLE_PID" "$RIGHT_KNUCKLE_CSV"
python3 docs/m3_capture_joint_effort.py "robotiq_85_right_finger_tip_joint" "$RIGHT_FINGER_TIP_CSV" \
  > "${LOGDIR}/m3_effort_right_finger_tip_watcher_${TS}.log" 2>&1 &
RIGHT_FINGER_TIP_PID=$!
printf '  right_finger_tip logger PID=%s, csv=%s\n' "$RIGHT_FINGER_TIP_PID" "$RIGHT_FINGER_TIP_CSV"
# left_finger_tip_joint added 2026-08-12: comparison joint for the
# right_finger_tip_joint runaway (velocity pinned at rate ceiling, effort well
# below its own ceiling, reconstructed mimic-law command disagreeing with
# measured velocity under load). left_finger_tip_joint correctly mimics the
# master directly (geometrically exact, unlike the pre-fix right side) --
# if it shows the same signature, the runaway is generic to any fingertip
# joint under load; if not, it's specific to the right chain.
python3 docs/m3_capture_joint_effort.py "robotiq_85_left_finger_tip_joint" "$LEFT_FINGER_TIP_CSV" \
  > "${LOGDIR}/m3_effort_left_finger_tip_watcher_${TS}.log" 2>&1 &
LEFT_FINGER_TIP_PID=$!
printf '  left_finger_tip logger PID=%s, csv=%s\n' "$LEFT_FINGER_TIP_PID" "$LEFT_FINGER_TIP_CSV"
# left_inner_knuckle_joint / right_inner_knuckle_joint added 2026-08-12 to test
# the joint_index-1 mimicked-index hypothesis from reading gz_ros2_control's
# actual write() source: right_finger_tip_joint's measured velocity matches a
# formula using left_finger_tip_joint's position (98.4%), not its configured
# reference -- and left_finger_tip is hardware_info.joints array index 4,
# exactly right_finger_tip's own index (5) minus one. If the same off-by-one
# holds generally, left_finger_tip_joint (index 4) should itself be reading
# right_inner_knuckle_joint (index 3)'s position, not its configured
# left_knuckle_joint (index 0).
python3 docs/m3_capture_joint_effort.py "robotiq_85_left_inner_knuckle_joint" "$LEFT_INNER_KNUCKLE_CSV" \
  > "${LOGDIR}/m3_effort_left_inner_knuckle_watcher_${TS}.log" 2>&1 &
LEFT_INNER_KNUCKLE_PID=$!
printf '  left_inner_knuckle logger PID=%s, csv=%s\n' "$LEFT_INNER_KNUCKLE_PID" "$LEFT_INNER_KNUCKLE_CSV"
python3 docs/m3_capture_joint_effort.py "robotiq_85_right_inner_knuckle_joint" "$RIGHT_INNER_KNUCKLE_CSV" \
  > "${LOGDIR}/m3_effort_right_inner_knuckle_watcher_${TS}.log" 2>&1 &
RIGHT_INNER_KNUCKLE_PID=$!
printf '  right_inner_knuckle logger PID=%s, csv=%s\n' "$RIGHT_INNER_KNUCKLE_PID" "$RIGHT_INNER_KNUCKLE_CSV"
sleep 2   # let them establish the /joint_states subscription before anything interesting happens

# ---------------------------------------------------------------------------
sec "6. Run m3_grasp (streamed live)"
timeout 120 ros2 launch ur5e_pick_place m3_grasp.launch.py csv_path:="$CSV_PATH" \
  2>&1 | tee "$GRASP_LOG"
RC="${PIPESTATUS[0]}"
if [[ $RC -eq 124 ]]; then
  printf '  m3_grasp launch hit the 120s bound, killing it now\n'
  pkill -9 -f "m3_grasp.launch.py" 2>/dev/null
else
  printf '  m3_grasp launch returned (rc=%s) before the 120s bound\n' "$RC"
fi

# ---------------------------------------------------------------------------
sec "7. Collect the effort loggers"
sleep 1
kill -INT "$WATCHER_PID" "$RIGHT_KNUCKLE_PID" "$RIGHT_FINGER_TIP_PID" "$LEFT_FINGER_TIP_PID" "$LEFT_INNER_KNUCKLE_PID" "$RIGHT_INNER_KNUCKLE_PID" 2>/dev/null
wait "$WATCHER_PID" "$RIGHT_KNUCKLE_PID" "$RIGHT_FINGER_TIP_PID" "$LEFT_FINGER_TIP_PID" "$LEFT_INNER_KNUCKLE_PID" "$RIGHT_INNER_KNUCKLE_PID" 2>/dev/null
echo "--- $WATCHER_LOG ---"
cat "$WATCHER_LOG"
echo "--- $EFFORT_CSV (row count) ---"
wc -l "$EFFORT_CSV" 2>/dev/null
echo "--- $RIGHT_KNUCKLE_CSV (row count) ---"
wc -l "$RIGHT_KNUCKLE_CSV" 2>/dev/null
echo "--- $RIGHT_FINGER_TIP_CSV (row count) ---"
wc -l "$RIGHT_FINGER_TIP_CSV" 2>/dev/null
echo "--- $LEFT_FINGER_TIP_CSV (row count) ---"
wc -l "$LEFT_FINGER_TIP_CSV" 2>/dev/null
echo "--- $LEFT_INNER_KNUCKLE_CSV (row count) ---"
wc -l "$LEFT_INNER_KNUCKLE_CSV" 2>/dev/null
echo "--- $RIGHT_INNER_KNUCKLE_CSV (row count) ---"
wc -l "$RIGHT_INNER_KNUCKLE_CSV" 2>/dev/null

# ---------------------------------------------------------------------------
sec "8. Gate AFTER (gz_assert_gripper_responsive)"
if gz_assert_gripper_responsive; then
  printf '  GATE_AFTER=OK\n'
else
  printf '  GATE_AFTER=FAIL -- sim degraded during/after the trial\n'
  if [[ "$RC" -eq 0 ]]; then RC=1; fi
fi

# ---------------------------------------------------------------------------
sec "9. Cleanup"
kill_sim
pkill -9 -f "move_group" 2>/dev/null

printf '\ndone: %s\n' "$(date -Is)"
printf 'artifacts:\n  sim_log=%s\n  movegroup_log=%s\n  grasp_log=%s\n  watcher_log=%s\n  effort_csv=%s\n  right_knuckle_csv=%s\n  right_finger_tip_csv=%s\n  left_finger_tip_csv=%s\n  grasp_csv=%s\n' \
  "$SIM_LOG" "$MG_LOG" "$GRASP_LOG" "$WATCHER_LOG" "$EFFORT_CSV" "$RIGHT_KNUCKLE_CSV" "$RIGHT_FINGER_TIP_CSV" "$LEFT_FINGER_TIP_CSV" "$CSV_PATH"

exit "$RC"

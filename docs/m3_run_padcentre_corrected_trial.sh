#!/usr/bin/env bash
# m3_run_padcentre_corrected_trial.sh — live grasp trial with
# grasp.pad_centre_offset updated from 0.013433 (empirical, object-drop) to
# 0.031573 (mesh-measured, scripts/lib/gripper_geometry.py's
# PAD_FACE_Z_CENTROID_M). Every prior trial in this project's stall family
# (0.1301/0.1309/0.1323) used the OLD value -- this is the first live run on
# the corrected one.
#
# The correction's SIGN was reasoned geometrically but never live-verified
# (scene.yaml's own long-standing note) -- this trial is that verification,
# not an assumption of success. GRIPPER_GOAL_REJECTED again, at a materially
# different stall angle or width_error, still isn't nothing: it discriminates
# whether the finger-fouling explanation is complete or partial.
cd "$(dirname "$0")/.." || exit 2

source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
source scripts/lib/gz_settle.sh
set -u

TS="$(date +%Y%m%d_%H%M%S)"
LOGDIR="docs"
SIM_LOG="${LOGDIR}/m3_padfix_sim_${TS}.log"
MG_LOG="${LOGDIR}/m3_padfix_movegroup_${TS}.log"
GRASP_LOG="${LOGDIR}/m3_padfix_grasp_${TS}.log"
CSV_PATH="m3_grasp_trace_padfix_${TS}.csv"

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
sec "5. Run m3_grasp (single trial, expected to exit after one cycle)"
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
sec "6. RUN SUMMARY line from the grasp log"
grep "RUN SUMMARY\|pad-centre-corrected offset\|pre-close:\|gripper_close_and_hold:\|grasp-success check:" "$GRASP_LOG"

# ---------------------------------------------------------------------------
sec "7. Gate AFTER (gz_assert_gripper_responsive)"
if gz_assert_gripper_responsive; then
  printf '  GATE_AFTER=OK\n'
else
  printf '  GATE_AFTER=FAIL -- sim degraded during/after the trial\n'
fi

# ---------------------------------------------------------------------------
sec "8. Cleanup"
kill_sim
pkill -9 -f "move_group" 2>/dev/null

printf '\ndone: %s\n' "$(date -Is)"
printf 'artifacts:\n  sim_log=%s\n  movegroup_log=%s\n  grasp_log=%s\n  grasp_csv=%s\n' \
  "$SIM_LOG" "$MG_LOG" "$GRASP_LOG" "$CSV_PATH"

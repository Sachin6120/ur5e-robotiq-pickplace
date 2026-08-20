#!/usr/bin/env bash
# m6_baseline_traj_capture.sh — identical bracketed single-cycle protocol to
# m3_run_full_cycle_trial_live.sh, run against the UNMODIFIED baseline
# config (object.size width=0.045m, fingertip_grasp_theta=0.4029), with one
# addition: joint_trajectory_recorder.py runs alongside stall_monitor.py for
# the whole cycle, logging every /joint_states sample of
# robotiq_85_left_knuckle_joint.
#
# WHY THIS SCRIPT EXISTS
#   M6 trajectory-capture run 3 (branch m6-width-30mm, object.size
#   width=0.030m) saw the master gripper joint swing 0 -> 0.716 -> -0.588
#   rad during the PRE-GRASP ARM MOVE -- before any gripper command that
#   cycle. That is a position-controlled joint exceeding its own URDF limit
#   with nothing commanding it to move at all, which is a correctness
#   question about the sim/controller stack, not about the width-30mm
#   config specifically. This script's ONLY job is to answer: does the same
#   excursion happen at the frozen baseline config too?
#
#   - If yes: it's a pre-existing latent bug, independent of M6, and
#     everything M6 has measured so far still stands (the anomaly isn't
#     something the width change caused).
#   - If no: it's specific to the width=0.030m config and becomes a real M6
#     finding in its own right -- arguably more important than the original
#     width question, since it would mean that config perturbs the arm
#     controller before the gripper is even touched.
#
#   Deliberately NOT combined with a 4th 30mm capture attempt or with the
#   aperture_m_fixed_tip(0.4310) self-jam arithmetic -- that computation is
#   a pure function of already-captured run-2 data and needs no new sim run
#   at all. This script answers one question only.
#
# HOW TO USE
#   The caller is responsible for having config/scene.yaml at the ORIGINAL
#   baseline (object.size: [0.045, 0.045, 0.045]) before invoking this --
#   this script does not touch git state itself, on purpose, so the
#   stash/restore around it stays an explicit, visible, separate step.
#
cd "$(dirname "$0")/.." || exit 2

source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
source scripts/lib/gz_settle.sh
set -u

TS="$(date +%Y%m%d_%H%M%S)_${RANDOM}"
LOGDIR="docs"
SIM_LOG="${LOGDIR}/m6_baseline_sim_${TS}.log"
MG_LOG="${LOGDIR}/m6_baseline_movegroup_${TS}.log"
GRASP_LOG="${LOGDIR}/m6_baseline_grasp_${TS}.log"
STALL_CSV="${LOGDIR}/m6_baseline_stalls_${TS}.csv"
STALL_LOG="${LOGDIR}/m6_baseline_stallmonitor_${TS}.log"
TRAJ_CSV="runs/m6_baseline_traj_${TS}.csv"
TRAJ_LOG="${LOGDIR}/m6_baseline_trajrecorder_${TS}.log"
CSV_PATH="m3_grasp_trace_m6baseline_${TS}.csv"

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
sec "0.5 Confirm scene.yaml is at baseline width (not the M6 30mm override)"
WIDTH=$(python3 -c "
import yaml
with open('config/scene.yaml') as f:
    d = yaml.safe_load(f)
print(d['object']['size'][0])
")
printf '  object.size[0] = %s\n' "$WIDTH"
if [[ "$WIDTH" != "0.045" ]]; then
  die "scene.yaml width is $WIDTH, not baseline 0.045 -- stash the M6 diff before running this"
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
sec "1.5 Start stall_monitor + joint_trajectory_recorder (whole cycle)"
python3 scripts/stall_monitor.py --out "$STALL_CSV" \
  > "$STALL_LOG" 2>&1 &
STALL_PID=$!
printf '  stall_monitor PID=%s, csv=%s\n' "$STALL_PID" "$STALL_CSV"

python3 scripts/joint_trajectory_recorder.py --out "$TRAJ_CSV" \
  --joint robotiq_85_left_knuckle_joint \
  > "$TRAJ_LOG" 2>&1 &
TRAJ_PID=$!
printf '  joint_trajectory_recorder PID=%s, csv=%s\n' "$TRAJ_PID" "$TRAJ_CSV"
sleep 1   # let both /joint_states subscriptions establish

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
sec "5. Run m3_grasp (streamed live, torn down on RUN SUMMARY)"
: > "$GRASP_LOG"
PYTHONUNBUFFERED=1 ros2 launch ur5e_pick_place m3_grasp.launch.py csv_path:="$CSV_PATH" \
  > "$GRASP_LOG" 2>&1 &
LAUNCH_PID=$!

tail -n +1 -F "$GRASP_LOG" &
TAIL_PID=$!

DEADLINE=$(( $(date +%s) + 100 ))
RC=1
while true; do
  if grep -q "RUN SUMMARY" "$GRASP_LOG" 2>/dev/null; then
    RC=0
    printf '  RUN SUMMARY seen, tearing down now instead of waiting on the launch''s own dead-wait\n'
    break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    RC=2
    printf '  m3_grasp launch exited on its own before RUN SUMMARY ever appeared\n'
    break
  fi
  if [[ $(date +%s) -ge $DEADLINE ]]; then
    RC=1
    printf '  RUN SUMMARY never appeared within the 100s bound -- treating as a hang\n'
    break
  fi
  sleep 0.2
done

sleep 1
kill "$TAIL_PID" 2>/dev/null; wait "$TAIL_PID" 2>/dev/null

if kill -0 "$LAUNCH_PID" 2>/dev/null; then
  pkill -9 -f "m3_grasp.launch.py" 2>/dev/null
  wait "$LAUNCH_PID" 2>/dev/null
fi

# ---------------------------------------------------------------------------
sec "6. Gate AFTER (gz_assert_gripper_responsive)"
if gz_assert_gripper_responsive; then
  printf '  GATE_AFTER=OK\n'
else
  printf '  GATE_AFTER=FAIL -- sim degraded during/after the trial\n'
  if [[ "$RC" -eq 0 ]]; then RC=1; fi
fi

# ---------------------------------------------------------------------------
sec "6.5 Stop stall_monitor + joint_trajectory_recorder"
kill -TERM "$STALL_PID" 2>/dev/null
wait "$STALL_PID" 2>/dev/null
kill -TERM "$TRAJ_PID" 2>/dev/null
wait "$TRAJ_PID" 2>/dev/null
echo "--- $STALL_LOG ---"
cat "$STALL_LOG"
echo "--- $TRAJ_LOG ---"
cat "$TRAJ_LOG"

# ---------------------------------------------------------------------------
sec "7. Cleanup"
kill_sim
pkill -9 -f "move_group" 2>/dev/null

printf '\ndone: %s\n' "$(date -Is)"
printf 'artifacts:\n  sim_log=%s\n  movegroup_log=%s\n  grasp_log=%s\n  stall_csv=%s\n  traj_csv=%s\n  grasp_csv=%s\n' \
  "$SIM_LOG" "$MG_LOG" "$GRASP_LOG" "$STALL_CSV" "$TRAJ_CSV" "$CSV_PATH"

exit "$RC"

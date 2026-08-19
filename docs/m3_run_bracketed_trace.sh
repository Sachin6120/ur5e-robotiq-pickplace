#!/usr/bin/env bash
# m3_run_bracketed_trace.sh — one bracketed M3 grasp trial with the goal-
# traffic + joint-angle tracer (docs/m3_trace_goal_traffic.py) running
# underneath it. Built 2026-08-10 to test whether the large aperture change
# visible in a screenshot pair (cube displaced, fingers wide open) came from
# (a) sim-degradation drift, (b) a goal m3_grasp.cpp itself sent that the
# source read missed, or (c) something else entirely commanding the
# gripper -- see docs/m3_trace_goal_traffic.py's header for the full
# reasoning.
#
# Bracketed clean on both sides, per this project's own protocol
# (gz_assert_gripper_responsive immediately before AND after the
# measurement, not once per session -- a session opening this same night
# found a gate pass-then-fail straddling an untrusted measurement).
cd "$(dirname "$0")/.." || exit 2

# ROS2's own setup.bash files reference unset variables internally --
# keep -u off while sourcing them, same as every other script in this
# project that sources gz_settle.sh alongside them.
source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
source scripts/lib/gz_settle.sh
set -u

TS="$(date +%Y%m%d_%H%M%S)"
LOGDIR="docs"
SIM_LOG="${LOGDIR}/m3_trace_sim_${TS}.log"
MG_LOG="${LOGDIR}/m3_trace_movegroup_${TS}.log"
GRASP_LOG="${LOGDIR}/m3_trace_grasp_${TS}.log"
TRACE_OUT="${LOGDIR}/m3_trace_goaltraffic_${TS}.txt"
STALL_CSV="${LOGDIR}/m3_trace_stalls_${TS}.csv"
STALL_LOG="${LOGDIR}/m3_trace_stallmonitor_${TS}.log"
CSV_PATH="m3_grasp_trace_${TS}.csv"

hr()  { printf '\n%s\n' "════════════════════════════════════════════════════════════"; }
sec() { hr; printf '§ %s\n' "$1"; hr; }
die() { printf '  [STOP] %s\n' "$1"; exit 2; }

# ---------------------------------------------------------------------------
sec "0. Clean slate"
kill_sim
pkill -9 -f "move_group" 2>/dev/null   # bare pattern -- "move_group.launch.py" doesn't match the spawned moveit_ros_move_group/move_group binary's own argv, confirmed leaking an orphan across two runs tonight
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
sleep 3   # buffer: node appears before planning pipeline finishes loading
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
GATE_BEFORE_OK=1

# ---------------------------------------------------------------------------
sec "5. Start tracer + stall monitor"
# TRACE_DURATION_S widened from 60: velocity=0.5->0.1 adds ~5s per gripper
# close, ~10s per cycle across two closes -- the old 60s bound was already
# tight against a ~24s trial, no longer safe.
TRACE_DURATION_S=90
python3 docs/m3_trace_goal_traffic.py "$TRACE_OUT" "$TRACE_DURATION_S" \
  gripper_controller "/world/empty/model/ur5e_robotiq/joint_state" &
TRACER_PID=$!

# scripts/stall_monitor.py: header-stamp (sim time) /joint_states gap
# monitor, runs until SIGTERM -- riding along on every trial from here per
# the staircase plan (step 1 of 3), not just dedicated tests.
python3 scripts/stall_monitor.py --out "$STALL_CSV" --min-gap-ms 20 --heartbeat-s 15 \
  > "$STALL_LOG" 2>&1 &
STALL_PID=$!
sleep 1   # let both rclpy nodes init and subscribe before the trial fires

# ---------------------------------------------------------------------------
sec "6. Run m3_grasp (single trial, expected to exit after one cycle)"
# `ros2 launch` does NOT return on its own once m3_grasp's own process dies --
# static_scene_tf is a persistent node and keeps the launch alive
# indefinitely (confirmed live 2026-08-10: m3_grasp exited cleanly at ~24s,
# the launch itself sat there for 300+s until killed by hand). Bound it
# explicitly; every observed trial has finished well under 90s.
timeout 90 ros2 launch ur5e_pick_place m3_grasp.launch.py csv_path:="$CSV_PATH" \
  > "$GRASP_LOG" 2>&1
RC=$?
if [[ $RC -eq 124 ]]; then
  printf '  m3_grasp launch hit the 90s bound (expected -- static_scene_tf never exits on its own), killing it now\n'
  pkill -9 -f "m3_grasp.launch.py" 2>/dev/null
else
  printf '  m3_grasp launch returned (rc=%s) before the 90s bound -- unexpected, check %s\n' "$RC" "$GRASP_LOG"
fi

# ---------------------------------------------------------------------------
sec "7. Tail: hold the tracer and stall monitor open after m3_grasp exits"
# Catches a goal delivered late, or one sent by a cleanup/leftover process
# after m3_grasp's own process has already died -- exactly the "external
# commander" hypothesis this trace exists to check.
TAIL_S=20
printf '  waiting %ss past m3_grasp exit before stopping the tracer...\n' "$TAIL_S"
sleep "$TAIL_S"

wait "$TRACER_PID" 2>/dev/null
printf '  tracer finished, output=%s\n' "$TRACE_OUT"

kill -TERM "$STALL_PID" 2>/dev/null
sleep 1
wait "$STALL_PID" 2>/dev/null
printf '  stall monitor finished, csv=%s\n' "$STALL_CSV"
cat "$STALL_LOG"

# ---------------------------------------------------------------------------
sec "8. Gate AFTER (gz_assert_gripper_responsive)"
if gz_assert_gripper_responsive; then
  printf '  GATE_AFTER=OK\n'
else
  printf '  GATE_AFTER=FAIL -- sim degraded during/after the trial; treat this trial''s numbers as suspect, do not discard the trace itself (still shows who sent what)\n'
fi

# ---------------------------------------------------------------------------
sec "9. Cleanup"
kill_sim
pkill -9 -f "move_group" 2>/dev/null   # bare pattern -- "move_group.launch.py" doesn't match the spawned moveit_ros_move_group/move_group binary's own argv, confirmed leaking an orphan across two runs tonight

printf '\ndone: %s\n' "$(date -Is)"
printf 'artifacts:\n  sim_log=%s\n  movegroup_log=%s\n  grasp_log=%s\n  trace=%s\n  stall_csv=%s\n  csv=%s\n' \
  "$SIM_LOG" "$MG_LOG" "$GRASP_LOG" "$TRACE_OUT" "$STALL_CSV" "$CSV_PATH"

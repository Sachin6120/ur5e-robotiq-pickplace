#!/usr/bin/env bash
# m3_run_streamed_hold_gap_trial.sh — same bracketed M3 grasp trial as
# m3_run_bracketed_trace.sh (goal-traffic tracer + stall monitor), with
# scripts/10_stream_joint_cmd.py added as a THIRD evidence stream: a
# continuous `gz topic -e` subscription (no per-sample subprocess spawn,
# ~1kHz) reconstructing the POSITION command from the velocity trace for the
# master gripper joint across the whole trial.
#
# WHY THIS EXISTS ON TOP OF THE EXISTING TRACER
#
#   docs/m3_trace_goal_traffic.py's GzJointPoller samples via `gz topic -e -n
#   1` per tick (~80ms poll_dt against a 500Hz control loop) -- every sample
#   in the hold-gap excursion trace it already caught came back railed
#   (+/-0.1 or worse), because the turnover the discriminator needs happens
#   40-100x faster than that. That poller answered "did the master move with
#   no active goal" (yes, confirmed: 0.0956->0.2546rad, one sample at 3.5x
#   its own clamp) but cannot answer WHY -- a smooth externally-driven
#   trajectory (#1) and a dual-writer fight (#3) both produce that same
#   undersampled signature.
#
#   scripts/10_stream_joint_cmd.py's discriminator: count unrailed samples
#   (|cmd-pos| < rail/gain = 0.002rad) at the turnover. >=5 -> #1 (a smooth
#   cmd cannot cross the band in fewer). <5 -> #3 (cmd jumped across it).
#   Verified end-to-end on this sim tonight: 992 samples/s sustained (well
#   above the ~250/s floor), no RTF sag, and the reconstruction identity
#   itself checked exact against a known idle hold (cmd_exact == pos ==
#   0.030000 on 9890/9890 samples, 0.000000 max deviation) before ever being
#   pointed at this trial.
#
#   Kept the existing tracer + stall monitor running unchanged alongside it:
#   the streamer only answers the #1-vs-#3 question for the master joint. It
#   does not show WHO sent a goal (that's GoalStatusTracer's GOAL stream,
#   ROS-level, no gz-level equivalent) and it does not show the OTHER five
#   linkage joints (that's GzJointPoller's follower-vs-master evidence,
#   already used to rule out the dragging-follower hypothesis). All three
#   streams share this run's TS in their filenames for after-the-fact
#   correlation.
#
# Bracketed clean on both sides, unchanged from m3_run_bracketed_trace.sh's
# own protocol (gz_assert_gripper_responsive immediately before AND after).
cd "$(dirname "$0")/.." || exit 2

source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
source scripts/lib/gz_settle.sh
set -u

TS="$(date +%Y%m%d_%H%M%S)"
LOGDIR="docs"
SIM_LOG="${LOGDIR}/m3_stream_sim_${TS}.log"
MG_LOG="${LOGDIR}/m3_stream_movegroup_${TS}.log"
GRASP_LOG="${LOGDIR}/m3_stream_grasp_${TS}.log"
TRACE_OUT="${LOGDIR}/m3_stream_goaltraffic_${TS}.txt"
STALL_CSV="${LOGDIR}/m3_stream_stalls_${TS}.csv"
STALL_LOG="${LOGDIR}/m3_stream_stallmonitor_${TS}.log"
CMD_CSV="${LOGDIR}/m3_stream_cmd_${TS}.csv"
CMD_SUMMARY="${LOGDIR}/m3_stream_cmd_summary_${TS}.log"
CSV_PATH="m3_grasp_trace_${TS}.csv"

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
GATE_BEFORE_OK=1

# ---------------------------------------------------------------------------
sec "5. Start tracer + stall monitor + streamed cmd-reconstruction"
TRACE_DURATION_S=90
python3 docs/m3_trace_goal_traffic.py "$TRACE_OUT" "$TRACE_DURATION_S" \
  gripper_controller "/world/empty/model/ur5e_robotiq/joint_state" &
TRACER_PID=$!

python3 scripts/stall_monitor.py --out "$STALL_CSV" --min-gap-ms 20 --heartbeat-s 15 \
  > "$STALL_LOG" 2>&1 &
STALL_PID=$!

# timeout sends SIGINT to `gz topic -e` at TRACE_DURATION_S; the python
# streamer sees EOF, falls out of its read loop, and prints its own summary
# (unrailed-run table) before exiting -- no signal needs to reach it directly.
timeout -s INT "$TRACE_DURATION_S" gz topic -e -t /world/empty/model/ur5e_robotiq/joint_state \
  | python3 scripts/10_stream_joint_cmd.py \
      --joint robotiq_85_left_knuckle_joint \
      --out "$CMD_CSV" \
  > "$CMD_SUMMARY" 2>&1 &
STREAM_PID=$!
printf '  tracer PID=%s, stall monitor PID=%s, streamer pipeline PID=%s\n' "$TRACER_PID" "$STALL_PID" "$STREAM_PID"

sleep 1   # let all subscribers init before the trial fires

# ---------------------------------------------------------------------------
sec "6. Run m3_grasp (single trial, expected to exit after one cycle)"
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
sec "7. Tail: hold all three streams open past m3_grasp exit"
TAIL_S=20
printf '  waiting %ss past m3_grasp exit before stopping the tracers...\n' "$TAIL_S"
sleep "$TAIL_S"

wait "$TRACER_PID" 2>/dev/null
printf '  goal-traffic tracer finished, output=%s\n' "$TRACE_OUT"

wait "$STREAM_PID" 2>/dev/null
printf '  cmd-reconstruction streamer finished, csv=%s\n' "$CMD_CSV"
echo "--- $CMD_SUMMARY ---"
cat "$CMD_SUMMARY"

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
  printf '  GATE_AFTER=FAIL -- sim degraded during/after the trial; treat this trial''s numbers as suspect, do not discard the traces themselves (they still show who/what/when)\n'
fi

# ---------------------------------------------------------------------------
sec "9. Cleanup"
kill_sim
pkill -9 -f "move_group" 2>/dev/null

printf '\ndone: %s\n' "$(date -Is)"
printf 'artifacts:\n  sim_log=%s\n  movegroup_log=%s\n  grasp_log=%s\n  goal_trace=%s\n  stall_csv=%s\n  cmd_csv=%s\n  cmd_summary=%s\n  grasp_csv=%s\n' \
  "$SIM_LOG" "$MG_LOG" "$GRASP_LOG" "$TRACE_OUT" "$STALL_CSV" "$CMD_CSV" "$CMD_SUMMARY" "$CSV_PATH"

#!/usr/bin/env bash
# m3_run_full_cycle_trial_live.sh — same bracketed protocol as
# m3_run_full_cycle_trial.sh, with ONE structural change: m3_grasp's launch
# output is teed to this script's own stdout LIVE (as each line is emitted),
# not buffered into a file and only grepped/printed after the whole cycle
# (including cleanup) has already finished.
#
# WHY THIS CHANGE IS NOT OPTIONAL FOR THE HARNESS
#   scripts/11_m3_cycles.sh's watcher does `tail -F` on THIS script's stdout
#   (captured as the harness's own per-cycle log) and reacts to "M3 STAGE 3
#   LIFT_DONE" / "M3 STAGE 4 TRANSPORT_DONE" the moment they appear, calling
#   sample_pose.py during transport.cpp's dwell window while the arm is
#   actually still holding position.
#
#   The original script redirected m3_grasp's entire launch output straight
#   into a file (`> "$GRASP_LOG" 2>&1`) and only grepped/printed matching
#   lines in section 6, AFTER the whole `timeout 120 ros2 launch ...` call had
#   already returned -- i.e. after transport, place, release, retreat, and the
#   launch's own ~70s dead-wait had all already happened. A watcher tailing
#   that script's stdout would see "LIFT_DONE" arrive tens of seconds after
#   the dwell window it was supposed to trigger sampling during had already
#   closed, or after cleanup had torn the sim down entirely. Silent, total
#   harness failure -- not a loud one, so this had to be caught before the
#   first harness run, not diagnosed after 3 empty CSVs.
#
#   Fix: `2>&1 | tee "$GRASP_LOG"` streams every line to stdout AS IT ARRIVES,
#   while still keeping the same on-disk log file. `${PIPESTATUS[0]}` recovers
#   `ros2 launch`'s own exit code from the pipeline -- $? after a pipe is
#   tee's exit code, not the launch's, and this project deliberately runs
#   without `set -o pipefail` (see 02_bootstrap_noble.sh's own header note),
#   so PIPESTATUS is the correct way to get it, not a workaround for one.
#
# WHY SECTION 5 NO LONGER USES `timeout 120 ros2 launch ... | tee`
#   ros2 launch never exits on its own here -- static_scene_tf and the rest
#   of the launch tree keep running after m3_grasp's own process reports
#   "process has finished cleanly" and logs RUN SUMMARY, so the launch
#   always ran out the `timeout 120` bound. That meant `${PIPESTATUS[0]}`
#   was 124 on every normal cycle AND on a genuine 120s hang -- the exit
#   code could not tell them apart, and `trial_exit` in 11_m3_cycles.sh's
#   CSV was useless across a sweep for exactly that reason (found live,
#   2026-08-11, reviewing the friction-fix cycle's own row). It also meant
#   every cycle paid the full ~70s of dead time after RUN SUMMARY before
#   teardown started.
#
#   Fix: launch in the background, tail its log to this script's own stdout
#   (same live-streaming property as the tee above), and poll for "RUN
#   SUMMARY" appearing. The moment it does, tear the launch down immediately
#   instead of waiting for it to exit on its own -- RC is set from what was
#   actually observed (0 = RUN SUMMARY seen, 1 = bound hit with no summary,
#   a real hang, 2 = the launch process died on its own before ever
#   producing one, also a real failure) rather than a `timeout` exit code
#   that could not distinguish any of those cases.
cd "$(dirname "$0")/.." || exit 2

source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
source scripts/lib/gz_settle.sh
set -u

TS="$(date +%Y%m%d_%H%M%S)_${RANDOM}"
LOGDIR="docs"
SIM_LOG="${LOGDIR}/m3_cyclelive_sim_${TS}.log"
MG_LOG="${LOGDIR}/m3_cyclelive_movegroup_${TS}.log"
GRASP_LOG="${LOGDIR}/m3_cyclelive_grasp_${TS}.log"
STALL_CSV="${LOGDIR}/m3_cyclelive_stalls_${TS}.csv"
STALL_LOG="${LOGDIR}/m3_cyclelive_stallmonitor_${TS}.log"
CSV_PATH="m3_grasp_trace_cyclelive_${TS}.csv"
# Set by scripts/11_m3_cycles.sh per cycle (its own $PRE prefix) when the
# sweep watcher needs a filesystem-write signal instead of parsing live
# stdout -- see transport.hpp's TransportParams::marker_file_prefix. Empty
# by default for standalone/non-sweep runs of this script.
MARKER_FILE_PREFIX="${M3_MARKER_PREFIX:-}"

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
sec "1.5 Start stall_monitor (passive /joint_states cadence recorder, whole cycle)"
# scripts/stall_monitor.py's own header: "Run this alongside everything and
# let the samples accumulate. Twenty M3 cycles is twenty samples for free."
# The standing decision after two dedicated reproduction hunts both failed --
# not wiring it in here would mean losing that data for good, not just
# deferring it. Started before move_group/object-spawn so it covers the
# whole cycle, stopped (section 6.5) before kill_sim tears the sim down, so
# the topic dying at teardown doesn't log as a fake stall.
python3 scripts/stall_monitor.py --out "$STALL_CSV" \
  > "$STALL_LOG" 2>&1 &
STALL_PID=$!
printf '  stall_monitor PID=%s, csv=%s, log=%s\n' "$STALL_PID" "$STALL_CSV" "$STALL_LOG"
sleep 1   # let its /joint_states subscription establish

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
sec "5. Run m3_grasp (streamed live, torn down on RUN SUMMARY -- see file header)"
: > "$GRASP_LOG"
# PYTHONUNBUFFERED=1, added 2026-08-12: `ros2 launch`'s own stdout, redirected
# straight to a file below (not a TTY), is Python -- it defaults to fully
# buffered, not line buffered, in that case. m3_grasp.launch.py's Node()
# entries already set emulate_tty=True, which fixes buffering at the NODE's
# own stdio level, but that is one layer below `ros2 launch` itself: the
# launch process aggregates and re-emits that output through its OWN stdout,
# and THAT stream is what actually lands in $GRASP_LOG. Confirmed live: three
# 11_m3_cycles.sh sweep cycles all reported result=SUCCESS in the completed
# log yet produced zero slip samples -- the stage markers a live tail -F
# watcher needs were present in the file eventually, never while the process
# was still running. Isolated first (not assumed): a standalone tail -F test
# against the exact truncate-then-rewrite pattern this harness uses worked
# correctly, ruling out the tail/file mechanism itself and pointing upstream,
# to buffering before the bytes ever reached $GRASP_LOG.
PYTHONUNBUFFERED=1 ros2 launch ur5e_pick_place m3_grasp.launch.py csv_path:="$CSV_PATH" \
  marker_file_prefix:="$MARKER_FILE_PREFIX" \
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

sleep 1   # let the last log lines reach the tail before it's killed
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
sec "6.5 Stop stall_monitor"
kill -TERM "$STALL_PID" 2>/dev/null
wait "$STALL_PID" 2>/dev/null
echo "--- $STALL_LOG ---"
cat "$STALL_LOG"

# ---------------------------------------------------------------------------
sec "7. Cleanup"
kill_sim
pkill -9 -f "move_group" 2>/dev/null

printf '\ndone: %s\n' "$(date -Is)"
printf 'artifacts:\n  sim_log=%s\n  movegroup_log=%s\n  grasp_log=%s\n  stall_csv=%s\n  grasp_csv=%s\n' \
  "$SIM_LOG" "$MG_LOG" "$GRASP_LOG" "$STALL_CSV" "$CSV_PATH"

exit "$RC"

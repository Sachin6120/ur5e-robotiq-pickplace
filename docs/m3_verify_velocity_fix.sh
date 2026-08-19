#!/usr/bin/env bash
# m3_verify_velocity_fix.sh — smoke test for the velocity=0.5->0.1 override
# (ur5e_robotiq_description/urdf/vendor/robotiq_2f_85_macro.urdf.xacro) and
# scripts/stall_monitor.py, before either is trusted as "deployed".
# Gripper-only, no arm/object needed. Checks:
#   1. a 0.1055rad close now takes ~5x longer (~1.0s vs ~0.2-0.4s) --
#      confirms the override actually took effect at runtime, not just in
#      the expanded URDF file.
#   2. stall_monitor.py runs cleanly end to end (starts, logs a sane
#      baseline gap, shuts down on SIGTERM without an exception, writes a
#      valid CSV with a summary row).
cd "$(dirname "$0")/.." || exit 2

source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
source scripts/lib/gz_settle.sh
set -u

TS="$(date +%Y%m%d_%H%M%S)"
SIM_LOG="docs/m3_verify_sim_${TS}.log"
STALL_CSV="docs/m3_verify_stalls_${TS}.csv"
MONITOR_LOG="docs/m3_verify_monitor_${TS}.log"

hr()  { printf '\n%s\n' "════════════════════════════════════════════════════════════"; }
sec() { hr; printf '§ %s\n' "$1"; hr; }
die() { printf '  [STOP] %s\n' "$1"; exit 2; }

sec "0. Clean slate"
kill_sim
pkill -9 -f "move_group" 2>/dev/null
sleep 1
gz_assert_clean_slate || die "refusing to start on a contaminated system"

sec "1. Launch sim"
nohup ros2 launch ur5e_robotiq_description ur5e_robotiq_sim_control.launch.py \
  > "$SIM_LOG" 2>&1 &
gz_wait_controller_active_bounded "gripper_controller" 30 || die "gripper_controller never active"

sec "2. Gate BEFORE"
gz_assert_gripper_responsive || die "sim degraded before the test started"

sec "3. Start stall_monitor.py"
python3 scripts/stall_monitor.py --out "$STALL_CSV" --min-gap-ms 20 --heartbeat-s 10 \
  > "$MONITOR_LOG" 2>&1 &
MONITOR_PID=$!
sleep 1.5   # let it subscribe and log its startup lines

sec "4. Time a 0.1055rad close from settled 0 -- expect ~1.0s now, not ~0.2-0.4s"
ros2 action send_goal "/gripper_controller/gripper_cmd" control_msgs/action/GripperCommand \
  "{command: {position: 0.0, max_effort: 50.0}}" > /dev/null 2>&1
gz_settle_joint "/world/empty/model/ur5e_robotiq/joint_state" 0.01 5.0 0.1 \
  robotiq_85_left_knuckle_joint

T0=$(date +%s.%N)
ros2 action send_goal "/gripper_controller/gripper_cmd" control_msgs/action/GripperCommand \
  "{command: {position: 0.1055, max_effort: 50.0}}" 2>&1 | sed 's/^/    /'
T1=$(date +%s.%N)
ELAPSED=$(python3 -c "print(f'{${T1}-${T0}:.3f}')")
printf '  0.1055rad close took %ss\n' "$ELAPSED"

sec "5. Stop the monitor (SIGTERM, matches its own signal handler)"
kill -TERM "$MONITOR_PID" 2>/dev/null
sleep 1
wait "$MONITOR_PID" 2>/dev/null
MONITOR_RC=$?
printf '  monitor exit code=%s\n' "$MONITOR_RC"
cat "$MONITOR_LOG"

sec "6. Gate AFTER"
if gz_assert_gripper_responsive; then
  printf '  GATE_AFTER=OK\n'
else
  printf '  GATE_AFTER=FAIL\n'
fi

sec "7. Cleanup"
kill_sim
pkill -9 -f "move_group" 2>/dev/null

printf '\ndone: %s\n' "$(date -Is)"
printf 'artifacts:\n  sim_log=%s\n  monitor_log=%s\n  stall_csv=%s\n' "$SIM_LOG" "$MONITOR_LOG" "$STALL_CSV"
echo "--- stall_csv contents ---"
cat "$STALL_CSV"

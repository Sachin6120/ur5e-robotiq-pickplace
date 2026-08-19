#!/usr/bin/env bash
# m3_check_probe_load.sh -- sim-launch-only check that JointCmdProbe loads
# and finds all six joints, before spending a full grasp-cycle trial on it.
cd "$(dirname "$0")/.." || exit 2

source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
source scripts/lib/gz_settle.sh
set -u

export GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/docs/gz_joint_cmd_probe/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
rm -f /tmp/joint_cmd_probe.csv

LOG="docs/m3_probe_check_$(date +%Y%m%d_%H%M%S).log"

kill_sim
pkill -9 -f "move_group" 2>/dev/null
sleep 1

nohup ros2 launch ur5e_robotiq_description ur5e_robotiq_sim_control.launch.py \
  > "$LOG" 2>&1 &
SIM_PID=$!
printf 'sim launch PID=%s, log=%s\n' "$SIM_PID" "$LOG"

if gz_wait_controller_active_bounded "gripper_controller" 30; then
  printf 'gripper_controller ACTIVE\n'
else
  printf 'gripper_controller NEVER WENT ACTIVE -- see %s\n' "$LOG"
fi
sleep 3

printf -- '--- grep for JointCmdProbe / error / not found in %s ---\n' "$LOG"
grep -iE 'JointCmdProbe|error.*plugin|not found' "$LOG" || printf '(no matches)\n'

printf -- '--- /tmp/joint_cmd_probe.csv ---\n'
if [[ -f /tmp/joint_cmd_probe.csv ]]; then
  wc -l /tmp/joint_cmd_probe.csv
  head -8 /tmp/joint_cmd_probe.csv
else
  printf 'FILE DOES NOT EXIST -- probe did not run or failed to open output\n'
fi

kill_sim
pkill -9 -f "move_group" 2>/dev/null

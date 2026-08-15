#!/usr/bin/env bash
# m3_check_mimic_chain_load.sh -- sim-launch-only check, no grasp cycle.
# Answers one question cheaply: does gz_ros2_control accept
# right_finger_tip_joint's mimic reference now pointing at right_knuckle_joint
# (itself a mimic of the master), or does it reject the mimic-of-a-mimic chain
# at load time? If it rejects, this is the fast way to find out -- no need to
# spend a full grasp-cycle trial discovering the sim never came up.
cd "$(dirname "$0")/.." || exit 2

source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
source scripts/lib/gz_settle.sh
set -u

LOG="docs/m3_mimic_chain_check_$(date +%Y%m%d_%H%M%S).log"

kill_sim
pkill -9 -f "move_group" 2>/dev/null
sleep 1

nohup ros2 launch ur5e_robotiq_description ur5e_robotiq_sim_control.launch.py \
  > "$LOG" 2>&1 &
SIM_PID=$!
printf 'sim launch PID=%s, log=%s\n' "$SIM_PID" "$LOG"

if gz_wait_controller_active_bounded "gripper_controller" 30; then
  printf 'RESULT: gripper_controller ACTIVE -- mimic chain accepted (or at least did not block controller activation)\n'
  RC=0
else
  printf 'RESULT: gripper_controller NEVER WENT ACTIVE -- see %s for the rejection\n' "$LOG"
  RC=1
fi

sleep 2
printf -- '--- grep for mimic/error/reject/fatal in %s ---\n' "$LOG"
grep -iE 'mimic|error|reject|fatal' "$LOG" || printf '(no matches)\n'

kill_sim
pkill -9 -f "move_group" 2>/dev/null
exit "$RC"

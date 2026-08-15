#!/usr/bin/env bash
# m3_check_state_topic.sh -- the cheap check before building JointCmdProbe:
# does /world/<world>/state already carry JointVelocityCmd via
# SceneBroadcaster's default serialization filters? If yes, no plugin needed.
# Expected answer is no (filters are narrow) but it costs nothing to rule out.
cd "$(dirname "$0")/.." || exit 2

source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
source scripts/lib/gz_settle.sh
set -u

LOG="docs/m3_topic_check_$(date +%Y%m%d_%H%M%S).log"

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
sleep 2

printf -- '--- gz topic list, state-related ---\n'
timeout 5 gz topic -l | grep -i state

printf -- '--- one message from /world/empty/state, searching for JointVelocityCmd ---\n'
timeout 8 gz topic -e -t /world/empty/state -n 1 > /tmp/m3_state_msg.txt 2>&1
if grep -qi "jointvelocitycmd\|velocity_cmd" /tmp/m3_state_msg.txt; then
  printf 'FOUND -- JointVelocityCmd is in the state message:\n'
  grep -A3 -i "jointvelocitycmd\|velocity_cmd" /tmp/m3_state_msg.txt
else
  printf 'NOT FOUND -- JointVelocityCmd is not serialized in /world/empty/state (%d bytes captured)\n' "$(wc -c < /tmp/m3_state_msg.txt)"
fi

kill_sim
pkill -9 -f "move_group" 2>/dev/null

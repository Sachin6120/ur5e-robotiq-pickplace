#!/usr/bin/env bash
# m3_check_topic_publishers.sh — one-shot check: is there more than one
# publisher on /joint_states? If so, every gap stall_monitor.py reports is
# an interleave of two streams, not one loop's cadence, and needs a
# publisher filter before being trusted for the sweep.
cd "$(dirname "$0")/.." || exit 2

source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
source scripts/lib/gz_settle.sh
set -u

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
  > /tmp/m3_pubcheck_sim.log 2>&1 &
gz_wait_controller_active_bounded "gripper_controller" 30 || die "gripper_controller never active"

sec "2. ros2 topic info /joint_states --verbose"
ros2 topic info /joint_states --verbose

sec "3. Cleanup"
kill_sim
pkill -9 -f "move_group" 2>/dev/null
printf 'done\n'

#!/usr/bin/env bash
# m3_launch_gui.sh — bring up the sim WITH the Gazebo GUI (gazebo_gui:=true),
# for the GUI look flagged as the direct next step on the 0.19 plateau /
# hold-gap drift investigation. Leaves the sim running in the foreground's
# background (nohup) rather than tearing it down -- this is for a human to
# look at, not a bracketed measurement.
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

sec "1. Launch sim WITH GUI"
nohup ros2 launch ur5e_robotiq_description ur5e_robotiq_sim_control.launch.py \
  gazebo_gui:=true > docs/m3_gui_sim.log 2>&1 &
SIM_PID=$!
printf '  sim launch PID=%s, log=docs/m3_gui_sim.log\n' "$SIM_PID"

gz_wait_controller_active_bounded "gripper_controller" 30 || die "gripper_controller never active"
gz_wait_controller_active_bounded "arm_controller" 10 || die "arm_controller never active"

sec "2. Launch move_group"
nohup ros2 launch ur5e_robotiq_moveit_config move_group.launch.py \
  > docs/m3_gui_movegroup.log 2>&1 &
printf '  move_group launched, log=docs/m3_gui_movegroup.log\n'

MG_T0=$(date +%s.%N)
MG_ELAPSED=0
until ros2 node list 2>/dev/null | grep -q '^/move_group$'; do
  MG_ELAPSED=$(python3 -c "import time; print(f'{time.time()-${MG_T0}:.1f}')")
  if python3 -c "import sys; sys.exit(0 if float('$MG_ELAPSED') > 40 else 1)"; then
    die "move_group node never appeared after 40s"
  fi
  sleep 0.5
done
sleep 3
printf '  move_group ready after %ss (+3s buffer)\n' "$MG_ELAPSED"

sec "3. Spawn pick object"
bash scripts/08_spawn_pick_object.sh || die "object spawn/settle failed"

sec "4. Gate check (informational only -- not tearing anything down either way)"
if gz_assert_gripper_responsive; then
  printf '  gripper responsive: OK\n'
else
  printf '  gripper responsive: FAILED -- sim may be degraded, restart if the GUI shows anything odd\n'
fi

hr
printf 'Sim is up WITH GUI, left running (not torn down).\n'
printf 'sim PID=%s\n' "$SIM_PID"
printf 'To run one grasp cycle while watching: ros2 launch ur5e_pick_place m3_grasp.launch.py\n'
printf 'To tear down when done: pkill -9 -f "gz sim" ; pkill -9 -f move_group ; pkill -9 -f robot_state_publisher ; pkill -9 -f parameter_bridge\n'

#!/usr/bin/env bash
# Run one Scene-A perception-driven cycle and own every process it starts.

set -uo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUN_TIMEOUT_S="${RUN_TIMEOUT_S:-240}"
STARTUP_TIMEOUT_S="${STARTUP_TIMEOUT_S:-60}"
SHUTDOWN_GRACE_S="${SHUTDOWN_GRACE_S:-10}"
OUT_DIR="${OUT_DIR:-${PROJECT_DIR}/evidence/perception_validation_pj_$(date +%Y%m%d_%H%M%S)}"

declare -a CHILD_PGIDS=()
declare -a CHILD_LAUNCHERS=()
CLEANED_UP=0

group_alive() {
  kill -0 -- "-$1" 2>/dev/null
}

stop_group() {
  local pgid="$1" deadline
  group_alive "$pgid" || return 0
  kill -TERM -- "-$pgid" 2>/dev/null || true
  deadline=$((SECONDS + SHUTDOWN_GRACE_S))
  while group_alive "$pgid" && (( SECONDS < deadline )); do
    sleep 0.2
  done
  if group_alive "$pgid"; then
    kill -KILL -- "-$pgid" 2>/dev/null || true
  fi
}

cleanup() {
  local i
  (( CLEANED_UP == 0 )) || return 0
  CLEANED_UP=1
  echo "=== Cleaning up validation child process groups ==="
  for ((i=${#CHILD_PGIDS[@]}-1; i>=0; i--)); do
    stop_group "${CHILD_PGIDS[$i]}"
    wait "${CHILD_LAUNCHERS[$i]}" 2>/dev/null || true
  done
}

on_signal() {
  local signal="$1" exit_code="$2"
  echo "Validation interrupted by ${signal}." >&2
  cleanup
  trap - EXIT
  exit "$exit_code"
}

trap cleanup EXIT
trap 'on_signal INT 130' INT
trap 'on_signal TERM 143' TERM

start_group() {
  local log_path="$1" pid_file deadline launcher
  shift
  pid_file="$OUT_DIR/.child_${#CHILD_PGIDS[@]}.pid"
  rm -f "$pid_file"
  # --fork makes session creation independent of the caller's job-control
  # mode. The session child reports its own PID (and therefore PGID) before
  # exec, while the --wait parent remains available for deterministic reap.
  setsid --fork --wait bash -c \
    'printf "%s\n" "$$" > "$1"; shift; exec "$@"' bash "$pid_file" "$@" \
    >"$log_path" 2>&1 &
  launcher=$!
  deadline=$((SECONDS + 5))
  while [[ ! -s "$pid_file" ]] && kill -0 "$launcher" 2>/dev/null && \
      (( SECONDS < deadline )); do
    sleep 0.05
  done
  if [[ ! -s "$pid_file" ]]; then
    echo "PROCESS_START_FAILURE: child session did not report its PGID." >&2
    wait "$launcher" 2>/dev/null || true
    return 1
  fi
  read -r STARTED_PGID < "$pid_file"
  CHILD_PGIDS+=("$STARTED_PGID")
  CHILD_LAUNCHERS+=("$launcher")
}

wait_for_command() {
  local description="$1" deadline
  shift
  deadline=$((SECONDS + STARTUP_TIMEOUT_S))
  while (( SECONDS < deadline )); do
    if "$@" >/dev/null 2>&1; then
      echo "${description} ready."
      return 0
    fi
    sleep 1
  done
  echo "STARTUP_TIMEOUT: ${description} was not ready within ${STARTUP_TIMEOUT_S}s." >&2
  return 1
}

if pgrep -f 'm3_grasp|static_scene_tf|move_group|object_detector|object_position_world|[g]z sim|robot_state_publisher|ros2_control_node' >/dev/null; then
  echo "CONTAMINATED_ENVIRONMENT: project ROS/Gazebo processes are already running; refusing to kill unowned processes." >&2
  exit 2
fi

set +u
source /opt/ros/jazzy/setup.bash
source "${PROJECT_DIR}/install/setup.bash"
set -u
mkdir -p "$OUT_DIR"
export ROS_LOG_DIR="$OUT_DIR/ros_logs"
mkdir -p "$ROS_LOG_DIR"

echo "=== Starting Gazebo + sim control ==="
start_group "$OUT_DIR/sim.log" ros2 launch ur5e_robotiq_description \
  ur5e_robotiq_sim_control.launch.py gripper_model:=parallel_jaw \
  enable_camera:=true gazebo_gui:=${GAZEBO_GUI:-false} || exit 9
SIM_PGID="$STARTED_PGID"
wait_for_command "arm_controller" bash -c \
  "ros2 control list_controllers 2>/dev/null | grep -q '^arm_controller.*active'" || exit 3

echo "=== Starting MoveIt ==="
start_group "$OUT_DIR/move_group.log" ros2 launch ur5e_robotiq_moveit_config \
  move_group.launch.py gripper_model:=parallel_jaw || exit 9
MOVEIT_PGID="$STARTED_PGID"
wait_for_command "move_group" bash -c \
  "ros2 node list 2>/dev/null | grep -qx '/move_group'" || exit 3

echo "=== Spawning Scene-A object ==="
python3 - "$PROJECT_DIR" <<'PY' || exit 4
import importlib.util
import pathlib
import sys

project = pathlib.Path(sys.argv[1])
path = project / "scripts/perception/milestone_f1_harness.py"
spec = importlib.util.spec_from_file_location("f1_harness", path)
harness = importlib.util.module_from_spec(spec)
spec.loader.exec_module(harness)
print("Remove old object:", harness.remove_object())
print("Spawn object:", harness.spawn_object(0.45, -0.15))
ok, message = harness.settle_object()
print("Settle result:", ok, message)
if not ok:
    raise SystemExit(1)
PY

echo "=== Starting perception nodes ==="
start_group "$OUT_DIR/detector.log" ros2 run ur5e_pick_place object_detector \
  --ros-args -p use_sim_time:=true || exit 9
DETECTOR_PGID="$STARTED_PGID"
start_group "$OUT_DIR/pos_world.log" ros2 run ur5e_pick_place object_position_world \
  --ros-args -p use_sim_time:=true || exit 9
POSITION_PGID="$STARTED_PGID"
wait_for_command "perceived world position" timeout 5 ros2 topic echo \
  /object_detector/position_world --once || exit 5

echo "=== Running m3_grasp (one Scene-A perception cycle) ==="
MARKER_PREFIX="$OUT_DIR/marker_scene_a"
M3_LOG="$OUT_DIR/m3_grasp.log"
start_group "$M3_LOG" ros2 launch ur5e_pick_place m3_grasp.launch.py \
  gripper_model:=parallel_jaw use_perceived_position:=true require_perception:=true \
  csv_path:="$OUT_DIR/scene_a_perceived_result.csv" \
  marker_file_prefix:="$MARKER_PREFIX" || exit 9
M3_PGID="$STARTED_PGID"

deadline=$((SECONDS + RUN_TIMEOUT_S))
while [[ ! -f "${MARKER_PREFIX}.run_summary_ready" ]]; do
  if ! group_alive "$M3_PGID"; then
    echo "M3_LAUNCH_EXITED_EARLY: launch ended before the terminal marker was written." >&2
    exit 6
  fi
  if (( SECONDS >= deadline )); then
    echo "M3_TIMEOUT: terminal marker not received within ${RUN_TIMEOUT_S}s." >&2
    exit 124
  fi
  sleep 0.2
done

# m3_grasp writes RUN SUMMARY and then the marker. Its launch remains alive
# because static_scene_tf is a long-running sibling, so the marker—not wait(1)
# on ros2 launch—is the completion boundary.
summary="$(grep 'RUN SUMMARY:' "$M3_LOG" | tail -n 1 || true)"
if [[ -z "$summary" ]]; then
  echo "M3_PROTOCOL_ERROR: terminal marker exists but RUN SUMMARY is absent." >&2
  exit 7
fi

echo "$summary"
if [[ "$summary" != *" result=SUCCESS "* ]]; then
  echo "M3_FAILED: m3_grasp completed with a non-success result." >&2
  exit 8
fi

echo "VALIDATION_COMPLETE: m3_grasp succeeded; evidence: $OUT_DIR"
exit 0

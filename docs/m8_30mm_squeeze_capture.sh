#!/usr/bin/env bash
# m8_30mm_squeeze_capture.sh — M8: ONE close-and-hold run of the 30 mm squeeze
# (2026-08-21). Same protocol as the M6 axis-fix validation, retargeted.
#
# WHAT THIS VALIDATES
#   M6 established that the grasp width is DERIVED, not hand-set:
#   scene_xacro_args.resolve_closing_axis() reads grasp.approach_axis and
#   grasp.gripper_roll and selects the object.size[] entry the fingers
#   actually close across. M8 exercises that derivation on the OTHER axis.
#   With gripper_roll = pi/2 = 1.57079632679 rad the gripper closes along
#   object +X, so the squeezed dimension is size[0]=0.030m -- the 30 mm
#   squeeze. Live configuration under test:
#       object.size            = [0.030, 0.045, 0.045] m
#       gripper_roll           = 1.57079632679 rad  (pi/2)
#       closing_axis_object    = (+1, 0, 0)  -> derived axis index 0
#       resolved width         = 0.030 m
#   Static prediction to falsify:
#       fingertip_grasp_theta = 0.538014762810753   (URDF, derived)
#       stall theta           = 0.538015            (zero net pad tilt)
#       expected_grip_angle   = 0.5378679450464813  (grasp_table 0.030 row)
#       within tolerance      = yes                 (0.0235 rad)
#
# PROTOCOL
#   Byte-identical to docs/m6_30mm_traj_capture.sh (itself identical to the
#   45mm baseline capture) except for exactly three things:
#     1. close_and_hold_only:=true  -- the run STOPS after the close/stall.
#        No lift/transport/place/release/retreat. This is the safety gate the
#        objective requires, and it is a node-level stage gate, NOT a process
#        freeze: no SIGSTOP, the node completes its close routine naturally
#        and writes its own RUN SUMMARY.
#     2. contact telemetry captured for both fingertips (the 30mm run's
#        contact logs came from an ad-hoc wrapper that was never committed).
#     3. output prefix m8_30mm_squeeze_* so NOTHING overwrites the historical
#        m6_30mm_* / m6_baseline_* / m7_fullcycle_axisfix_* evidence.
#   No physics parameter, controller gain, effort limit, stall threshold,
#   URDF or mesh is touched by this script.
#
# The width guard checks the DERIVED width, not the deprecated hand-set
# object.grasp_width_axis index -- guarding that index is exactly the mistake
# M6 exists to close out. For M8 the derivation must resolve to size[0]=0.030m.

cd "$(dirname "$0")/.." || exit 2

source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
source scripts/lib/gz_settle.sh
set -u

TS="$(date +%Y%m%d_%H%M%S)_${RANDOM}"
LOGDIR="docs"
SIM_LOG="${LOGDIR}/m8_30mm_squeeze_sim_${TS}.log"
MG_LOG="${LOGDIR}/m8_30mm_squeeze_movegroup_${TS}.log"
GRASP_LOG="${LOGDIR}/m8_30mm_squeeze_grasp_${TS}.log"
STALL_CSV="${LOGDIR}/m8_30mm_squeeze_stalls_${TS}.csv"
STALL_LOG="${LOGDIR}/m8_30mm_squeeze_stallmonitor_${TS}.log"
TRAJ_CSV="runs/m8_30mm_squeeze_traj_${TS}.csv"
TRAJ_LOG="${LOGDIR}/m8_30mm_squeeze_trajrecorder_${TS}.log"
CONTACT_L="${LOGDIR}/m8_30mm_squeeze_contact_left_${TS}.log"
CONTACT_R="${LOGDIR}/m8_30mm_squeeze_contact_right_${TS}.log"
CSV_PATH="m3_grasp_trace_m8_30mm_squeeze_${TS}.csv"

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
sec "0.5 Confirm the DERIVED closing axis (must be object X) and width (0.030)"
python3 - <<'PY' || die "derived-config guard failed -- do not run"
import importlib.util, sys, yaml
def load(n, p):
    s = importlib.util.spec_from_file_location(n, p)
    m = importlib.util.module_from_spec(s); s.loader.exec_module(m); return m
sxa = load("sxa", "config/scene_xacro_args.py")
gg = load("gg", "scripts/lib/gripper_geometry.py")
scene = yaml.safe_load(open("config/scene.yaml"))
r = sxa.resolve_closing_axis(scene)
tg = gg.theta_for_width(r["width_m"])
print(f"  object.size          = {scene['object']['size']}")
print(f"  closing_axis_object  = {tuple(round(v,12) for v in r['closing_axis_object'])}")
print(f"  derived axis index   = {r['axis_index']}  (cross-check {r['configured_axis']})")
print(f"  resolved width       = {r['width_m']} m")
print(f"  theta_grasp          = {tg!r}")
co = r["closing_axis_object"]
print(f"  |dot(closing,X)|     = {abs(co[0]):.15f}")
print(f"  |dot(closing,Y)|     = {abs(co[1]):.15f}")
ok = (list(scene["object"]["size"]) == [0.030, 0.045, 0.045]
      and r["axis_index"] == 0 and abs(r["width_m"] - 0.030) < 1e-12
      and abs(tg - 0.538015) < 5e-7 and abs(abs(co[0]) - 1.0) < 1e-9)
print(f"  GUARD = {'OK' if ok else 'FAIL'}")
sys.exit(0 if ok else 1)
PY

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
sec "1.2 Confirm the SPAWNED gripper carries the derived fingertip angle"
grep -o 'fingertip_grasp_theta[^ ]*' "$SIM_LOG" | head -3
printf '  (see also §0.5 above -- the xacro arg comes from the same helper)\n'

# ---------------------------------------------------------------------------
sec "1.4 Start contact telemetry on both fingertips"
CT_L=$(gz topic -l 2>/dev/null | grep -i "left_finger_tip.*contact" | head -1)
CT_R=$(gz topic -l 2>/dev/null | grep -i "right_finger_tip.*contact" | head -1)
printf '  left  topic = %s\n' "${CT_L:-<none>}"
printf '  right topic = %s\n' "${CT_R:-<none>}"
[[ -z "$CT_L" || -z "$CT_R" ]] && die "contact topics not found -- cannot validate contact geometry"
nohup gz topic -e -t "$CT_L" > "$CONTACT_L" 2>&1 &
CTL_PID=$!
nohup gz topic -e -t "$CT_R" > "$CONTACT_R" 2>&1 &
CTR_PID=$!
printf '  contact echo PIDs: left=%s right=%s\n' "$CTL_PID" "$CTR_PID"

# ---------------------------------------------------------------------------
sec "1.5 Start stall_monitor + joint_trajectory_recorder (whole cycle)"
python3 scripts/stall_monitor.py --out "$STALL_CSV" > "$STALL_LOG" 2>&1 &
STALL_PID=$!
printf '  stall_monitor PID=%s, csv=%s\n' "$STALL_PID" "$STALL_CSV"

python3 scripts/joint_trajectory_recorder.py --out "$TRAJ_CSV" \
  --joint robotiq_85_left_knuckle_joint > "$TRAJ_LOG" 2>&1 &
TRAJ_PID=$!
printf '  joint_trajectory_recorder PID=%s, csv=%s\n' "$TRAJ_PID" "$TRAJ_CSV"
sleep 1

# ---------------------------------------------------------------------------
sec "2. Launch move_group"
nohup ros2 launch ur5e_robotiq_moveit_config move_group.launch.py > "$MG_LOG" 2>&1 &
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
sec "2.5 IK REACHABILITY GATE (compute_ik, before any grasp attempt)"
# The 90-degree wrist roll is a configuration this project has never planned
# for -- scene.yaml's own base-height note records that reachability was
# verified only for the straight-down roll=0 pose. Check it explicitly here
# rather than discovering it as a mid-run planning failure.
# tool0 orientation at roll=pi/2 is a 180-deg rotation about X -> q=(1,0,0,0),
# derived from the same _basis_from_approach_axis() the node uses.
IK_FAIL=0
for Z in 1.024478 0.924478; do
  OUT=$(ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK \
    "{ik_request: {group_name: 'arm', ik_link_name: 'tool0', robot_state: {is_diff: true}, avoid_collisions: true, pose_stamped: {header: {frame_id: 'world'}, pose: {position: {x: 0.45, y: -0.15, z: ${Z}}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}, timeout: {sec: 5, nanosec: 0}}}" 2>&1)
  CODE=$(printf '%s' "$OUT" | grep -o "val=[-0-9]*" | tail -1)
  printf '  tool0 z=%s -> error_code %s %s\n' "$Z" "${CODE:-<none>}" \
    "$([ "$CODE" = "val=1" ] && echo '(SUCCESS)' || echo '(NOT SUCCESS)')"
  [ "$CODE" = "val=1" ] || IK_FAIL=1
done
if [ "$IK_FAIL" -ne 0 ]; then
  kill -TERM "$CTL_PID" "$CTR_PID" "$STALL_PID" "$TRAJ_PID" 2>/dev/null
  kill_sim; pkill -9 -f "move_group" 2>/dev/null; pkill -9 -f "gz topic -e" 2>/dev/null
  die "M8 REACHABILITY FAILURE -- compute_ik found no solution at the 90-degree wrist roll. Not running the squeeze."
fi
printf '  IK REACHABILITY: PASS (both pre-grasp and grasp poses solvable)\n'

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
sec "5. Run m3_grasp, close_and_hold_only:=true"
: > "$GRASP_LOG"
PYTHONUNBUFFERED=1 ros2 launch ur5e_pick_place m3_grasp.launch.py \
  csv_path:="$CSV_PATH" close_and_hold_only:=true \
  > "$GRASP_LOG" 2>&1 &
LAUNCH_PID=$!

tail -n +1 -F "$GRASP_LOG" &
TAIL_PID=$!

DEADLINE=$(( $(date +%s) + 100 ))
RC=1
while true; do
  if grep -q "RUN SUMMARY" "$GRASP_LOG" 2>/dev/null; then
    RC=0
    printf '  RUN SUMMARY seen, tearing down now\n'
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

sleep 1
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
sec "6.5 Stop loggers"
kill -TERM "$CTL_PID" "$CTR_PID" 2>/dev/null
kill -TERM "$STALL_PID" 2>/dev/null; wait "$STALL_PID" 2>/dev/null
kill -TERM "$TRAJ_PID" 2>/dev/null; wait "$TRAJ_PID" 2>/dev/null
echo "--- $STALL_LOG ---"; cat "$STALL_LOG"
echo "--- $TRAJ_LOG ---";  cat "$TRAJ_LOG"

# ---------------------------------------------------------------------------
sec "7. Cleanup"
kill_sim
pkill -9 -f "move_group" 2>/dev/null
pkill -9 -f "gz topic -e" 2>/dev/null

printf '\ndone: %s\n' "$(date -Is)"
printf 'artifacts:\n  sim_log=%s\n  movegroup_log=%s\n  grasp_log=%s\n  stall_csv=%s\n  traj_csv=%s\n  contact_left=%s\n  contact_right=%s\n  grasp_csv=%s\n' \
  "$SIM_LOG" "$MG_LOG" "$GRASP_LOG" "$STALL_CSV" "$TRAJ_CSV" "$CONTACT_L" "$CONTACT_R" "$CSV_PATH"

exit "$RC"

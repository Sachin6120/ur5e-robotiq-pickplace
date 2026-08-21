#!/usr/bin/env bash
# m7_fullcycle_axisfix_capture.sh — M7: ONE controlled FULL pick-and-place
# cycle on the M6-corrected configuration (2026-08-21).
#
# WHAT THIS VALIDATES
#   scene.yaml's object.grasp_width_axis used to be a hand-set index naming
#   object.size[0]=0.030m. The gripper physically closes along object/world
#   +Y, so the squeezed dimension is size[1]=0.045m. The width is now DERIVED
#   (scene_xacro_args.resolve_closing_axis()). Static prediction to falsify:
#       fingertip_grasp_theta = 0.402892701551987   (URDF, derived)
#       stall theta           = 0.402893            (zero net pad tilt)
#       expected_grip_angle   = 0.405532            (grasp_table 0.045 row)
#       within tolerance      = yes                 (0.0235 rad)
#
# PROTOCOL
#   Byte-identical to docs/m6_30mm_traj_capture.sh (itself identical to the
#   45mm baseline capture) except for exactly three things:
#     1. close_and_hold_only is NOT passed (defaults false) -- the node runs
#        the COMPLETE cycle: pre-grasp, descent, grasp, lift, transport,
#        place, release, retreat. M6's close-and-hold gate is deliberately
#        off; M7 is the dynamic test that gate was suppressing.
#     2. contact telemetry captured for both fingertips (the 30mm run's
#        contact logs came from an ad-hoc wrapper that was never committed).
#     3. continuous Gazebo pose telemetry for the whole cycle, so slip can be
#        measured as a TRAJECTORY (slip.py's flange-relative definition) and
#        not just at two settled endpoints.
#     4. output prefix m7_fullcycle_axisfix_* so NOTHING overwrites the
#        m6_30mm_* / m6_baseline_* / m6_axisfix_* evidence.
#   No physics parameter, controller gain, effort limit, stall threshold,
#   URDF or mesh is touched by this script.
#
# The width guard checks the DERIVED width, not size[0] -- guarding the old
# hand-set index is exactly the mistake this run exists to close out.

cd "$(dirname "$0")/.." || exit 2

source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
source scripts/lib/gz_settle.sh
set -u

TS="$(date +%Y%m%d_%H%M%S)_${RANDOM}"
LOGDIR="docs"
SIM_LOG="${LOGDIR}/m7_fullcycle_axisfix_sim_${TS}.log"
MG_LOG="${LOGDIR}/m7_fullcycle_axisfix_movegroup_${TS}.log"
GRASP_LOG="${LOGDIR}/m7_fullcycle_axisfix_grasp_${TS}.log"
STALL_CSV="${LOGDIR}/m7_fullcycle_axisfix_stalls_${TS}.csv"
STALL_LOG="${LOGDIR}/m7_fullcycle_axisfix_stallmonitor_${TS}.log"
TRAJ_CSV="runs/m7_fullcycle_axisfix_traj_${TS}.csv"
TRAJ_LOG="${LOGDIR}/m7_fullcycle_axisfix_trajrecorder_${TS}.log"
CONTACT_L="${LOGDIR}/m7_fullcycle_axisfix_contact_left_${TS}.log"
CONTACT_R="${LOGDIR}/m7_fullcycle_axisfix_contact_right_${TS}.log"
POSE_LOG="${LOGDIR}/m7_fullcycle_axisfix_pose_${TS}.log"
CSV_PATH="m3_grasp_trace_m7_fullcycle_axisfix_${TS}.csv"

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
sec "0.5 Confirm the DERIVED closing axis and width"
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
ok = (list(scene["object"]["size"]) == [0.030, 0.045, 0.045]
      and r["axis_index"] == 1 and abs(r["width_m"] - 0.045) < 1e-12
      and abs(tg - 0.402893) < 5e-7)
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

POSE_TOPIC="/world/empty/pose/info"
nohup gz topic -e -t "$POSE_TOPIC" > "$POSE_LOG" 2>&1 &
POSE_PID=$!
printf '  pose echo PID=%s topic=%s -> %s\n' "$POSE_PID" "$POSE_TOPIC" "$POSE_LOG"

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
sec "5. Run m3_grasp — FULL CYCLE (close_and_hold_only defaults false)"
: > "$GRASP_LOG"
PYTHONUNBUFFERED=1 ros2 launch ur5e_pick_place m3_grasp.launch.py \
  csv_path:="$CSV_PATH" \
  > "$GRASP_LOG" 2>&1 &
LAUNCH_PID=$!

tail -n +1 -F "$GRASP_LOG" &
TAIL_PID=$!

DEADLINE=$(( $(date +%s) + 180 ))
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
    printf '  RUN SUMMARY never appeared within the 180s bound -- treating as a hang\n'
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
kill -TERM "$CTL_PID" "$CTR_PID" "$POSE_PID" 2>/dev/null
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
printf 'artifacts:\n  sim_log=%s\n  movegroup_log=%s\n  grasp_log=%s\n  stall_csv=%s\n  traj_csv=%s\n  contact_left=%s\n  contact_right=%s\n  pose_log=%s\n  grasp_csv=%s\n' \
  "$SIM_LOG" "$MG_LOG" "$GRASP_LOG" "$STALL_CSV" "$TRAJ_CSV" "$CONTACT_L" "$CONTACT_R" "$POSE_LOG" "$CSV_PATH"

exit "$RC"

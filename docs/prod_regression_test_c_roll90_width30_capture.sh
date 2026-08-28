#!/usr/bin/env bash
# prod_reg_test_c_roll90_width30_capture.sh — M8: ONE controlled FULL pick-and-place cycle
# on the 30 mm squeeze configuration (2026-08-21).
#
# WHAT THIS VALIDATES
#   M8's close-and-hold run established that gripper_roll = pi/2 rotates the
#   DERIVED closing axis onto object +X, so the squeezed dimension is
#   size[0]=0.030m, and that the resulting grasp is geometrically correct:
#   bilateral contact on the object's +-X faces at X=0.465 / X=0.435 m,
#   theta_achieved=0.538203 rad (+0.188 mrad vs derived prediction).
#   That run stopped at the hold. THIS run removes that gate and tests
#   whether the 30 mm grasp survives DYNAMIC load: lift, transport, place,
#   release, retreat.
#       object.size            = [0.030, 0.045, 0.045] m
#       gripper_roll           = 1.57079632679 rad  (pi/2)
#       closing_axis_object    = (+1, 0, 0)  -> derived axis index 0
#       resolved width         = 0.030 m
#       fingertip_grasp_theta  = 0.538014762810753   (URDF, derived)
#       expected_grip_angle    = 0.5378679450464813  (grasp_table 0.030 row)
#       tolerance              = 0.0235 rad
#
# PROTOCOL
#   Same structure as docs/m7_fullcycle_axisfix_capture.sh, retargeted:
#     1. close_and_hold_only is NOT passed (defaults false) -- the node runs
#        the COMPLETE cycle. This is the gate M8's first run held shut.
#     2. contact telemetry on both fingertips, for the whole cycle.
#     3. pose telemetry as a FILTERED CSV (pick_target + wrist_3_link) rather
#        than a raw echo of every link. slip.py's definition is
#        flange-relative, so those two entities are exactly what a slip
#        TRAJECTORY needs; the raw echo M7 used was 35 MB of mostly visuals.
#     4. an IK reachability gate covering the PLACE side as well as the pick
#        side. The pi/2 wrist roll was proven reachable at the pick pose by
#        M8's close-and-hold run, but nothing has ever planned to the place
#        pose (y=+0.200) at this roll. Checked before any Gazebo work rather
#        than discovered as a mid-cycle planning abort.
#     5. output prefix prod_reg_test_c_roll90_width30_* so NOTHING overwrites the
#        m6_* / m7_fullcycle_axisfix_* / m8_30mm_squeeze_* evidence.
#   No physics parameter, controller gain, effort limit, stall threshold,
#   tcp_offset, URDF or mesh is touched by this script.
#
# The width guard checks the DERIVED width, not the deprecated hand-set
# object.grasp_width_axis index. For M8 the derivation must resolve to
# size[0]=0.030m.

cd "$(dirname "$0")/.." || exit 2

source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
source scripts/lib/gz_settle.sh
set -u

TS="$(date +%Y%m%d_%H%M%S)_${RANDOM}"
LOGDIR="docs"
SIM_LOG="${LOGDIR}/prod_reg_test_c_roll90_width30_sim_${TS}.log"
MG_LOG="${LOGDIR}/prod_reg_test_c_roll90_width30_movegroup_${TS}.log"
GRASP_LOG="${LOGDIR}/prod_reg_test_c_roll90_width30_grasp_${TS}.log"
STALL_CSV="${LOGDIR}/prod_reg_test_c_roll90_width30_stalls_${TS}.csv"
STALL_LOG="${LOGDIR}/prod_reg_test_c_roll90_width30_stallmonitor_${TS}.log"
TRAJ_CSV="runs/prod_reg_test_c_roll90_width30_traj_${TS}.csv"
TRAJ_LOG="${LOGDIR}/prod_reg_test_c_roll90_width30_trajrecorder_${TS}.log"
CONTACT_L="${LOGDIR}/prod_reg_test_c_roll90_width30_contact_left_${TS}.log"
CONTACT_R="${LOGDIR}/prod_reg_test_c_roll90_width30_contact_right_${TS}.log"
POSE_CSV="runs/prod_reg_test_c_roll90_width30_pose_${TS}.csv"
POSE_LOG="${LOGDIR}/prod_reg_test_c_roll90_width30_poserecorder_${TS}.log"
MARKER_DIR="${LOGDIR}/prod_reg_test_c_roll90_width30_markers_${TS}"
CSV_PATH="m3_grasp_trace_prod_reg_test_c_roll90_width30_${TS}.csv"

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
mkdir -p "$MARKER_DIR"

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
rel = gg.theta_for_width(r["width_m"] + 0.010)   # RELEASE_CLEARANCE_M, launch file
print(f"  object.size          = {scene['object']['size']}")
print(f"  gripper_roll         = {scene['grasp']['gripper_roll']}")
print(f"  closing_axis_object  = {tuple(round(v,12) for v in r['closing_axis_object'])}")
print(f"  derived axis index   = {r['axis_index']}  (cross-check {r['configured_axis']})")
print(f"  resolved width       = {r['width_m']} m")
print(f"  theta_grasp          = {tg!r}")
print(f"  theta_release (+10mm)= {rel!r}")
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
sec "1.45 Start ground-truth pose recorder (pick_target + wrist_3_link)"
# Filtered CSV, not a raw echo. Gazebo's own pose topic, never TF -- slip.py's
# rule since M0. wrist_3_link is the flange at the physics level: tool0,
# flange, ur_to_robotiq_link and robotiq_85_base_link are all fixed-joint
# lumped into it, and every joint in that chain has zero translation, so its
# world position IS the flange's.
nohup python3 - "$POSE_CSV" <<'PY' > "$POSE_LOG" 2>&1 &
import importlib.util, subprocess, sys, time, csv
spec = importlib.util.spec_from_file_location("sp", "scripts/lib/sample_pose.py")
sp = importlib.util.module_from_spec(spec); spec.loader.exec_module(sp)
OUT = sys.argv[1]
WANT = ("pick_target", "wrist_3_link")
fh = open(OUT, "w", newline="", buffering=1)
w = csv.writer(fh)
w.writerow(["wall_t", "entity", "x", "y", "z", "qx", "qy", "qz", "qw"])
proc = subprocess.Popen(["gz", "topic", "-e", "-t", "/world/empty/pose/info"],
                        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
n = 0
try:
    for lines in sp.stream(proc.stdout):
        poses = sp.parse_pose_v(lines)
        t = time.time()
        for want in WANT:
            # exact name only: 'wrist_3_link' must not also match
            # 'wrist_3_link_visual' or the lumped visual entities
            if want in poses:
                p = poses[want]
                w.writerow([f"{t:.6f}", want] + [f"{v:.9f}" for v in p])
                n += 1
finally:
    proc.terminate(); fh.close()
    print(f"pose recorder wrote {n} rows to {OUT}")
PY
POSE_PID=$!
printf '  pose recorder PID=%s, csv=%s\n' "$POSE_PID" "$POSE_CSV"

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
sec "2.5 IK REACHABILITY GATE — pick side AND place side, at roll=pi/2"
# tool0 orientation at roll=pi/2 is a 180-deg rotation about X -> q=(1,0,0,0),
# from the same _basis_from_approach_axis() the node uses. z values:
#   0.924478 = grasp/place height  (object centre 0.7725 + corrected 0.151978)
#   1.024478 = pre-grasp standoff  (+0.100)
#   1.044478 = lift/transport      (+0.120 retreat)
IK_FAIL=0
ik_check() {  # $1=x $2=y $3=z $4=label
  local OUT CODE
  OUT=$(ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK \
    "{ik_request: {group_name: 'arm', ik_link_name: 'tool0', robot_state: {is_diff: true}, avoid_collisions: true, pose_stamped: {header: {frame_id: 'world'}, pose: {position: {x: $1, y: $2, z: $3}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}, timeout: {sec: 5, nanosec: 0}}}" 2>&1)
  CODE=$(printf '%s' "$OUT" | grep -o "val=[-0-9]*" | tail -1)
  printf '  %-26s (%.3f, %+.3f, %.6f) -> %s %s\n' "$4" "$1" "$2" "$3" "${CODE:-<none>}" \
    "$([ "$CODE" = "val=1" ] && echo '(SUCCESS)' || echo '(NOT SUCCESS)')"
  [ "$CODE" = "val=1" ] || IK_FAIL=1
}
ik_check 0.45 -0.15 1.024478 "pick pre-grasp"
ik_check 0.45 -0.15 0.924478 "pick grasp"
ik_check 0.45 -0.15 1.044478 "pick lift"
ik_check 0.45  0.20 1.044478 "place transport"
ik_check 0.45  0.20 0.924478 "place descend"
if [ "$IK_FAIL" -ne 0 ]; then
  kill -TERM "$CTL_PID" "$CTR_PID" "$POSE_PID" "$STALL_PID" "$TRAJ_PID" 2>/dev/null
  kill_sim; pkill -9 -f "move_group" 2>/dev/null; pkill -9 -f "gz topic -e" 2>/dev/null
  die "M8 REACHABILITY FAILURE -- compute_ik found no solution at the 90-degree wrist roll. Not running the cycle."
fi
printf '  IK REACHABILITY: PASS (all five cycle waypoints solvable)\n'

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
  csv_path:="$CSV_PATH" marker_file_prefix:="${MARKER_DIR}/stage" \
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

sleep 2
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
echo "--- $STALL_LOG ---"; tail -3 "$STALL_LOG"
echo "--- $TRAJ_LOG ---";  cat "$TRAJ_LOG"
echo "--- $POSE_LOG ---";  cat "$POSE_LOG"
echo "--- stage markers ---"; ls -1 "$MARKER_DIR" 2>/dev/null

# ---------------------------------------------------------------------------
sec "7. Cleanup"
kill_sim
pkill -9 -f "move_group" 2>/dev/null
pkill -9 -f "gz topic -e" 2>/dev/null

printf '\ndone: %s\n' "$(date -Is)"
printf 'artifacts:\n  sim_log=%s\n  movegroup_log=%s\n  grasp_log=%s\n  stall_csv=%s\n  traj_csv=%s\n  contact_left=%s\n  contact_right=%s\n  pose_csv=%s\n  markers=%s\n  grasp_csv=%s\n' \
  "$SIM_LOG" "$MG_LOG" "$GRASP_LOG" "$STALL_CSV" "$TRAJ_CSV" "$CONTACT_L" "$CONTACT_R" "$POSE_CSV" "$MARKER_DIR" "$CSV_PATH"

exit "$RC"

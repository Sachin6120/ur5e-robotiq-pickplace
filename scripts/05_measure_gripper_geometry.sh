#!/usr/bin/env bash
# 05_measure_gripper_geometry.sh — sweep the actuated joint, measure two things
#                                  from the SAME fingertip positions at each
#                                  sample: opening width and tcp_offset.
#
# WHY ONE SCRIPT PRODUCES BOTH
#   gripper.width_map (joint_rad -> width_m) and grasp.tcp_offset (a distance
#   along tool0's approach axis) were treated as independent scene.yaml fields,
#   but they are not independent measurements — they come from the same pair
#   of fingertip pad positions at the same joint angle. Deriving them
#   separately risks them drifting apart if object.size or the actuated
#   joint's range ever changes and only one field gets re-measured.
#
#   Concretely: tcp_offset was measured once at a single object-adjacent
#   aperture (0.4 rad) as a placeholder. That number is already known to be
#   wrong by up to 13.6mm at other apertures — see
#   docs/M-1_reference_report.md §6 item 5. Once width_map exists, tcp_offset
#   should be READ from this same table at the aperture width_map computes for
#   the real object, not carried as a separately-measured scalar.
#
# METHODOLOGY (same discipline as M0-C and the original tcp_offset measurement)
#   Ground truth is Gazebo's own pose topic, not TF — this should be measured
#   independent of what MoveIt/robot_state_publisher believes, not assumed to
#   agree with it.
#
#   tool0 itself never appears in /world/<world>/pose/info. It, flange,
#   robotiq_85_base_link and ur_to_robotiq_link are all fixed-joint-lumped
#   into wrist_3_link at the physics level (visible as
#   "wrist_3_link_fixed_joint_lump__..." entries in the same topic). Since
#   every joint in that chain has zero translation (confirmed by recon),
#   tool0's world POSITION equals wrist_3_link's; only orientation differs, by
#   two known fixed rotations (wrist_3_link -> flange -> tool0). This script
#   reconstructs tool0's pose from wrist_3_link's rather than assuming a
#   simpler shortcut exists.
#
#   width  = straight-line distance between the two fingertip link centers.
#   offset = (fingertip_midpoint - tool0_position) projected onto tool0's
#            local Z ("front", ROS-Industrial tool0 convention).
#   perp   = residual component NOT along that axis — printed as a sanity
#            check. It stayed ~2e-7 m across the whole range when this was
#            first measured; if a future run shows it growing, the lateral-
#            symmetry assumption (not just the constant-reach one) has also
#            broken, and that is worth knowing explicitly rather than average
#            away.
#
# PRECONDITION
#   The merged sim is already up in another terminal, gripper_controller active.
#
# USAGE
#   bash scripts/05_measure_gripper_geometry.sh 2>&1 | \
#     tee docs/gripper_geometry_$(date +%Y%m%d_%H%M%S).log

set -u

GRIPPER_CTRL="${GRIPPER_CTRL:-gripper_controller}"
WORLD="${WORLD:-empty}"
MODEL="${MODEL:-ur5e_robotiq}"
GZ_POSE_TOPIC="/world/${WORLD}/pose/info"

# Sweep points. 0.0 and near the mechanical limit (0.767, not the nominal 0.8,
# since the URDF's ros2_control initial_value never quite reaches 0.8) are
# included as endpoints; the rest are even spacing across the range.
SAMPLES="${SAMPLES:-0.0 0.2 0.4 0.6 0.767}"

hr()   { printf '\n%s\n' "════════════════════════════════════════════════════════════"; }
sec()  { hr; printf '§ %s\n' "$1"; hr; }
note() { printf '  [note] %s\n' "$1"; }

printf 'gripper geometry sweep: %s\n' "$(date -Is)"
printf 'samples: %s\n' "$SAMPLES"

sec "0. Preconditions"
TOPICS=$(gz topic -l 2>/dev/null || true)
if ! printf '%s\n' "$TOPICS" | grep -qF "$GZ_POSE_TOPIC"; then
  printf '  [STOP] pose topic missing: %s — is the sim running?\n' "$GZ_POSE_TOPIC"
  exit 2
fi
printf '  [ok] %s present\n' "$GZ_POSE_TOPIC"

sec "1. Sweep"
RESULTS_FILE="${RESULTS_FILE:-/tmp/gripper_geometry_samples.txt}"
: > "$RESULTS_FILE"

for target in $SAMPLES; do
  ros2 action send_goal "/${GRIPPER_CTRL}/gripper_cmd" \
    control_msgs/action/GripperCommand \
    "{command: {position: ${target}, max_effort: 50.0}}" >/dev/null 2>&1
  sleep 1.5

  gz topic -e -t "$GZ_POSE_TOPIC" -n 1 2>/dev/null > "/tmp/geom_pose_${target}.txt"

  python3 - "/tmp/geom_pose_${target}.txt" "$target" >> "$RESULTS_FILE" <<'PY'
import sys, re, math

path, target = sys.argv[1], sys.argv[2]
txt = open(path).read()

def get_link(name):
    m = re.search(
        r'name:\s*"' + re.escape(name) + r'"\s*\n\s*id:.*?position\s*\{(.*?)\}\s*orientation\s*\{(.*?)\}',
        txt, re.S)
    if not m:
        return None
    pos_txt, ori_txt = m.groups()
    pos = {k: float(v) for k, v in re.findall(r'([xyz]):\s*(-?[\d.eE+-]+)', pos_txt)}
    ori = {k: float(v) for k, v in re.findall(r'([xyzw]):\s*(-?[\d.eE+-]+)', ori_txt)}
    p = (pos.get('x', 0.0), pos.get('y', 0.0), pos.get('z', 0.0))
    q = (ori.get('x', 0.0), ori.get('y', 0.0), ori.get('z', 0.0), ori.get('w', 1.0))
    return p, q

def quat_to_R(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/n, y/n, z/n, w/n
    return [
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ]

def R_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = [[1,0,0],[0,cr,-sr],[0,sr,cr]]
    Ry = [[cp,0,sp],[0,1,0],[-sp,0,cp]]
    Rz = [[cy,-sy,0],[sy,cy,0],[0,0,1]]
    return matmul(matmul(Rz, Ry), Rx)

def matmul(A, B):
    return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

def matvec(A, v):
    return tuple(sum(A[i][k]*v[k] for k in range(3)) for i in range(3))

def sub(a, b): return tuple(a[i]-b[i] for i in range(3))
def dot(a, b): return sum(a[i]*b[i] for i in range(3))
def norm(a): return math.sqrt(dot(a, a))

wrist3 = get_link("wrist_3_link")
left   = get_link("robotiq_85_left_finger_tip_link")
right  = get_link("robotiq_85_right_finger_tip_link")

if not (wrist3 and left and right):
    print(f"{target} ERROR missing_link")
    sys.exit(0)

wrist3_pos, wrist3_q = wrist3
left_pos, _ = left
right_pos, _ = right

R_wrist3 = quat_to_R(wrist3_q)
# Fixed chain confirmed by recon, zero translation throughout:
# wrist_3_link -> flange (rpy 0,-pi/2,-pi/2) -> tool0 (rpy pi/2,0,pi/2).
R_tool0 = matmul(matmul(R_wrist3, R_from_rpy(0, -math.pi/2, -math.pi/2)),
                 R_from_rpy(math.pi/2, 0, math.pi/2))
tool0_pos = wrist3_pos
tool0_z_world = matvec(R_tool0, (0, 0, 1))

mid = tuple((left_pos[i] + right_pos[i]) / 2.0 for i in range(3))
vec = sub(mid, tool0_pos)
offset = dot(vec, tool0_z_world)
perp = norm(sub(vec, tuple(offset * c for c in tool0_z_world)))
width = norm(sub(left_pos, right_pos))

print(f"{target} {width:.6f} {offset:.6f} {perp:.2e}")
PY
done

sec "2. Results  (joint_rad, width_m, tcp_offset_m, perp_residual_m)"
column -t "$RESULTS_FILE"

sec "3. What to do with this"
cat <<'EOF'
  This is raw measurement, not a judged pass/fail. To use it:

  1. gripper.width_map in scene.yaml: pair each (joint_rad, width_m) here as
     [[joint_rad, width_m], ...].
  2. grasp.tcp_offset: for a SPECIFIC object.size[grasp_width_axis], solve
     width_map for the joint angle that width corresponds to, then read
     tcp_offset at that angle from this same table (interpolate between
     samples if it falls between them) — not a value read off by eye for a
     "close enough" object width.
  3. If the perp_residual column grows beyond ~1e-5 m at any sample, the
     lateral-symmetry assumption has broken too, not just constant-reach.
     That would mean grasp_tcp needs an X/Y correction as well as the Z
     (tcp_offset) one — worth flagging loudly, not averaging away.
EOF

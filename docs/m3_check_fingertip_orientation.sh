#!/usr/bin/env bash
# m3_check_fingertip_orientation.sh -- the decisive, model-free check: command
# the gripper open (theta ~ 0), settle, and read left_finger_tip_link's
# orientation RELATIVE to left_finger_link directly from Gazebo's own
# simulated rigid-body pose (not TF -- TF for a fixed joint is 100%
# determined by the URDF and would just circularly echo back whatever was
# typed there; Gazebo's pose goes through libsdformat's URDF->SDF conversion
# and DART's actual body simulation, an independent code path).
#
# Since left_finger_joint (finger_link's own parent joint) is ALSO fixed,
# and left_finger_tip_joint (finger_tip_link's parent) is fixed at
# rpy="0 fingertip_grasp_theta 0", the relative rotation between
# left_finger_link and left_finger_tip_link is EXACTLY that fixed joint's
# own rotation -- regardless of what theta (the master's angle) actually is,
# regardless of arm pose, regardless of gripper mounting rotation. No need to
# actually command theta=0 for correctness; done anyway for a clean read.
cd "$(dirname "$0")/.." || exit 2

source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
source scripts/lib/gz_settle.sh
set -u

LOG="docs/m3_orient_check_$(date +%Y%m%d_%H%M%S).log"

kill_sim
pkill -9 -f "move_group" 2>/dev/null
sleep 1

nohup ros2 launch ur5e_robotiq_description ur5e_robotiq_sim_control.launch.py \
  > "$LOG" 2>&1 &
SIM_PID=$!
printf 'sim launch PID=%s, log=%s\n' "$SIM_PID" "$LOG"

if ! gz_wait_controller_active_bounded "gripper_controller" 30; then
  printf 'gripper_controller NEVER WENT ACTIVE -- see %s\n' "$LOG"
  kill_sim
  exit 1
fi
sleep 2

printf -- '--- gz topic list, pose-related ---\n'
timeout 5 gz topic -l | grep -i pose

printf -- '--- commanding gripper open (theta=0) via action ---\n'
timeout 15 ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 0.0}}" \
  --feedback > /tmp/gripper_open_result.txt 2>&1
tail -5 /tmp/gripper_open_result.txt
sleep 1

printf -- '--- one pose/info message, extracting finger_link/finger_tip_link ---\n'
printf -- '(dynamic_pose/info only carries links below a non-fixed joint --\n'
printf -- ' left_finger_link/left_finger_tip_link are both fixed now, so use\n'
printf -- ' pose/info, the full static+dynamic scene graph, instead)\n'
timeout 8 gz topic -e -t /world/empty/pose/info -n 1 > /tmp/m3_pose_msg.txt 2>&1
grep -c "^pose" /tmp/m3_pose_msg.txt || true
python3 - /tmp/m3_pose_msg.txt << 'PY'
import sys, re

path = sys.argv[1]
text = open(path).read()
# Split on 'pose {' blocks and extract name + orientation quaternion (x,y,z,w)
blocks = re.split(r'\npose \{', text)
found = {}
for b in blocks:
    m_name = re.search(r'name:\s*"([^"]+)"', b)
    if not m_name:
        continue
    name = m_name.group(1)
    if 'left_finger_link' not in name and 'left_finger_tip_link' not in name:
        continue
    orient = re.search(r'orientation \{([^}]*)\}', b)
    if not orient:
        continue
    ob = orient.group(1)
    def getf(tag, default=0.0):
        m = re.search(rf'{tag}:\s*(-?[\d.eE+-]+)', ob)
        return float(m.group(1)) if m else default
    qx, qy, qz, qw = getf('x'), getf('y'), getf('z'), getf('w', 1.0)
    found[name] = (qx, qy, qz, qw)

for k, v in found.items():
    print(f"{k}: quat={v}")

if len(found) < 2:
    print("DID NOT FIND BOTH LINKS -- check the link names above / topic content")
    sys.exit(1)

import math
def quat_conj(q):
    x,y,z,w = q
    return (-x,-y,-z,w)
def quat_mul(q1, q2):
    x1,y1,z1,w1 = q1
    x2,y2,z2,w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )

parent_key = [k for k in found if 'left_finger_link' in k and 'tip' not in k][0]
child_key = [k for k in found if 'left_finger_tip_link' in k][0]
q_parent = found[parent_key]
q_child = found[child_key]

q_rel = quat_mul(quat_conj(q_parent), q_child)
x,y,z,w = q_rel
# pitch (rotation about Y) from quaternion, standard formula
pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
print(f"\nrelative quat (finger_link -> finger_tip_link): {q_rel}")
print(f"pitch (deg): {math.degrees(pitch):.3f}")
print(f"fingertip_grasp_theta (deg): {math.degrees(0.402892701551987):.3f}")
PY

kill_sim
pkill -9 -f "move_group" 2>/dev/null

#!/usr/bin/env bash
# 08_spawn_pick_object.sh — spawn (or remove) config/scene.yaml's object:
# block at object.pick_pose, for M3 grasp-node test runs.
#
# WHY THIS EXISTS
#   m3_grasp.launch.py deliberately does not spawn a test object itself —
#   that's a harness concern, per docs/HANDOFF_M3.md's M3 grasp node
#   section. Test 2's first live run spawned pick_target ad hoc and found
#   the world had no table under it; the table gap is now fixed by
#   ur5e_robotiq_sim_control.launch.py's own table spawn (see
#   config/scene_table_sdf.py). This script is the object-side half: derive
#   the object SDF from scene.yaml's object: block, same single-source-of-
#   truth discipline as everything else in this project, rather than typing
#   pick_pose/size/mass into a one-off gz service call by hand.
#
# PRECONDITION
#   The merged sim is already up (ur5e_robotiq_sim_control.launch.py),
#   which now includes the table.
#
# USAGE
#   bash scripts/08_spawn_pick_object.sh            # spawn at object.pick_pose
#   REMOVE=1 bash scripts/08_spawn_pick_object.sh    # remove it again

set -u

source "$(dirname "$0")/lib/gz_settle.sh"

WORLD="${WORLD:-empty}"
REMOVE="${REMOVE:-0}"

hr()   { printf '\n%s\n' "════════════════════════════════════════════════════════════"; }
sec()  { hr; printf '§ %s\n' "$1"; hr; }
die()  { printf '  [STOP] %s\n' "$1"; exit 2; }

GZ_POSE="/world/${WORLD}/pose/info"
GZ_CREATE="/world/${WORLD}/create"
GZ_REMOVE="/world/${WORLD}/remove"

# ---------------------------------------------------------------------------
sec "0. Read object: block from scene.yaml"
read -r OBJ_NAME OBJ_SX OBJ_SY OBJ_SZ OBJ_MASS OBJ_MU PX PY PZ ROLL PITCH YAW <<EOF
$(python3 -c "
import yaml
scene = yaml.safe_load(open('config/scene.yaml'))
obj = scene['object']
p = obj['pick_pose']
s = obj['surface']
print(obj['name'], *obj['size'], obj['mass'], s['mu'],
      p['x'], p['y'], p['z'], p['roll'], p['pitch'], p['yaw'])
")
EOF
printf '  object=%s size=%s %s %s m mass=%s kg mu=%s\n' \
       "$OBJ_NAME" "$OBJ_SX" "$OBJ_SY" "$OBJ_SZ" "$OBJ_MASS" "$OBJ_MU"
printf '  pick_pose=%s %s %s  rpy=%s %s %s\n' "$PX" "$PY" "$PZ" "$ROLL" "$PITCH" "$YAW"

if [[ "$REMOVE" == "1" ]]; then
  sec "1. Remove ${OBJ_NAME}"
  gz service -s "$GZ_REMOVE" --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
    --timeout 5000 --req "name: \"${OBJ_NAME}\", type: MODEL"
  exit 0
fi

# ---------------------------------------------------------------------------
sec "1. Spawn ${OBJ_NAME} at pick_pose"
SDF="<?xml version='1.0'?><sdf version='1.9'><model name='${OBJ_NAME}'>
<pose>${PX} ${PY} ${PZ} ${ROLL} ${PITCH} ${YAW}</pose>
<link name='link'>
<inertial><mass>${OBJ_MASS}</mass>
  <inertia><ixx>1e-4</ixx><iyy>1e-4</iyy><izz>1e-4</izz>
           <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
</inertial>
<collision name='c'><geometry><box><size>${OBJ_SX} ${OBJ_SY} ${OBJ_SZ}</size></box></geometry>
  <surface><friction><ode><mu>${OBJ_MU}</mu><mu2>${OBJ_MU}</mu2></ode></friction></surface>
</collision>
<visual name='v'><geometry><box><size>${OBJ_SX} ${OBJ_SY} ${OBJ_SZ}</size></box></geometry>
  <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>1 1 1 1</specular></material>
</visual>
</link></model></sdf>"

# protobuf text-format cannot have a string literal cross a physical line
# boundary — flatten before --req, same hazard 04/06/m0_verify.sh document.
SDF_FLAT="${SDF//$'\n'/ }"
CREATE_OUT=$(gz service -s "$GZ_CREATE" \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 5000 --req "sdf: \"${SDF_FLAT//\"/\\\"}\", name: \"${OBJ_NAME}\"" 2>&1)
printf '%s\n' "$CREATE_OUT" | sed 's|^|    |'
if printf '%s\n' "$CREATE_OUT" | grep -qi "error\|cannot cross line"; then
  die "object spawn request was rejected — see the error above."
fi

sec "2. Settle onto the table (gravity only, not contact-loaded)"
if ! gz_settle_pose "$GZ_POSE" 0.0005 5.0 0.15 "$OBJ_NAME"; then
  die "object did not settle after spawn — check the table is actually present (ur5e_robotiq_sim_control.launch.py's table spawn) before trusting anything downstream."
fi

python3 -c "
import re, sys
raw = sys.stdin.read()
blocks = re.findall(r'name:\s*\"${OBJ_NAME}\"(.*?)position\s*\{(.*?)\}', raw, re.S)
if blocks:
    xyz = dict(re.findall(r'([xyz]):\s*(-?[\d.eE+-]+)', blocks[-1][1]))
    print(f\"  settled z = {xyz.get('z', '?')} (expected table.surface_z + size.z/2 = {${PZ}:.5f})\")
" < <(gz topic -e -t "$GZ_POSE" -n 1 2>/dev/null)

printf '\ndone: %s\n' "$(date -Is)"

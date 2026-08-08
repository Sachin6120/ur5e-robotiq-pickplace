#!/usr/bin/env bash
# 06_measure_grasp_table.sh — one contact experiment per object width,
#                              measuring grip angle, tcp_offset-at-grip, and
#                              pad-centre offset from the SAME closed-state
#                              fingertip/box samples.
#
# WHY THIS SCRIPT, SEPARATE FROM 05
#   05_measure_gripper_geometry.sh sweeps COMMANDED joint angle in free space
#   (no object) and measures clearance width + tcp_offset at each angle —
#   purely geometric, no compliance, no contact.
#
#   grip width and pad-centre offset are NOT the same kind of quantity: they
#   only exist once an object is actually caught between the pads and the
#   mechanism has stalled against it. The URDF confirms there's no shortcut
#   around this — the fingertip <collision> geometry is a raw STL mesh
#   (left_finger_tip.stl / right_finger_tip.stl) with no <origin> offset to
#   read out, so the true contact surface isn't a queryable scalar; it's
#   whatever the solver resolves when mesh meets object. That has to be
#   measured, not derived from the URDF.
#
#   So this script is indexed by OBJECT WIDTH, not commanded angle, and
#   spawns a real object for every sample. clearance_width for a given
#   object width is a SEPARATE LOOKUP against 05's already-existing
#   free-space table — this script does not recompute or duplicate it.
#
# WHAT EACH ROW MEASURES (one contact experiment, three numbers from it)
#   grip_angle_rad       — achieved master position at stall. The empirical,
#                           compliance-included counterpart to clearance.
#   tcp_offset_m          — SAME computation 05 uses (fingertip midpoint,
#                           projected onto tool0's local Z, relative to
#                           tool0 reconstructed from wrist_3_link), just fed
#                           the CONTACT-LOADED fingertip poses at stall
#                           instead of free-space ones. Literally the same
#                           measurement 05 takes, viewed at a different
#                           (empirical, not commanded) aperture.
#   pad_centre_offset_m   — vertical drop of the object from its spawn/rest
#                           height to its settled height after overclose.
#                           Per the Blocker-2 falsification test
#                           (docs/HANDOFF_M3.md), this drop is geometric
#                           seating, not friction: the object falls from the
#                           fingertip-link-origin height (where it was
#                           placed and where tcp_offset/grasp_tcp are
#                           currently anchored) down to the true pad centre.
#
#   Five widths means five (grip_angle, pad_centre_offset) pairs — enough to
#   actually check whether pad-centre offset correlates with aperture (the
#   hypothesis floated after the 2-point ad hoc check in Blocker 2's
#   investigation came back inconclusive), instead of guessing from two
#   near-duplicate stall angles.
#
# TIMEOUT DISCIPLINE (Blocker 2's action item, applied here from the start)
#   The overclose action call is bounded by gripper.command_timeout_s from
#   scene.yaml (falls back to 5.0s if absent) via `timeout`, not left to
#   hang the way the bare `ros2 action send_goal` in 04/m0_verify.sh can —
#   see docs/HANDOFF_M3.md's +3mm finding: a near-boundary grasp can leave
#   the object still sliding after the gripper itself reaches full closure,
#   and the action server never publishes a result. Bounding the call
#   converts that hang into a recorded TIMEOUT outcome for that width, not a
#   fix for that width's grasp — same distinction as GRIPPER_GOAL_REJECTED
#   in ur5e_pick_place: correct behaviour, still a lost sample.
#
#   Per-width TIMEOUT is recorded and the sweep continues to the next width
#   — no retry. A single bad width does not invalidate the others, and per
#   the project's own rule (docs/HANDOFF_M3.md, "never auto-retry a failed
#   measurement"), the failure itself is data: which widths land near the
#   capture-window boundary is exactly what this table should reveal.
#
# PRECONDITION
#   The merged sim is already up in another terminal, gripper_controller
#   active, arm at its initial pose.
#
# USAGE
#   bash scripts/06_measure_grasp_table.sh 2>&1 | \
#     tee docs/grasp_table_$(date +%Y%m%d_%H%M%S).log

set -u

source "$(dirname "$0")/lib/gz_settle.sh"

WORLD="${WORLD:-empty}"
MODEL="${MODEL:-ur5e_robotiq}"
MASTER="${MASTER:-robotiq_85_left_knuckle_joint}"
GRIPPER_CTRL="${GRIPPER_CTRL:-gripper_controller}"

# Precondition tolerance: this is very likely the actual root cause of the
# reproducibility failure documented in docs/HANDOFF_M3.md. The gripper
# spawns near-closed (~0.767rad) on every fresh launch tested this session
# (5/5), not open (~0rad) as every probe script in this project assumed
# without checking. PRECLOSE angles computed here (and everywhere else) were
# computed as if starting from open; the actual starting aperture was never
# what any of these scripts believed it was.
OPEN_TOL_RAD="${OPEN_TOL_RAD:-0.05}"

# Object widths to sweep. Narrowed around the one validated anchor point
# (40mm box, 0.45 rad preclose, from Blocker 2's height-sweep tests) rather
# than the originally-proposed 25-65mm spread: a first live run at that wider
# range showed the width-dependent PRECLOSE formula below doesn't generalize
# past the anchor (see docs/HANDOFF_M3.md) -- widths far from 40mm missed the
# object entirely (fell through before the box even settled at spawn) or
# hung. Widen this only after that formula (or a real replacement for it) is
# validated at more points.
WIDTHS="${WIDTHS:-0.030 0.035 0.040 0.045 0.050}"

# Spawn-validity tolerance: after the post-spawn settle, the box must be
# within this many metres of the intended spawn height, or it wasn't caught
# by the pre-close cradle at all -- it fell through and is resting somewhere
# else entirely (usually the floor). A prior run let exactly this case
# through as a false OK: the box was stationary (settle correctly saw no
# motion) but stationary on the floor, not between the pads, and the
# subsequent overclose then closed on nothing. All prior valid captures
# settled within <0.2mm of target; 10mm is well above that noise floor and
# well below "fell a metre."
SPAWN_TOL_M="${SPAWN_TOL_M:-0.010}"
BOX_H="${BOX_H:-0.060}"
BOX_MASS="${BOX_MASS:-0.15}"
BOX_MU="${BOX_MU:-1.0}"
OVERCLOSE="${OVERCLOSE:-0.8}"

# Settle-before-sample thresholds — see lib/gz_settle.py. Same values used
# throughout the rest of this project's contact-load measurements.
POSE_EPS_M="${POSE_EPS_M:-0.0005}"
JOINT_EPS_RAD_S="${JOINT_EPS_RAD_S:-0.02}"
SETTLE_TIMEOUT_S="${SETTLE_TIMEOUT_S:-5.0}"
SETTLE_POLL_S="${SETTLE_POLL_S:-0.15}"

# The box settling against the gripper AFTER overclose is contact/compliance
# settling, not free motion -- consecutive-poll gz_settle_pose (above)
# declares it "settled" after ~0.5s while it is still measurably sinking for
# up to ~120s (confirmed live 2026-08-08, docs/HANDOFF_M3.md, "box-settle
# false-quiescence" -- this is what made every pad_centre_offset value in
# this script's pre-2026-08-08 output an undercount, some by 5-10x). Use the
# windowed variant with a long enough window and timeout to actually resolve
# that creep, ONLY for this one post-overclose box settle -- the fingertip
# and pre-close settles above are known-fast, non-contact motion and stay on
# the cheap consecutive-poll check.
BOX_SETTLE_WINDOW_S="${BOX_SETTLE_WINDOW_S:-10.0}"
BOX_SETTLE_TIMEOUT_S="${BOX_SETTLE_TIMEOUT_S:-150.0}"

LEFT_TIP="robotiq_85_left_finger_tip_link"
RIGHT_TIP="robotiq_85_right_finger_tip_link"
WRIST3="wrist_3_link"

OUTDIR="${OUTDIR:-$(pwd)/grasp_table_out}"
mkdir -p "$OUTDIR"

hr()   { printf '\n%s\n' "════════════════════════════════════════════════════════════"; }
sec()  { hr; printf '§ %s\n' "$1"; hr; }
note() { printf '  [note] %s\n' "$1"; }
die()  { printf '  [STOP] %s\n' "$1"; exit 2; }

GZ_JS="/world/${WORLD}/model/${MODEL}/joint_state"
GZ_POSE="/world/${WORLD}/pose/info"
GZ_CREATE="/world/${WORLD}/create"
GZ_REMOVE="/world/${WORLD}/remove"

printf 'grasp table sweep: %s\n' "$(date -Is)"
printf 'world=%s model=%s widths=%s\n' "$WORLD" "$MODEL" "$WIDTHS"

# ---------------------------------------------------------------------------
sec "0. Preconditions"
TOPICS=$(gz topic -l 2>/dev/null || true)
[[ -z "$TOPICS" ]] && die "no gz topics — is the sim running?"
for t in "$GZ_JS" "$GZ_POSE"; do
  printf '%s\n' "$TOPICS" | grep -qF "$t" && printf '  [ok] topic %s\n' "$t" \
    || die "missing topic $t — fix WORLD=/MODEL= to match, then re-run"
done

if ! gz_assert_joint "$GZ_JS" "$MASTER" 0.0 "$OPEN_TOL_RAD" "gripper open"; then
  die "precondition failed -- see message above. Command the gripper open (position 0.0) and re-run."
fi

# gripper.command_timeout_s from scene.yaml — the timeout this script binds
# to. Falls back to 5.0s if the field or file is missing, matching
# scene.yaml's own current value.
CMD_TIMEOUT_S=$(python3 -c "
import yaml
try:
    scene = yaml.safe_load(open('config/scene.yaml'))
    print(scene.get('gripper', {}).get('command_timeout_s', 5.0))
except Exception:
    print(5.0)
")
printf '  [ok] gripper.command_timeout_s = %ss (from config/scene.yaml)\n' "$CMD_TIMEOUT_S"

RESULTS_FILE="${RESULTS_FILE:-$OUTDIR/grasp_table.txt}"
printf '# box_width_m  grip_angle_rad  width_at_grip_m  tcp_offset_at_grip_m  pad_centre_offset_m  outcome\n' > "$RESULTS_FILE"

# sample_master_state -> "position velocity" for $MASTER, or empty on miss
sample_master_state() {
  python3 - "$1" <<PY
import sys
sys.path.insert(0, "$(dirname "$0")/lib")
import gz_settle as gs
txt = gs._gz_echo("$GZ_JS")
j = {}
import re
for blk in re.findall(r'joint\s*\{(.*?)\n\}', txt, re.S):
    n = re.search(r'name:\s*"([^"]+)"', blk)
    p = re.search(r'position:\s*(-?[\d.eE+-]+)', blk)
    if n and p:
        j[n.group(1)] = float(p.group(1))
print(j.get(sys.argv[1], ''))
PY
}

# tcp_geometry <label> -> "width_m tcp_offset_m" computed from wrist_3_link +
# both fingertip links at the current instant, reusing 05's exact math.
tcp_geometry() {
  gz topic -e -t "$GZ_POSE" -n 1 2>/dev/null > "$OUTDIR/pose_$1.txt"
  python3 - "$OUTDIR/pose_$1.txt" <<'PY'
import sys, re, math

txt = open(sys.argv[1]).read()

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
    print("ERROR missing_link")
    sys.exit(0)

wrist3_pos, wrist3_q = wrist3
left_pos, _ = left
right_pos, _ = right

R_wrist3 = quat_to_R(wrist3_q)
R_tool0 = matmul(matmul(R_wrist3, R_from_rpy(0, -math.pi/2, -math.pi/2)),
                 R_from_rpy(math.pi/2, 0, math.pi/2))
tool0_pos = wrist3_pos
tool0_z_world = matvec(R_tool0, (0, 0, 1))

mid = tuple((left_pos[i] + right_pos[i]) / 2.0 for i in range(3))
vec = sub(mid, tool0_pos)
offset = dot(vec, tool0_z_world)
width = norm(sub(left_pos, right_pos))

print(f"{width:.6f} {offset:.6f}")
PY
}

# box_z <label> -> box's world z, or empty on miss
box_z() {
  gz topic -e -t "$GZ_POSE" -n 1 2>/dev/null > "$OUTDIR/box_$1.txt"
  python3 - "$OUTDIR/box_$1.txt" "$2" <<'PY'
import sys, re
txt, want = open(sys.argv[1]).read(), sys.argv[2]
for blk in re.findall(r'pose\s*\{(.*?)\n\}', txt, re.S):
    n = re.search(r'name:\s*"([^"]+)"', blk)
    if n and n.group(1) == want:
        pos = re.search(r'position\s*\{(.*?)\}', blk, re.S)
        if pos:
            d = dict(re.findall(r'([xyz]):\s*(-?[\d.eE+-]+)', pos.group(1)))
            print(d.get('z', ''))
            break
PY
}

cleanup_width() {
  local box_name="$1"
  gz service -s "$GZ_REMOVE" --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
    --timeout 3000 --req "name: \"${box_name}\", type: MODEL" >/dev/null 2>&1
  timeout "$CMD_TIMEOUT_S" ros2 action send_goal "/${GRIPPER_CTRL}/gripper_cmd" \
    control_msgs/action/GripperCommand \
    "{command: {position: 0.0, max_effort: 50.0}}" >/dev/null 2>&1
}

# ---------------------------------------------------------------------------
sec "1. Sweep"

for W in $WIDTHS; do
  BOX_NAME="grasp_table_box"
  printf '\n  --- width=%s m ---\n' "$W"

  # PRECLOSE via interpolation against 05_measure_gripper_geometry.sh's
  # ACTUAL measured free-space width_map, not an assumed spec constant.
  #
  # First attempt at this used (max_opening - w)/max_opening*0.8 with
  # max_opening=0.085 (04's commented-but-never-implemented formula, the 2F-85
  # spec figure). That formula gave 0.4235 rad for the 40mm box -- the exact
  # width already validated in Blocker 2 at PRECLOSE=0.45 -- and the
  # discrepancy alone (0.4235 vs the working 0.45) was enough to make every
  # width in that run miss or hang, including 40mm itself, when re-tested
  # under the formula's own value. The formula was an assumed constant, never
  # checked against a measurement, and it was wrong even at the one point
  # already measured.
  #
  # Fix: use 05's real 5-point table directly, and target a free-space
  # clearance of (object_width + 0.050m). That +50mm margin is not
  # independently derived -- it's back-computed from the one validated
  # anchor (40mm object, 0.45rad measured to work) so this reproduces 0.45
  # almost exactly at 40mm (0.4523rad, 2.3mm-equivalent off) by construction,
  # then extrapolates the SAME margin to nearby widths rather than a
  # different, unverified constant.
  PRECLOSE=$(python3 -c "
table = [(0.0,0.135515),(0.2,0.116784),(0.4,0.095834),(0.6,0.073503),(0.767,0.054442)]
w = ${W}
target = w + 0.050
p = None
for i in range(len(table)-1):
    a0, w0 = table[i]; a1, w1 = table[i+1]
    if w1 <= target <= w0:
        frac = (w0 - target) / (w0 - w1)
        p = a0 + frac * (a1 - a0)
        break
if p is None:
    p = 0.0 if target > table[0][1] else 0.75
print(f'{max(0.0, min(0.75, p)):.4f}')
")
  printf '  pre-closing %s to %s rad (heuristic for this width)\n' "$MASTER" "$PRECLOSE"

  ros2 action send_goal "/${GRIPPER_CTRL}/gripper_cmd" \
    control_msgs/action/GripperCommand \
    "{command: {position: ${PRECLOSE}, max_effort: 50.0}}" >/dev/null 2>&1

  if ! gz_settle_pose "$GZ_POSE" "$POSE_EPS_M" "$SETTLE_TIMEOUT_S" "$SETTLE_POLL_S" "$LEFT_TIP" "$RIGHT_TIP"; then
    printf '  [STOP] fingertips did not settle after pre-close at width=%s — skipping this width\n' "$W"
    printf '%s\t\t\t\tTIMEOUT_PRECLOSE\n' "$W" >> "$RESULTS_FILE"
    continue
  fi

  BOX_XYZ=$(gz topic -e -t "$GZ_POSE" -n 1 2>/dev/null | python3 -c "
import sys, re
txt = sys.stdin.read()
tips = {}
for blk in re.findall(r'pose\s*\{(.*?)\n\}', txt, re.S):
    n = re.search(r'name:\s*\"([^\"]+)\"', blk)
    if not n or not n.group(1).endswith('finger_tip_link'):
        continue
    pos = re.search(r'position\s*\{(.*?)\}', blk, re.S)
    if pos:
        d = dict(re.findall(r'([xyz]):\s*(-?[\d.eE+-]+)', pos.group(1)))
        tips[n.group(1)] = tuple(float(d.get(k, 0)) for k in 'xyz')
if len(tips) >= 2:
    vs = list(tips.values())
    print(' '.join(f'{sum(c[i] for c in vs)/len(vs):.5f}' for i in range(3)))
")
  if [[ -z "$BOX_XYZ" ]]; then
    printf '  [STOP] could not locate fingertip links at width=%s — skipping\n' "$W"
    printf '%s\t\t\t\tMISSING_FINGERTIPS\n' "$W" >> "$RESULTS_FILE"
    continue
  fi
  printf '  box placement (fingertip midpoint): %s\n' "$BOX_XYZ"
  read -r BX BY BZ <<<"$BOX_XYZ"

  SDF="<?xml version='1.0'?><sdf version='1.9'><model name='${BOX_NAME}'>
<pose>${BX} ${BY} ${BZ} 0 0 0</pose><link name='link'>
<inertial><mass>${BOX_MASS}</mass><inertia><ixx>1e-4</ixx><iyy>1e-4</iyy>
<izz>1e-4</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
<collision name='c'><geometry><box><size>${W} ${W} ${BOX_H}</size></box>
</geometry><surface><friction><ode><mu>${BOX_MU}</mu><mu2>${BOX_MU}</mu2></ode></friction></surface>
</collision>
<visual name='v'><geometry><box><size>${W} ${W} ${BOX_H}</size></box>
</geometry></visual></link></model></sdf>"
  SDF_FLAT="${SDF//$'\n'/ }"
  CREATE_OUT=$(gz service -s "$GZ_CREATE" --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean --timeout 5000 \
    --req "sdf: \"${SDF_FLAT//\"/\\\"}\", name: \"${BOX_NAME}\"" 2>&1)
  if printf '%s\n' "$CREATE_OUT" | grep -qi "error\|cannot cross line"; then
    printf '  [STOP] box spawn rejected at width=%s — skipping\n' "$W"
    printf '%s\t\t\t\tSPAWN_REJECTED\n' "$W" >> "$RESULTS_FILE"
    continue
  fi

  if ! gz_settle_pose "$GZ_POSE" "$POSE_EPS_M" "$SETTLE_TIMEOUT_S" "$SETTLE_POLL_S" "$BOX_NAME"; then
    printf '  [STOP] box did not settle after spawn at width=%s — skipping\n' "$W"
    printf '%s\t\t\t\tTIMEOUT_SPAWN\n' "$W" >> "$RESULTS_FILE"
    cleanup_width "$BOX_NAME"
    continue
  fi
  Z_BEFORE=$(box_z before "$BOX_NAME")

  if [[ -z "$Z_BEFORE" ]]; then
    printf '  [STOP] could not read box z after spawn at width=%s — skipping\n' "$W"
    printf '%s\t\t\t\tMISSING_BOX_POSE\n' "$W" >> "$RESULTS_FILE"
    cleanup_width "$BOX_NAME"
    continue
  fi
  SPAWN_DRIFT=$(python3 -c "print(abs(${Z_BEFORE} - ${BZ}))")
  if python3 -c "exit(0 if abs(${Z_BEFORE} - ${BZ}) > ${SPAWN_TOL_M} else 1)"; then
    printf '  [STOP] box settled %sm from its spawn height at width=%s — not caught by the\n' \
           "$SPAWN_DRIFT" "$W"
    printf '         pre-close cradle, it fell through. Skipping overclose against nothing.\n'
    printf '%s\t\t\t\tMISSED_AT_SPAWN\n' "$W" >> "$RESULTS_FILE"
    cleanup_width "$BOX_NAME"
    continue
  fi

  printf '  commanding %s -> %s rad against a %s m box (bounded at %ss)\n' \
         "$MASTER" "$OVERCLOSE" "$W" "$CMD_TIMEOUT_S"
  gripper_close_and_hold "$GRIPPER_CTRL" "$MASTER" "$OVERCLOSE" 50.0 \
    "$CMD_TIMEOUT_S" "$GZ_JS" "$OUTDIR/action_${W}.txt"
  printf '  gripper_close_and_hold: %s, now holding %s rad (not left driving toward %s)\n' \
         "$GRIPPER_HOLD_RESULT" "$GRIPPER_HOLD_POSITION" "$OVERCLOSE"

  if [[ "$GRIPPER_HOLD_RESULT" == "TIMED_OUT_HELD" || "$GRIPPER_HOLD_RESULT" == "UNKNOWN_NO_SAMPLE" ]]; then
    printf '  [STOP] overclose action produced no result within %ss at width=%s — TIMEOUT, not retrying\n' \
           "$CMD_TIMEOUT_S" "$W"
    printf '%s\t\t\t\tTIMEOUT_OVERCLOSE\n' "$W" >> "$RESULTS_FILE"
    cleanup_width "$BOX_NAME"
    continue
  fi

  if ! gz_settle_joint "$GZ_JS" "$JOINT_EPS_RAD_S" "$SETTLE_TIMEOUT_S" "$SETTLE_POLL_S" "$MASTER"; then
    printf '  [note] master joint did not settle within %ss at width=%s — recording anyway, flagged\n' \
           "$SETTLE_TIMEOUT_S" "$W"
  fi
  if ! gz_settle_pose_windowed "$GZ_POSE" "$POSE_EPS_M" "$BOX_SETTLE_TIMEOUT_S" "$SETTLE_POLL_S" \
       "$BOX_SETTLE_WINDOW_S" "$BOX_NAME"; then
    printf '  [note] box did not settle within %ss (window=%ss) at width=%s — recording anyway, flagged\n' \
           "$BOX_SETTLE_TIMEOUT_S" "$BOX_SETTLE_WINDOW_S" "$W"
  fi

  GRIP_ANGLE=$(sample_master_state "$MASTER")
  read -r WIDTH_AT_GRIP TCP_OFFSET_AT_GRIP <<<"$(tcp_geometry closed_${W})"
  Z_AFTER=$(box_z after "$BOX_NAME")

  if [[ -z "$GRIP_ANGLE" || -z "$WIDTH_AT_GRIP" || -z "$Z_BEFORE" || -z "$Z_AFTER" ]]; then
    printf '  [STOP] incomplete sample at width=%s — recording as such\n' "$W"
    printf '%s\t\t\t\tINCOMPLETE_SAMPLE\n' "$W" >> "$RESULTS_FILE"
    cleanup_width "$BOX_NAME"
    continue
  fi

  PAD_CENTRE_OFFSET=$(python3 -c "print(f'{${Z_BEFORE} - ${Z_AFTER}:.6f}')")
  printf '  grip_angle=%s width_at_grip=%s tcp_offset_at_grip=%s pad_centre_offset=%s\n' \
         "$GRIP_ANGLE" "$WIDTH_AT_GRIP" "$TCP_OFFSET_AT_GRIP" "$PAD_CENTRE_OFFSET"
  printf '%s\t%s\t%s\t%s\t%s\tOK\n' \
         "$W" "$GRIP_ANGLE" "$WIDTH_AT_GRIP" "$TCP_OFFSET_AT_GRIP" "$PAD_CENTRE_OFFSET" \
         >> "$RESULTS_FILE"

  cleanup_width "$BOX_NAME"
done

# ---------------------------------------------------------------------------
sec "2. Results"
column -t "$RESULTS_FILE"

sec "3. What to do with this"
cat <<'EOF'
  Raw measurement, not a judged pass/fail. To use it:

  1. clearance_width for a given object width: look up SEPARATELY in
     05_measure_gripper_geometry.sh's output table (joint_rad -> width_m,
     free-space). This script deliberately does not recompute it.
  2. grasp composition (M3): aperture from clearance_width's inverted
     lookup; TCP target from tcp_offset_at_grip at the matching row here
     (not the single scalar grasp.tcp_offset currently in scene.yaml);
     approach shifted by pad_centre_offset at that same row.
  3. Any row marked TIMEOUT_* or INCOMPLETE_SAMPLE is itself a finding —
     that object width landed near (or past) the capture-window boundary.
     Do not retry it. Investigate why before trusting a nearby width's OK
     row as safe margin.
  4. Check whether pad_centre_offset correlates with grip_angle across the
     OK rows — this is the test the 2-point ad hoc check in Blocker 2
     couldn't settle. Five points should be enough to say yes or no.
EOF

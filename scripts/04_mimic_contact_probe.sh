#!/usr/bin/env bash
# 04_mimic_contact_probe.sh — does contact oppose the mimic linkage?
#
# THIS SCRIPT DELIBERATELY DOES NOT PASS OR FAIL ANYTHING.
#
# It is a calibration run. §3.5 of the M-1 report established that dartsim
# refuses to create mimic constraints and that gz_ros2_control is (almost
# certainly) writing follower positions in software instead. Free-space tracking
# already works — that is measured and not in question. What is unknown is
# whether that mechanism yields when a fingertip meets an object.
#
# Nobody knows yet what "yields" looks like numerically on this setup: how many
# radians short of command counts as stalling, what residual velocity is normal,
# how much interpenetration is just contact softness. Inventing those thresholds
# now and baking them into m0_verify.sh would encode a guess as a gate.
#
# So: this reports raw numbers. Read them, decide the thresholds, THEN update
# m0_verify.sh's M0-C with a criterion grounded in observation.
#
# PRECONDITION
#   The merged sim is already up in another terminal, controllers active.
#   Arm at its initial pose, gripper OPEN.
#
# USAGE
#   bash scripts/04_mimic_contact_probe.sh 2>&1 | tee docs/probe_$(date +%Y%m%d_%H%M%S).log
#
# VERIFY-DON'T-ASSUME: the gz service and topic names below are the Harmonic
# spellings as I understand them, not names I have run on this machine. §0
# checks each one exists before use and tells you what is actually available if
# not. Fix the variable, do not fight the script.

set -u

source "$(dirname "$0")/lib/gz_settle.sh"

WORLD="${WORLD:-empty}"
MODEL="${MODEL:-ur5e_robotiq}"
MASTER="${MASTER:-robotiq_85_left_knuckle_joint}"
GRIPPER_CTRL="${GRIPPER_CTRL:-gripper_controller}"

# Precondition tolerance: this script's own PRECONDITION comment says
# "gripper OPEN" and nothing ever checked it -- false on every fresh sim
# launch tested this session (5/5 spawned near-closed, ~0.767rad, not open;
# see docs/HANDOFF_M3.md). Every probe run before this fix was measuring
# contact behavior on top of an unverified, and often unmet, starting
# aperture.
OPEN_TOL_RAD="${OPEN_TOL_RAD:-0.05}"

# Test object. Width is what matters: it sets the joint angle at which the
# fingers SHOULD stop.
BOX_NAME="${BOX_NAME:-probe_box}"
BOX_W="${BOX_W:-0.040}"      # m, across the closing axis
BOX_H="${BOX_H:-0.060}"      # m
BOX_MASS="${BOX_MASS:-0.15}" # kg
BOX_MU="${BOX_MU:-1.0}"

# Command the master PAST the angle the box physically permits. The whole point
# is to demand more closure than the object allows and see what gives.
OVERCLOSE="${OVERCLOSE:-0.8}"   # rad; 0.8 = fully closed per recon
SETTLE="${SETTLE:-3.0}"         # s to let contact resolve

OUTDIR="${OUTDIR:-$(pwd)/probe_out}"
mkdir -p "$OUTDIR"

# Settle-before-sample thresholds — see lib/gz_settle.py. Every fixed sleep
# below used to be a guess at "probably long enough"; these poll until motion
# actually stops (or fail loudly) instead.
POSE_EPS_M="${POSE_EPS_M:-0.0005}"
JOINT_EPS_RAD_S="${JOINT_EPS_RAD_S:-0.02}"
SETTLE_POLL_S="${SETTLE_POLL_S:-0.15}"
LEFT_TIP="robotiq_85_left_finger_tip_link"
RIGHT_TIP="robotiq_85_right_finger_tip_link"

hr()   { printf '\n%s\n' "════════════════════════════════════════════════════════════"; }
sec()  { hr; printf '§ %s\n' "$1"; hr; }
note() { printf '  [note] %s\n' "$1"; }
die()  { printf '  [STOP] %s\n' "$1"; exit 2; }

GZ_JS="/world/${WORLD}/model/${MODEL}/joint_state"
GZ_POSE="/world/${WORLD}/pose/info"
GZ_CREATE="/world/${WORLD}/create"
GZ_REMOVE="/world/${WORLD}/remove"

printf 'mimic contact probe: %s\n' "$(date -Is)"
printf 'world=%s model=%s box_width=%s m overclose=%s rad\n' \
       "$WORLD" "$MODEL" "$BOX_W" "$OVERCLOSE"

# ---------------------------------------------------------------------------
sec "0. Preconditions"
TOPICS=$(gz topic -l 2>/dev/null || true)
[[ -z "$TOPICS" ]] && die "no gz topics — is the sim running?"

for t in "$GZ_JS" "$GZ_POSE"; do
  if printf '%s\n' "$TOPICS" | grep -qF "$t"; then
    printf '  [ok] topic %s\n' "$t"
  else
    printf '  [MISSING] topic %s\n' "$t"
    printf '  available topics containing joint_state or pose:\n'
    printf '%s\n' "$TOPICS" | grep -iE 'joint_state|pose' | sed 's|^|      |'
    die "fix WORLD=/MODEL= to match, then re-run"
  fi
done

SERVICES=$(gz service -l 2>/dev/null || true)
for s in "$GZ_CREATE" "$GZ_REMOVE"; do
  if printf '%s\n' "$SERVICES" | grep -qF "$s"; then
    printf '  [ok] service %s\n' "$s"
  else
    printf '  [MISSING] service %s\n' "$s"
    printf '%s\n' "$SERVICES" | grep -iE 'create|remove|spawn' | sed 's|^|      |'
    die "spawn/remove service name differs — set GZ_CREATE/GZ_REMOVE"
  fi
done

if ! gz_assert_joint "$GZ_JS" "$MASTER" 0.0 "$OPEN_TOL_RAD" "gripper open"; then
  die "precondition failed -- see message above. Command the gripper open (position 0.0) and re-run."
fi

# ---------------------------------------------------------------------------
# sample_joints <label> -> writes "<name> <position> <velocity>" lines
sample_joints() {
  gz topic -e -t "$GZ_JS" -n 1 2>/dev/null > "$OUTDIR/raw_js_$1.txt"
  python3 - "$OUTDIR/raw_js_$1.txt" <<'PY'
import sys, re
txt = open(sys.argv[1]).read()
for blk in re.findall(r'joint\s*\{(.*?)\n\}', txt, re.S):
    n = re.search(r'name:\s*"([^"]+)"', blk)
    p = re.search(r'position:\s*(-?[\d.eE+-]+)', blk)
    v = re.search(r'velocity:\s*(-?[\d.eE+-]+)', blk)
    if n and p:
        print(f"{n.group(1)} {p.group(1)} {v.group(1) if v else 'NA'}")
PY
}

# sample_box_pose <label>
sample_box_pose() {
  gz topic -e -t "$GZ_POSE" -n 1 2>/dev/null > "$OUTDIR/raw_pose_$1.txt"
  python3 - "$OUTDIR/raw_pose_$1.txt" "$BOX_NAME" <<'PY'
import sys, re
txt, want = open(sys.argv[1]).read(), sys.argv[2]
for blk in re.findall(r'pose\s*\{(.*?)\n\}', txt, re.S):
    n = re.search(r'name:\s*"([^"]+)"', blk)
    if n and want in n.group(1):
        pos = re.search(r'position\s*\{(.*?)\}', blk, re.S)
        if pos:
            xyz = dict(re.findall(r'([xyz]):\s*(-?[\d.eE+-]+)', pos.group(1)))
            print(f"{n.group(1)} {xyz.get('x','0')} {xyz.get('y','0')} {xyz.get('z','0')}")
PY
}

# ---------------------------------------------------------------------------
sec "1. Baseline — gripper open, no object"
sample_joints baseline | tee "$OUTDIR/joints_baseline.txt" | sed 's|^|    |'

sec "1b. Pre-close to a cradle aperture before placing the box"
# At full-open (master ~0 rad) the 2F-85's ~85 mm span leaves a 40 mm box
# touching neither pad — it free-falls through the gap under gravity before
# the closing command ever reaches it. Observed directly: a box spawned at the
# full-open fingertip midpoint landed ~16 cm lower, on the ground plane, by
# the time the "closed" sample was taken. Pre-closing first means the box is
# actually flanked by the pads at spawn time instead of floating in open air.
#
# PRECLOSE is a rough heuristic (linear opening-vs-angle guess: (max_opening -
# box_width) / max_opening * 0.8), NOT a calibrated value — scene.yaml's
# width_map is still null for exactly this reason, the 2F-85 four-bar linkage
# isn't linear. If the box still isn't caught between the pads, adjust
# PRECLOSE and re-run; it is a starting point, not a claim.
PRECLOSE="${PRECLOSE:-0.45}"
printf '  pre-closing %s to %s rad (heuristic cradle aperture, not measured)\n' "$MASTER" "$PRECLOSE"
ros2 action send_goal "/${GRIPPER_CTRL}/gripper_cmd" \
  control_msgs/action/GripperCommand \
  "{command: {position: ${PRECLOSE}, max_effort: 50.0}}" 2>&1 \
  | tail -10 | sed 's|^|    |'
if ! gz_settle_pose "$GZ_POSE" "$POSE_EPS_M" 5.0 "$SETTLE_POLL_S" "$LEFT_TIP" "$RIGHT_TIP"; then
  die "fingertips did not settle after pre-close — box placement below would use a moving target"
fi

note "place the box at the (now pre-closed) fingertip midpoint. If your setup"
note "keeps the arm at its initial pose, read the fingertip link poses from"
note "$GZ_POSE and set BOX_XYZ accordingly. Set BOX_XYZ explicitly to skip"
note "auto-placement."

BOX_XYZ="${BOX_XYZ:-}"
if [[ -z "$BOX_XYZ" ]]; then
  gz topic -e -t "$GZ_POSE" -n 1 2>/dev/null > "$OUTDIR/raw_pose_probe.txt"
  BOX_XYZ=$(python3 - "$OUTDIR/raw_pose_probe.txt" <<'PY'
import sys, re
txt = open(sys.argv[1]).read()
tips = {}
for blk in re.findall(r'pose\s*\{(.*?)\n\}', txt, re.S):
    n = re.search(r'name:\s*"([^"]+)"', blk)
    # Exact suffix match, not substring: /world/.../pose/info also reports
    # nested sub-frames like "...finger_tip_link_visual" at a near-(0,0,0)
    # LOCAL offset, which still contains "finger_tip_link" as a substring.
    # Averaging those in silently pulled the computed point to roughly HALF
    # the real link position in every axis — confirmed by hand: real fingertip
    # midpoint was (0.492, 0.133, 0.367), the buggy substring match produced
    # (0.246, 0.067, 0.183). That's why the box fell straight to the ground
    # both prior runs regardless of gripper aperture — it was never anywhere
    # near the gripper to begin with.
    if not n or not n.group(1).endswith('finger_tip_link'):
        continue
    pos = re.search(r'position\s*\{(.*?)\}', blk, re.S)
    if pos:
        xyz = dict(re.findall(r'([xyz]):\s*(-?[\d.eE+-]+)', pos.group(1)))
        tips[n.group(1)] = tuple(float(xyz.get(k, 0)) for k in 'xyz')
if len(tips) >= 2:
    vs = list(tips.values())
    print(" ".join(f"{sum(c[i] for c in vs)/len(vs):.5f}" for i in range(3)))
PY
)
fi

if [[ -z "$BOX_XYZ" ]]; then
  die "could not locate fingertip links in $GZ_POSE — set BOX_XYZ='x y z' manually"
fi
printf '\n  box placement (fingertip midpoint): %s\n' "$BOX_XYZ"

# ---------------------------------------------------------------------------
sec "2. Spawn the test object"
read -r BX BY BZ <<<"$BOX_XYZ"
SDF="<?xml version='1.0'?><sdf version='1.9'>
<model name='${BOX_NAME}'>
  <pose>${BX} ${BY} ${BZ} 0 0 0</pose>
  <link name='link'>
    <inertial><mass>${BOX_MASS}</mass>
      <inertia><ixx>1e-4</ixx><iyy>1e-4</iyy><izz>1e-4</izz>
               <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
    </inertial>
    <collision name='c'><geometry><box><size>${BOX_W} ${BOX_W} ${BOX_H}</size></box></geometry>
      <surface><friction><ode><mu>${BOX_MU}</mu><mu2>${BOX_MU}</mu2></ode></friction></surface>
    </collision>
    <visual name='v'><geometry><box><size>${BOX_W} ${BOX_W} ${BOX_H}</size></box></geometry></visual>
  </link>
</model></sdf>"

# protobuf text-format does not allow a string literal to cross a physical
# line boundary. $SDF above is written multi-line for readability; flatten it
# to a single line here before it goes anywhere near --req, or gz's text-format
# parser rejects the whole request ("String literals cannot cross line
# boundaries") — silently as far as this script's control flow is concerned,
# since nothing downstream checks this command's exit status. That failure
# mode is dangerous specifically here: the gripper still closes in the empty
# air where the box should have been, "succeeds," and produces a shortfall
# that looks identical to a real through-the-object result.
SDF_FLAT="${SDF//$'\n'/ }"
CREATE_OUT=$(gz service -s "$GZ_CREATE" \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 5000 --req "sdf: \"${SDF_FLAT//\"/\\\"}\", name: \"${BOX_NAME}\"" 2>&1)
printf '%s\n' "$CREATE_OUT" | sed 's|^|    |'
if printf '%s\n' "$CREATE_OUT" | grep -qi "error\|cannot cross line"; then
  die "box spawn request was rejected — see the error above. Nothing to test; fix before reading any result as real."
fi
if ! gz_settle_pose "$GZ_POSE" "$POSE_EPS_M" 5.0 "$SETTLE_POLL_S" "$BOX_NAME"; then
  die "box did not settle after spawn — it is still falling/bouncing, not resting against the pads"
fi
sample_box_pose spawned | sed 's|^|    box @ |'

# ---------------------------------------------------------------------------
sec "3. Command closure PAST what the object permits"
printf '  commanding %s -> %s rad (box is %s m wide)\n' "$MASTER" "$OVERCLOSE" "$BOX_W"
ros2 action send_goal "/${GRIPPER_CTRL}/gripper_cmd" \
  control_msgs/action/GripperCommand \
  "{command: {position: ${OVERCLOSE}, max_effort: 50.0}}" 2>&1 \
  | tail -20 | sed 's|^|    |'

# $SETTLE is now a timeout ceiling, not a blind wait: settle on the master
# joint's velocity AND the box's pose, sequentially, each capped at $SETTLE.
# A timeout on either is reported, not silently treated as "rest".
if ! gz_settle_joint "$GZ_JS" "$JOINT_EPS_RAD_S" "$SETTLE" "$SETTLE_POLL_S" "$MASTER"; then
  note "master joint did not settle within ${SETTLE}s — sampling anyway, but the"
  note "shortfall number below may include residual motion, not just the stall."
fi
if ! gz_settle_pose "$GZ_POSE" "$POSE_EPS_M" "$SETTLE" "$SETTLE_POLL_S" "$BOX_NAME"; then
  note "box did not settle within ${SETTLE}s — it may still be sliding/ejecting."
  note "the displacement number below is a snapshot mid-motion, not a rest state."
fi

sample_joints closed | tee "$OUTDIR/joints_closed.txt" | sed 's|^|    |'
sample_box_pose closed | sed 's|^|    box @ |'

# ---------------------------------------------------------------------------
sec "4. Numbers  (interpret these; the script will not)"
python3 - "$OUTDIR/joints_baseline.txt" "$OUTDIR/joints_closed.txt" \
         "$MASTER" "$OVERCLOSE" "$BOX_W" <<'PY'
import sys
def load(p):
    d = {}
    for line in open(p):
        f = line.split()
        if len(f) >= 2:
            d[f[0]] = (float(f[1]), None if f[2] == 'NA' else float(f[2]))
    return d

base, clos = load(sys.argv[1]), load(sys.argv[2])
master, cmd, w = sys.argv[3], float(sys.argv[4]), float(sys.argv[5])

mp, mv = clos.get(master, (float('nan'), None))
print(f"  commanded master angle : {cmd:.4f} rad")
print(f"  achieved  master angle : {mp:.4f} rad")
print(f"  shortfall              : {cmd - mp:+.4f} rad")
print(f"  master velocity at rest: {mv}")
print()
print("  THE QUESTION THIS ANSWERS:")
print("    shortfall ~0        -> the linkage drove to command THROUGH the object.")
print("                           Contact is not opposing it. Kinematic override")
print("                           confirmed. Friction grasping cannot work as specified.")
print("    shortfall clearly >0 -> contact IS opposing the linkage. The override")
print("                           yields. Friction grasping may be viable; calibrate")
print("                           m0_verify.sh's threshold from this number.")
print()
print("  followers (position, velocity):")
for n in sorted(clos):
    if n == master or 'robotiq' not in n:
        continue
    p, v = clos[n]
    b = base.get(n, (float('nan'), None))[0]
    print(f"    {n:<42} {p:+.5f}  (moved {p-b:+.5f})  vel={v}")
print()
print("  Also check the box pose printed in §3 against §2:")
print("    unchanged        -> held, or never contacted (check finger separation)")
print("    small shift      -> pushed by one finger before the other met it")
print("    large/rising     -> ejected. Solver resolving an impossible overlap.")
print("    inside the pads  -> interpenetration. Look at it in the GUI to be sure.")
PY

# ---------------------------------------------------------------------------
sec "5. Cleanup"
gz service -s "$GZ_REMOVE" \
  --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
  --timeout 3000 --req "name: \"${BOX_NAME}\", type: MODEL" 2>&1 | sed 's|^|    |'
ros2 action send_goal "/${GRIPPER_CTRL}/gripper_cmd" \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.0, max_effort: 50.0}}" >/dev/null 2>&1
note "gripper reopened, box removed. Raw samples kept in $OUTDIR"

hr
cat <<'EOF'
  NEXT, once you have read the numbers:

  1. Decide the stall threshold from the observed shortfall, not from intuition.
  2. Rewrite m0_verify.sh §M0-C around it, replacing the free-space sweep.
     Keep the sweep as a subordinate check — it still catches a broken merge —
     but it must no longer be what M0-C passes on.
  3. If the shortfall is ~0, do NOT start tuning friction. Go to §3.5
     Consequence 2 and work the physics-engine decision instead.
EOF

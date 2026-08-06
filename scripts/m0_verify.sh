#!/usr/bin/env bash
# m0_verify.sh — Milestone 0 stack verification. A / B / C.
#
# The deliverable of M0 is a written pass/fail note with log lines.
# Not "it works". This script produces that note.
#
# PRECONDITION
#   M-1 is done: merged URDF spawns cleanly in Harmonic, MoveIt config exists.
#   Bring the sim up in another terminal FIRST, then run this.
#
# USAGE
#   Terminal 1:  ros2 launch <your_bringup> sim_moveit.launch.py
#   Terminal 2:  source /opt/ros/jazzy/setup.bash && source install/setup.bash
#                bash scripts/m0_verify.sh 2>&1 | tee docs/m0_$(date +%Y%m%d_%H%M%S).log
#
# EXIT CODES
#   0 = all three checks PASS
#   1 = at least one FAIL   (do not proceed to M1)
#   2 = could not run       (sim not up, tooling missing)

set -uo pipefail

source "$(dirname "$0")/lib/gz_settle.sh"

WORLD="${WORLD:-empty}"                  # gz world name
MODEL="${MODEL:-ur5e_robotiq}"           # gz model name of the merged robot
ACTUATED="${ACTUATED:-robotiq_85_left_knuckle_joint}"  # was: finger_joint — that
                                                        # name does not exist in
                                                        # ros-jazzy-robotiq-description
GRIPPER_CTRL="${GRIPPER_CTRL:-gripper_controller}"
ARM_CTRL="${ARM_CTRL:-arm_controller}"
MOVEIT_CTRL_YAML="${MOVEIT_CTRL_YAML:-config/moveit_controllers.yaml}"
SPAWN_WAIT="${SPAWN_WAIT:-45}"           # donor reports 10-15s; be generous
# Precondition tolerance for M0-C: gripper must be open (~0 rad) at start.
# Never checked before this session; false on 5/5 fresh launches tested
# (spawned ~0.767rad, not open) -- see docs/HANDOFF_M3.md.
OPEN_TOL_RAD="${OPEN_TOL_RAD:-0.05}"

PASS_A=UNKNOWN; PASS_B=UNKNOWN; PASS_C=UNKNOWN
EVID_DIR="${EVID_DIR:-$(pwd)/m0_evidence}"
mkdir -p "$EVID_DIR"

hr()   { printf '\n%s\n' "════════════════════════════════════════════════════════════"; }
sec()  { hr; printf '§ %s\n' "$1"; hr; }
ok()   { printf '  [PASS] %s\n' "$1"; }
bad()  { printf '  [FAIL] %s\n' "$1"; }
warn() { printf '  [WARN] %s\n' "$1"; }
note() { printf '  [note] %s\n' "$1"; }
cmd()  { printf '\n  $ %s\n' "$*"; }

printf 'M0 verification run: %s\n' "$(date -Is)"
printf 'world=%s model=%s actuated_joint=%s\n' "$WORLD" "$MODEL" "$ACTUATED"

# ---------------------------------------------------------------------------
sec "M0-A  Gazebo version binding"
# Requirement: confirm Harmonic (gz-sim 8.x), with no stray dependency pulling
# in Fortress or Classic. Evidence = gz sim --versions + resolved dep list.

A_FAIL=0

if ! command -v gz >/dev/null 2>&1; then
  bad "gz binary not on PATH"; A_FAIL=1
else
  cmd "gz sim --versions"
  GZ_VER=$(gz sim --versions 2>&1 | tee "$EVID_DIR/A_gz_versions.txt")
  printf '%s\n' "$GZ_VER" | sed 's|^|      |'

  GZ_MAJOR=$(printf '%s' "$GZ_VER" | grep -oE '^[0-9]+' | head -1)
  if [[ "$GZ_MAJOR" == "8" ]]; then
    ok "gz-sim major version = 8 (Harmonic)"
  else
    bad "gz-sim major version = ${GZ_MAJOR:-<unparsed>}; expected 8 (Harmonic)"
    A_FAIL=1
  fi
fi

cmd "dpkg -l | grep -Ei 'gz-sim|ignition|gazebo'"
dpkg -l 2>/dev/null | grep -Ei 'gz-sim|ignition|gazebo' \
  | awk '{printf "      %-44s %s\n", $2, $3}' | tee "$EVID_DIR/A_deps.txt"

# A stray Fortress package is the specific failure this check exists to catch.
if dpkg -l 2>/dev/null | grep -qiE 'ignition-gazebo|gz-sim6|gazebo11|gazebo-common'; then
  bad "Fortress/Classic-era package present in the resolved dependency list"
  warn "trace it: apt-cache rdepends <pkg>  — something is pulling it in"
  A_FAIL=1
else
  ok "no Fortress or Classic packages in the resolved dependency list"
fi

cmd "gz topic -l  (is a Harmonic transport actually up?)"
if gz topic -l 2>/dev/null | head -20 | tee "$EVID_DIR/A_gz_topics.txt" | sed 's|^|      |'; then
  if [[ ! -s "$EVID_DIR/A_gz_topics.txt" ]]; then
    bad "gz transport listed no topics — is the sim actually running?"; A_FAIL=1
  fi
else
  bad "gz topic -l failed"; A_FAIL=1
fi

[[ $A_FAIL -eq 0 ]] && PASS_A=PASS || PASS_A=FAIL
printf '\n  ==> M0-A: %s\n' "$PASS_A"

# ---------------------------------------------------------------------------
sec "M0-B  Gripper controller path in simulation"
# Requirement: the controller name(s) actually spawned by the merged bringup
# must match what moveit_controllers.yaml expects for the gripper planning
# group. This is the seam that silently breaks.

B_FAIL=0

printf '  waiting up to %ss for controllers to spawn...\n' "$SPAWN_WAIT"
for ((i=0; i<SPAWN_WAIT; i++)); do
  if ros2 control list_controllers >/dev/null 2>&1; then break; fi
  sleep 1
done

cmd "ros2 control list_controllers"
if ! ros2 control list_controllers 2>&1 | tee "$EVID_DIR/B_controllers.txt" | sed 's|^|      |'; then
  bad "controller_manager not reachable — sim not up?"
  PASS_B=FAIL; B_FAIL=1
else
  for c in "$ARM_CTRL" "$GRIPPER_CTRL" joint_state_broadcaster; do
    if grep -qE "^${c}\b.*\bactive\b" "$EVID_DIR/B_controllers.txt"; then
      ok "$c is active"
    else
      bad "$c not found in 'active' state"
      B_FAIL=1
    fi
  done

  # What TYPE is the gripper controller? This decides whether the squeeze knob
  # is position-space or effort-space. The spec requires this be stated
  # explicitly, not assumed.
  #
  # Format-agnostic extraction: strips any brackets, then takes the first
  # whitespace-separated token containing '/', which is the plugin type in
  # both the bracketed layout an older ros2_control CLI used and the plain-
  # column layout this Jazzy build actually prints. The old bracket-only
  # pattern silently returned empty on this build's output and always fell
  # through to "unrecognised" — found by running it, not by reading it.
  GTYPE=$(awk -v c="$GRIPPER_CTRL" '
    index($0, c) == 1 {
      line = $0
      gsub(/[][]/, " ", line)
      n = split(line, f, /[ \t]+/)
      for (i = 1; i <= n; i++)
        if (index(f[i], "/") > 0) { print f[i]; exit }
    }
  ' "$EVID_DIR/B_controllers.txt")
  printf '\n  gripper controller type: %s\n' "${GTYPE:-<not found>}"
  # Match on the CONTROLLER NAMESPACE, not the class name, and test effort
  # FIRST. effort_controllers/GripperActionController is a real type and
  # contains the substring "GripperActionController" — a *GripperActionController*
  # branch tried first swallows it and reports EFFORT control as POSITION
  # control, leaving scene.yaml's radian-space squeeze knob in force against a
  # controller that takes newtons. That misclassification is exactly what this
  # ordering exists to prevent; caught by testing the effort case directly.
  case "$GTYPE" in
    effort_controllers/*|*Effort*System*)
      warn "EFFORT-controlled. The position squeeze formula DOES NOT APPLY."
      warn "Set scene.yaml gripper.position_controlled: false and switch the knob"
      warn "to commanded force/effort. State this explicitly in the run log."
      ;;
    position_controllers/GripperActionController)
      ok "POSITION-controlled. scene.yaml gripper.squeeze applies, in RADIANS."
      note "max_effort in the GripperCommand action is a ceiling, not the knob."
      ;;
    position_controllers/*)
      ok "POSITION-controlled ($GTYPE)."
      note "not GripperActionController — confirm it exposes a gripper_cmd action"
      note "and that stalling is reported, or a successful grasp reads as failure."
      ;;
    "")
      bad "could not parse the gripper controller type from list_controllers"
      note "raw line(s) matching '${GRIPPER_CTRL}':"
      grep -n "$GRIPPER_CTRL" "$EVID_DIR/B_controllers.txt" | sed 's|^|      |'
      note "if the format changed again, fix the awk above rather than assuming"
      B_FAIL=1
      ;;
    *)
      bad "unrecognised gripper controller type: $GTYPE"
      B_FAIL=1
      ;;
  esac

  # Cross-check against what MoveIt believes.
  cmd "cat $MOVEIT_CTRL_YAML"
  if [[ -f "$MOVEIT_CTRL_YAML" ]]; then
    cat "$MOVEIT_CTRL_YAML" | sed 's|^|      |' | tee "$EVID_DIR/B_moveit_yaml.txt"
    if grep -q "$GRIPPER_CTRL" "$MOVEIT_CTRL_YAML"; then
      ok "moveit_controllers.yaml references '$GRIPPER_CTRL' — names match"
    else
      bad "moveit_controllers.yaml does NOT reference '$GRIPPER_CTRL'"
      warn "this is the exact mismatch M0-B exists to catch"
      B_FAIL=1
    fi
  else
    bad "moveit_controllers.yaml not found at $MOVEIT_CTRL_YAML"
    B_FAIL=1
  fi

  cmd "ros2 action list | grep -i gripper"
  ros2 action list 2>/dev/null | grep -i gripper | sed 's|^|      |' \
    | tee "$EVID_DIR/B_actions.txt" \
    || { bad "no gripper action server advertised"; B_FAIL=1; }

  [[ $B_FAIL -eq 0 ]] && PASS_B=PASS || PASS_B=FAIL
fi
printf '\n  ==> M0-B: %s\n' "$PASS_B"

# ---------------------------------------------------------------------------
# =============================================================================
# WHAT CHANGED AND WHY
#   The old M0-C swept the actuated joint in free space and passed if the
#   followers tracked. Per report §3.5 that test now returns green while
#   proving the wrong thing: dartsim creates no mimic constraint, gz_ros2_control
#   writes follower positions in software, and free-space tracking is therefore
#   expected regardless of whether contact can oppose the linkage.
#
#   C1 (sweep) is DEMOTED to a subordinate merge-sanity check. It still catches
#   a broken merge, and it no longer decides M0-C.
#   C2 (contact load) is what M0-C now passes on.
#
#   Thresholds are calibrated from the 2026-08-03 probe run (report §3.5), with
#   margin — not fitted to the observed numbers:
#     observed shortfall 0.342 rad   -> threshold 0.10
#     observed box drop   ~13 mm     -> threshold 30 mm
#     observed follower spread <1e-5 -> threshold 0.05 (admits the known
#                                       right_knuckle divergence of ~0.027)
# =============================================================================

sec "M0-C  Mimic linkage under contact load"
# Ground truth is Gazebo's own joint_state, never /joint_states. The donor repo
# documents an open gz_ros2_control readback defect, and /joint_states is in any
# case what ros2_control believes — which is the thing under test.

C_FAIL=0
GZ_JS_TOPIC="/world/${WORLD}/model/${MODEL}/joint_state"
GZ_POSE_TOPIC="/world/${WORLD}/pose/info"
GZ_CREATE="/world/${WORLD}/create"
GZ_REMOVE="/world/${WORLD}/remove"

BOX_NAME="${BOX_NAME:-m0c_box}"
BOX_W="${BOX_W:-0.040}"
BOX_H="${BOX_H:-0.060}"
BOX_MASS="${BOX_MASS:-0.15}"
OVERCLOSE="${OVERCLOSE:-0.8}"
SETTLE="${SETTLE:-3.0}"

# gripper.command_timeout_s from scene.yaml — this section's overclose call
# used to be unbounded (see docs/HANDOFF_M3.md, Blocker 2's ACTION FOR M3:
# "Also fix the probe scripts themselves (04_mimic_contact_probe.sh,
# m0_verify.sh's M0-C)... they currently call ros2 action send_goal with no
# cap, which is exactly what just hung"). Falls back to 5.0s if the field or
# file is missing, matching config/scene.yaml's own current value.
CMD_TIMEOUT_S=$(python3 -c "
import yaml
try:
    scene = yaml.safe_load(open('config/scene.yaml'))
    print(scene.get('gripper', {}).get('command_timeout_s', 5.0))
except Exception:
    print(5.0)
")

# Calibrated thresholds — see header.
MIN_SHORTFALL="${MIN_SHORTFALL:-0.10}"
MAX_BOX_DISP="${MAX_BOX_DISP:-0.030}"
MAX_FOLLOWER_ERR="${MAX_FOLLOWER_ERR:-0.05}"
FLOOR_Z="${FLOOR_Z:-0.05}"

# Settle-before-sample thresholds — see lib/gz_settle.py. Fixed sleeps here
# used to be the only thing standing between a sample and a still-moving
# system; a single unlucky sample lands in this gate as false slip/false
# stall with mu unchanged. C2 gates on these; C1 (subordinate, does not
# decide M0-C) only warns.
POSE_EPS_M="${POSE_EPS_M:-0.0005}"
JOINT_EPS_RAD_S="${JOINT_EPS_RAD_S:-0.02}"
SETTLE_POLL_S="${SETTLE_POLL_S:-0.15}"
LEFT_TIP="robotiq_85_left_finger_tip_link"
RIGHT_TIP="robotiq_85_right_finger_tip_link"

_gz_joints() {
  gz topic -e -t "$GZ_JS_TOPIC" -n 1 2>/dev/null > "$EVID_DIR/C_js_$1.txt"
  python3 - "$EVID_DIR/C_js_$1.txt" <<'PY'
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

_gz_box() {
  gz topic -e -t "$GZ_POSE_TOPIC" -n 1 2>/dev/null > "$EVID_DIR/C_pose_$1.txt"
  python3 - "$EVID_DIR/C_pose_$1.txt" "$BOX_NAME" <<'PY'
import sys, re
txt, want = open(sys.argv[1]).read(), sys.argv[2]
for blk in re.findall(r'pose\s*\{(.*?)\n\}', txt, re.S):
    n = re.search(r'name:\s*"([^"]+)"', blk)
    if n and n.group(1) == want:
        pos = re.search(r'position\s*\{(.*?)\}', blk, re.S)
        if pos:
            d = dict(re.findall(r'([xyz]):\s*(-?[\d.eE+-]+)', pos.group(1)))
            print(f"{d.get('x','0')} {d.get('y','0')} {d.get('z','0')}")
            break
PY
}

_cleanup_c() {
  gz service -s "$GZ_REMOVE" --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
    --timeout 3000 --req "name: \"${BOX_NAME}\", type: MODEL" >/dev/null 2>&1
  ros2 action send_goal "/${GRIPPER_CTRL}/gripper_cmd" \
    control_msgs/action/GripperCommand \
    "{command: {position: 0.0, max_effort: 50.0}}" >/dev/null 2>&1
}

# ---------------------------------------------------------------------------
cmd "gz topic -l | grep joint_state"
GZ_TOPICS=$(gz topic -l 2>/dev/null || true)
printf '%s\n' "$GZ_TOPICS" | grep -i joint_state | sed 's|^|      |' || true

if ! printf '%s\n' "$GZ_TOPICS" | grep -qF "$GZ_JS_TOPIC"; then
  bad "Gazebo ground-truth topic missing: $GZ_JS_TOPIC"
  warn "Add the JointStatePublisher system plugin to the model's <gazebo> tags."
  warn "Without it M0-C has no independent source and CANNOT pass."
  warn "Do not substitute /joint_states. That defeats the entire check."
  PASS_C=FAIL
elif ! gz_assert_joint "$GZ_JS_TOPIC" "$ACTUATED" 0.0 "${OPEN_TOL_RAD:-0.05}" "gripper open"; then
  bad "precondition failed: $ACTUATED not open at M0-C start -- see message above"
  warn "Every M0-C run before this check assumed the gripper spawned open and"
  warn "never verified it; on this project's own fresh-launch testing that was"
  warn "false 5/5 times (see docs/HANDOFF_M3.md). Command the gripper open"
  warn "(position 0.0) before running m0_verify.sh, or fix the bringup launch"
  warn "to do so automatically."
  PASS_C=FAIL
else
  ok "Gazebo ground-truth topic present"

  # -------------------------------------------------------------------------
  printf '\n  --- C1 (SUBORDINATE): free-space tracking ---\n'
  note "merge sanity only. Per report §3.5 this passes even when the linkage is"
  note "unforced, so it does NOT decide M0-C. A failure here means the merge is"
  note "broken; a pass here means nothing about grasping."

  ros2 param get /robot_state_publisher robot_description 2>/dev/null \
    | sed '1d' > "$EVID_DIR/C_robot_description.urdf" || true

  python3 - "$EVID_DIR/C_robot_description.urdf" "$ACTUATED" \
      > "$EVID_DIR/C_mimic_spec.txt" 2>/dev/null <<'PY'
import sys, xml.etree.ElementTree as ET
try:
    root = ET.parse(sys.argv[1]).getroot()
except Exception as e:
    print(f"ERR {e}"); sys.exit(0)
act = sys.argv[2]
for j in root.findall('joint'):
    m = j.find('mimic')
    if m is not None and m.get('joint') == act:
        print(f"{j.get('name')} {m.get('multiplier','1.0')} {m.get('offset','0.0')}")
PY

  NMIMIC=$(grep -c . "$EVID_DIR/C_mimic_spec.txt" 2>/dev/null || echo 0)
  printf '  mimic joints declared off %s: %s\n' "$ACTUATED" "$NMIMIC"
  sed 's|^|      |' "$EVID_DIR/C_mimic_spec.txt" 2>/dev/null

  if [[ "$NMIMIC" -ne 5 ]]; then
    bad "expected 5 mimic joints in robot_description, found $NMIMIC — merge is wrong"
    C_FAIL=1
  else
    ok "5 mimic joints declared (merge structurally intact)"
  fi

  ros2 action send_goal "/${GRIPPER_CTRL}/gripper_cmd" \
    control_msgs/action/GripperCommand \
    "{command: {position: 0.0, max_effort: 50.0}}" >/dev/null 2>&1
  if ! gz_settle_joint "$GZ_JS_TOPIC" "$JOINT_EPS_RAD_S" 5.0 "$SETTLE_POLL_S" "$ACTUATED"; then
    warn "master joint did not settle before the free-space sample — C1 is"
    warn "subordinate/evidence-only, so this does not fail M0-C, but the"
    warn "captured sample below may not be a true free-space rest state."
  fi
  _gz_joints openfree > "$EVID_DIR/C_joints_openfree.txt"
  ok "free-space open sample captured (evidence only, not a gate)"

  # -------------------------------------------------------------------------
  printf '\n  --- C2 (PASS CRITERION): does contact oppose the linkage? ---\n'

  # Pre-close to a cradle aperture BEFORE placing the box. Two independent
  # reasons, both found by running this, not by inspection:
  #  1. At full-open (~0 rad, ~85mm span) a 40mm box touches neither pad and
  #     free-falls through the gap under gravity before the closing command
  #     ever reaches it.
  #  2. Commanding the FULL 0->0.8 rad range in the same settle window as a
  #     partial-range close means a much higher average closing speed by the
  #     time the fingers actually reach the object — confirmed directly: a
  #     first run of this exact section (no pre-close) closed nearly the full
  #     range (0.0 -> 0.79, stalled=false) in the same 3s SETTLE that a
  #     pre-closed 0.44 -> 0.8 (0.36 rad) run used to stall cleanly at 0.458.
  #     The box was ejected (363mm) in the fast case, retained (13mm) in the
  #     slow case. This isn't a placement workaround, it models how a real
  #     grasp approach works: pre-shape near the object's width, then close
  #     gently, not full-open to full-closed in one commanded motion.
  # PRECLOSE is a rough heuristic ((max_opening - box_width)/max_opening*0.8),
  # not a measured value — scene.yaml's width_map is still null. Adjust and
  # re-run if the box still isn't caught.
  PRECLOSE="${PRECLOSE:-0.45}"
  printf '  pre-closing %s to %s rad (heuristic cradle aperture, not measured)\n' \
         "$ACTUATED" "$PRECLOSE"
  ros2 action send_goal "/${GRIPPER_CTRL}/gripper_cmd" \
    control_msgs/action/GripperCommand \
    "{command: {position: ${PRECLOSE}, max_effort: 50.0}}" >/dev/null 2>&1
  if ! gz_settle_pose "$GZ_POSE_TOPIC" "$POSE_EPS_M" 5.0 "$SETTLE_POLL_S" "$LEFT_TIP" "$RIGHT_TIP"; then
    bad "fingertips did not settle after pre-close — box placement below would target a moving pair of pads"
    C_FAIL=1
  fi

  # Place the box at the (now pre-closed) fingertip midpoint.
  if [[ -z "${BOX_XYZ:-}" ]]; then
    gz topic -e -t "$GZ_POSE_TOPIC" -n 1 2>/dev/null > "$EVID_DIR/C_tips.txt"
    BOX_XYZ=$(python3 - "$EVID_DIR/C_tips.txt" <<'PY'
import sys, re
txt = open(sys.argv[1]).read(); tips = {}
for blk in re.findall(r'pose\s*\{(.*?)\n\}', txt, re.S):
    n = re.search(r'name:\s*"([^"]+)"', blk)
    if not n or not n.group(1).endswith('finger_tip_link'):
        continue
    pos = re.search(r'position\s*\{(.*?)\}', blk, re.S)
    if pos:
        d = dict(re.findall(r'([xyz]):\s*(-?[\d.eE+-]+)', pos.group(1)))
        tips[n.group(1)] = tuple(float(d.get(k, 0)) for k in 'xyz')
if len(tips) >= 2:
    vs = list(tips.values())
    print(" ".join(f"{sum(c[i] for c in vs)/len(vs):.5f}" for i in range(3)))
PY
)
  fi

  if [[ -z "$BOX_XYZ" ]]; then
    bad "could not locate fingertip links in $GZ_POSE_TOPIC"
    warn "set BOX_XYZ='x y z' explicitly and re-run"
    C_FAIL=1
  else
    printf '  box placement (fingertip midpoint): %s\n' "$BOX_XYZ"
    read -r BX BY BZ <<<"$BOX_XYZ"

    SDF="<?xml version='1.0'?><sdf version='1.9'><model name='${BOX_NAME}'>
<pose>${BX} ${BY} ${BZ} 0 0 0</pose><link name='link'>
<inertial><mass>${BOX_MASS}</mass><inertia><ixx>1e-4</ixx><iyy>1e-4</iyy>
<izz>1e-4</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
<collision name='c'><geometry><box><size>${BOX_W} ${BOX_W} ${BOX_H}</size></box>
</geometry></collision>
<visual name='v'><geometry><box><size>${BOX_W} ${BOX_W} ${BOX_H}</size></box>
</geometry></visual></link></model></sdf>"

    # protobuf text-format cannot have a string literal cross a physical line
    # boundary — flatten before it goes anywhere near --req, or gz's parser
    # rejects the request and, since nothing here checks that exit status, the
    # box silently never spawns while the rest of the test sails on regardless.
    SDF_FLAT="${SDF//$'\n'/ }"
    CREATE_OUT=$(gz service -s "$GZ_CREATE" --reqtype gz.msgs.EntityFactory \
      --reptype gz.msgs.Boolean --timeout 5000 \
      --req "sdf: \"${SDF_FLAT//\"/\\\"}\", name: \"${BOX_NAME}\"" 2>&1)
    printf '%s\n' "$CREATE_OUT" | sed 's|^|      |'
    if ! gz_settle_pose "$GZ_POSE_TOPIC" "$POSE_EPS_M" 5.0 "$SETTLE_POLL_S" "$BOX_NAME"; then
      bad "box did not settle after spawn — it is still falling/bouncing, not resting against the pads"
      C_FAIL=1
    fi

    BOX_BEFORE=$(_gz_box before)
    printf '  box before close: %s\n' "${BOX_BEFORE:-<not found>}"
    if [[ -z "$BOX_BEFORE" ]]; then
      bad "test box did not spawn — cannot run the contact test"
      C_FAIL=1
    else
      printf '  commanding %s -> %s rad against a %s m box (bounded at %ss)\n' \
             "$ACTUATED" "$OVERCLOSE" "$BOX_W" "$CMD_TIMEOUT_S"
      gripper_close_and_hold "$GRIPPER_CTRL" "$ACTUATED" "$OVERCLOSE" 50.0 \
        "$CMD_TIMEOUT_S" "$GZ_JS_TOPIC" "$EVID_DIR/C_action_result.txt"
      printf '  gripper_close_and_hold: %s, now holding %s rad (not left driving toward %s)\n' \
             "$GRIPPER_HOLD_RESULT" "$GRIPPER_HOLD_POSITION" "$OVERCLOSE"
      if [[ "$GRIPPER_HOLD_RESULT" == "UNKNOWN_NO_SAMPLE" ]]; then
        bad "overclose call produced no result AND ground truth could not be sampled"
        C_FAIL=1
      fi

      # $SETTLE is now a timeout ceiling, not a blind wait. This is the
      # sample the C2 pass/fail criteria are computed from, so an unsettled
      # system here is a hard fail, not a warning — a mid-motion sample would
      # be measurement noise reported as slip or as a stall.
      if ! gz_settle_joint "$GZ_JS_TOPIC" "$JOINT_EPS_RAD_S" "$SETTLE" "$SETTLE_POLL_S" "$ACTUATED"; then
        bad "master joint did not settle within ${SETTLE}s of the overclose command"
        C_FAIL=1
      fi
      if ! gz_settle_pose "$GZ_POSE_TOPIC" "$POSE_EPS_M" "$SETTLE" "$SETTLE_POLL_S" "$BOX_NAME"; then
        bad "box did not settle within ${SETTLE}s of the overclose command"
        C_FAIL=1
      fi

      _gz_joints closed > "$EVID_DIR/C_joints_closed.txt"
      BOX_AFTER=$(_gz_box after)
      printf '  box after close : %s\n' "${BOX_AFTER:-<not found>}"

      # Read from GRIPPER_HOLD_RESULT, not by re-grepping C_action_result.txt:
      # that file now also contains the hold-position call's own SUCCEEDED
      # result appended after the original close's, and a bare tail-1 grep
      # for stalled:/reached_goal: would pick up the (near-instant, always
      # reached_goal:true) hold call instead of the actual grasp attempt.
      case "$GRIPPER_HOLD_RESULT" in
        STALLED)       STALLED=true;  REACHED=false ;;
        REACHED_GOAL)  STALLED=false; REACHED=true  ;;
        *)             STALLED="";    REACHED=""    ;;
      esac

      python3 "$(dirname "$0")/m0c_eval.py" \
               "$EVID_DIR/C_joints_closed.txt" "$EVID_DIR/C_mimic_spec.txt" \
               "$ACTUATED" "$OVERCLOSE" "$BOX_BEFORE" "$BOX_AFTER" \
               "$MIN_SHORTFALL" "$MAX_BOX_DISP" "$MAX_FOLLOWER_ERR" "$FLOOR_Z" \
               "${STALLED:-}" "${REACHED:-}"
      RC=$?
      [[ $RC -ne 0 ]] && C_FAIL=1
    fi
    _cleanup_c
  fi

  [[ $C_FAIL -eq 0 ]] && PASS_C=PASS || PASS_C=FAIL
fi
printf '\n  ==> M0-C: %s\n' "$PASS_C"

# ---------------------------------------------------------------------------
sec "M0 SUMMARY"
printf '  M0-A  Gazebo version binding ............ %s\n' "$PASS_A"
printf '  M0-B  Gripper controller path ........... %s\n' "$PASS_B"
printf '  M0-C  Mimic joint tracking .............. %s\n' "$PASS_C"
printf '\n  evidence: %s\n' "$EVID_DIR"

if [[ "$PASS_A" == "PASS" && "$PASS_B" == "PASS" && "$PASS_C" == "PASS" ]]; then
  printf '\n  M0 PASS — cleared to start M1.\n'
  exit 0
else
  printf '\n  M0 FAIL — do NOT proceed to M1. Fix the failing check first.\n'
  printf '  A failure here will be misdiagnosed later as a grasp problem.\n'
  exit 1
fi

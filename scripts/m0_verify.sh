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

WORLD="${WORLD:-empty}"                  # gz world name
MODEL="${MODEL:-ur5e_robotiq}"           # gz model name of the merged robot
ACTUATED="${ACTUATED:-finger_joint}"
GRIPPER_CTRL="${GRIPPER_CTRL:-gripper_controller}"
ARM_CTRL="${ARM_CTRL:-arm_controller}"
MOVEIT_CTRL_YAML="${MOVEIT_CTRL_YAML:-config/moveit_controllers.yaml}"
SPAWN_WAIT="${SPAWN_WAIT:-45}"           # donor reports 10-15s; be generous

PASS_A=UNKNOWN; PASS_B=UNKNOWN; PASS_C=UNKNOWN
EVID_DIR="${EVID_DIR:-$(pwd)/m0_evidence}"
mkdir -p "$EVID_DIR"

hr()   { printf '\n%s\n' "════════════════════════════════════════════════════════════"; }
sec()  { hr; printf '§ %s\n' "$1"; hr; }
ok()   { printf '  [PASS] %s\n' "$1"; }
bad()  { printf '  [FAIL] %s\n' "$1"; }
warn() { printf '  [WARN] %s\n' "$1"; }
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
  GTYPE=$(grep -E "^${GRIPPER_CTRL}\b" "$EVID_DIR/B_controllers.txt" \
          | grep -oE '\[[^]]+\]' | tr -d '[]')
  printf '\n  gripper controller type: %s\n' "${GTYPE:-<not found>}"
  case "$GTYPE" in
    *GripperActionController*)
      ok "POSITION-controlled. scene.yaml gripper.squeeze applies, in RADIANS."
      ;;
    *effort*|*Effort*)
      warn "EFFORT-controlled. The position squeeze formula DOES NOT APPLY."
      warn "Set scene.yaml gripper.position_controlled: false and switch the"
      warn "knob to commanded force/effort. State this in the run log."
      ;;
    *)
      bad "unrecognised gripper controller type — resolve before M1"
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
sec "M0-C  Mimic joint tracking"
# Requirement: confirm all mimic joints track the actuated joint correctly in
# gz_ros2_control, CROSS-CHECKED AGAINST GAZEBO'S OWN STATE OUTPUT, not
# /joint_states.
#
# Why this matters here specifically: the donor repo documents an unresolved
# gz_ros2_control state-readback defect on /joint_states. So /joint_states
# agreeing with itself proves nothing. Gazebo's JointStatePublisher system
# plugin on /world/<world>/model/<model>/joint_state is the independent source.

C_FAIL=0
GZ_JS_TOPIC="/world/${WORLD}/model/${MODEL}/joint_state"

cmd "gz topic -l | grep joint_state"
gz topic -l 2>/dev/null | grep -i joint_state | sed 's|^|      |'

if ! gz topic -l 2>/dev/null | grep -q "$GZ_JS_TOPIC"; then
  bad "Gazebo joint_state topic not found: $GZ_JS_TOPIC"
  warn "Add the JointStatePublisher system plugin to the model SDF/URDF gz tags."
  warn "WITHOUT IT THERE IS NO INDEPENDENT GROUND TRUTH AND M0-C CANNOT PASS."
  warn "Do not substitute /joint_states here. That defeats the entire check."
  PASS_C=FAIL; C_FAIL=1
else
  ok "Gazebo ground-truth joint_state topic present"

  # Read expected mimic relationships straight from the parsed URDF.
  ros2 param get /robot_state_publisher robot_description 2>/dev/null \
    | sed '1d' > "$EVID_DIR/C_robot_description.urdf" || true

  python3 - "$EVID_DIR/C_robot_description.urdf" "$ACTUATED" \
      > "$EVID_DIR/C_mimic_spec.txt" 2>/dev/null <<'PY'
import sys, xml.etree.ElementTree as ET
try:
    root = ET.parse(sys.argv[1]).getroot()
except Exception as e:
    print(f"ERR could not parse URDF: {e}"); sys.exit(0)
act = sys.argv[2]
for j in root.findall('joint'):
    m = j.find('mimic')
    if m is not None and m.get('joint') == act:
        print(f"{j.get('name')} {m.get('multiplier','1.0')} {m.get('offset','0.0')}")
PY

  NMIMIC=$(grep -c . "$EVID_DIR/C_mimic_spec.txt" 2>/dev/null || echo 0)
  printf '\n  mimic joints declared off %s: %s\n' "$ACTUATED" "$NMIMIC"
  sed 's|^|      |' "$EVID_DIR/C_mimic_spec.txt" 2>/dev/null

  if [[ "$NMIMIC" -eq 0 ]]; then
    bad "no mimic joints found in robot_description — merge is wrong"
    C_FAIL=1
  elif [[ "$NMIMIC" -ne 5 ]]; then
    warn "expected 5 mimic joints per the donor repo; found $NMIMIC"
    warn "not automatically a failure, but reconcile before proceeding"
  fi

  # Sweep the actuated joint and sample Gazebo's own state at each step.
  printf '\n  sweeping %s and sampling Gazebo ground truth...\n' "$ACTUATED"
  echo "step,commanded,gz_actuated,joint,gz_value,expected,abs_err" \
    > "$EVID_DIR/C_mimic_track.csv"

  STEP=0
  for TARGET in 0.0 0.2 0.4 0.6 0.8 0.4 0.0; do
    STEP=$((STEP+1))
    ros2 action send_goal "/${GRIPPER_CTRL}/gripper_cmd" \
      control_msgs/action/GripperCommand \
      "{command: {position: ${TARGET}, max_effort: 50.0}}" >/dev/null 2>&1
    sleep 2.0

    gz topic -e -t "$GZ_JS_TOPIC" -n 1 > "$EVID_DIR/C_gz_sample_${STEP}.txt" 2>/dev/null

    python3 - "$EVID_DIR/C_gz_sample_${STEP}.txt" "$EVID_DIR/C_mimic_spec.txt" \
             "$ACTUATED" "$TARGET" "$STEP" \
      >> "$EVID_DIR/C_mimic_track.csv" <<'PY'
import sys, re
sample, spec, act, target, step = sys.argv[1:6]
txt = open(sample).read()

# gz msgs.Model text format: repeated joint { name: "x" axis1 { position: v } }
vals = {}
for blk in re.findall(r'joint\s*\{(.*?)\n\}', txt, re.S):
    n = re.search(r'name:\s*"([^"]+)"', blk)
    p = re.search(r'position:\s*(-?[\d.eE+-]+)', blk)
    if n and p:
        vals[n.group(1)] = float(p.group(1))

a = vals.get(act)
if a is None:
    print(f"{step},{target},NA,{act},NA,NA,NA")
    sys.exit(0)

for line in open(spec):
    parts = line.split()
    if len(parts) != 3:
        continue
    name, mult, off = parts[0], float(parts[1]), float(parts[2])
    exp = mult * a + off
    got = vals.get(name)
    if got is None:
        print(f"{step},{target},{a:.6f},{name},ABSENT,{exp:.6f},NA")
    else:
        print(f"{step},{target},{a:.6f},{name},{got:.6f},{exp:.6f},{abs(got-exp):.6f}")
PY
  done

  printf '\n  -- tracking results (Gazebo ground truth vs URDF mimic spec) --\n'
  column -s, -t < "$EVID_DIR/C_mimic_track.csv" | sed 's|^|      |'

  # Tolerance: 1e-3 rad. Anything looser and a partially-tracking linkage slips
  # through, which is exactly the failure mode that masquerades as bad friction.
  MAXERR=$(awk -F, 'NR>1 && $7!="NA" && $7+0>m {m=$7+0} END{printf "%.6f", m+0}' \
           "$EVID_DIR/C_mimic_track.csv")
  ABSENT=$(awk -F, 'NR>1 && $5=="ABSENT"' "$EVID_DIR/C_mimic_track.csv" | wc -l)

  printf '\n  max |error| across sweep: %s rad\n' "$MAXERR"
  printf '  samples where a mimic joint was absent from Gazebo state: %s\n' "$ABSENT"

  if [[ "$ABSENT" -gt 0 ]]; then
    bad "one or more mimic joints missing from Gazebo's own state output"
    warn "the linkage is not being simulated, only drawn"
    C_FAIL=1
  fi
  if awk -v m="$MAXERR" 'BEGIN{exit !(m>0.001)}'; then
    bad "mimic tracking error $MAXERR rad exceeds 1e-3 tolerance"
    warn "a partially-tracking linkage LOOKS like a grip/friction problem."
    warn "fix this before spending any time on M3 friction tuning."
    C_FAIL=1
  else
    ok "all mimic joints track within 1e-3 rad against Gazebo ground truth"
  fi

  # Independent-source comparison. If these two disagree, ros2_control's
  # readback is lying and every downstream measurement is suspect.
  printf '\n  -- cross-check: /joint_states vs Gazebo ground truth --\n'
  ros2 topic echo /joint_states --once 2>/dev/null \
    | tee "$EVID_DIR/C_ros_joint_states.txt" | head -30 | sed 's|^|      |'
  warn "compare $ACTUATED position in C_ros_joint_states.txt against the final"
  warn "gz sample. Divergence => ros2_control readback defect; trust Gazebo."

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

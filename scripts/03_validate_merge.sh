#!/usr/bin/env bash
# 03_validate_merge.sh — expand the merged xacro and check it against the values
#                        recon actually observed. Runs with NO simulation.
#
# This is not M-1's "spawns cleanly" evidence. It is the cheaper check that runs
# first, so that a spawn failure is never something a text-level check could
# have caught in two seconds.
#
# USAGE
#   source /opt/ros/jazzy/setup.bash
#   source ~/ur5e_ws/install/setup.bash     # if the merge pkg is in a workspace
#   bash scripts/03_validate_merge.sh 2>&1 | tee docs/merge_$(date +%Y%m%d_%H%M%S).log
#
# EXIT 0 = merged description is structurally sound, go spawn it
# EXIT 1 = a check failed; the failure names the specific expectation violated

# No pipefail — see the note at the top of 02_bootstrap_noble.sh.
set -u

XACRO="${XACRO:-ur5e_robotiq_description/urdf/ur5e_robotiq.urdf.xacro}"
OUT="${OUT:-/tmp/ur5e_robotiq_merged.urdf}"
MASTER="${MASTER:-robotiq_85_left_knuckle_joint}"
FAILED=0

hr()   { printf '\n%s\n' "════════════════════════════════════════════════════════════"; }
sec()  { hr; printf '§ %s\n' "$1"; hr; }
ok()   { printf '  [PASS] %s\n' "$1"; }
bad()  { printf '  [FAIL] %s\n' "$1"; FAILED=1; }
note() { printf '  [note] %s\n' "$1"; }

printf 'merge validation: %s\n' "$(date -Is)"
printf 'xacro: %s\n' "$XACRO"

# ---------------------------------------------------------------------------
sec "1. Expand the xacro"
if [[ ! -f "$XACRO" ]]; then
  bad "xacro not found at $XACRO"
  note "run this from the repo root, or set XACRO=<path>"
  exit 1
fi

# name:= must be passed explicitly, even though the xacro declares a default
# for it. `$(arg name)` is used on the root <robot> tag itself, and xacro
# resolves substitutions top-down — by the time that attribute is evaluated,
# the in-document <xacro:arg name="name"> declaration hasn't been registered
# yet. This is not specific to this file: the proven-working upstream
# ur_simulation_gz/urdf/ur_gz.urdf.xacro has the identical pattern and fails
# the identical way if you invoke it with no args at all. Their launch files
# always pass name:= explicitly; do the same here.
if xacro "$XACRO" name:=ur5e_robotiq > "$OUT" 2>/tmp/xacro_err.txt; then
  ok "expanded to $OUT ($(wc -c <"$OUT") bytes)"
else
  bad "xacro expansion failed"
  sed 's|^|      |' /tmp/xacro_err.txt | head -40
  note "common causes, in order of likelihood:"
  note "  - coupler_output_link name wrong (see §3 below for the real link list)"
  note "  - a ur_description config/<ur_type>/*.yaml path that does not exist"
  note "  - macro param name mismatch"
  exit 1
fi

# ---------------------------------------------------------------------------
sec "2. Structural parse"
python3 - "$OUT" "$MASTER" <<'PY'
import sys, xml.etree.ElementTree as ET
from collections import Counter

path, master = sys.argv[1], sys.argv[2]
root = ET.parse(path).getroot()

links  = [l.get('name') for l in root.findall('link')]
joints = root.findall('joint')
fail = 0

def ok(m):  print(f"  [PASS] {m}")
def bad(m):
    global fail
    print(f"  [FAIL] {m}"); fail = 1

print(f"  links: {len(links)}   joints: {len(joints)}")

# --- duplicate links/joints -------------------------------------------------
for kind, names in (("link", links), ("joint", [j.get('name') for j in joints])):
    dupes = [n for n, c in Counter(names).items() if c > 1]
    if dupes:
        bad(f"duplicate {kind} names: {dupes}")
    else:
        ok(f"no duplicate {kind} names")

# --- tree connectivity: every link except the root must be a joint child -----
children = {j.find('child').get('link') for j in joints if j.find('child') is not None}
parents  = {j.find('parent').get('link') for j in joints if j.find('parent') is not None}
roots = [l for l in links if l not in children]
if len(roots) == 1:
    ok(f"single kinematic root: {roots[0]}")
else:
    bad(f"expected exactly 1 root link, found {len(roots)}: {roots}")

orphans = [l for l in links if l not in children and l not in parents]
if orphans and orphans != roots:
    bad(f"orphan links (in no joint): {[o for o in orphans if o not in roots]}")
else:
    ok("no orphan links")

# --- the UR flange chain, confirmed by recon --------------------------------
for want in ('wrist_3_link', 'flange', 'tool0'):
    if any(l.endswith(want) for l in links):
        ok(f"flange chain link present: {want}")
    else:
        bad(f"flange chain link MISSING: {want}")

# --- gripper present --------------------------------------------------------
grip_links = [l for l in links if 'robotiq' in l or '85' in l]
if grip_links:
    ok(f"gripper links present ({len(grip_links)})")
else:
    bad("no Robotiq links found — gripper macro did not expand into the tree")

# --- master actuated joint --------------------------------------------------
mj = [j for j in joints if j.get('name', '').endswith(master)]
if not mj:
    bad(f"master joint '{master}' NOT found")
    print("       revolute/continuous joints actually present:")
    for j in joints:
        if j.get('type') in ('revolute', 'continuous'):
            print(f"         {j.get('name')}  ({j.get('type')})")
else:
    j = mj[0]
    lim = j.find('limit')
    lo = float(lim.get('lower')) if lim is not None and lim.get('lower') else None
    hi = float(lim.get('upper')) if lim is not None and lim.get('upper') else None
    ok(f"master joint present: {j.get('name')} ({j.get('type')}) limits [{lo}, {hi}]")
    # Recon observed 0.0 -> 0.8 rad, 0.0 = open.
    if lo is not None and hi is not None:
        if abs(lo) < 1e-6 and abs(hi - 0.8) < 0.05:
            ok("limits match recon (0.0 open -> 0.8 closed)")
        else:
            bad(f"limits [{lo}, {hi}] differ from recon's [0.0, 0.8] — "
                "scene.yaml squeeze is in rad and assumes that range")

# --- mimic joints -----------------------------------------------------------
expected = {
    'robotiq_85_right_knuckle_joint':       -1.0,
    'robotiq_85_left_inner_knuckle_joint':   1.0,
    'robotiq_85_right_inner_knuckle_joint': -1.0,
    'robotiq_85_left_finger_tip_joint':     -1.0,
    'robotiq_85_right_finger_tip_joint':     1.0,
}
mimics = {}
for j in joints:
    m = j.find('mimic')
    if m is not None:
        mimics[j.get('name')] = (m.get('joint'), float(m.get('multiplier', 1.0)),
                                 float(m.get('offset', 0.0)))

print(f"\n  mimic joints found: {len(mimics)} (recon observed 5)")
for n, (src, mult, off) in sorted(mimics.items()):
    print(f"    {n:<40} -> {src}  mult={mult}  off={off}")

if len(mimics) != 5:
    bad(f"expected 5 mimic joints, found {len(mimics)}")
else:
    ok("5 mimic joints present")

for name, want_mult in expected.items():
    hit = [k for k in mimics if k.endswith(name)]
    if not hit:
        bad(f"mimic joint missing: {name}")
        continue
    src, mult, off = mimics[hit[0]]
    if not src.endswith(master):
        bad(f"{name} mimics '{src}', expected '{master}'")
    elif abs(mult - want_mult) > 1e-6:
        bad(f"{name} multiplier {mult}, recon observed {want_mult}")
    else:
        ok(f"{name} -> mult {mult}")

# --- velocity limits across the whole linkage -------------------------------
# gz_ros2_control drives the mimic followers with a 500/s correction and no
# gain factor -- ten times stiffer than the master's own 50/s P-loop (see
# gz_system.cpp, write(), the mimic block at the end). A follower with no
# <limit> has no ceiling on that correction, and the four inner-knuckle and
# finger-tip joints shipped without one.
#
# Measured consequence before this was fixed: with no gripper goal active at
# all, the master was dragged 0.0956 -> 0.2546 rad while its own controller
# commanded full retreat, and showed a sample at 3.5x its own velocity clamp.
# A joint cannot beat its clamp under its own command; the followers were
# moving it.
#
# So every joint in the linkage must carry a velocity limit, and they must all
# be equal -- every multiplier is +/-1.0, so a correctly-tracking follower
# moves at exactly the master's rate. Equal limits mean a dragging follower can
# move at most as fast as the master's retreat, which is the property that
# lets the master fight to a draw. Any joint left unbounded gives that away.
linkage = [
    'robotiq_85_left_knuckle_joint',
    'robotiq_85_right_knuckle_joint',
    'robotiq_85_left_inner_knuckle_joint',
    'robotiq_85_right_inner_knuckle_joint',
    'robotiq_85_left_finger_tip_joint',
    'robotiq_85_right_finger_tip_joint',
]

print("\n  linkage velocity limits:")
vels = {}
for want in linkage:
    hit = [j for j in joints if j.get('name', '').endswith(want)]
    if not hit:
        bad(f"linkage joint not found: {want}")
        continue
    lim = hit[0].find('limit')
    v = lim.get('velocity') if lim is not None else None
    if v is None:
        bad(
            f"{want}: NO velocity limit. This joint is driven by the 500/s "
            "mimic correction with no ceiling and can drag the master."
        )
        continue
    vels[want] = float(v)
    print(f"    {want:<44} velocity={v}")

if len(vels) == len(linkage):
    distinct = sorted(set(vels.values()))
    if len(distinct) == 1:
        ok(f"all 6 linkage joints share velocity limit {distinct[0]}")
    else:
        bad(
            f"linkage velocity limits are not equal: {distinct}. The fastest "
            "joint sets how hard the linkage can drag the master, so a split "
            "here is the asymmetry, whichever side looks 'correct'."
        )

sys.exit(fail)
PY
[[ $? -ne 0 ]] && FAILED=1

# ---------------------------------------------------------------------------
sec "3. Link inventory  (read this if §1 failed on the coupler link name)"
python3 - "$OUT" <<'PY'
import sys, xml.etree.ElementTree as ET
root = ET.parse(sys.argv[1]).getroot()
names = [l.get('name') for l in root.findall('link')]
for n in names:
    mark = "  <-- candidate coupler output" if ('mount' in n or 'coupler' in n or 'adapter' in n) else ""
    print(f"    {n}{mark}")
PY

# ---------------------------------------------------------------------------
sec "4. ros2_control blocks"
python3 - "$OUT" <<'PY'
import sys, xml.etree.ElementTree as ET
root = ET.parse(sys.argv[1]).getroot()
blocks = root.findall('ros2_control')
print(f"  <ros2_control> blocks: {len(blocks)}")
fail = 0
for b in blocks:
    hw = b.find('hardware')
    plugin = hw.find('plugin').text.strip() if hw is not None and hw.find('plugin') is not None else "<none>"
    joints = [j.get('name') for j in b.findall('joint')]
    cmd = [j.get('name') for j in b.findall('joint') if j.find('command_interface') is not None]
    print(f"\n    name={b.get('name')}  type={b.get('type')}")
    print(f"      plugin: {plugin}")
    print(f"      joints ({len(joints)}): {joints}")
    print(f"      with command interfaces ({len(cmd)}): {cmd}")
    if plugin == "<none>":
        print("      [FAIL] no hardware plugin declared"); fail = 1
    if 'fake' in plugin.lower() or 'mock' in plugin.lower():
        print("      [FAIL] MOCK hardware plugin — this grasps nothing but looks healthy")
        print("             set sim_ignition:=true, not use_fake_hardware")
        fail = 1

if len(blocks) < 2:
    print("\n  [FAIL] expected 2 blocks (arm + gripper). The gripper macro emits its")
    print("         own when include_ros2_control is true — check it wasn't disabled.")
    fail = 1
else:
    print("\n  [PASS] both arm and gripper ros2_control blocks present")

sys.exit(fail)
PY
[[ $? -ne 0 ]] && FAILED=1

# ---------------------------------------------------------------------------
sec "5. Mimic joints must NOT carry command interfaces"
# Under sim_ignition the followers get only <param name="mimic"/multiplier>.
# If any follower has a command_interface, a controller can claim it and fight
# the constraint solver. The symptom is a jittering gripper that reads as a
# friction problem.
python3 - "$OUT" "$MASTER" <<'PY'
import sys, xml.etree.ElementTree as ET
root = ET.parse(sys.argv[1]).getroot()
master = sys.argv[2]
mimic_names = {j.get('name') for j in root.findall('joint') if j.find('mimic') is not None}
fail = 0
for b in root.findall('ros2_control'):
    for j in b.findall('joint'):
        if j.get('name') in mimic_names and j.find('command_interface') is not None:
            print(f"  [FAIL] mimic joint {j.get('name')} has a command_interface")
            fail = 1
if not fail:
    print("  [PASS] no mimic joint exposes a command interface")
print(f"\n  reminder: the gripper planning group and gripper controller must each")
print(f"  claim exactly one joint — {master} — and none of the 5 followers.")
sys.exit(fail)
PY
[[ $? -ne 0 ]] && FAILED=1

# ---------------------------------------------------------------------------
sec "SUMMARY"
if [[ $FAILED -eq 0 ]]; then
  cat <<EOF
  MERGE VALIDATES.

  Structure is sound and matches what recon observed. That is NOT the same as
  M-1 being done — it proves the text is consistent, not that it spawns.

  Next:
    1. spawn it in Harmonic, confirm no errors and the arm does not free-fall
    2. ros2 control list_hardware_interfaces  -> BOTH components must be active
    3. MoveIt Setup Assistant, two groups: arm, gripper
    4. then, and only then, M0

  Still open by design: grasp.tcp_offset in config/scene.yaml. Measure it from
  the expanded URDF now that one exists — tool0 forward to the finger-pad
  contact midpoint.
EOF
  exit 0
else
  printf '\n  MERGE VALIDATION FAILED — fix before spawning.\n'
  printf '  A structural error here becomes an unexplained grasp failure later.\n'
  exit 1
fi

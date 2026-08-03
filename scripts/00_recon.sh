#!/usr/bin/env bash
# 00_recon.sh — ground-truth collection before any URDF merge is written.
#
# WHY THIS EXISTS
#   The URDF merge needs exact xacro macro signatures, frame names, joint limits
#   and mimic multipliers. Those have changed across ROS 2 releases, and the
#   reference repo's README is Humble-first. Writing the merge from inference
#   would produce something that looks right and is wrong.
#
#   So: collect facts here, then write the merge from the facts.
#
# USAGE
#   source /opt/ros/jazzy/setup.bash
#   bash scripts/00_recon.sh 2>&1 | tee docs/recon_$(date +%Y%m%d_%H%M%S).log
#
# This script is READ-ONLY. It installs nothing and launches no simulation.

set -uo pipefail

RECON_DIR="${RECON_DIR:-$(pwd)/recon_out}"
mkdir -p "$RECON_DIR"

hr()  { printf '\n%s\n' "════════════════════════════════════════════════════════════"; }
sec() { hr; printf '§ %s\n' "$1"; hr; }
note(){ printf '  [note] %s\n' "$1"; }
miss(){ printf '  [MISSING] %s\n' "$1"; }

printf 'recon run: %s\n' "$(date -Is)"
printf 'output dir: %s\n' "$RECON_DIR"

# ---------------------------------------------------------------------------
sec "0. Host and distro"
# Jazzy requires Ubuntu 24.04 Noble. Anything else and the rest is moot.
grep -E 'PRETTY_NAME|VERSION_ID' /etc/os-release
printf 'ROS_DISTRO      = %s\n' "${ROS_DISTRO:-<unset>}"
printf 'ROS_VERSION     = %s\n' "${ROS_VERSION:-<unset>}"
printf 'RMW             = %s\n' "${RMW_IMPLEMENTATION:-<default>}"
printf 'GZ_VERSION      = %s\n' "${GZ_VERSION:-<unset>}"
printf 'kernel          = %s\n' "$(uname -r)"

if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
  note "ROS_DISTRO is not 'jazzy'. Did you source /opt/ros/jazzy/setup.bash?"
fi

# ---------------------------------------------------------------------------
sec "1. Gazebo version binding  (feeds M0-A)"
# The spec requires evidence, not assumption, that we are on Harmonic (gz-sim 8.x)
# and that nothing is dragging in Fortress (gz-sim 6) or Classic.
if command -v gz >/dev/null 2>&1; then
  printf '$ gz sim --versions\n'; gz sim --versions
  printf '\n$ gz --versions\n';    gz --versions
else
  miss "gz binary not on PATH — Gazebo Harmonic may not be installed"
fi

if command -v ign >/dev/null 2>&1; then
  note "!! 'ign' binary present. That is Fortress-era. Investigate whether"
  note "!! anything actually depends on it, or M0-A cannot pass cleanly."
  ign gazebo --versions 2>/dev/null || true
fi

printf '\n-- installed gz apt packages --\n'
dpkg -l 2>/dev/null | grep -Ei 'gz-(sim|tools|msgs|transport|physics|rendering)|ignition|gazebo' \
  | awk '{printf "  %-42s %s\n", $2, $3}' || miss "no gz packages found via dpkg"

printf '\n-- DART physics plugin present? (Harmonic default engine) --\n'
find /usr/lib /opt -name '*physics-dartsim*' 2>/dev/null | head -5 \
  || miss "dartsim physics plugin not located"

# ---------------------------------------------------------------------------
sec "2. Relevant ROS 2 package inventory"
for pkg in \
    ur_description ur_simulation_gz ur_moveit_config ur_robot_driver \
    robotiq_description robotiq_controllers \
    gz_ros2_control ros_gz_sim ros_gz_bridge \
    moveit_ros_planning_interface moveit_kinematics moveit_planners_ompl \
    controller_manager joint_trajectory_controller position_controllers
do
  if p=$(ros2 pkg prefix "$pkg" 2>/dev/null); then
    v=$(dpkg-query -W -f='${Version}' "ros-${ROS_DISTRO}-${pkg//_/-}" 2>/dev/null || echo "source")
    printf '  %-34s %-46s %s\n' "$pkg" "$p" "$v"
  else
    printf '  %-34s %s\n' "$pkg" "*** NOT FOUND ***"
  fi
done

note "robotiq_description missing is EXPECTED and is the core M-1 problem."
note "UR ships no integrated gripper — that package is what we must source."

# ---------------------------------------------------------------------------
sec "3. UR xacro macro signature  (open question #1)"
# Parameter names on ur_macro.xacro have changed between releases. Read them.
UR_DESC=$(ros2 pkg prefix ur_description 2>/dev/null)/share/ur_description
if [[ -d "$UR_DESC" ]]; then
  printf 'ur_description share dir: %s\n\n' "$UR_DESC"
  printf -- '-- urdf/ tree --\n'
  find "$UR_DESC/urdf" -maxdepth 2 -type f | sed 's|^|  |'

  for f in "$UR_DESC"/urdf/ur_macro.xacro "$UR_DESC"/urdf/ur.urdf.xacro; do
    [[ -f "$f" ]] || continue
    printf '\n-- macro/arg signature: %s --\n' "$(basename "$f")"
    grep -nE '<xacro:macro|<xacro:arg|<xacro:property name="(prefix|tf_prefix)"' "$f" \
      | sed 's|^|  |'
  done

  printf '\n-- declared tool/flange frames --\n'
  grep -rnoE '"(tool0|flange|wrist_3_link|tool0_controller)"' "$UR_DESC/urdf" \
    | sort -u | head -20 | sed 's|^|  |'
else
  miss "ur_description not installed — cannot read macro signature"
fi

# ---------------------------------------------------------------------------
sec "4. Robotiq description  (open questions #2, #4)"
ROBOTIQ=$(ros2 pkg prefix robotiq_description 2>/dev/null)/share/robotiq_description
if [[ -d "$ROBOTIQ" ]]; then
  printf 'robotiq_description share dir: %s\n\n' "$ROBOTIQ"
  find "$ROBOTIQ/urdf" -maxdepth 2 -type f 2>/dev/null | sed 's|^|  |'

  printf '\n-- xacro macro signatures --\n'
  grep -rnE '<xacro:macro' "$ROBOTIQ/urdf" 2>/dev/null | sed 's|^|  |'

  printf '\n-- mimic joint declarations (expect 5 off finger_joint) --\n'
  grep -rnE '<mimic ' "$ROBOTIQ/urdf" 2>/dev/null | sed 's|^|  |'

  printf '\n-- finger_joint limit block --\n'
  grep -rnA4 'name="finger_joint"' "$ROBOTIQ/urdf" 2>/dev/null | sed 's|^|  |'
else
  miss "robotiq_description NOT installed."
  note "Candidate sources, in order of preference:"
  note "  a) apt:  ros-jazzy-robotiq-description   (check availability first)"
  note "  b) src:  https://github.com/PickNikRobotics/ros2_robotiq_gripper"
  note "  c) src:  darshmenon/UR3_ROS2_PICK_AND_PLACE  (vendored copy, Humble-first)"
  printf '\n$ apt-cache policy ros-%s-robotiq-description\n' "${ROS_DISTRO:-jazzy}"
  apt-cache policy "ros-${ROS_DISTRO:-jazzy}-robotiq-description" 2>/dev/null \
    || note "apt-cache lookup failed"
fi

# ---------------------------------------------------------------------------
sec "5. Arm-only baseline sanity  (does UR's own path work at all?)"
# The spec's premise is that ur_sim_moveit.launch.py works out of the box.
# Confirm the launch file exists before relying on it. Do NOT run it here.
UR_GZ=$(ros2 pkg prefix ur_simulation_gz 2>/dev/null)/share/ur_simulation_gz
if [[ -d "$UR_GZ" ]]; then
  printf 'ur_simulation_gz launch files:\n'
  find "$UR_GZ/launch" -type f 2>/dev/null | sed 's|^|  |'
  printf '\n-- gz version referenced in package.xml --\n'
  grep -E 'gz|ign|gazebo' "$UR_GZ/package.xml" 2>/dev/null | sed 's|^|  |' \
    || note "no gz dependency string found in package.xml"
else
  miss "ur_simulation_gz not installed"
fi

# ---------------------------------------------------------------------------
sec "6. Generate and parse arm-only URDF  (baseline before merge)"
# Produces the pre-merge reference. After the merge, diff against this to see
# exactly what the gripper added.
if [[ -f "$UR_DESC/urdf/ur.urdf.xacro" ]]; then
  OUT="$RECON_DIR/ur5e_armonly.urdf"
  if xacro "$UR_DESC/urdf/ur.urdf.xacro" ur_type:=ur5e name:=ur5e > "$OUT" 2>"$RECON_DIR/xacro_err.txt"; then
    printf 'wrote %s (%s bytes)\n\n' "$OUT" "$(stat -c%s "$OUT")"
    python3 - "$OUT" <<'PY'
import sys, xml.etree.ElementTree as ET
r = ET.parse(sys.argv[1]).getroot()
joints = r.findall('joint')
links  = r.findall('link')
print(f"  links : {len(links)}")
print(f"  joints: {len(joints)}")
print("\n  -- non-fixed joints, with limits --")
for j in joints:
    if j.get('type') == 'fixed':
        continue
    lim = j.find('limit')
    lo = lim.get('lower', '?') if lim is not None else '?'
    hi = lim.get('upper', '?') if lim is not None else '?'
    ef = lim.get('effort', '?') if lim is not None else '?'
    print(f"    {j.get('name'):<28} {j.get('type'):<10} [{lo}, {hi}] effort={ef}")
mim = [j for j in joints if j.find('mimic') is not None]
print(f"\n  -- mimic joints: {len(mim)} (expect 0 pre-merge) --")
for j in mim:
    m = j.find('mimic')
    print(f"    {j.get('name')} mimics {m.get('joint')} "
          f"mult={m.get('multiplier','1')} off={m.get('offset','0')}")
print("\n  -- candidate TCP frames --")
for l in links:
    if l.get('name') in ('tool0','flange','wrist_3_link','tool0_controller'):
        print(f"    {l.get('name')}")
PY
  else
    miss "xacro expansion failed — see $RECON_DIR/xacro_err.txt"
    head -30 "$RECON_DIR/xacro_err.txt" | sed 's|^|  |'
  fi
else
  miss "ur.urdf.xacro not found; skipping URDF generation"
fi

# ---------------------------------------------------------------------------
sec "7. Summary — what still needs answering"
cat <<'EOF'
  Transcribe these into docs/M-1_reference_report.md §6 and into scene.yaml,
  replacing every `# RECON` placeholder. Do not write the URDF merge until all
  five are filled with observed values.

    [ ] 1. ur_macro.xacro parameter names          -> §3 above
    [ ] 2. robotiq_description source + version    -> §4 above
    [ ] 3. UR5e flange frame name + mount xyz/rpy  -> §3, §6 above
    [ ] 4. finger_joint limits + 5 mimic mult/off  -> §4 above
    [ ] 5. flange -> finger-pad-midpoint offset    -> measure post-merge

  Then, and only then:
    - write the merge xacro
    - run MoveIt Setup Assistant for two groups: `arm`, `gripper`
    - confirm clean spawn in Harmonic
    - THAT is M-1 done. M0 comes after.
EOF

hr
printf 'recon complete: %s\n' "$(date -Is)"
printf 'artifacts in: %s\n' "$RECON_DIR"

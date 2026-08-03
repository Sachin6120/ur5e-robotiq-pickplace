#!/usr/bin/env bash
# 02_bootstrap_noble.sh — fresh Ubuntu 24.04 (Noble) -> ROS 2 Jazzy + Gazebo Harmonic
#                          + UR5e + MoveIt 2 + gz_ros2_control, ready for recon.
#
# WHY A SCRIPT AND NOT A CHECKLIST
#   M0-A exists to catch a stray Fortress/Classic dependency. The cheapest way to
#   pass M0-A is to never create the condition: install from the ROS 2 Jazzy repo
#   only, let it pull its default Gazebo, and verify the result before moving on.
#   A hand-typed install is exactly where a stray `gazebo11` or `ignition-*` gets
#   in, usually from a blog post written for Fortress.
#
# USAGE
#   bash scripts/02_bootstrap_noble.sh 2>&1 | tee bootstrap_$(date +%Y%m%d_%H%M%S).log
#
#   Idempotent — safe to re-run. Prompts for sudo.
#
# EXIT CODES
#   0 = stack installed and verified, proceed to 00_recon.sh
#   1 = a verification step failed; read the FAIL line, do not proceed
#   2 = wrong host OS
#
# NOT DONE HERE (deliberately)
#   - the URDF merge            (blocked on recon output)
#   - the MoveIt config         (Setup Assistant, after the merge)
#   - anything MTC or MongoDB   (deferred by the project spec)

set -uo pipefail

WS="${WS:-$HOME/ur5e_ws}"
ROS_DISTRO_TARGET=jazzy
FAILED=0

hr()   { printf '\n%s\n' "════════════════════════════════════════════════════════════"; }
sec()  { hr; printf '§ %s\n' "$1"; hr; }
ok()   { printf '  [OK]   %s\n' "$1"; }
bad()  { printf '  [FAIL] %s\n' "$1"; FAILED=1; }
warn() { printf '  [WARN] %s\n' "$1"; }
note() { printf '  [note] %s\n' "$1"; }
run()  { printf '\n  $ %s\n' "$*"; "$@"; }

printf 'bootstrap run: %s\n' "$(date -Is)"
printf 'workspace: %s\n' "$WS"

# ---------------------------------------------------------------------------
sec "0. Host guard"
. /etc/os-release
printf '  detected: %s (%s)\n' "$PRETTY_NAME" "$VERSION_ID"
if [[ "$VERSION_ID" != "24.04" ]]; then
  bad "ROS 2 Jazzy requires Ubuntu 24.04 Noble. Found $VERSION_ID."
  note "Jazzy has no binaries for other releases. Building from source is not"
  note "worth it here — reimage or use a 24.04 VM/container instead."
  exit 2
fi
ok "Ubuntu 24.04 Noble confirmed"

if [[ $EUID -eq 0 ]]; then
  bad "do not run this as root; it uses sudo where needed"
  exit 2
fi

# Catch a pre-existing contaminated system before we add to it.
if dpkg -l 2>/dev/null | grep -qiE '^ii\s+(gazebo11|libgazebo11|ignition-gazebo|ignition-common)'; then
  warn "Fortress/Classic-era packages ALREADY present on this machine:"
  dpkg -l | grep -iE '^ii\s+(gazebo11|libgazebo11|ignition-)' | awk '{print "         " $2}'
  warn "On a genuinely fresh install this list should be empty."
  warn "These will make M0-A fail. Remove them before continuing:"
  warn "  sudo apt remove --purge 'gazebo11*' 'libgazebo11*' 'ignition-*' && sudo apt autoremove"
else
  ok "no Fortress/Classic packages present (clean starting point)"
fi

# ---------------------------------------------------------------------------
sec "1. Base system + universe repo"
run sudo apt update
run sudo apt install -y software-properties-common curl gnupg lsb-release \
                        build-essential cmake git python3-pip
run sudo add-apt-repository -y universe
run sudo apt update
ok "base tooling installed"

# ---------------------------------------------------------------------------
sec "2. ROS 2 apt source"
# Current mechanism is the ros2-apt-source .deb from ros-infrastructure, which
# carries the keyring and sources.list entry together. This replaced the older
# manual apt-key / raw sources.list approach — do not mix the two, a leftover
# manual entry plus this package gives duplicate-source warnings and can pin
# the wrong versions.
#
# Authoritative reference (fetch it yourself if anything below misbehaves):
#   https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

if [[ -f /etc/apt/sources.list.d/ros2.list ]] && \
   ! dpkg -l ros2-apt-source >/dev/null 2>&1; then
  warn "legacy /etc/apt/sources.list.d/ros2.list found without ros2-apt-source"
  warn "remove it to avoid a duplicate source: sudo rm /etc/apt/sources.list.d/ros2.list"
fi

if dpkg -l ros2-apt-source >/dev/null 2>&1; then
  ok "ros2-apt-source already installed"
else
  ROS_APT_SOURCE_VERSION=$(curl -fsSL \
    https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -oP '"tag_name":\s*"\K[^"]+' || true)

  if [[ -z "$ROS_APT_SOURCE_VERSION" ]]; then
    bad "could not resolve latest ros-apt-source release from the GitHub API"
    note "check network, or follow the official docs page manually:"
    note "  https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html"
    exit 1
  fi
  printf '  resolved ros-apt-source version: %s\n' "$ROS_APT_SOURCE_VERSION"

  DEB="/tmp/ros2-apt-source.deb"
  run curl -fsSL -o "$DEB" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
  run sudo dpkg -i "$DEB"
fi

run sudo apt update
if apt-cache policy ros-${ROS_DISTRO_TARGET}-desktop 2>/dev/null | grep -q 'Candidate:'; then
  ok "ROS 2 ${ROS_DISTRO_TARGET} packages visible to apt"
else
  bad "apt cannot see ros-${ROS_DISTRO_TARGET}-* packages — apt source is not working"
  exit 1
fi

# ---------------------------------------------------------------------------
sec "3. ROS 2 Jazzy + dev tools"
run sudo apt install -y "ros-${ROS_DISTRO_TARGET}-desktop" ros-dev-tools
ok "ros-${ROS_DISTRO_TARGET}-desktop installed"

# ---------------------------------------------------------------------------
sec "4. Gazebo Harmonic via the ROS 2 pairing"
# Jazzy's DEFAULT Gazebo is Harmonic. Installing ros-jazzy-ros-gz from the ROS 2
# repo pulls Harmonic as a dependency. This is the low-risk path.
#
# Do NOT add packages.osrfoundation.org as an extra apt source unless you have a
# specific reason. Mixing it with the ROS 2 repo is a common way to end up with
# a version mismatch that M0-A then has to catch.
run sudo apt install -y "ros-${ROS_DISTRO_TARGET}-ros-gz" \
                        "ros-${ROS_DISTRO_TARGET}-ros-gz-sim" \
                        "ros-${ROS_DISTRO_TARGET}-ros-gz-bridge" \
                        "ros-${ROS_DISTRO_TARGET}-gz-ros2-control"
ok "ros_gz + gz_ros2_control installed"

# ---------------------------------------------------------------------------
sec "5. MoveIt 2, controllers, UR packages"
run sudo apt install -y \
  "ros-${ROS_DISTRO_TARGET}-moveit" \
  "ros-${ROS_DISTRO_TARGET}-moveit-setup-assistant" \
  "ros-${ROS_DISTRO_TARGET}-moveit-planners-ompl" \
  "ros-${ROS_DISTRO_TARGET}-moveit-ros-planning-interface" \
  "ros-${ROS_DISTRO_TARGET}-ros2-control" \
  "ros-${ROS_DISTRO_TARGET}-ros2-controllers" \
  "ros-${ROS_DISTRO_TARGET}-controller-manager" \
  "ros-${ROS_DISTRO_TARGET}-joint-trajectory-controller" \
  "ros-${ROS_DISTRO_TARGET}-position-controllers" \
  "ros-${ROS_DISTRO_TARGET}-joint-state-broadcaster" \
  "ros-${ROS_DISTRO_TARGET}-ros2controlcli" \
  "ros-${ROS_DISTRO_TARGET}-xacro" \
  "ros-${ROS_DISTRO_TARGET}-tf2-ros" \
  "ros-${ROS_DISTRO_TARGET}-tf2-geometry-msgs"
ok "MoveIt 2 + ros2_control stack installed"

# UR packages: prefer binaries, fall back to source. Report which happened —
# the M-1 report needs to record this.
mkdir -p "$WS/src"
UR_FROM=binary
for p in ur-description ur-simulation-gz ur-moveit-config; do
  if apt-cache policy "ros-${ROS_DISTRO_TARGET}-${p}" 2>/dev/null | grep -q 'Candidate: [^(]'; then
    run sudo apt install -y "ros-${ROS_DISTRO_TARGET}-${p}"
  else
    warn "no binary for ros-${ROS_DISTRO_TARGET}-${p}"
    UR_FROM=source
  fi
done

if [[ "$UR_FROM" == "source" ]]; then
  note "cloning UR packages from source into $WS/src"
  cd "$WS/src"
  [[ -d Universal_Robots_ROS2_Description ]] || \
    run git clone -b "${ROS_DISTRO_TARGET}" \
      https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
  [[ -d Universal_Robots_ROS2_GZ_Simulation ]] || \
    run git clone -b "${ROS_DISTRO_TARGET}" \
      https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git
fi
printf '\n  UR packages sourced from: %s\n' "$UR_FROM"

# ---------------------------------------------------------------------------
sec "6. Robotiq description  (the actual M-1 problem)"
# UR ships no integrated gripper. This is the package the whole merge depends on
# and the one most likely to be missing on Jazzy.
ROBOTIQ_FROM=none
if apt-cache policy "ros-${ROS_DISTRO_TARGET}-robotiq-description" 2>/dev/null \
     | grep -q 'Candidate: [^(]'; then
  run sudo apt install -y "ros-${ROS_DISTRO_TARGET}-robotiq-description"
  ROBOTIQ_FROM=binary
  ok "robotiq_description installed from apt"
else
  warn "no apt binary for robotiq_description on ${ROS_DISTRO_TARGET}"
  note "falling back to source: PickNikRobotics/ros2_robotiq_gripper"
  note "NOTE: that repo's main branch targets Humble/Iron/Rolling. Jazzy support"
  note "      is claimed but unverified. If the build fails, that failure is"
  note "      M-1 information — record it, do not paper over it."
  cd "$WS/src"
  [[ -d ros2_robotiq_gripper ]] || \
    run git clone https://github.com/PickNikRobotics/ros2_robotiq_gripper.git
  ROBOTIQ_FROM=source-picknik
fi
printf '\n  robotiq description sourced from: %s\n' "$ROBOTIQ_FROM"

# ---------------------------------------------------------------------------
sec "7. Resolve dependencies and build the workspace"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  run sudo rosdep init || warn "rosdep already initialised"
fi
run rosdep update

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_TARGET}/setup.bash"

cd "$WS"
if [[ -n "$(ls -A "$WS/src" 2>/dev/null)" ]]; then
  run rosdep install --from-paths src --ignore-src -r -y \
      --rosdistro "${ROS_DISTRO_TARGET}" || warn "rosdep reported unresolved keys — read them"
  if run colcon build --symlink-install; then
    ok "workspace built"
  else
    bad "colcon build failed — this is M-1 information, record the error"
    note "most likely cause: a source package targeting Humble, not Jazzy"
  fi
else
  note "src/ is empty (everything came from binaries) — nothing to build yet"
fi

# ---------------------------------------------------------------------------
sec "8. Verification  (pre-flight for M0-A)"
# This is not M0-A. M0-A runs against the merged robot with the sim up. This is
# the weaker check that the toolchain itself is sane before we invest in a merge.

if command -v gz >/dev/null 2>&1; then
  GZV=$(gz sim --versions 2>&1 | head -1)
  printf '  gz sim --versions -> %s\n' "$GZV"
  if [[ "${GZV%%.*}" == "8" ]]; then
    ok "gz-sim 8.x (Harmonic) — correct pairing for Jazzy"
  else
    bad "gz-sim major version is ${GZV%%.*}, expected 8 (Harmonic)"
  fi
else
  bad "gz binary not on PATH after install"
fi

if dpkg -l 2>/dev/null | grep -qiE '^ii\s+(gazebo11|libgazebo11|ignition-gazebo)'; then
  bad "Fortress/Classic packages present after install — something pulled them in"
  dpkg -l | grep -iE '^ii\s+(gazebo11|libgazebo11|ignition-)' | awk '{print "         " $2}'
  note "trace with: apt-cache rdepends <pkgname>"
else
  ok "no Fortress/Classic contamination"
fi

for pkg in ur_description gz_ros2_control moveit_ros_planning_interface controller_manager; do
  if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    ok "$pkg resolvable"
  else
    bad "$pkg NOT resolvable"
  fi
done

if ros2 pkg prefix robotiq_description >/dev/null 2>&1; then
  ok "robotiq_description resolvable"
else
  warn "robotiq_description not resolvable yet"
  note "if it was cloned to src/, source the workspace and re-check:"
  note "  source $WS/install/setup.bash && ros2 pkg prefix robotiq_description"
fi

# ---------------------------------------------------------------------------
sec "9. Shell setup"
SETUP_LINE="source /opt/ros/${ROS_DISTRO_TARGET}/setup.bash"
if ! grep -qF "$SETUP_LINE" "$HOME/.bashrc" 2>/dev/null; then
  printf '\n# ROS 2 %s\n%s\n' "$ROS_DISTRO_TARGET" "$SETUP_LINE" >> "$HOME/.bashrc"
  ok "added ROS setup to ~/.bashrc"
else
  ok "~/.bashrc already sources ROS"
fi
note "workspace overlay is NOT auto-sourced on purpose — source it per-terminal:"
note "  source $WS/install/setup.bash"

# ---------------------------------------------------------------------------
sec "SUMMARY"
printf '  UR packages ........... %s\n' "$UR_FROM"
printf '  robotiq description ... %s\n' "$ROBOTIQ_FROM"
printf '  workspace ............. %s\n' "$WS"

if [[ $FAILED -eq 0 ]]; then
  cat <<EOF

  BOOTSTRAP OK.

  Next, in a NEW terminal:

    source /opt/ros/${ROS_DISTRO_TARGET}/setup.bash
    source $WS/install/setup.bash        # if src/ was non-empty
    cd <this repo>
    bash scripts/00_recon.sh 2>&1 | tee docs/recon_\$(date +%Y%m%d_%H%M%S).log

  Then send me that log. The URDF merge gets written from it, not from guesses.

  Optional sanity check before recon — UR's own arm-only path should already
  work, with no gripper involved:

    ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e

  If that does NOT come up, stop. The problem is upstream of anything we add,
  and every later milestone would inherit it.
EOF
  exit 0
else
  cat <<'EOF'

  BOOTSTRAP FAILED — see the [FAIL] lines above.

  Do not proceed to recon or the URDF merge. Every failure here reappears later
  wearing a different costume, usually as an unexplained grasp problem.
EOF
  exit 1
fi

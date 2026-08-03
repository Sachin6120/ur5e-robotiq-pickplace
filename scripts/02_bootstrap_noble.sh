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

# NOTE: deliberately NOT using `set -o pipefail`.
#   `cmd | grep -q PAT` is unsafe under pipefail: grep -q exits the moment it
#   matches, closing the pipe, which kills `cmd` with SIGPIPE (141). pipefail
#   then reports the pipeline as FAILED precisely because the pattern MATCHED.
#   That silently inverts every such test. Learned the hard way — it made this
#   script claim apt couldn't see ros-jazzy-* on a machine where ros-jazzy-desktop
#   was already installed.
#   Pipelines below capture into a variable first and test the variable.
set -u

WS="${WS:-$HOME/ur5e_ws}"
ROS_DISTRO_TARGET=jazzy
FAILED=0

# Build parallelism. Left unset = colcon decides (one worker per core), which is
# what OOM-kills a MoveIt build inside WSL2's default memory cap. Auto-throttled
# in §7 based on available RAM; override explicitly with:
#   COLCON_WORKERS=2 bash scripts/02_bootstrap_noble.sh
COLCON_WORKERS="${COLCON_WORKERS:-auto}"

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
# Capture first, then test — see the pipefail note at the top of this file.
# Under pipefail this test would have reported "clean" on a DIRTY machine.
CONTAM=$(dpkg -l 2>/dev/null | grep -iE '^ii[[:space:]]+(gazebo11|libgazebo11|ignition-gazebo|ignition-common)' || true)
if [[ -n "$CONTAM" ]]; then
  warn "Fortress/Classic-era packages ALREADY present on this machine:"
  printf '%s\n' "$CONTAM" | awk '{print "         " $2}'
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

# Don't just assert failure — show what apt actually sees, so the cause is
# visible in the log rather than requiring a second round trip.
NJAZZY=$(apt-cache search "^ros-${ROS_DISTRO_TARGET}-" 2>/dev/null | wc -l)
CAND=$(apt-cache policy "ros-${ROS_DISTRO_TARGET}-desktop" 2>/dev/null \
       | awk '/Candidate:/{print $2}')

printf '  ros-%s-* packages visible to apt: %s\n' "$ROS_DISTRO_TARGET" "$NJAZZY"
printf '  ros-%s-desktop candidate: %s\n' "$ROS_DISTRO_TARGET" "${CAND:-<package unknown to apt>}"

if [[ "$NJAZZY" -gt 100 && -n "$CAND" && "$CAND" != "(none)" ]]; then
  ok "ROS 2 ${ROS_DISTRO_TARGET} packages visible to apt"
else
  bad "apt cannot resolve ros-${ROS_DISTRO_TARGET}-desktop"

  printf '\n  -- configured ROS apt sources --\n'
  for f in /etc/apt/sources.list.d/ros2.sources /etc/apt/sources.list.d/ros2.list; do
    [[ -f "$f" ]] && { printf '  %s:\n' "$f"; sed 's|^|      |' "$f"; }
  done

  printf '\n  -- ROS distros apt CAN see --\n'
  apt-cache search '^ros-' 2>/dev/null \
    | sed -n 's/^ros-\([a-z]\+\)-.*/\1/p' | sort | uniq -c | sort -rn \
    | head -8 | sed 's|^|      |'

  printf '\n  -- apt list state --\n'
  ls -la /var/lib/apt/lists/ 2>/dev/null | grep -i 'packages.ros.org' \
    | sed 's|^|      |' || printf '      (no packages.ros.org lists cached)\n'

  note "if the distro histogram above shows kilted/rolling but not ${ROS_DISTRO_TARGET},"
  note "this machine inherited an apt source from an earlier setup. Fix with:"
  note "    sudo rm -f /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2.sources"
  note "    sudo apt purge -y ros2-apt-source"
  note "  then re-run this script — it will reinstall the source cleanly."
  exit 1
fi

# On a re-run against a machine that already has some ros-jazzy-* packages
# installed from a much older sync, `apt install` below only pulls in NEW
# packages/deps at current versions while leaving already-installed ones
# (e.g. libfastcdr) pinned to their old build. That version skew is an ABI
# break waiting to happen: a freshly-installed package's typesupport lib
# expects symbols from a current lib{fastcdr,fastrtps}, the stale one on
# disk doesn't have them, and you get a runtime "undefined symbol" crash
# from move_group/rviz2 with no compile-time warning. `apt upgrade` (not
# full-upgrade — this never removes packages) closes that gap before we
# install anything new on top of it.
UPGRADABLE=$(apt list --upgradable 2>/dev/null | grep -c '^ros-')
if [[ "$UPGRADABLE" -gt 0 ]]; then
  warn "$UPGRADABLE ros-${ROS_DISTRO_TARGET}-* packages are stale — upgrading before installing more"
  run sudo apt upgrade -y
else
  ok "all ros-${ROS_DISTRO_TARGET}-* packages already current"
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
# has_binary <pkgname> -> 0 if apt has a real (non-"(none)") candidate.
# Capture-then-test; do not pipe into grep -q. See pipefail note at top.
has_binary() {
  local cand
  cand=$(apt-cache policy "$1" 2>/dev/null | awk '/Candidate:/{print $2; exit}')
  [[ -n "$cand" && "$cand" != "(none)" ]]
}

UR_FROM=binary
# ur_moveit_config's launch file hard-codes a node from ur_robot_driver even in
# sim-only mode (config/launch/ur_moveit.launch.py), so it's a real runtime
# dependency here, not just a real-hardware package.
for p in ur-description ur-simulation-gz ur-moveit-config ur-robot-driver; do
  if has_binary "ros-${ROS_DISTRO_TARGET}-${p}"; then
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
if has_binary "ros-${ROS_DISTRO_TARGET}-robotiq-description"; then
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

# ROS 2's setup.bash references its own internal variables (e.g.
# AMENT_TRACE_SETUP_FILES) without guarding them, so it is not nounset-safe.
# Suspend `set -u` for this one line only, then restore it immediately.
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_TARGET}/setup.bash"
set -u

cd "$WS"
if [[ -n "$(ls -A "$WS/src" 2>/dev/null)" ]]; then
  run rosdep install --from-paths src --ignore-src -r -y \
      --rosdistro "${ROS_DISTRO_TARGET}" || warn "rosdep reported unresolved keys — read them"

  # Decide parallelism BEFORE building. An OOM-kill here wastes 20 minutes and
  # reports as a confusing compiler crash rather than "out of memory".
  if [[ "$COLCON_WORKERS" == "auto" ]]; then
    MEM_GB=$(awk '/MemTotal/ {printf "%d", $2/1024/1024}' /proc/meminfo)
    CORES=$(nproc)
    printf '  detected: %s GB RAM, %s cores\n' "$MEM_GB" "$CORES"
    # MoveIt/OMPL translation units want ~2 GB each at peak.
    SAFE=$(( MEM_GB / 2 )); [[ $SAFE -lt 1 ]] && SAFE=1
    if [[ $SAFE -lt $CORES ]]; then
      COLCON_WORKERS=$SAFE
      warn "throttling to $COLCON_WORKERS workers — ${MEM_GB}GB RAM is tight for $CORES-way parallel"
      note "WSL2 caps memory by default. To raise it, create C:\\Users\\<you>\\.wslconfig:"
      note "    [wsl2]"
      note "    memory=12GB"
      note "  then run 'wsl --shutdown' from PowerShell and reopen."
    else
      COLCON_WORKERS=""
      ok "memory is comfortable for $CORES-way parallel build"
    fi
  fi

  BUILD_ARGS=(--symlink-install)
  [[ -n "$COLCON_WORKERS" ]] && BUILD_ARGS+=(--parallel-workers "$COLCON_WORKERS")

  if run colcon build "${BUILD_ARGS[@]}"; then
    ok "workspace built"
  else
    bad "colcon build failed — this is M-1 information, record the error"
    note "if the log shows a killed compiler / signal 9, that is an OOM kill,"
    note "not a code error. Re-run with fewer workers:"
    note "    COLCON_WORKERS=1 bash scripts/02_bootstrap_noble.sh"
    note "otherwise the likely cause is a source package targeting Humble, not Jazzy"
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

CONTAM_POST=$(dpkg -l 2>/dev/null | grep -iE '^ii[[:space:]]+(gazebo11|libgazebo11|ignition-gazebo)' || true)
if [[ -n "$CONTAM_POST" ]]; then
  bad "Fortress/Classic packages present after install — something pulled them in"
  printf '%s\n' "$CONTAM_POST" | awk '{print "         " $2}'
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

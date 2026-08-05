#!/usr/bin/env bash
# 07_check_gripper_spawn_state.sh — is the gripper's spawn state deterministic?
#
# WHY THIS EXISTS
#   docs/HANDOFF_M3.md's "NEW, SEVERE, UNRESOLVED" section: the validated
#   40mm/0.45rad anchor stopped reproducing on two fresh sim instances. One
#   of those instances read the master joint (robotiq_85_left_knuckle_joint)
#   at 0.7668 rad in its "gripper OPEN" baseline sample — nearly fully
#   closed, not open. 0.7668 rad is not an arbitrary number: it's the exact
#   rest position recorded for this linkage under gravity, unforced, in the
#   original free-space mimic test (M-1 report).
#
#   Mechanism: the xacro sets ros2_control initial_value for the six arm
#   joints but NOT for robotiq_85_left_knuckle_joint. The gripper
#   controller takes 10-15s to spawn and activate. Until it does, nothing
#   holds the finger joint — mimic followers are software-overridden BY
#   THE CONTROLLER, which isn't running yet either — so gravity is free to
#   close the linkage before the controller ever takes hold. Whether a
#   fresh spawn ends up open or closed is then a race between controller
#   activation and gravity, which is exactly the shape of "worked 5/5, then
#   stopped reproducing" on later fresh launches.
#
#   This script tests that directly: launch fresh N times, sample the
#   master joint's position at a FIXED delay after controllers report
#   active (same delay every time — this is a bimodality check, not an
#   average), and report every reading. Do not average them. A mix of ~0
#   and ~0.77 readings is the signature; a tight cluster at ~0.77 (or ~0)
#   across all N means the initial state is deterministic and something
#   else explains the reproducibility failure.
#
#   UPDATE after the deterministic-open launch-file fix landed: a fixed
#   POST_ACTIVE_DELAY_S is the wrong tool for validating that fix, and stayed
#   in as SAMPLE_MODE=fixed only to keep reproducing the original diagnostic
#   run byte-for-byte. The open command is chained off gripper_controller
#   activation but itself has to wait for its action server to become
#   discoverable -- confirmed directly in a validation run's launch log
#   ("[ros2-8] Waiting for an action server to become available...") still
#   printing past the 0.5s mark. Sampling early there reads a
#   command-in-flight, not the outcome, and reproduces the same 0.7668rad
#   number by coincidence (it's the joint's pre-command rest position, not a
#   sign the fix failed).
#
#   SAMPLE_MODE=settle (now the default) does NOT use velocity-settle either
#   -- tried that first, and it has the identical blind spot: a joint that
#   hasn't started moving yet reads as zero-velocity just like one that
#   finished moving, so it "settles" instantly at whatever position it
#   already happened to rest at. Confirmed directly: two launches settled in
#   <0.5s at the pre-command rest position while their open-command process
#   was still blocked on the same "Waiting for an action server" line.
#   SAMPLE_MODE=settle instead polls the joint's POSITION until it actually
#   converges to open (+/- OPEN_TOL_RAD) or SETTLE_TIMEOUT_S elapses -- that
#   is what actually distinguishes "the open command finished" from "it
#   never got the chance to start".
#
# USAGE
#   bash scripts/07_check_gripper_spawn_state.sh 2>&1 | \
#     tee docs/spawn_state_check_$(date +%Y%m%d_%H%M%S).log
#
# NOTE: this script owns sim lifecycle (kills and relaunches it N times).
# Do not run this while relying on a sim instance for something else.

# ROS's own setup.bash references unset variables internally (e.g.
# AMENT_TRACE_SETUP_FILES) -- source it before turning set -u on, not after,
# or sourcing itself is what crashes the script.
source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
cd ~/ur5e_pickplace
source scripts/lib/gz_settle.sh

set -u

N="${N:-5}"
MASTER="${MASTER:-robotiq_85_left_knuckle_joint}"
WORLD="${WORLD:-empty}"
MODEL="${MODEL:-ur5e_robotiq}"
POST_ACTIVE_DELAY_S="${POST_ACTIVE_DELAY_S:-0.5}"
SAMPLE_MODE="${SAMPLE_MODE:-settle}"   # settle (default) or fixed (the original diagnostic mode)
OPEN_TOL_RAD="${OPEN_TOL_RAD:-0.05}"   # matches the tolerance every probe script's gz_assert_joint uses
ACTIVATION_BOUND_S="${ACTIVATION_BOUND_S:-20}"  # 6.8-13.1s is healthy; 40+s means the system is suspect, not just slow
SETTLE_TIMEOUT_S="${SETTLE_TIMEOUT_S:-20.0}"
SETTLE_POLL_S="${SETTLE_POLL_S:-0.15}"
GZ_JS="/world/${WORLD}/model/${MODEL}/joint_state"

hr()  { printf '\n%s\n' "════════════════════════════════════════════════════════════"; }
sec() { hr; printf '§ %s\n' "$1"; hr; }

# kill_sim and the clean-slate/bounded-activation preamble now live in
# scripts/lib/gz_settle.sh (kill_sim, gz_assert_clean_slate,
# gz_wait_controller_active_bounded) -- `ros2 launch` spawning children that
# outlive its parent is a property of `ros2 launch` itself, not of this
# script, so every script that owns sim lifecycle inherits the same fix
# rather than growing its own teardown. See that file's comments for the
# 18-orphan/15-minute-hang history. This script no longer defines kill_sim
# locally.

sample_master_position() {
  python3 -c "
import sys, re
sys.path.insert(0, 'scripts/lib')
import gz_settle as gs
txt = gs._gz_echo('$GZ_JS')
for blk in re.findall(r'joint\s*\{(.*?)\n\}', txt, re.S):
    n = re.search(r'name:\s*\"([^\"]+)\"', blk)
    p = re.search(r'position:\s*(-?[\d.eE+-]+)', blk)
    if n and p and n.group(1) == '$MASTER':
        print(p.group(1))
        break
"
}

printf 'spawn-state bimodality check: %s\n' "$(date -Is)"
if [[ "$SAMPLE_MODE" == "settle" ]]; then
  printf 'N=%s launches, master=%s, sample_mode=settle (open_tol=%s rad, timeout=%ss)\n' \
    "$N" "$MASTER" "$OPEN_TOL_RAD" "$SETTLE_TIMEOUT_S"
else
  printf 'N=%s launches, master=%s, sample_mode=fixed, post-active delay=%ss\n' "$N" "$MASTER" "$POST_ACTIVE_DELAY_S"
fi

sec "0. Ensuring clean slate"
kill_sim
gz_assert_clean_slate || { echo "  [STOP] stray processes survived kill_sim -- aborting rather than launching on top of them"; exit 1; }
echo "  [ok] no prior sim instance running"

declare -a READINGS
declare -a SPAWN_WAITS

for ((i=1; i<=N; i++)); do
  sec "Launch $i / $N"
  ros2 launch ur5e_robotiq_description ur5e_robotiq_sim_control.launch.py \
    > "/tmp/spawn_check_launch_${i}.log" 2>&1 &

  # gz_wait_controller_active_bounded matches m0_verify.sh's controller-active
  # check (word-boundary on both the controller name and "active" -- a bare
  # `grep -q "active"` also matches "inactive" as a substring and doesn't
  # check gripper_controller specifically; that bug made every prior run of
  # this script sample the master joint before gripper_controller had ever
  # loaded). It also enforces ACTIVATION_BOUND_S: activation time is a known
  # health signal for this stack, not just a wait -- 40+s means orphaned
  # processes or some other contamination, not "give it more time" (see
  # docs/HANDOFF_M3.md, "orphaned processes"). A bound breach is recorded as
  # a failure for this launch, same as TIMEOUT/MISSING below -- not retried.
  if ! gz_wait_controller_active_bounded gripper_controller "$ACTIVATION_BOUND_S"; then
    READINGS+=("ACTIVATION_TIMEOUT")
    SPAWN_WAITS+=("$GZ_LAST_WAIT_S")
    kill_sim
    continue
  fi
  SPAWN_WAITS+=("$GZ_LAST_WAIT_S")

  if [[ "$SAMPLE_MODE" == "settle" ]]; then
    # NOT gz_settle_joint: velocity-settle can't tell "open command finished"
    # apart from "open command hasn't reached the sim yet". Both read as
    # zero velocity -- confirmed directly: two launches this session settled
    # in <0.5s at the joint's untouched gravity-rest position while their
    # open-command process was still printing "Waiting for an action server
    # to become available" in the launch log, i.e. it had not even sent the
    # goal yet. Poll for actual convergence to the open position instead,
    # giving the whole chain (gripper_controller activation -> action server
    # discovery -> goal execution) real time to finish. A timeout here is a
    # recorded failure, per this project's standing rule -- not retried.
    DEADLINE=$(python3 -c "import time; print(time.time() + $SETTLE_TIMEOUT_S)")
    RESULT="TIMEOUT"
    while python3 -c "import sys, time; sys.exit(0 if time.time() < $DEADLINE else 1)"; do
      POS_NOW=$(sample_master_position)
      if [[ -n "$POS_NOW" ]] && python3 -c "import sys; sys.exit(0 if abs(float('$POS_NOW')) <= $OPEN_TOL_RAD else 1)" 2>/dev/null; then
        RESULT="$POS_NOW"
        break
      fi
      sleep "$SETTLE_POLL_S"
    done
    if [[ "$RESULT" == "TIMEOUT" ]]; then
      printf '  [STOP] %s did not reach open (+/- %s rad) within %ss -- recording as TIMEOUT for this launch\n' \
        "$MASTER" "$OPEN_TOL_RAD" "$SETTLE_TIMEOUT_S"
      printf '  last read: %s rad\n' "${POS_NOW:-unreadable}"
      READINGS+=("TIMEOUT(${POS_NOW:-unreadable})")
      kill_sim
      continue
    fi
    POS="$RESULT"
    printf '  %s = %s rad (reached open within %ss)\n' "$MASTER" "$POS" "$SETTLE_TIMEOUT_S"
  else
    sleep "$POST_ACTIVE_DELAY_S"
    POS=$(sample_master_position)
    if [[ -z "$POS" ]]; then
      printf '  [STOP] could not read %s -- treating as MISSING for this launch\n' "$MASTER"
      READINGS+=("MISSING")
      kill_sim
      continue
    fi
    printf '  %s = %s rad (fixed delay=%ss after active)\n' "$MASTER" "$POS" "$POST_ACTIVE_DELAY_S"
  fi

  READINGS+=("$POS")
  kill_sim
done

sec "Results (raw, not averaged -- this is a bimodality check)"
printf '%-8s %-18s %-14s\n' "launch" "controller_wait_s" "master_pos_rad"
for ((i=0; i<N; i++)); do
  printf '%-8s %-18s %-14s\n' "$((i+1))" "${SPAWN_WAITS[$i]}" "${READINGS[$i]}"
done

sec "Interpretation -- read the raw values above, this section will not judge"
if [[ "$SAMPLE_MODE" == "settle" ]]; then
  cat <<'EOF'
  This run validates the deterministic-open launch-file fix, not the
  original un-commanded race (use SAMPLE_MODE=fixed to reproduce that
  diagnostic). Each reading here is taken AFTER the master joint settled
  (velocity < eps), i.e. after any open command chained off
  gripper_controller activation has actually finished, not mid-flight.

  All N settle near ~0 (open): the fix works -- deterministic startup is
  confirmed, not just implemented.

  Any reading near ~0.77 (near-closed) or TIMEOUT after settling: the fix
  did NOT take effect for that launch even after waiting for the joint to
  stop moving. Do not re-run and average past this -- check that launch's
  /tmp/spawn_check_launch_<i>.log for whether the open command's process
  ([ros2-N] tag) ever started, reached "Waiting for an action server", and
  what result (if any) it got. A TIMEOUT with no error usually means the
  command never fired at all (event handler didn't trigger, or a stale
  workspace copy without the fix -- diff the installed launch file against
  this repo's before assuming the code is at fault).
EOF
else
  cat <<'EOF'
  A mix of ~0 (open) and ~0.77 (near-closed) readings across launches is
  the bimodality signature: the gripper controller's activation is racing
  gravity, and which one wins varies launch to launch. This explains
  reproducibility failures directly -- any script that assumes "gripper
  OPEN at start" without checking is measuring from an undefined initial
  condition on some fraction of launches.

  All readings clustered near ~0.77 (or all near ~0): the initial state is
  deterministic. Something else explains the reproducibility failure --
  do not conclude the race hypothesis from this data alone in that case.

  Next steps if bimodal, per docs/HANDOFF_M3.md:
  1. Deterministic startup: command the gripper open once, after
     controllers activate, in the bringup launch -- OR set ros2_control
     initial_value for robotiq_85_left_knuckle_joint (means patching the
     package that owns the macro, more invasive).
  2. Precondition assertion: every measurement script should verify the
     starting aperture (open, settle, check within tolerance) rather than
     assume it, and abort with a named error otherwise -- same discipline
     already applied to base_link pose (M2's CONFIG_ERROR guard) and mimic
     joint count (M0-C's C1 check).
EOF
fi

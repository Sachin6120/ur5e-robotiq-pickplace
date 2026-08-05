#!/usr/bin/env bash
# gz_settle.sh — bash wrappers around gz_settle.py. Source this, then call
# gz_settle_joint / gz_settle_pose to WAIT before taking a ground-truth
# sample, instead of a fixed `sleep N` guessed to be "probably long enough".
# See gz_settle.py's header for why fixed sleeps are a measurement-integrity
# bug, not a style preference.
#
# Both functions return nonzero (and gz_settle.py prints a [STOP] line to
# stderr) if the poll times out without settling. Treat that as a hard stop
# for the calling script's current sample — do not follow either call with
# `|| true` and sample anyway.

GZ_SETTLE_PY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/gz_settle.py"

# gz_settle_joint <gz_joint_state_topic> <vel_eps_rad_s> <timeout_s> <poll_dt_s> <joint_name> [joint_name ...]
gz_settle_joint() {
  local topic="$1" eps="$2" timeout="$3" poll="$4"; shift 4
  python3 "$GZ_SETTLE_PY" joint --topic "$topic" --eps "$eps" \
    --timeout "$timeout" --poll "$poll" "$@"
}

# gz_settle_pose <gz_pose_info_topic> <pos_eps_m> <timeout_s> <poll_dt_s> <link_name> [link_name ...]
gz_settle_pose() {
  local topic="$1" eps="$2" timeout="$3" poll="$4"; shift 4
  python3 "$GZ_SETTLE_PY" pose --topic "$topic" --eps "$eps" \
    --timeout "$timeout" --poll "$poll" "$@"
}

# gz_assert_joint <gz_joint_state_topic> <joint_name> <expected_rad> <tol_rad> [label]
#
# PRECONDITION assertion, not a settle wait: reads the joint's CURRENT
# position once and fails loudly if it isn't where the caller assumed. Every
# probe script in this project stated "gripper OPEN" as a precondition in a
# comment and never checked it -- it was false on every fresh sim launch this
# session (confirmed 5/5, ~0.767rad not ~0rad; see docs/HANDOFF_M3.md). Call
# this before any script proceeds on an assumed starting joint position.
gz_assert_joint() {
  local topic="$1" name="$2" expected="$3" tol="$4" label="${5:-}"
  python3 "$GZ_SETTLE_PY" assert-joint --topic "$topic" \
    --expected "$expected" --tol "$tol" ${label:+--label "$label"} "$name"
}

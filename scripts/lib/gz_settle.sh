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

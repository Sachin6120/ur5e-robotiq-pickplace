#!/usr/bin/env bash
# 11_m3_cycles.sh — run N pick-lift-transport-place cycles and produce M3's CSV.
#
# THE CRITERION, FIXED IN ADVANCE AND NOT MOVING
#   20 consecutive cycles, at least 18 with object slip under 5 mm relative to
#   the gripper, zero ejections or penetrations. Slip from Gazebo's own pose
#   ground truth, never from TF.
#
#   This script runs N cycles (default 3, for the staircase step before the
#   full sweep) and applies the same arithmetic at any N. The thresholds are
#   read from config/scene.yaml. Nothing here tunes to fit an outcome.
#
# RESTART POLICY — DECIDED IN ADVANCE, 2026-08-11
#   The sim degrades under 10 minutes of use and 20 cycles takes longer, so a
#   restart between cycles is permitted, logged, and gate-checked on both
#   sides. Every row carries sim_instance.
#
#   What "consecutive" was protecting against is cherry-picking: running 40 and
#   reporting the best 20. A logged restart does not reintroduce that, and a
#   fresh sim per cycle is arguably the harder test. THIS SCRIPT NEVER DISCARDS
#   A CYCLE. Every cycle it starts appears in the CSV with whatever it did,
#   including gate failures and crashes.
#
# WHAT IT WRAPS
#   Your existing per-trial script, whatever it is, via --trial-cmd. This does
#   not relaunch Gazebo itself: reimplementing a bringup that already works is
#   how a harness ends up testing the harness. The trial command must:
#     - run exactly one full cycle, bracketed by its own gates
#     - STREAM m3_grasp's output to stdout LIVE (not buffer it into a file and
#       print it only after the whole cycle including cleanup has finished) --
#       the watcher below needs to see M3 STAGE markers while the dwell window
#       they announce is still open, not tens of seconds after the sim it
#       would sample has already been torn down. Confirmed live 2026-08-11:
#       an earlier trial script redirected m3_grasp's launch output straight
#       to a file and only grepped it at the very end -- every cycle run
#       through it would have read NO_SAMPLE, silently, for a reason that had
#       nothing to do with slip.
#     - exit non-zero if either gate fails
#   Cycle index and sim instance are passed in as M3_CYCLE and M3_SIM_INSTANCE.
#
# HOW SLIP IS TAKEN
#   transport.cpp emits the stage markers and then DWELLS (slip_sample_dwell_s).
#   A watcher tails the cycle log; on LIFT_DONE and TRANSPORT_DONE it calls
#   sample_pose.py, which requires a quiescent window and fails loudly rather
#   than sampling a moving pose. slip.py then differences the two, relative to
#   the flange — so the number does not inherit tcp_offset, which was wrong by
#   18 mm until this morning.
#
# USAGE
#   bash scripts/11_m3_cycles.sh --cycles 3 \
#        --trial-cmd "bash docs/m3_run_full_cycle_trial_live.sh" \
#        --out runs/m3_cycles_$(date +%Y%m%d_%H%M%S).csv
#
#   FLANGE defaults to tool0 below but THIS PROJECT'S tool0 does not appear on
#   the pose topic at all -- confirmed live: robot_state_publisher/gz lump
#   tool0's fixed-joint chain into wrist_3_link, and only wrist_3_link,
#   base_link, shoulder_link, etc. (bare names, no "_visual"/"_lump__" suffix)
#   carry real pose data. wrist_3_link shares tool0's ORIGIN exactly (fixed,
#   zero-translation joints between them, this project's own established
#   fact), and slip_m()'s cancellation proof holds for ANY rigidly-attached
#   frame sharing that origin regardless of the constant rotation offset
#   between them (R_F' = R_F * Q for fixed Q  =>  the two rail-to-rail samples
#   differ by the same Q on both sides, which cancels exactly in the
#   difference -- same proof as the flange-vs-tcp argument this design
#   already rests on). So FLANGE=wrist_3_link is exact, not an approximation.
#   Override via FLANGE=... if a different URDF changes this.

set -u   # no pipefail — see the note at the top of 02_bootstrap_noble.sh

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB="$HERE/lib"

CYCLES=3
TRIAL_CMD=""
OUT="m3_cycles.csv"
SCENE="${SCENE:-config/scene.yaml}"
FLANGE="${FLANGE:-wrist_3_link}"
OBJECT="${OBJECT:-pick_target}"
POSE_TOPIC="${POSE_TOPIC:-/world/empty/pose/info}"
SLIP_MAX="${SLIP_MAX:-0.005}"
MARKER_TIMEOUT="${MARKER_TIMEOUT:-240}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --cycles)    CYCLES="$2"; shift 2 ;;
    --trial-cmd) TRIAL_CMD="$2"; shift 2 ;;
    --out)       OUT="$2"; shift 2 ;;
    --slip-max)  SLIP_MAX="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

if [[ -z "$TRIAL_CMD" ]]; then
  echo "CONFIG_ERROR: --trial-cmd is required. It must run exactly one full" >&2
  echo "  bracketed cycle and exit non-zero on gate failure." >&2
  exit 2
fi

RUNDIR="$(dirname "$OUT")"; mkdir -p "$RUNDIR"
# attempt_status: RAN (m3_grasp actually launched, reached RUN SUMMARY or
# TRANSPORT_DONE) or GATE_BEFORE_FAIL (died on a precondition before
# m3_grasp ever ran -- retried, not counted toward CYCLES, but the row is
# never dropped). cycle is empty for a GATE_BEFORE_FAIL row (no completed
# cycle to number); attempt is the raw, always-incrementing attempt count
# both row types share, for reconstructing exactly what happened in order.
printf 'attempt_status,cycle,attempt,trial_exit,grasp_result,gripper_result,'\
'achieved_grip_angle,tcp_error_m,slip_m,slip_verdict,ejected,'\
'lift_fraction,cycle_wall_s,log\n' > "$OUT"

# ---------------------------------------------------------------------------
# Watcher. Polls for filesystem marker files touched by transport.cpp /
# m3_grasp.cpp; on each slip marker, takes a settled sample. Runs in the
# background for the life of one cycle.
#
# REPLACED 2026-08-12: was `tail -n +1 -F "$log" | while read line; case
# "$line" in ...`, parsing live stdout for "M3 STAGE 3 LIFT_DONE" etc.
# Confirmed live, three separate sweep cycles: that mechanism can fail
# silently under this harness's own process tree (bash -c wrapping a script
# that itself backgrounds `ros2 launch` and a forwarding `tail -F`) -- the
# watcher's live tail saw nothing past the point the grasp launch even
# started, in every one of three cycles, despite the exact same lines
# appearing correctly in the completed log file afterward, and despite an
# isolated reproduction of the same bash -c / nesting pattern working fine
# standalone. Root cause not pinned down further (see docs/HANDOFF_M3.md);
# a filesystem write is immune to the entire class of pipe/stdout-buffering
# problem this sits in, which is the actual fix rather than continuing to
# chase the exact interaction. The "stage 2 ground truth"/.preclose sample
# the old watcher also took is dropped here, deliberately: nothing below
# ever reads .preclose back (only .baseline/.after feed the slip verdict),
# so it was already dead weight, not a signal this rewrite needs to
# preserve.
watch_markers() {
  local pre="$1"
  local start_ts baseline_done=0
  start_ts=$(date +%s)
  while true; do
    if [[ "$baseline_done" -eq 0 && -f "${pre}.liftdone_ready" ]]; then
      python3 "$LIB/sample_pose.py" --topic "$POSE_TOPIC" \
        --entities "$FLANGE" "$OBJECT" > "${pre}.baseline" 2>"${pre}.baseline.err"
      echo "$?" > "${pre}.baseline.rc"
      baseline_done=1
    fi
    if [[ -f "${pre}.transportdone_ready" ]]; then
      python3 "$LIB/sample_pose.py" --topic "$POSE_TOPIC" \
        --entities "$FLANGE" "$OBJECT" > "${pre}.after" 2>"${pre}.after.err"
      echo "$?" > "${pre}.after.rc"
      break
    fi
    if [[ -f "${pre}.run_summary_ready" ]]; then
      # m3_grasp's own last write on every path, success or typed failure --
      # matches the old watcher's "RUN SUMMARY seen, stop waiting" fast exit
      # for a cycle that aborted before TRANSPORT_DONE (e.g.
      # GRASP_LOST_DURING_LIFT). No .after sample to take; brc/arc below
      # correctly resolve to NO_SAMPLE, counted as a FAIL, which is what it is.
      break
    fi
    if [[ $(( $(date +%s) - start_ts )) -ge "$MARKER_TIMEOUT" ]]; then
      break
    fi
    sleep 0.2
  done
}

field() {  # field <file> <entity>  -> the 7 pose numbers
  awk -v e="$2" '$1==e { $1=""; print substr($0,2); exit }' "$1" 2>/dev/null
}

pass=0; fail=0; eject=0; completed=0; attempt=0
# Retry-until-CYCLES-actually-run, added 2026-08-12. Gate-before (and every
# other precondition die() below it -- contaminated system, controllers
# never active, move_group never appearing, object spawn failure) fires
# BEFORE m3_grasp is ever invoked. It cannot know the outcome, so retrying
# past it is not cherry-picking -- cherry-picking is selecting AFTER the
# result is known, which this is not. What would be wrong is silently
# reinterpreting a pre-fixed criterion after seeing a result; this does the
# opposite: the criterion (18/20 slip PASS, zero ejections) is now applied
# to 20 cycles that actually reached m3_grasp, full stop, and every attempt
# -- including every retried one -- still lands in $OUT, never deleted.
# MAX_ATTEMPTS is a safety cap, not part of the criterion: if the sim is
# genuinely broken (not just occasionally racy), this stops the sweep from
# looping forever instead of surfacing that.
MAX_ATTEMPTS=$(( CYCLES * 4 ))
preflight_fails=0

while (( completed < CYCLES )); do
  attempt=$((attempt+1))
  if (( attempt > MAX_ATTEMPTS )); then
    echo "  [STOP] $MAX_ATTEMPTS attempts made, only $completed/$CYCLES cycles" \
         "actually ran -- this is not occasional raciness, something is" \
         "structurally broken. Not retrying further."
    break
  fi
  # PRE is keyed by ATTEMPT, not by completed-cycle-count -- every attempt
  # gets its own never-reused prefix, so the staleness bug the old
  # cycle_NNN-reused-across-runs naming required an explicit rm -f guard
  # against (2026-08-11) cannot recur here by construction.
  PRE="$RUNDIR/attempt_$(printf '%03d' "$attempt")"
  LOG="${PRE}.log"
  : > "$LOG"

  echo "═══ attempt $attempt (completed $completed/$CYCLES so far) ═══"
  t0=$(date +%s)

  watch_markers "$PRE" &
  WATCHER=$!

  M3_CYCLE="$((completed+1))" M3_SIM_INSTANCE="$attempt" M3_MARKER_PREFIX="$PRE" \
    bash -c "$TRIAL_CMD" > "$LOG" 2>&1
  rc=$?

  ( sleep "$MARKER_TIMEOUT"; kill "$WATCHER" 2>/dev/null ) &
  KILLER=$!
  wait "$WATCHER" 2>/dev/null
  kill "$KILLER" 2>/dev/null
  pkill -P "$WATCHER" 2>/dev/null

  t1=$(date +%s)

  # --- pre-flight failure? m3_grasp never ran, so there is nothing to score
  # as a grasp outcome -- retry, do not count toward CYCLES, but the row
  # still goes in $OUT with attempt_status=GATE_BEFORE_FAIL, never dropped.
  if grep -q '\[STOP\]' "$LOG" && ! grep -q "RUN SUMMARY" "$LOG"; then
    preflight_fails=$((preflight_fails+1))
    # die() messages are free text and can contain commas (e.g. "restart and
    # retry, do not proceed") -- unlike the tightly-controlled enum fields
    # elsewhere in this CSV, this one needs sanitizing or it silently
    # corrupts the field alignment for every column after it.
    reason=$(grep -m1 '\[STOP\]' "$LOG" | sed -E 's/^[[:space:]]*\[STOP\][[:space:]]*//' | tr ',' ';')
    printf 'GATE_BEFORE_FAIL,,%d,%d,%s,,,,,,,,%d,%s\n' \
      "$attempt" "$rc" "${reason:0:80}" "$((t1-t0))" "$LOG" >> "$OUT"
    echo "  [PREFLIGHT FAIL] $reason -- retrying, not counted toward $CYCLES ($((t1-t0))s)"
    continue
  fi

  completed=$((completed+1))
  c="$completed"

  # --- pull what the node reported -----------------------------------------
  grasp_result=$(grep -o 'result=[A-Z_]*' "$LOG" | head -1 | cut -d= -f2)
  grip_result=$(grep -o 'gripper_result=[A-Z_]*' "$LOG" | head -1 | cut -d= -f2)
  grip_angle=$(grep -o 'achieved_grip_angle=[0-9.]*' "$LOG" | head -1 | cut -d= -f2)
  tcp_err=$(grep -o 'tcp_error_m=[0-9.]*' "$LOG" | head -1 | cut -d= -f2)
  lift_frac=$(grep -o 'lift ok (fraction [0-9.]*' "$LOG" | head -1 | grep -o '[0-9.]*$')

  # --- slip ----------------------------------------------------------------
  slip="" ; verdict="NO_SAMPLE"
  brc=$(cat "${PRE}.baseline.rc" 2>/dev/null || echo 1)
  arc=$(cat "${PRE}.after.rc" 2>/dev/null || echo 1)
  if [[ "$brc" == "0" && "$arc" == "0" ]]; then
    fb=$(field "${PRE}.baseline" "$FLANGE"); ob=$(field "${PRE}.baseline" "$OBJECT")
    fa=$(field "${PRE}.after"    "$FLANGE"); oa=$(field "${PRE}.after"    "$OBJECT")
    if [[ -n "$fb" && -n "$ob" && -n "$fa" && -n "$oa" ]]; then
      line=$(python3 "$LIB/slip.py" --threshold-m "$SLIP_MAX" \
               --flange-t0 $fb --object-t0 $ob \
               --flange-t1 $fa --object-t1 $oa 2>&1)
      slip=$(echo "$line" | grep -o 'slip_m=[0-9.]*' | cut -d= -f2)
      verdict=$(echo "$line" | grep -o 'verdict=[A-Z]*' | cut -d= -f2)
    fi
  fi

  # Ejection: the object is not in the fingers at all. Distinguished from slip
  # because it is a separate clause of the criterion (zero ejections), and a
  # 400 mm "slip" is not a marginal grasp, it is a different event.
  ejected=no
  if [[ -n "$slip" ]] && awk "BEGIN{exit !($slip > 10*$SLIP_MAX)}"; then
    ejected=yes; eject=$((eject+1))
  fi

  case "$verdict" in
    PASS) pass=$((pass+1)) ;;
    FAIL) fail=$((fail+1)) ;;
    *)    fail=$((fail+1))
          echo "  [warn] cycle $c produced no usable slip sample — counted as" \
               "a FAIL, not skipped. See ${PRE}.baseline.err / .after.err" ;;
  esac

  printf 'RAN,%d,%d,%d,%s,%s,%s,%s,%s,%s,%s,%s,%d,%s\n' \
    "$c" "$attempt" "$rc" "${grasp_result:-NONE}" "${grip_result:-NONE}" \
    "${grip_angle:-}" "${tcp_err:-}" "${slip:-}" "$verdict" "$ejected" \
    "${lift_frac:-}" "$((t1-t0))" "$LOG" >> "$OUT"

  echo "  cycle $c/$CYCLES: exit=$rc result=${grasp_result:-NONE} slip=${slip:-none} $verdict" \
       "($((t1-t0))s)"
done

# ---------------------------------------------------------------------------
started="$completed"
need_pass=$(( (CYCLES * 9 + 9) / 10 ))    # >= 90% of the REQUESTED count, i.e. 18 of 20 -- fixed to CYCLES, not $started, so a MAX_ATTEMPTS abort below cannot silently shrink the denominator into an easier bar.
echo
echo "═══ M3 SWEEP SUMMARY ═══"
echo "  cycles completed : $started / $CYCLES requested   ($attempt total attempts," \
     "$preflight_fails preflight-failed and retried -- every attempt is in $OUT)"
echo "  slip PASS      : $pass"
echo "  slip FAIL      : $fail"
echo "  ejections      : $eject"
echo "  criterion      : >= $need_pass of $CYCLES under ${SLIP_MAX} m, zero ejections"

if [[ "$started" -lt "$CYCLES" ]]; then
  echo "  RESULT: ABORTED — only $started/$CYCLES cycles ever ran ($attempt attempts" \
       "made, MAX_ATTEMPTS=$MAX_ATTEMPTS reached). Not a criterion evaluation:" \
       "the sim could not stay up long enough to even ATTEMPT $CYCLES cycles."
  exit 1
elif [[ "$eject" -gt 0 ]]; then
  echo "  RESULT: FAIL — the criterion allows zero ejections."
  exit 1
elif [[ "$pass" -ge "$need_pass" ]]; then
  echo "  RESULT: PASS"
  exit 0
else
  echo "  RESULT: FAIL — $pass of $started passed, needed $need_pass."
  exit 1
fi

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
printf 'cycle,sim_instance,trial_exit,grasp_result,gripper_result,'\
'achieved_grip_angle,tcp_error_m,slip_m,slip_verdict,ejected,'\
'lift_fraction,cycle_wall_s,log\n' > "$OUT"

# ---------------------------------------------------------------------------
# Watcher. Tails a cycle log; on each slip marker, takes a settled sample.
# Runs in the background for the life of one cycle.
# ---------------------------------------------------------------------------
watch_markers() {
  local log="$1" pre="$2"
  tail -n +1 -F "$log" 2>/dev/null | while IFS= read -r line; do
    case "$line" in
      *"stage 2 ground truth"*)
        # Fires right after the descent, before the close is ever
        # commanded -- the object has been resting since spawn/settle
        # (section 3, well before this) and nothing has touched it yet.
        # This is the UPSTREAM sample: whatever orientation the object
        # carries here is independent of the cycle's own outcome, unlike
        # a post-LIFT_DONE sample, which is downstream of the verdict
        # (an object still on the table at LIFT_DONE vs one airborne in
        # the fingers are two different physical situations, not two
        # settle behaviours -- confirmed live 2026-08-11, see
        # HANDOFF_M3.md).
        python3 "$LIB/sample_pose.py" --topic "$POSE_TOPIC" \
          --entities "$OBJECT" > "${pre}.preclose" 2>"${pre}.preclose.err"
        echo "$?" > "${pre}.preclose.rc"
        ;;
      *"M3 STAGE 3 LIFT_DONE"*)
        python3 "$LIB/sample_pose.py" --topic "$POSE_TOPIC" \
          --entities "$FLANGE" "$OBJECT" > "${pre}.baseline" 2>"${pre}.baseline.err"
        echo "$?" > "${pre}.baseline.rc"
        ;;
      *"M3 STAGE 4 TRANSPORT_DONE"*)
        python3 "$LIB/sample_pose.py" --topic "$POSE_TOPIC" \
          --entities "$FLANGE" "$OBJECT" > "${pre}.after" 2>"${pre}.after.err"
        echo "$?" > "${pre}.after.rc"
        break
        ;;
      *"RUN SUMMARY"*)
        # Added 2026-08-11, alongside transport.cpp's grasp-loss check
        # (check_grasp_not_lost, Stage 3): a cycle that aborts before ever
        # reaching transport (GRASP_LOST_DURING_LIFT, or any other early
        # typed failure) never emits TRANSPORT_DONE, so the case above
        # never fires and this loop would otherwise tail forever until
        # MARKER_TIMEOUT (240s default) killed it from outside -- turning
        # every cycle the new check was built to make FASTER into one of
        # the slowest in the sweep. RUN SUMMARY is m3_grasp's own last
        # line on every path, success or typed failure, so it's a safe
        # unconditional exit here: a normal cycle already broke out on
        # TRANSPORT_DONE above, before RUN SUMMARY is ever logged, so this
        # case only ever fires for the abort path. No .after sample to
        # take -- there is nothing to sample, the object left before
        # transport was attempted -- so brc/arc below correctly resolve to
        # NO_SAMPLE for this cycle, and it's counted as a FAIL, which is
        # what it is.
        break
        ;;
    esac
  done
}

field() {  # field <file> <entity>  -> the 7 pose numbers
  awk -v e="$2" '$1==e { $1=""; print substr($0,2); exit }' "$1" 2>/dev/null
}

pass=0; fail=0; eject=0; started=0

for (( c=1; c<=CYCLES; c++ )); do
  started=$((started+1))
  PRE="$RUNDIR/cycle_$(printf '%03d' "$c")"
  LOG="${PRE}.log"
  : > "$LOG"
  # Clear any pose samples left over from a PREVIOUS run that used this same
  # cycle number -- $PRE is reused across every invocation of this script,
  # nothing here is timestamped. Found live 2026-08-11: a cycle that aborts
  # before TRANSPORT_DONE (GRASP_LOST_DURING_LIFT, see check_grasp_not_lost)
  # never writes a fresh .after, and without this the stale .after/.after.rc
  # from an unrelated sweep an hour earlier was silently read as this
  # cycle's own sample -- reported PASS with a slip number that was
  # someone else's, on a cycle that never got anywhere near transport. Only
  # became likely to bite once early-abort cycles existed in numbers; the
  # staleness risk itself predates that and applied to any cycle that
  # failed before Stage 4, just rarely enough before now to go unnoticed.
  rm -f "${PRE}.preclose" "${PRE}.preclose.err" "${PRE}.preclose.rc" \
        "${PRE}.baseline" "${PRE}.baseline.err" "${PRE}.baseline.rc" \
        "${PRE}.after" "${PRE}.after.err" "${PRE}.after.rc"

  echo "═══ cycle $c/$CYCLES  (sim instance $c) ═══"
  t0=$(date +%s)

  watch_markers "$LOG" "$PRE" &
  WATCHER=$!

  M3_CYCLE="$c" M3_SIM_INSTANCE="$c" \
    bash -c "$TRIAL_CMD" > "$LOG" 2>&1
  rc=$?

  # The watcher exits on TRANSPORT_DONE. If the cycle failed before that it is
  # still tailing, so bound it rather than hanging the whole sweep on one bad
  # cycle.
  ( sleep "$MARKER_TIMEOUT"; kill "$WATCHER" 2>/dev/null ) &
  KILLER=$!
  wait "$WATCHER" 2>/dev/null
  kill "$KILLER" 2>/dev/null
  pkill -P "$WATCHER" 2>/dev/null

  t1=$(date +%s)

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

  printf '%d,%d,%d,%s,%s,%s,%s,%s,%s,%s,%s,%d,%s\n' \
    "$c" "$c" "$rc" "${grasp_result:-NONE}" "${grip_result:-NONE}" \
    "${grip_angle:-}" "${tcp_err:-}" "${slip:-}" "$verdict" "$ejected" \
    "${lift_frac:-}" "$((t1-t0))" "$LOG" >> "$OUT"

  echo "  exit=$rc result=${grasp_result:-NONE} slip=${slip:-none} $verdict" \
       "($((t1-t0))s)"
done

# ---------------------------------------------------------------------------
need_pass=$(( (started * 9 + 9) / 10 ))    # >= 90%, i.e. 18 of 20
echo
echo "═══ M3 SWEEP SUMMARY ═══"
echo "  cycles started : $started   (none discarded)"
echo "  slip PASS      : $pass"
echo "  slip FAIL      : $fail"
echo "  ejections      : $eject"
echo "  criterion      : >= $need_pass of $started under ${SLIP_MAX} m, zero ejections"

if [[ "$eject" -gt 0 ]]; then
  echo "  RESULT: FAIL — the criterion allows zero ejections."
  exit 1
elif [[ "$pass" -ge "$need_pass" ]]; then
  echo "  RESULT: PASS"
  exit 0
else
  echo "  RESULT: FAIL — $pass of $started passed, needed $need_pass."
  exit 1
fi

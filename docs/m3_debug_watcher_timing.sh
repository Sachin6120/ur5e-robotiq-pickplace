#!/usr/bin/env bash
# m3_debug_watcher_timing.sh -- run ONE cycle via m3_run_full_cycle_trial_live.sh
# directly, with a live tail -F that timestamps each NEW line the instant it
# sees it, into a side file. Compares those arrival times against the total
# cycle duration to distinguish "buffered, arrives in a late burst" from
# "genuinely never visible to a live tail".
cd "$(dirname "$0")/.." || exit 2
set -u

OUTLOG="docs/m3_debugwatch_$(date +%Y%m%d_%H%M%S).log"
ARRIVALS="docs/m3_debugwatch_arrivals_$(date +%Y%m%d_%H%M%S).log"
: > "$OUTLOG"
: > "$ARRIVALS"

T0=$(date +%s.%N)

( tail -n +1 -F "$OUTLOG" 2>/dev/null | while IFS= read -r line; do
    now=$(date +%s.%N)
    elapsed=$(echo "$now - $T0" | bc)
    printf '[t+%6.1fs] %s\n' "$elapsed" "$line" >> "$ARRIVALS"
  done ) &
TAIL_WATCHER=$!
echo "tail-watcher PID=$TAIL_WATCHER, arrivals=$ARRIVALS"

bash docs/m3_run_full_cycle_trial_live.sh > "$OUTLOG" 2>&1
RC=$?
T1=$(date +%s.%N)
echo "trial finished, rc=$RC, total_wall=$(echo "$T1 - $T0" | bc)s"

sleep 2   # let the tail-watcher catch up on any final flush
kill "$TAIL_WATCHER" 2>/dev/null
pkill -P "$TAIL_WATCHER" 2>/dev/null

echo "--- arrival times for the key markers ---"
grep -E "stage 2 ground truth|M3 STAGE 3 LIFT_DONE|M3 STAGE 4 TRANSPORT_DONE|RUN SUMMARY" "$ARRIVALS"

#!/usr/bin/env python3
"""STAGE-3C C3C preserved-evidence reanalysis, revision 2 (NO runtime, NO Gazebo).

Revision 2 of scripts/reanalyze_c3c_evidence.py. It changes NOTHING about
production and reruns NOTHING: it launches no process, starts no simulator,
performs no manipulation, and modifies no evidence file. It reads the same raw
artifacts and recomputes the same gates, with ONE gate materially strengthened.

WHAT CHANGED, AND WHY A SEPARATE ARTIFACT
-----------------------------------------
v1's Phase-2 causal gate was presence-only::

    phase2_began_after_attempt1_accepted =
        original['c3c_causal']['t_phase2_begin_sim'] is not None

It asserted that a number exists, not that anything happened in the required
order. v2 replaces it with an explicit ordering proof computed from raw log
line order (scripts/lib/stage3c_c3c_causal.py):

    CANDIDATE_VALIDATION_OK
      < FJT_GOAL_ACCEPTED attempt=1
      < last tick still showing a LIVE obstacle stream  (transition not yet begun)
      < first tick showing the stream INTERRUPTED       (Phase-2 despawn observed)
      <= FUTURE_PATH_INVALID naming dynamic_obstacle_0  (Phase-2 respawn observed)

all within one file, one process, one logger clock. The harness's
``t_phase2_begin_sim`` is NOT used by any gate: it comes from a different
node's lagging sim clock (recorded, quantified, and explained under
``harness_clock_lag_observation``).

Because that gate's logic changed materially, this writes a NEW immutable
artifact -- ``qualification_results_reanalysis_v2.json`` -- and leaves both
``qualification_results.json`` (the original NEEDS_CORRECTION runtime record)
and ``qualification_results_reanalysis.json`` (v1) untouched. Repository
convention here is append-only audit history; nothing is overwritten.
"""
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(REPO / 'scripts'), str(REPO / 'scripts/lib'), str(REPO / 'scripts/perception')]

import reanalyze_c3c_evidence as v1
import stage3c_c3c_causal as causal


def save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, default=str) + '\n')


def reanalyze_v2(evidence_dir: Path) -> dict:
    # Every v1 gate, recomputed from the raw artifacts by v1's own code so the
    # two revisions cannot silently drift apart.
    result = v1.reanalyze(evidence_dir)
    original = json.loads((evidence_dir / 'qualification_results.json').read_text())
    log = (evidence_dir / 'm3_grasp.log').read_text(errors='replace')

    result.update(
        reanalysis_revision=2,
        runtime_rerun=False,
        production_runtime_unchanged=True,
        parser_evidence_analysis_revision=True,
        original_evidence_modified=False,
        supersedes='qualification_results_reanalysis.json',
        v1_result_file=str(evidence_dir / 'qualification_results_reanalysis.json'),
    )

    causal_proof = causal.analyze_phase2_causal_order(log)
    result['phase2_causal_order'] = causal_proof
    result['harness_clock_lag_observation'] = causal.harness_clock_lag(
        log, original.get('c3c_causal', {}).get('t_phase2_begin_sim'))

    # v1's weak presence-only gate, kept ONLY as a labelled record of what the
    # previous revision asserted. It is not part of the v2 verdict.
    result['gates_v1_superseded'] = dict(
        phase2_began_after_attempt1_accepted_presence_only=result['gates'].pop(
            'phase2_began_after_attempt1_accepted'))

    # Every v1 gate still applies, plus the strengthened causal-order gate.
    result['gates']['phase2_causal_order_from_raw_log_order'] = bool(
        causal_proof['phase2_causal_order_ok'])
    result['verdict'] = 'PASS' if all(result['gates'].values()) else 'NEEDS_CORRECTION'
    return result


def main():
    evidence_dir = REPO / 'evidence' / 'stage3c_c3c_20260909_024120'
    if len(sys.argv) > 1:
        evidence_dir = Path(sys.argv[1]).resolve()
    result = reanalyze_v2(evidence_dir)
    out_path = evidence_dir / 'qualification_results_reanalysis_v2.json'
    if out_path.exists():
        print(f'NOTE: overwriting existing {out_path.name} (same revision, regenerated)')
    save(out_path, result)
    print(f'Reanalysis v2 written to: {out_path}')
    print(json.dumps(dict(
        original_verdict=result['original_verdict'],
        reanalysis_revision=result['reanalysis_revision'],
        reanalysis_verdict=result['verdict'],
        runtime_rerun=result['runtime_rerun'],
        production_runtime_unchanged=result['production_runtime_unchanged'],
        parser_evidence_analysis_revision=result['parser_evidence_analysis_revision'],
        gate_count=len(result['gates']),
        phase2_causal_order_line_indices=(
            result['phase2_causal_order'].get('ordering') or {}).get('line_indices'),
        harness_clock_lag_s=result['harness_clock_lag_observation'].get('harness_clock_lag_s'),
        failed_gates=[k for k, v in result['gates'].items() if not v],
    ), indent=2))
    return 0 if result['verdict'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())

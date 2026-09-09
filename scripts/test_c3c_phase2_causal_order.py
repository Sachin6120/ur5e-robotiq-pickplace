#!/usr/bin/env python3
"""Offline regression tests for the STAGE-3C C3C Phase-2 causal-order gate.

No simulator, no ROS, no manipulation, no evidence modification. Half the
cases are synthetic logs that pin the gate's decision rules; the rest run
against the PRESERVED evidence log to prove the gate actually holds on the
real artifact and is not merely satisfiable in principle.
"""
import json
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(REPO / 'scripts'), str(REPO / 'scripts/lib')]

import stage3c_c3c_causal as causal

EVIDENCE = REPO / 'evidence' / 'stage3c_c3c_20260909_024120'


def line(stamp, body, proc='m3_grasp-2'):
    return f'[{proc}] [INFO] [{stamp:.9f}] [m3_grasp]: {body}'


def synthetic_log(*, live_age_ms=12.0, include_interruption=True,
                  include_respawn=True, transition_before_accept=False):
    """Builds a minimal log with the same event vocabulary as production."""
    rows = [
        line(100.000, 'M3 C3 SCENE_B_ACQUIRED t_request=1.0 t_response=1.1 latency_ms=0.2 '
                      'obstacle_age_ms=0.000 pose_match_error_m=0.000000'),
        line(100.001, 'M3 C3 CANDIDATE_VALIDATION_OK max_payload_tilt_deg=0.03 '
                      'max_tool_tilt_deg=0.03'),
        line(100.002, 'M3 C1 TRANSPORT_MONITOR_START t_monitor_start=1.2 rate_target_hz=10.000'),
    ]
    if transition_before_accept:
        # The defect this gate exists to catch: the stream is already dead
        # BEFORE the attempt-1 goal is accepted, so no post-acceptance tick can
        # show a live stream.
        rows.append(line(100.003, 'M3 C1 TRANSPORT_MONITOR_TICK tick=1 scene_age_ms=800.000 '
                                  'scene_stale=1 progress_fraction=0.1'))
    rows.append(line(100.004, 'M3 C3 FJT_GOAL_ACCEPTED attempt=1 goal_uuid=abc123'))
    if not transition_before_accept:
        rows.append(line(100.100, f'M3 C1 TRANSPORT_MONITOR_TICK tick=2 '
                                  f'scene_age_ms={live_age_ms:.3f} scene_stale=0 '
                                  f'progress_fraction=0.2'))
    if include_interruption:
        rows.append(line(100.200, 'M3 C1 TRANSPORT_MONITOR_TICK tick=3 scene_age_ms=292.000 '
                                  'scene_stale=1 progress_fraction=0.3'))
    if include_respawn:
        rows.append(line(100.300, 'M3 C1 TRANSPORT_MONITOR_FUTURE_PATH_INVALID tick=4 '
                                  'first_invalid_time_s=2.8460 '
                                  'collision_pairs="dynamic_obstacle_0<->ur_to_robotiq_link; "'))
    return '\n'.join(rows)


class SyntheticGate(unittest.TestCase):
    def test_well_ordered_log_passes(self):
        r = causal.analyze_phase2_causal_order(synthetic_log())
        self.assertTrue(r['phase2_causal_order_ok'], r['reason'])
        self.assertEqual(r['ordering']['line_indices'], sorted(r['ordering']['line_indices']))
        self.assertTrue(r['ordering']['same_process'])

    def test_gate_is_not_presence_only(self):
        """The v1 gate passed on ANY log carrying a t_phase2_begin_sim number.
        This one must fail when the ordering evidence is absent."""
        r = causal.analyze_phase2_causal_order(
            synthetic_log(include_interruption=False, include_respawn=False))
        self.assertFalse(r['phase2_causal_order_ok'])
        self.assertIn('anchor', r['reason'])

    def test_missing_respawn_evidence_fails(self):
        r = causal.analyze_phase2_causal_order(synthetic_log(include_respawn=False))
        self.assertFalse(r['phase2_causal_order_ok'])

    def test_missing_interruption_evidence_fails(self):
        r = causal.analyze_phase2_causal_order(synthetic_log(include_interruption=False))
        self.assertFalse(r['phase2_causal_order_ok'])

    def test_transition_already_underway_before_acceptance_fails(self):
        """No post-acceptance tick shows a live stream, so there is no positive
        evidence that the transition had not already begun -> gate FAILS.
        This is the direction the gate must be able to reject."""
        r = causal.analyze_phase2_causal_order(synthetic_log(transition_before_accept=True))
        self.assertFalse(r['phase2_causal_order_ok'])
        self.assertIsNone(r['last_live_stream_tick'])

    def test_age_above_live_bound_is_not_live_evidence(self):
        """An age at or above one monitor tick period is not proof the stream was
        still being fed, so it must not be accepted as the live-stream anchor."""
        r = causal.analyze_phase2_causal_order(synthetic_log(live_age_ms=99.0))
        self.assertFalse(r['phase2_causal_order_ok'])
        r_ok = causal.analyze_phase2_causal_order(
            synthetic_log(live_age_ms=99.0), live_stream_max_age_ms=150.0)
        self.assertTrue(r_ok['phase2_causal_order_ok'], r_ok['reason'])

    def test_missing_candidate_validation_fails(self):
        log = '\n'.join(l for l in synthetic_log().splitlines()
                        if 'CANDIDATE_VALIDATION_OK' not in l)
        r = causal.analyze_phase2_causal_order(log)
        self.assertFalse(r['phase2_causal_order_ok'])

    def test_attempt0_goal_acceptance_is_not_mistaken_for_attempt1(self):
        log = synthetic_log().replace('FJT_GOAL_ACCEPTED attempt=1',
                                      'FJT_GOAL_ACCEPTED attempt=0')
        r = causal.analyze_phase2_causal_order(log)
        self.assertFalse(r['phase2_causal_order_ok'])
        self.assertIsNone(r['attempt1_accepted'])

    def test_no_cross_node_clock_timestamps_are_used(self):
        """The gate must not difference two nodes' independently-sampled
        snapshots of sim time. Both are the same nominal ROS sim-time domain;
        the problem is node-local observation latency, not a clock domain."""
        r = causal.analyze_phase2_causal_order(synthetic_log())
        self.assertFalse(r['uses_cross_node_clock_timestamps'])
        self.assertFalse(r['uses_harness_sim_timestamp'])


@unittest.skipUnless((EVIDENCE / 'm3_grasp.log').exists(), 'preserved evidence not present')
class PreservedEvidence(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.log = (EVIDENCE / 'm3_grasp.log').read_text(errors='replace')
        cls.result = causal.analyze_phase2_causal_order(cls.log)

    def test_gate_holds_on_the_preserved_run(self):
        self.assertTrue(self.result['phase2_causal_order_ok'], self.result['reason'])

    def test_exact_anchor_line_indices(self):
        """Pinned to the preserved artifact: a change here means either the
        evidence or the gate's anchor selection moved."""
        self.assertEqual(self.result['ordering']['line_indices'], [106, 116, 120, 123, 127])
        self.assertTrue(self.result['ordering']['line_order_strictly_increasing'])
        self.assertTrue(self.result['ordering']['log_stamp_order_non_decreasing'])

    def test_all_anchors_come_from_one_process(self):
        self.assertTrue(self.result['ordering']['same_process'])
        self.assertEqual(set(self.result['ordering']['processes']), {'m3_grasp-2'})

    def test_live_stream_anchor_is_genuinely_fresh(self):
        anchor = self.result['last_live_stream_tick']
        self.assertLessEqual(anchor['scene_age_ms'], causal.LIVE_STREAM_MAX_AGE_MS)
        self.assertEqual(anchor['scene_stale'], '0')

    def test_respawn_evidence_names_the_phase2_obstacle(self):
        self.assertIn('dynamic_obstacle_0', self.result['phase2_respawn_evidence']['text'])

    def test_harness_clock_lag_is_measured_and_excluded_from_gates(self):
        original = json.loads((EVIDENCE / 'qualification_results.json').read_text())
        lag = causal.harness_clock_lag(
            self.log, original['c3c_causal']['t_phase2_begin_sim'])
        self.assertTrue(lag['measurable'])
        # The harness number trails m3_grasp's own sim clock; the sign is the
        # whole explanation for the apparent 47.924 < 48.006 discrepancy.
        self.assertGreater(lag['harness_clock_lag_s'], 0.0)
        self.assertAlmostEqual(lag['harness_clock_lag_s'], 0.082, places=3)
        self.assertFalse(self.result['uses_harness_sim_timestamp'])

    def test_original_and_v1_artifacts_are_not_touched_by_analysis(self):
        original = json.loads((EVIDENCE / 'qualification_results.json').read_text())
        self.assertEqual(original['verdict'], 'NEEDS_CORRECTION')
        v1 = json.loads((EVIDENCE / 'qualification_results_reanalysis.json').read_text())
        self.assertEqual(v1['verdict'], 'PASS')
        self.assertFalse(v1['runtime_rerun'])


if __name__ == '__main__':
    unittest.main(verbosity=2)

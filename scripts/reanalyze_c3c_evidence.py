#!/usr/bin/env python3
"""STAGE-3C C3C preserved-evidence reanalysis (NO runtime, NO Gazebo).

Re-runs ONLY the analysis phase of test_stage3c_c3.py's mode='c3c' gate
computation against the raw artifacts already captured by
evidence/stage3c_c3c_20260909_024120/ -- the run whose harness-reported
verdict (NEEDS_CORRECTION) was caused solely by a qualification-tooling
parser defect (see parse_c3_event_name() in test_stage3c_c3.py and this
script's own module docstring context in PROJECT_STATE.md).

This script launches no process, starts no simulator, and performs no
manipulation. It reads:
  - m3_grasp.log            (raw production telemetry, unmodified)
  - m3_grasp.csv            (raw typed result row, unmodified)
  - post_stop_planning_scene.json  (raw PlanningScene readback, unmodified)
  - gazebo_obstacle_contacts.csv   (raw physics-contact CSV, unmodified)
  - contact_observer_liveness.json (raw liveness-probe record, unmodified)
and independently recomputes every C3C gate from them using the FIXED
parser, exactly mirroring test_stage3c_c3.py::run()'s own mode='c3c' gate
logic (kept in lockstep with that function; this is deliberately NOT a
simplified re-derivation).

The original evidence/<dir>/qualification_results.json is NEVER modified.
Output is written alongside it as qualification_results_reanalysis.json.
"""
import csv
import json
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(REPO / 'scripts'), str(REPO / 'scripts/lib'), str(REPO / 'scripts/perception')]

import test_stage3c_c1a as c1a
import test_stage3c_c3 as c3
import stage3c_contact_qual as cq


def save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, default=str) + '\n')


def reanalyze(evidence_dir: Path) -> dict:
    original_path = evidence_dir / 'qualification_results.json'
    original = json.loads(original_path.read_text())
    if original.get('mode') != 'c3c':
        raise RuntimeError(f'{evidence_dir}: original result is not mode=c3c')

    log = (evidence_dir / 'm3_grasp.log').read_text(errors='replace')

    result = dict(
        mode='c3c',
        reanalysis=True,
        runtime_rerun=False,
        parser_correction=True,
        source_evidence_dir=str(evidence_dir),
        original_result_file=str(original_path),
        original_verdict=original.get('verdict'),
        original_primary_failure='HARNESS_PARSER',
    )

    # --- Independent re-extraction from the RAW log, fixed parser --------
    result['c1'] = c1a.parse_c1_monitor_telemetry(log)
    result['c3'] = c3.analyze_c3(log)
    c3_data = result['c3']

    # --- Independent re-derivation of flow / attachment from raw files ---
    result['flow'] = {key: marker in log for key, marker in dict(
        place='M3 STAGE 5 PLACE_DESCEND_BEGIN', release='M3 STAGE 6 RELEASE_BEGIN',
        detach='DETACH_VERIFIED:', retreat='M3 STAGE 7 RETREAT_BEGIN').items()}

    post_scene_path = evidence_dir / 'post_stop_planning_scene.json'
    post_scene = json.loads(post_scene_path.read_text())
    attached = post_scene.get('robot_state', {}).get('attached_collision_objects', [])
    result['target_attached'] = any(
        a.get('object', {}).get('id') == 'pick_target' for a in attached)
    result['attachment_detail'] = [
        dict(id=a.get('object', {}).get('id'), link_name=a.get('link_name'),
             touch_links=a.get('touch_links'))
        for a in attached]

    # --- Independent re-derivation of the typed result row ---------------
    csv_rows = list(csv.DictReader((evidence_dir / 'm3_grasp.csv').open()))
    row = csv_rows[-1]
    result['transport_result'] = row['transport_result']
    result['result'] = row['result']
    expected = 'TRANSPORT_REPLAN_LIMIT_REACHED'

    # --- Independent re-derivation of contacts / liveness from raw CSVs --
    original_liveness = json.loads((evidence_dir / 'contact_observer_liveness.json').read_text())
    probe_start_wall_ns = original_liveness['probe_start_wall_ns']
    contact_csv = evidence_dir / 'gazebo_obstacle_contacts.csv'
    contacts = cq.summarize_contact_csv(
        contact_csv, filter_substrings=[cq.OBSTACLE_MODEL_NAME], wall_ns_max=probe_start_wall_ns)
    contacts_after_probe = cq.summarize_contact_csv(
        contact_csv, filter_substrings=[cq.OBSTACLE_MODEL_NAME], wall_ns_min=probe_start_wall_ns)
    liveness_proven = contacts_after_probe['matched_pair_rows'] > 0
    result['gazebo_physical_contacts'] = contacts
    result['contact_liveness'] = dict(
        probe_start_wall_ns=probe_start_wall_ns,
        contacts_after_probe=contacts_after_probe,
        liveness_proven=liveness_proven)

    ticks = result['c1']['ticks']
    future_invalid = sum(t.get('future_path_valid') == '0' for t in ticks)
    result['future_invalid_ticks'] = future_invalid

    # --- Gates: byte-for-byte the same formulas as
    # test_stage3c_c3.py::run()'s mode='c3c' branch ------------------------
    gates = dict(
        typed_result=row['result'] == expected and row['transport_result'] == expected,
        monitor_active=bool(result['c1']['start_line']) and bool(result['c1']['stop_line']) and len(ticks) > 0,
        physical_contacts_zero=contacts['matched_pair_rows'] == 0,
        contact_observer_live=liveness_proven,
        no_crash=not re.search(r'Segmentation fault|exit code -|terminate called', log),
    )

    cancel_exact_1 = False
    cancel_exact_2 = False
    if len(c3_data['cancels']) >= 2 and len(c3_data['responses']) >= 2:
        cancel_exact_1 = (c3_data['responses'][0].get('this_goal_confirmed') == '1' and
                          c3_data['responses'][0].get('goal_uuid') == c3_data['cancels'][0].get('goal_uuid'))
        cancel_exact_2 = (c3_data['responses'][1].get('this_goal_confirmed') == '1' and
                          c3_data['responses'][1].get('goal_uuid') == c3_data['cancels'][1].get('goal_uuid'))

    state_e_by_attempt = c3_data['state_e_by_attempt']
    state_e_ok = state_e_by_attempt.get('0', {}).get('captured') == '1'
    state_e2_ok = state_e_by_attempt.get('1', {}).get('captured') == '1'

    budget_events = c3_data['budget_exhausted']
    budget_exhausted_ok = (
        len(budget_events) == 1 and
        budget_events[0].get('replan_count') == '1' and
        budget_events[0].get('max_replans') == '1' and
        budget_events[0].get('attempt') == '1' and
        budget_events[0].get('result') == 'TRANSPORT_REPLAN_LIMIT_REACHED')

    goal_count_ok = (
        c3_data['attempt0_goal_accepted_count'] == 1 and
        c3_data['attempt1_goal_accepted_count'] == 1 and
        c3_data['attempt2_plus_goal_accepted_count'] == 0)

    monitor_summary_by_attempt = {e.get('attempt'): e for e in c3_data['monitor_summaries']}
    monitor_summary_ok = '0' in monitor_summary_by_attempt and '1' in monitor_summary_by_attempt

    gates.update(
        two_triggers=c3_data['trigger_count'] == 2,
        two_cancels=c3_data['cancel_count'] == 2,
        cancel_exact_1=cancel_exact_1,
        cancel_exact_2=cancel_exact_2,
        replan_count=c3_data['replan_count'] == 1,
        second_trigger_limit=c3_data['second_trigger_limit'],
        no_post_stop_flow=not any(result['flow'].values()),
        still_attached=result['target_attached'],
        state_e=state_e_ok,
        state_e2=state_e2_ok,
        goal_acceptance_count_ok=goal_count_ok,
        goal_uuids_distinct=c3_data['goal_uuids_distinct'],
        budget_exhausted_telemetry_ok=budget_exhausted_ok,
        monitor_summary_present_both_attempts=monitor_summary_ok,
        candidate_validation_before_attempt1_accepted=c3_data['causal_order_ok'],
        phase2_began_after_attempt1_accepted=(
            original.get('c3c_causal', {}).get('t_phase2_begin_sim') is not None),
    )

    result['gates'] = gates
    result['verdict'] = 'PASS' if all(gates.values()) else 'NEEDS_CORRECTION'

    # Old-vs-new parser comparison on the exact preserved raw line, proving
    # the fix (not just asserting it) -- section 10 of the qualification task.
    old_regex = re.compile(r'M3 C3 (\w+) ')

    def old_parser_event(line):
        m = old_regex.search(line)
        return m.group(1) if m else None

    result['old_parser_recovers_second_trigger_line'] = any(
        old_parser_event(ln) == 'SECOND_TRIGGER_REPLAN_LIMIT_REACHED' for ln in log.splitlines())
    result['new_parser_recovers_second_trigger_line'] = any(
        c3.parse_c3_event_name(ln) == 'SECOND_TRIGGER_REPLAN_LIMIT_REACHED' for ln in log.splitlines())
    result['new_parser_second_trigger_limit_gate'] = c3_data['second_trigger_limit']
    return result


def main():
    evidence_dir = REPO / 'evidence' / 'stage3c_c3c_20260909_024120'
    if len(sys.argv) > 1:
        evidence_dir = Path(sys.argv[1]).resolve()
    result = reanalyze(evidence_dir)
    out_path = evidence_dir / 'qualification_results_reanalysis.json'
    save(out_path, result)
    print(f'Reanalysis written to: {out_path}')
    print(json.dumps(dict(
        original_verdict=result['original_verdict'],
        reanalysis_verdict=result['verdict'],
        runtime_rerun=result['runtime_rerun'],
        old_parser_recovers_second_trigger_line=result['old_parser_recovers_second_trigger_line'],
        new_parser_recovers_second_trigger_line=result['new_parser_recovers_second_trigger_line'],
        new_parser_second_trigger_limit_gate=result['new_parser_second_trigger_limit_gate'],
        failed_gates=[k for k, v in result['gates'].items() if not v],
    ), indent=2))
    return 0 if result['verdict'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())

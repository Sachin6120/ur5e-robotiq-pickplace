#!/usr/bin/env python3
"""Stage-3C C3 qualification harness: one-replan reactive recovery.

Modes:
  c3a: Parity under production B1 obstacle (0 triggers, 0 replans, full cycle SUCCESS).
  c3b: Replan recovery under C1B transient obstacle (1 trigger, exact cancel, settle, State E,
       SCENE_A, replacement plan, SCENE_B, candidate validation, attempt 1 NATURAL, full cycle SUCCESS).
  c3c: Second-trigger replan budget limit (trigger 1 -> replan 1 -> trigger 2 on replacement ->
       exact cancel, settle, State E2, TRANSPORT_REPLAN_LIMIT_REACHED, target attached).

Sequential gate: c3b requires passing c3a; c3c requires passing c3b.
No auto-retry.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rosidl_runtime_py.convert import message_to_ordereddict
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(REPO / 'scripts'), str(REPO / 'scripts/lib'),
                str(REPO / 'scripts/perception')]
import test_stage3c_c1a as c1a
import test_stage3c_c1b as c1b
import stage3c_contact_qual as cq
import milestone_f1_harness as f1
import stage2a_analyzer as analyzer


def save(path, value):
    path.write_text(json.dumps(value, indent=2, default=str) + '\n')


def process_table():
    rows = {}
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            stat = (path / 'stat').read_text().rsplit(')', 1)[1].split()
            argv = (path / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            rows[int(path.name)] = dict(state=stat[0], ppid=int(stat[1]),
                pgid=int(stat[2]), start=stat[19], argv=argv)
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    return rows


def strays():
    table = process_table()
    excluded = {os.getpid()}
    parent = os.getppid()
    while parent in table and parent not in excluded:
        excluded.add(parent)
        parent = table[parent]['ppid']
    names = ('gz sim', 'gz-sim', 'ruby /usr/bin/gz', 'move_group', 'm3_grasp',
             'dynamic_obstacle_scene_node', 'parameter_bridge', 'dynamic_obstacle_bridge',
             'robot_state_publisher', 'controller_manager/spawner', 'static_scene_tf',
             'gz_contact_observer', 'gz_pose_observer', 'gz topic -e',
             'object_detector', 'object_position_world', 'test_stage3c_')
    return {pid: row for pid, row in table.items() if pid not in excluded
            and row['state'] != 'Z' and any(name in row['argv'] for name in names)}


class Session:
    def __init__(self, evidence):
        self.evidence = evidence
        self.procs = []
        self.files = []
        self.env = os.environ.copy()
        setup = (REPO.parent.parent / "install" / "setup.bash"
                 if REPO.parent.name == "src" else REPO / "install" / "setup.bash")
        if setup.is_file() and 'ur5e_robotiq_description' not in self.env.get('AMENT_PREFIX_PATH', ''):
            out = subprocess.check_output(['bash', '-c', f'source {setup} && env'], text=True)
            for line in out.splitlines():
                k, sep, v = line.partition('=')
                if sep:
                    self.env[k] = v
        lib = str(REPO.parents[1] / 'install/ur5e_robotiq_description/lib')
        for var in ('GZ_SIM_SYSTEM_PLUGIN_PATH', 'LD_LIBRARY_PATH'):
            self.env[var] = lib + ':' + self.env.get(var, '')

    def run(self, args, name, timeout=30):
        result = subprocess.run(args, env=self.env, capture_output=True, text=True, timeout=timeout)
        (self.evidence / (name + '.log')).write_text(result.stdout + result.stderr)
        if result.returncode:
            raise RuntimeError(f'{name}: return code {result.returncode}; see saved log')
        return result.stdout + result.stderr

    def launch(self, name, args):
        f = (self.evidence / (name + '.log')).open('w')
        self.files.append(f)
        p = subprocess.Popen(args, env=self.env, stdout=f, stderr=subprocess.STDOUT,
                             start_new_session=True)
        self.procs.append(p)
        return p

    def cleanup(self):
        table = process_table()
        groups = {p.pid for p in self.procs}
        owned = {pid: row for pid, row in table.items() if row['pgid'] in groups}
        save(self.evidence / 'cleanup_before.json', owned)
        for p in reversed(self.procs):
            try:
                os.killpg(p.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            for p in self.procs:
                p.poll()
            current = process_table()
            live = {pid: row for pid, row in current.items()
                    if row['state'] != 'Z' and (row['pgid'] in groups or
                        (pid in owned and row['start'] == owned[pid]['start']))}
            if not live:
                break
            time.sleep(0.1)
        for pid, row in live.items():
            current = process_table().get(pid)
            if current and current['start'] == row['start']:
                os.kill(pid, signal.SIGKILL)
        for p in self.procs:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
        for f in self.files:
            f.close()
        remaining = strays()
        save(self.evidence / 'cleanup_after.json', remaining)
        return not remaining


ANSI_ESCAPE = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')


def fields(line):
    clean = ANSI_ESCAPE.sub('', line)
    return {k: quoted if quoted else plain for k, quoted, plain in
            re.findall(r'(\w+)=(?:"([^"]*)"|(\S+))', clean)}


# STAGE-3C C3C qualification-tooling fix (this closeout; NEVER a production
# telemetry change -- see PROJECT_STATE.md's matching section).
#
# Defect: analyze_c3() extracted M3 C3 event names with
# `re.search(r'M3 C3 (\w+) ', line)`, which requires a SPACE immediately
# after the event name. Production has always used TWO established
# delimiter forms for its own M3 C3 lines (confirmed by a full source grep
# of every "M3 C3 <NAME>" emission site in transport_coordinator.cpp and
# transport_executor.cpp):
#   FORM A: "M3 C3 EVENT_NAME key=value ..."   (space-delimited)
#   FORM B: "M3 C3 EVENT_NAME: human text ..."  (colon-delimited)
# The old regex only recognized FORM A. Every FORM-B event
# (SECOND_TRIGGER_REPLAN_LIMIT_REACHED and every *_FAILED/*_TIMEOUT/
# *_INVALID/*_LOST diagnostic) was silently invisible to analyze_c3(),
# even though production logged it correctly -- directly observed in
# evidence/stage3c_c3c_20260909_024120/m3_grasp.log line 148, where
# "M3 C3 SECOND_TRIGGER_REPLAN_LIMIT_REACHED: replacement collision
# stopped; ..." is present exactly once but the old regex returned no
# match on it. This is the same class of qualification-tooling defect as
# the 2026-09-08 C2 ANSI-escape parser fix (HANDOFF.md) -- never a
# production defect, since production's own telemetry was correct
# throughout.
C3_EVENT_PATTERN = re.compile(r'M3 C3 ([A-Z0-9_]+)(?:[: ]|$)')


def parse_c3_event_name(line):
    """Extracts the M3 C3 event name from one log line, accepting BOTH
    established telemetry forms (see module comment above). Returns the
    event name, or None if the line carries no well-formed M3 C3 event.

    Deliberately narrow, not a broad match-anything-after-prefix pattern:
      - the literal prefix must be exactly "M3 C3 " (a trailing digit, e.g.
        "M3 C30 ...", cannot match -- that string does not even contain the
        literal substring "M3 C3 ", since the space is checked, not just
        the "C3" characters).
      - the event name itself must be exactly the family's established
        naming convention, [A-Z0-9_]+ (no lowercase, punctuation, or
        embedded whitespace).
      - immediately after the event name there must be a delimiter: a
        single space (FORM A), a colon (FORM B), or end-of-line. Any other
        following character (e.g. a hyphen, period, or exclamation mark)
        makes the WHOLE match fail at that position -- it does not fall
        back to silently accepting a truncated event name.
    """
    clean = ANSI_ESCAPE.sub('', line)
    m = C3_EVENT_PATTERN.search(clean)
    return m.group(1) if m else None


def analyze_c3(log):
    c2_events = {}
    c3_events = {}
    for line_index, line in enumerate(log.splitlines()):
        m2 = re.search(r'M3 C2 (\w+) ', line)
        if m2:
            row = fields(line)
            row['_line_index'] = line_index
            c2_events.setdefault(m2.group(1), []).append(row)
        event_name = parse_c3_event_name(line)
        if event_name:
            row = fields(line)
            row['_line_index'] = line_index
            c3_events.setdefault(event_name, []).append(row)

    def first(d, k):
        return d.get(k, [{}])[0]

    triggers = c2_events.get('COLLISION_TRIGGER', [])
    cancels = c2_events.get('CANCEL_REQUEST', [])
    responses = c2_events.get('CANCEL_RESPONSE', [])
    terminals = c2_events.get('TERMINAL', [])
    settles = c2_events.get('PHYSICAL_SETTLE_CONFIRMED', [])
    state_es = c2_events.get('STATE_E', [])

    # Stage-3C C3C attempt-aware FJT telemetry (design task section 6/8).
    fjt_send_requests = c3_events.get('FJT_SEND_REQUEST', [])
    fjt_goal_accepted = c3_events.get('FJT_GOAL_ACCEPTED', [])
    fjt_terminal = c3_events.get('FJT_TERMINAL', [])
    settled_state_events = c3_events.get('SETTLED_STATE', [])
    monitor_summaries = c3_events.get('REPLACEMENT_MONITOR_SUMMARY', [])
    budget_exhausted = c3_events.get('REPLAN_BUDGET_EXHAUSTED', [])

    def accepted_uuids_for_attempt(attempt: str):
        return [e.get('goal_uuid') for e in fjt_goal_accepted if e.get('attempt') == attempt]

    attempt0_uuids = accepted_uuids_for_attempt('0')
    attempt1_uuids = accepted_uuids_for_attempt('1')
    attempt2_plus_uuids = [e.get('goal_uuid') for e in fjt_goal_accepted
                            if e.get('attempt') not in ('0', '1')]

    # Causal-order proof (design task section 5/17): SCENE_B candidate
    # validation must appear in the log strictly BEFORE the replacement
    # (attempt=1) FJT goal is accepted. Proven by LOG-LINE ORDER, not
    # timestamp arithmetic -- both events are logged synchronously on the
    # same coordinator control-flow thread in transport_coordinator.cpp,
    # in this fixed source order (candidate validation -> preSendValidate ->
    # FJT send -> goal_response_callback fires FJT_GOAL_ACCEPTED), so line
    # index is a robust, simple proxy for "happened before" here.
    candidate_validation_line = first(c3_events, 'CANDIDATE_VALIDATION_OK').get('_line_index')
    attempt1_accepted_line = next(
        (e.get('_line_index') for e in fjt_goal_accepted if e.get('attempt') == '1'), None)
    causal_order_ok = (
        candidate_validation_line is not None and attempt1_accepted_line is not None and
        candidate_validation_line < attempt1_accepted_line)

    return dict(
        c2_events=c2_events,
        c3_events=c3_events,
        triggers=triggers,
        cancels=cancels,
        responses=responses,
        terminals=terminals,
        settles=settles,
        state_es=state_es,
        trigger_count=len(triggers),
        cancel_count=sum(e.get('cause') == 'COLLISION_STOP' for e in cancels),
        watchdog_count=sum(e.get('cause') == 'WATCHDOG_CLEANUP' for e in cancels),
        stop_count=sum(e.get('result') == 'TRANSPORT_COLLISION_STOPPED' for e in c2_events.get('STOP_RESULT', [])),
        replan_triggered=bool(c3_events.get('REPLAN_TRIGGERED')),
        replan_count=1 if c3_events.get('REPLAN_TRIGGERED') else 0,
        scene_a=first(c3_events, 'SCENE_A_ACQUIRED'),
        replan_plan=first(c3_events, 'REPLAN_PLAN_OK'),
        scene_b=first(c3_events, 'SCENE_B_ACQUIRED'),
        candidate_validation=first(c3_events, 'CANDIDATE_VALIDATION_OK'),
        replacement_send=first(c3_events, 'REPLACEMENT_SEND_READY'),
        replacement_success=bool(c3_events.get('REPLACEMENT_SUCCESS')),
        second_trigger_limit=bool(c3_events.get('SECOND_TRIGGER_REPLAN_LIMIT_REACHED')),
        # --- C3C attempt-aware telemetry ---
        fjt_send_requests=fjt_send_requests,
        fjt_goal_accepted=fjt_goal_accepted,
        fjt_terminal=fjt_terminal,
        settled_state_events=settled_state_events,
        monitor_summaries=monitor_summaries,
        budget_exhausted=budget_exhausted,
        attempt0_goal_accepted_count=len(attempt0_uuids),
        attempt1_goal_accepted_count=len(attempt1_uuids),
        attempt2_plus_goal_accepted_count=len(attempt2_plus_uuids),
        attempt0_uuid=attempt0_uuids[0] if attempt0_uuids else None,
        attempt1_uuid=attempt1_uuids[0] if attempt1_uuids else None,
        goal_uuids_distinct=(bool(attempt0_uuids) and bool(attempt1_uuids) and
                              attempt0_uuids[0] != attempt1_uuids[0]),
        state_e_by_attempt={e.get('attempt'): e for e in settled_state_events},
        causal_order_ok=causal_order_ok,
    )


# --- Audited C3B obstacle parameters (Candidate 5: Transit_Symmetric) ---
C3B_OBSTACLE_BOX = [0.05, 0.05, 0.10]
C3B_CENTER_X = 0.450
C3B_CENTER_Z = 0.860
C3B_Y_MIN = -0.05
C3B_Y_MAX = 0.05
C3B_PERIOD_S = 1.0
C3B_ACTIVE_WINDOW_S = 10.0

# --- Stage-3C C3C Phase-2 (goal-anchored) obstacle parameters ---
#
# Design ruling (STAGE-3C C3C PRE-QUALIFICATION task): the ORIGINAL
# speculative C3C scenario (same C3B region X=0.450/Z=0.860/Y=[-0.05,+0.05],
# only a longer active window) is REJECTED -- C3B's own evidence
# (evidence/stage3c_c3b_20260908_135359/dynamic_scene.log +
# gz_pose_stream.csv) proves the accepted 389-waypoint replacement path
# never re-enters that region while the obstacle is live (minimum payload
# surface gap during the obstacle's actual live window: 48.0 mm, never
# negative). Extending the window only re-checks the same clear path.
#
# Selected mechanism instead: after the replacement FJT goal is accepted,
# relocate the SAME dynamic_obstacle_0 into the region the replacement
# trajectory approaches ONLY AT ITS VERY END (the above_place goal
# vicinity). Offline geometric analysis against the recovered C3B
# replacement payload path (same evidence directory) found this region to
# have >250 mm clearance at any plausible Phase-2 onset time (0.5-10 s
# after send) and to be entered by the remaining path only near
# trajectory-time ~38-39 s of the 38.7124 s planned duration (see
# /tmp scratchpad c3c_phase2_proof.py output recorded in this task's
# closeout report; not persisted evidence, per this project's own
# precedent for un-persisted analytical steps -- e.g. Stage-3B Steps 1/2).
C3C_PHASE2_CENTER_X = 0.450
C3C_PHASE2_CENTER_Z = 0.8725
C3C_PHASE2_Y_MIN = 0.15
C3C_PHASE2_Y_MAX = 0.25
C3C_PHASE2_PERIOD_S = 1.0
# Bounded wait for "FJT_GOAL_ACCEPTED attempt=1" to appear in the log
# before concluding Phase 2 was never reachable (e.g. replacement plan
# failed, or attempt 1 was never sent) -- NOT a fixed sleep before acting;
# see wait_for_log_line().
C3C_PHASE2_GATE_TIMEOUT_S = 60.0


def create_c3b_obstacle_sdf(out_path: Path, center_x=C3B_CENTER_X, center_z=C3B_CENTER_Z,
                            y_min=C3B_Y_MIN, y_max=C3B_Y_MAX, period_s=C3B_PERIOD_S):
    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="dynamic_obstacle">
    <pose>{center_x:.4f} {y_max:.4f} {center_z:.4f} 0 0 0</pose>
    <static>false</static>
    <link name="obstacle_link">
      <gravity>false</gravity>
      <kinematic>true</kinematic>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.001</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.001</iyy><iyz>0.0</iyz><izz>0.001</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{C3B_OBSTACLE_BOX[0]} {C3B_OBSTACLE_BOX[1]} {C3B_OBSTACLE_BOX[2]}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{C3B_OBSTACLE_BOX[0]} {C3B_OBSTACLE_BOX[1]} {C3B_OBSTACLE_BOX[2]}</size></box></geometry>
        <material>
          <ambient>1.0 0.05 0.85 1.0</ambient>
          <diffuse>1.0 0.05 0.85 1.0</diffuse>
          <specular>0.5 0.5 0.5 1.0</specular>
        </material>
      </visual>
    </link>
    <plugin filename="libdeterministic_motion_system.so" name="ur5e_robotiq_sim::DeterministicMotion">
      <center_x>{center_x:.4f}</center_x>
      <center_z>{center_z:.4f}</center_z>
      <y_min>{y_min:.4f}</y_min>
      <y_max>{y_max:.4f}</y_max>
      <period>{period_s:.2f}</period>
    </plugin>
    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
      <publish_link_pose>false</publish_link_pose>
      <publish_model_pose>true</publish_model_pose>
      <publish_visual_pose>false</publish_visual_pose>
      <publish_collision_pose>false</publish_collision_pose>
      <publish_sensor_pose>false</publish_sensor_pose>
      <publish_nested_model_pose>false</publish_nested_model_pose>
      <use_pose_vector_msg>false</use_pose_vector_msg>
      <update_frequency>50</update_frequency>
    </plugin>
  </model>
</sdf>
"""
    out_path.write_text(cq.add_contact_sensor(sdf_content))


# --- Stage-3C C3C Phase H (CLEAR HOLD) obstacle parameters -----------------
#
# Locked HOLD authority from the deterministic-scenario design task
# (evidence/stage3c_c3c_hold_contact_probe_20260909_020320/). Keeps Phase 1's
# X and Z and moves only along the axis the motion plugin already sweeps, to a
# region 0.80 m clear of the table's Y[-0.40,+0.40] footprint and outside the
# arm's reachable |Y| at that X. Measured there, with the payload attached:
# State E valid at 1.0315 m tool0 clearance, above_place valid at 1.3750 m,
# 11/12 orientation-constrained planning-only trials succeeded and every
# successful path was collision-free at every waypoint.
#
# The 20 mm residual sweep is deliberate: a geometrically frozen obstacle
# risks a pose stream that stops changing, and the production freshness gate
# requires an update newer than the baseline AND <= 250 ms old.
C3C_HOLD_CENTER_X = 0.450
C3C_HOLD_CENTER_Z = 0.860
C3C_HOLD_Y_MIN = -1.210
C3C_HOLD_Y_MAX = -1.190
C3C_HOLD_PERIOD_S = 1.0

# Generic production parameter name; the harness advertises this service and
# production calls it once, after State E, before SCENE_A.
C3C_GATE_SERVICE_NAME = '/stage3c_c3c_pre_replan_scene_gate'


# --- Stage-3C C3C safe despawn/respawn barrier -----------------------------
#
# The measured safe barrier below (request remove -> POLL until an
# authoritative Gazebo query proves the entity is actually absent -> request
# same-name create -> require exactly one entity with that name -> wait for
# the Phase-2 pose topic to resume -> wait for the PlanningScene pose to
# converge on the Phase-2 region) is the direct product of the STAGE-3C C3C
# PRE-QUALIFICATION FINAL GATE infrastructure-only probe
# (evidence/stage3c_c3c_probe_20260908_151829/probe_results.json, verdict
# PASS). That probe measured, on this exact installed Gazebo/ros_gz_bridge
# stack: remove-response-to-actually-absent latency 844.67 ms (one measured
# run -- not assumed to be a safety-certified bound, hence the poll below,
# never a blind sleep); create succeeded exactly once with exactly one
# resulting entity (`allow_renaming: false` in spawn_model() already makes a
# premature same-name create FAIL outright rather than silently produce a
# renamed duplicate, independent of this barrier); dynamic_obstacle_scene_node's
# own `add_count_` stayed at exactly 1 across the whole transition (one ADD
# total, MOVE-only afterward, confirmed from the raw dynamic_scene.log); the
# PlanningScene retained dynamic_obstacle_0 at its last Phase-1 pose for the
# entire gap with zero CollisionObject REMOVE operations observed in either
# dynamic_scene.log or move_group.log; and the PlanningScene pose converged
# on the Phase-2 Y range after respawn.
POLL_INTERVAL_S = 0.05
ABSENT_TIMEOUT_S = 10.0
DUPLICATE_CHECK_RETRIES = 3
POSE_RESUME_TIMEOUT_S = 10.0
SCENE_CONVERGE_TIMEOUT_S = 10.0


POSE_INFO_TOPIC = '/world/empty/pose/info'


def pose_census(session: 'Session', timeout: float = 10.0):
    """One authoritative world pose census: {name: count} from a single fresh
    gz.msgs.Pose_V message on /world/empty/pose/info.

    Replaces `gz model`, whose /gazebo/worlds service lookup was directly
    observed timing out against a healthy, running simulator whose model had
    already spawned successfully. A transport-topic read needs no such service
    round-trip, and one message answers presence, absence AND duplicate count
    for every entity at a single instant.

    Counts `name:` OCCURRENCES rather than using sample_pose.parse_pose_v(),
    which returns a DICT and therefore silently collapses same-name duplicates
    -- exactly the condition this census exists to detect. R1 evidence
    (evidence/stage3c_c3c_r1_pose_census_20260909_021756/) confirms the
    obstacle appears model-level as the bare name "dynamic_obstacle".
    """
    try:
        r = subprocess.run(['gz', 'topic', '-e', '-t', POSE_INFO_TOPIC, '-n', '1'],
                           capture_output=True, text=True, timeout=timeout, env=session.env)
    except subprocess.TimeoutExpired:
        return None
    raw = r.stdout
    if not raw.strip():
        return None
    counts = {}
    for m in re.finditer(r'name:\s*"([^"]*)"', raw):
        counts[m.group(1)] = counts.get(m.group(1), 0) + 1
    return counts


def gazebo_entity_count(session: 'Session', model_name: str):
    """Authoritative live count of entities with this exact name, or None if
    the census could not be read (ambiguous -- never reported as absent)."""
    counts = pose_census(session)
    return None if counts is None else counts.get(model_name, 0)


def model_pose_topic_publishing(session: 'Session', model_name: str, timeout: float = 8.0) -> bool:
    """Create-success signal B: the new entity is actually publishing on its
    OWN pose topic. Required in addition to the census count, because the
    /world/empty/create Boolean reply is not trustworthy on its own -- gz-sim
    logged "Entity not spawned" while the client still received `data: true`
    (evidence/stage3c_c3c_20260909_014313/sim.log)."""
    try:
        r = subprocess.run(
            ['gz', 'topic', '-e', '-t', f'/model/{model_name}/pose', '-n', '1'],
            capture_output=True, text=True, timeout=timeout, env=session.env)
    except subprocess.TimeoutExpired:
        return False
    return 'position' in r.stdout


def despawn_gazebo_model_only(session: 'Session', model_name: str, name: str):
    """Removes a Gazebo entity via /world/empty/remove WITHOUT publishing a
    MoveIt /collision_object REMOVE, then POLLS an authoritative Gazebo
    query until the entity is confirmed genuinely absent -- never trusting
    the synchronous remove-service response alone (measured, not assumed;
    see the probe note above). The PlanningScene therefore retains the
    object at its last known pose (DynamicObstacleSceneNode's own ADD-once
    counter, dynamic_obstacle_scene_node.cpp `add_count_`, never resets on a
    pose-stream gap -- confirmed BOTH by direct source read and by this
    probe's own runtime dynamic_scene.log) so a respawn under the SAME model
    name resumes as MOVE, not a second ADD.
    """
    removal = session.run(['gz', 'service', '-s', '/world/empty/remove',
        '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '5000', '--req', f'name: "{model_name}", type: MODEL'],
        name)
    if 'true' not in removal.lower():
        raise RuntimeError(f'{name}: Gazebo model despawn failed')
    deadline = time.monotonic() + ABSENT_TIMEOUT_S
    while time.monotonic() < deadline:
        count = gazebo_entity_count(session, model_name)
        if count == 0:
            return
        time.sleep(POLL_INTERVAL_S)
    raise RuntimeError(
        f'{name}: entity "{model_name}" never became absent within {ABSENT_TIMEOUT_S}s '
        'of a successful remove response -- refusing to respawn under the same name')


def respawn_gazebo_model_same_name(
    session: 'Session', model_name: str, sdf_text: str, name: str,
    evidence_pose_log: Path = None,
):
    """Creates model_name from sdf_text -- must be called only after
    despawn_gazebo_model_only() has already confirmed absence. Requires
    exactly one resulting entity with this name (spawn_model() already
    passes allow_renaming: false, so a premature attempt fails outright
    rather than silently producing a dynamic_obstacle_1-style duplicate;
    this is a second, independent confirmation from live Gazebo state, not
    reliance on that flag alone). Then waits (bounded poll, not a sleep)
    for the model's own pose topic to resume publishing -- required before
    returning, so the caller never proceeds believing Phase 2 is active
    when it is not.
    """
    spawn = cq.spawn_model(model_name, sdf_text)
    if evidence_pose_log is not None:
        evidence_pose_log.write_text(spawn.stdout + spawn.stderr)
    if 'true' not in spawn.stdout.lower():
        raise RuntimeError(f'{name}: same-name Gazebo model respawn failed')
    # Create success requires BOTH an exactly-one census count AND the new
    # entity publishing on its own pose topic -- never the create Boolean.
    count = None
    for _ in range(DUPLICATE_CHECK_RETRIES):
        count = gazebo_entity_count(session, model_name)
        if count == 1:
            break
        time.sleep(POLL_INTERVAL_S)
    if count != 1:
        raise RuntimeError(
            f'{name}: expected exactly 1 Gazebo entity named "{model_name}" after '
            f'respawn, world pose census reported {count}')
    if not model_pose_topic_publishing(session, model_name):
        raise RuntimeError(
            f'{name}: /model/{model_name}/pose is not publishing after respawn -- '
            'create Boolean alone is not accepted as proof of creation')
    # `ros2 topic echo --once` blocks for the NEXT published message on a
    # volatile topic (no history replay) -- so any successfully-parsed pose
    # here is necessarily a fresh post-respawn message, not a stale cached
    # Phase-1 echo. Exact Phase-2-region confirmation is a separate step
    # (wait_for_planning_scene_phase2_convergence()), not this one.
    pose_topic = f'/model/{model_name}/pose'
    deadline = time.monotonic() + POSE_RESUME_TIMEOUT_S
    while time.monotonic() < deadline:
        try:
            r = subprocess.run(['ros2', 'topic', 'echo', pose_topic, '--once'],
                capture_output=True, text=True, timeout=3, env=session.env)
            if 'position:' in r.stdout:
                return
        except subprocess.TimeoutExpired:
            pass
        time.sleep(0.1)
    raise RuntimeError(f'{name}: Phase-2 pose stream on {pose_topic} never resumed '
                        f'within {POSE_RESUME_TIMEOUT_S}s of respawn')


def wait_for_planning_scene_phase2_convergence(monitor, y_threshold: float, timeout_s: float = SCENE_CONVERGE_TIMEOUT_S):
    """Polls /get_planning_scene (via the harness's own C1BMonitorNode
    client -- no new service client) until dynamic_obstacle_0's stored pose
    is unambiguously inside the Phase-2 region (Y > y_threshold, which must
    be chosen strictly above Phase 1's own Y range so no Phase-1 sample can
    satisfy it). Fails loudly, rather than silently continuing, if the
    PlanningScene ever reports zero or more than one dynamic_obstacle_0
    object -- an ADD reset or an accidental duplicate must never pass
    unnoticed.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(monitor, timeout_sec=0.05)
        scene = monitor.query_planning_scene(timeout_sec=2)
        if not scene:
            continue
        matches = [co for co in scene.world.collision_objects if co.id == c1b.OBSTACLE_ID]
        if len(matches) not in (0, 1):
            raise RuntimeError(
                f'PlanningScene reports {len(matches)} objects with id={c1b.OBSTACLE_ID} '
                '(expected 0 or 1) -- duplicate or ownership conflict during Phase-2 transition')
        if matches and matches[0].pose.position.y > y_threshold:
            return matches[0].pose
        time.sleep(0.1)
    raise RuntimeError(
        f'PlanningScene pose for {c1b.OBSTACLE_ID} never converged to Y > {y_threshold} '
        f'within {timeout_s}s of Phase-2 respawn')


# --- Perception-readiness two-gate probe (harness-only correction) --------
#
# Prior single-probe defect (STAGE-3C C3C harness correction task): the
# readiness check was one `ros2 topic echo /object_detector/position_world
# --once` call with a 30 s subprocess-kill timeout. `ros2 topic echo --once`
# does NOT poll/block waiting for a publisher to appear in the ROS graph --
# it resolves the topic's type from the graph once and exits nonzero
# immediately if that resolution fails, regardless of the timeout given to
# `subprocess.run`. Observed in evidence/stage3c_c3c_20260909_013351/: exit
# code 1 after 345 ms ("does not appear to be published yet" /
# "Could not determine the type"), aborting the run before m3_grasp ever
# launched -- a qualification-harness race, not a C3 production defect
# (manipulation_started=false; no obstacle spawned; no FJT goal sent). The
# same step took 2.57 s (C3A, evidence/stage3c_c3a_20260908_125925/) and
# 2.95 s (C3B, evidence/stage3c_c3b_20260908_135359/) when it happened to
# win the race.
#
# Correction: two separately-timed, bounded gates, replacing the one
# combined probe.
#   GATE A -- ROS graph / type discovery: poll `ros2 topic type <topic>`
#     (never blocks; returns empty/nonzero -- WAITING, not a failure -- iff
#     unresolved) until it resolves to exactly PERCEPTION_EXPECTED_TYPE.
#     Continuously checks the perception node processes have not exited;
#     a process death fails immediately rather than waiting out the full
#     timeout.
#   GATE B -- first actual sample: only once Gate A confirms topic+type,
#     run one bounded `ros2 topic echo --once` for the remaining timeout
#     budget, proving actual data flow (not just graph advertisement).
PERCEPTION_TOPIC = '/object_detector/position_world'
PERCEPTION_EXPECTED_TYPE = 'geometry_msgs/msg/PointStamped'
PERCEPTION_POLL_INTERVAL_S = 0.1
PERCEPTION_READY_TIMEOUT_S = 30.0


class PerceptionReadinessError(RuntimeError):
    """Perception-readiness failure, tagged with one of the taxonomy codes
    below rather than surfacing as an undifferentiated subprocess
    return-code RuntimeError."""
    def __init__(self, code: str, message: str):
        super().__init__(f'{code}: {message}')
        self.code = code


def classify_topic_type_poll(resolved_type, expected_type):
    """Pure predicate for Gate A's per-poll outcome, factored out for unit
    testing without a subprocess/ROS graph. Returns one of:
    'WAITING' (resolved_type is None -- topic/type not yet resolvable,
    not a failure while inside the timeout), 'READY' (resolved_type ==
    expected_type), or 'PERCEPTION_TOPIC_TYPE_MISMATCH' (resolved to a
    different, wrong type)."""
    if resolved_type is None:
        return 'WAITING'
    if resolved_type == expected_type:
        return 'READY'
    return 'PERCEPTION_TOPIC_TYPE_MISMATCH'


def _resolve_topic_type(session: 'Session', topic: str, timeout: float = 5.0):
    """Gate A primitive: `ros2 topic type <topic>` -- returns the resolved
    type string, or None if not yet resolvable. Never raises on the
    ordinary not-yet-published state (nonzero return / empty output)."""
    try:
        r = subprocess.run(['ros2', 'topic', 'type', topic], env=session.env,
                            capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return None
    out = r.stdout.strip()
    return out if (r.returncode == 0 and out) else None


def wait_for_perception_ready(session: 'Session', evidence: Path, node_procs,
                               topic: str = PERCEPTION_TOPIC,
                               expected_type: str = PERCEPTION_EXPECTED_TYPE,
                               timeout_s: float = PERCEPTION_READY_TIMEOUT_S,
                               poll_interval_s: float = PERCEPTION_POLL_INTERVAL_S):
    """Runs Gate A then Gate B (see module-level comment above). Writes
    perception_readiness.json with the required telemetry regardless of
    outcome, and perception_ready.log with Gate B's raw output on success.
    Raises PerceptionReadinessError with an exact taxonomy code on failure.
    """
    telemetry = dict(
        perception_launch_monotonic=time.monotonic(),
        topic_discovered_monotonic=None,
        type_resolved_monotonic=None,
        first_sample_monotonic=None,
        poll_count=0,
        resolved_topic_type=None,
        process_alive_at_ready=None,
        gate_failed=None,
    )
    launch_t = telemetry['perception_launch_monotonic']
    deadline = launch_t + timeout_s

    def finish(code=None, message=None):
        if code is not None:
            telemetry['gate_failed'] = code
        save(evidence / 'perception_readiness.json', telemetry)
        if code is not None:
            raise PerceptionReadinessError(code, message)

    # --- Gate A: ROS graph / type discovery ---
    resolved_type = None
    while True:
        for p in node_procs:
            if p.poll() is not None:
                finish('PERCEPTION_PROCESS_EXITED',
                       f'perception node process (pid {p.pid}) exited with code '
                       f'{p.returncode} before {topic} became discoverable')
        telemetry['poll_count'] += 1
        resolved_type = _resolve_topic_type(session, topic)
        outcome = classify_topic_type_poll(resolved_type, expected_type)
        if outcome == 'READY':
            now = time.monotonic()
            telemetry['topic_discovered_monotonic'] = now
            telemetry['type_resolved_monotonic'] = now
            telemetry['resolved_topic_type'] = resolved_type
            break
        if outcome == 'PERCEPTION_TOPIC_TYPE_MISMATCH':
            telemetry['topic_discovered_monotonic'] = time.monotonic()
            telemetry['resolved_topic_type'] = resolved_type
            finish('PERCEPTION_TOPIC_TYPE_MISMATCH',
                   f'{topic} resolved to type {resolved_type!r}, expected {expected_type!r}')
        # outcome == 'WAITING': not a failure, keep polling within the timeout.
        if time.monotonic() >= deadline:
            finish('PERCEPTION_TOPIC_DISCOVERY_TIMEOUT',
                   f'{topic} type never resolved within {timeout_s}s of node launch '
                   f'({telemetry["poll_count"]} polls)')
        time.sleep(poll_interval_s)

    # --- Gate B: first actual sample ---
    remaining = max(0.5, deadline - time.monotonic())
    try:
        r = subprocess.run(['ros2', 'topic', 'echo', topic, '--once'],
                            env=session.env, capture_output=True, text=True, timeout=remaining)
        output = r.stdout + r.stderr
        (evidence / 'perception_ready.log').write_text(output)
        sample_ok = (r.returncode == 0 and 'x:' in output)
    except subprocess.TimeoutExpired as exc:
        (evidence / 'perception_ready.log').write_text(
            (exc.stdout or '') + (exc.stderr or '') if isinstance(exc.stdout, str) else '')
        sample_ok = False
    if not sample_ok:
        finish('PERCEPTION_FIRST_SAMPLE_TIMEOUT',
               f'no valid sample observed on {topic} within {remaining:.1f}s of type resolution; '
               'see perception_ready.log')

    now = time.monotonic()
    telemetry['first_sample_monotonic'] = now
    telemetry['process_alive_at_ready'] = all(p.poll() is None for p in node_procs)
    telemetry['launch_to_topic_discovery_ms'] = (telemetry['topic_discovered_monotonic'] - launch_t) * 1000.0
    telemetry['launch_to_first_sample_ms'] = (now - launch_t) * 1000.0
    finish()
    return telemetry


class PreReplanGateServer:
    """Harness-side std_srvs/srv/Trigger server implementing the Phase-1 ->
    HOLD transition contract for C3C.

    Runs on its OWN node and executor thread, never the monitor node: the
    callback must itself wait on /get_planning_scene, and doing that from a
    callback of a node the main loop is spin_once()-ing would be re-entrant.

    Contract (replies success=true only if ALL of these hold):
      1. remove the Phase-1 Gazebo entity
      2. confirm absence via the world pose census (never `gz model`)
      3. same-name create from the HOLD SDF
      4. confirm EXACTLY ONE entity by census -- never the create Boolean
      5. confirm the new entity publishes on its own /model/<name>/pose topic
      6. confirm the PlanningScene's dynamic_obstacle_0 converged into HOLD
    No UR5e commands. No MoveIt CollisionObject REMOVE.
    """

    def __init__(self, session, evidence, hold_sdf_path, service_name=C3C_GATE_SERVICE_NAME):
        from std_srvs.srv import Trigger
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from moveit_msgs.srv import GetPlanningScene

        self.session = session
        self.evidence = Path(evidence)
        self.hold_sdf_path = Path(hold_sdf_path)
        self.telemetry = dict(requests=0, success=None, reason=None, timeline={})

        self.node = Node('c3c_pre_replan_gate_server')
        group = ReentrantCallbackGroup()
        self.scene_cli = self.node.create_client(
            GetPlanningScene, '/get_planning_scene', callback_group=group)
        self.srv = self.node.create_service(
            Trigger, service_name, self._handle, callback_group=group)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self._stop = False
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self):
        while not self._stop:
            self.executor.spin_once(timeout_sec=0.1)

    def shutdown(self):
        self._stop = True
        self.thread.join(timeout=5.0)
        self.node.destroy_node()

    def _scene_obstacle_pose(self, timeout_s=2.0):
        from moveit_msgs.srv import GetPlanningScene
        from moveit_msgs.msg import PlanningSceneComponents
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        future = self.scene_cli.call_async(req)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if future.done():
                res = future.result()
                if res is None:
                    return None
                for co in res.scene.world.collision_objects:
                    if co.id == c1b.OBSTACLE_ID:
                        return co.pose
                return None
            time.sleep(0.01)
        return None

    def _handle(self, request, response):
        t0 = time.monotonic()
        tl = {}
        self.telemetry['requests'] += 1

        def fail(reason):
            self.telemetry.update(success=False, reason=reason, timeline=tl)
            response.success = False
            response.message = reason
            print(f'  [gate server] FAILED: {reason}', flush=True)
            return response

        try:
            tl['request_monotonic'] = t0
            # 1/2. remove, then census-confirmed absence
            despawn_gazebo_model_only(self.session, cq.OBSTACLE_MODEL_NAME, 'gate_phase1_despawn')
            tl['absent_monotonic'] = time.monotonic()

            # 3/4/5. same-name create + exactly-one census + own pose topic
            respawn_gazebo_model_same_name(
                self.session, cq.OBSTACLE_MODEL_NAME, self.hold_sdf_path.read_text(),
                'gate_hold_respawn',
                evidence_pose_log=self.evidence / 'obstacle_hold_spawn.log')
            tl['created_and_publishing_monotonic'] = time.monotonic()

            # 6. PlanningScene must actually carry the obstacle into HOLD.
            threshold = C3C_HOLD_Y_MAX + 0.01   # strictly above HOLD's whole sweep
            deadline = time.monotonic() + SCENE_CONVERGE_TIMEOUT_S
            converged = None
            while time.monotonic() < deadline:
                pose = self._scene_obstacle_pose()
                if pose is not None and pose.position.y < threshold:
                    converged = pose
                    break
                time.sleep(0.05)
            if converged is None:
                return fail('PLANNING_SCENE_HOLD_CONVERGENCE_TIMEOUT')
            tl['scene_hold_converged_monotonic'] = time.monotonic()
            tl['scene_hold_pose'] = dict(
                x=converged.position.x, y=converged.position.y, z=converged.position.z)
        except Exception as exc:  # noqa: BLE001 - any failure must reply success=false
            return fail(repr(exc))

        tl['response_monotonic'] = time.monotonic()
        tl['total_ms'] = (tl['response_monotonic'] - t0) * 1000.0
        self.telemetry.update(success=True, reason=None, timeline=tl)
        response.success = True
        response.message = 'pre-replan scene transition complete'
        print(f"  [gate server] SUCCESS in {tl['total_ms']:.1f} ms, "
              f"scene y={tl['scene_hold_pose']['y']:.4f}", flush=True)
        return response


def run(mode, prerequisite):
    if mode != 'c3a':
        previous = json.loads(Path(prerequisite).read_text())
        expected = 'c3a' if mode == 'c3b' else 'c3b'
        if previous.get('mode') != expected or previous.get('verdict') != 'PASS':
            raise RuntimeError(f'{mode} requires a passing {expected} evidence result')
    evidence = REPO / 'evidence' / ('stage3c_' + mode + '_' + time.strftime('%Y%m%d_%H%M%S'))
    evidence.mkdir(parents=True, exist_ok=False)
    print(f'Evidence directory: {evidence}', flush=True)
    session = Session(evidence)
    result = dict(mode=mode, verdict='NEEDS_CORRECTION', manipulation_started=False)
    monitor = None
    gate_server = None
    try:
        existing = strays()
        save(evidence / 'prelaunch_process_census.json', existing)
        if existing:
            raise RuntimeError('Pre-existing runtime processes; no launch attempted')
        session.run(['ps', '-eo', 'pid,cmd'], 'raw_process_census')
        session.run(['bash', '-c', "source scripts/lib/gz_settle.sh; "
            "ps() { command ps \"$@\" | awk '$2 != \"/usr/libexec/gvfsd-trash\"'; }; "
            "gz_assert_clean_slate"], 'clean_slate')

        is_parity = (mode == 'c3a')
        is_transient = (mode == 'c3b')
        is_second_trigger = (mode == 'c3c')

        obstacle_sdf = evidence / 'qualification_obstacle.sdf'
        if is_parity:
            result['scenario'] = dict(center_x=0.70, center_z=0.85,
                y_min=0.25, y_max=0.45, period_s=4.0)
            save(evidence / 'obstacle_sdf_provenance.json', cq.derive_contact_instrumented_sdf(
                REPO / 'ur5e_robotiq_description/models/dynamic_obstacle/model.sdf', obstacle_sdf))
        elif is_transient:
            create_c3b_obstacle_sdf(obstacle_sdf)
            result['scenario'] = dict(center_x=C3B_CENTER_X, center_z=C3B_CENTER_Z,
                y_min=C3B_Y_MIN, y_max=C3B_Y_MAX, period_s=C3B_PERIOD_S,
                active_window_s=C3B_ACTIVE_WINDOW_S)
        else:  # c3c
            # Phase 1 is BYTE-EQUIVALENT to the qualified C3B scenario -- same
            # SDF generator, same center/Y-range/period -- so attempt-0's
            # first trigger, State E, SCENE_A, replacement planning, SCENE_B,
            # and candidate validation are all reproduced unmodified. Phase 1
            # has NO active_window_s: it is not time-limited here at all --
            # removal is superseded by the Phase-2 relocation below, which is
            # causally gated on FJT_GOAL_ACCEPTED attempt=1, never a
            # hardcoded/derived timestamp (design task section 5).
            create_c3b_obstacle_sdf(obstacle_sdf)
            hold_sdf = evidence / 'qualification_obstacle_hold.sdf'
            create_c3b_obstacle_sdf(hold_sdf, center_x=C3C_HOLD_CENTER_X,
                center_z=C3C_HOLD_CENTER_Z, y_min=C3C_HOLD_Y_MIN,
                y_max=C3C_HOLD_Y_MAX, period_s=C3C_HOLD_PERIOD_S)
            phase2_sdf = evidence / 'qualification_obstacle_phase2.sdf'
            create_c3b_obstacle_sdf(phase2_sdf, center_x=C3C_PHASE2_CENTER_X,
                center_z=C3C_PHASE2_CENTER_Z, y_min=C3C_PHASE2_Y_MIN,
                y_max=C3C_PHASE2_Y_MAX, period_s=C3C_PHASE2_PERIOD_S)
            result['scenario'] = dict(
                phase1=dict(center_x=C3B_CENTER_X, center_z=C3B_CENTER_Z,
                    y_min=C3B_Y_MIN, y_max=C3B_Y_MAX, period_s=C3B_PERIOD_S),
                phase2=dict(center_x=C3C_PHASE2_CENTER_X, center_z=C3C_PHASE2_CENTER_Z,
                    y_min=C3C_PHASE2_Y_MIN, y_max=C3C_PHASE2_Y_MAX,
                    period_s=C3C_PHASE2_PERIOD_S),
                hold=dict(center_x=C3C_HOLD_CENTER_X, center_z=C3C_HOLD_CENTER_Z,
                    y_min=C3C_HOLD_Y_MIN, y_max=C3C_HOLD_Y_MAX,
                    period_s=C3C_HOLD_PERIOD_S),
                pre_replan_gate_service=C3C_GATE_SERVICE_NAME,
                phase2_gate='FJT_GOAL_ACCEPTED attempt=1',
                phase2_gate_timeout_s=C3C_PHASE2_GATE_TIMEOUT_S)

        session.launch('gz_pose_observer', ['python3', str(REPO / 'scripts/perception/gz_pose_observer.py'),
                       '--out', str(evidence / 'gz_pose_stream.csv')])
        print('Launching fresh native-Linux simulator', flush=True)
        session.launch('sim', ['ros2', 'launch', 'ur5e_robotiq_description',
            'ur5e_robotiq_sim_control.launch.py', 'gripper_model:=parallel_jaw',
            'enable_camera:=true', 'gazebo_gui:=false'])
        for controller in ('arm_controller', 'parallel_jaw_gripper_controller'):
            session.run(['bash', '-c', 'source scripts/lib/gz_settle.sh; '
                'gz_wait_controller_active_bounded ' + controller + ' 45'],
                controller + '_ready', timeout=60)
        for i, position in enumerate((0.0, 0.005, 0.0)):
            output = session.run(['ros2', 'action', 'send_goal',
                '/parallel_jaw_gripper_controller/gripper_cmd', 'control_msgs/action/GripperCommand',
                '{command: {position: ' + str(position) + ', max_effort: 5.0}}'],
                f'gripper_precondition_{i}', timeout=15)
            if 'reached_goal: true' not in output or 'stalled: true' in output:
                raise RuntimeError('Parallel-jaw free-air responsiveness precondition failed')
        session.run(['bash', '-c', 'source scripts/lib/gz_settle.sh; '
            'gz_assert_joint /world/empty/model/ur5e_robotiq/joint_state gripper_jaw_joint 0.0 0.0005 C3_OPEN'],
            'gripper_open_verified')
        session.launch('move_group', ['ros2', 'launch', 'ur5e_robotiq_moveit_config',
                       'move_group.launch.py', 'gripper_model:=parallel_jaw'])
        rclpy.init()
        monitor = c1b.C1BMonitorNode()
        if not monitor.get_scene_cli.wait_for_service(timeout_sec=20):
            raise RuntimeError('PlanningScene service unavailable')
        if not monitor.validity_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError('MoveIt validity service unavailable')
        session.launch('dynamic_scene', ['ros2', 'launch', 'ur5e_pick_place',
                       'stage3b_dynamic_scene.launch.py', 'use_sim_time:=true'])
        contact_csv = evidence / 'gazebo_obstacle_contacts.csv'
        contact_observer = session.launch('contact_observer', ['python3',
            str(REPO / 'scripts/perception/gz_contact_observer.py'), '--topic', cq.CONTACT_TOPIC,
            '--out', str(contact_csv)])

        if is_parity:
            spawn = cq.spawn_model(cq.OBSTACLE_MODEL_NAME, obstacle_sdf.read_text())
            (evidence / 'obstacle_spawn.log').write_text(spawn.stdout + spawn.stderr)
            if 'true' not in spawn.stdout.lower():
                raise RuntimeError('B1 obstacle spawn failed')
            deadline = time.monotonic() + 20
            registered = False
            while time.monotonic() < deadline:
                rclpy.spin_once(monitor, timeout_sec=0.05)
                scene = monitor.query_planning_scene(timeout_sec=2)
                if scene and any(co.id == c1b.OBSTACLE_ID for co in scene.world.collision_objects):
                    registered = True
                    break
            if not registered:
                raise RuntimeError('B1 obstacle failed to register')
            start = monitor.get_clock().now().nanoseconds * 1e-9
            deadline = time.monotonic() + 15
            while monitor.get_clock().now().nanoseconds * 1e-9 - start < 4.5:
                if time.monotonic() >= deadline:
                    raise RuntimeError('B1 motion observation timed out')
                rclpy.spin_once(monitor, timeout_sec=0.05)
            ys = [p['y'] for p in monitor.ros_poses]
            save(evidence / 'b1_envelope.json', dict(y_min=min(ys), y_max=max(ys), n=len(ys)))
            if min(ys) > 0.26 or max(ys) < 0.44:
                raise RuntimeError('B1 envelope not observed')

        print('Preparing settled Scene-A object and perception', flush=True)
        f1.remove_object()
        spawned = f1.spawn_object(0.45, -0.15)
        (evidence / 'target_spawn.log').write_text(spawned)
        if 'true' not in spawned.lower():
            raise RuntimeError('Target spawn failed')
        settled, detail = f1.settle_object(timeout=20)
        (evidence / 'target_initial_settle.log').write_text(detail)
        if not settled:
            raise RuntimeError('Initial target settle failed')
        save(evidence / 'init_settled_pose.json', f1.instantaneous_object_pose())
        perception_procs = [session.launch(node, ['ros2', 'run', 'ur5e_pick_place', node,
                                 '--ros-args', '-p', 'use_sim_time:=true'])
                            for node in ('object_detector', 'object_position_world')]
        perception_readiness = wait_for_perception_ready(session, evidence, perception_procs)
        result['perception_readiness'] = perception_readiness
        if is_second_trigger:
            # Advertise the pre-replan scene gate BEFORE m3_grasp starts, so the
            # coordinator's bounded availability wait never races startup.
            gate_server = PreReplanGateServer(session, evidence, hold_sdf)
            print(f'Pre-replan scene gate advertised at {C3C_GATE_SERVICE_NAME}', flush=True)

        args = ['ros2', 'launch', 'ur5e_pick_place', 'm3_grasp.launch.py',
                'gripper_model:=parallel_jaw', 'use_perceived_position:=true',
                'require_perception:=true', 'csv_path:=' + str(evidence / 'm3_grasp.csv'),
                'marker_file_prefix:=' + str(evidence / 'stage')]
        if is_second_trigger:
            args.append('transport_pre_replan_gate_service_name:=' + C3C_GATE_SERVICE_NAME)
        result['m3_command'] = args
        save(evidence / 'predeclared_run.json', result)
        print(f'Executing the ONE {mode} manipulation run', flush=True)
        result['manipulation_started'] = True
        m3 = session.launch('m3_grasp', args)
        deadline = time.monotonic() + 240
        spawned_at = None
        removed_at = None
        validity_samples = []
        next_validity = 0.0

        active_window_s = result['scenario'].get('active_window_s', 0.0)
        phase2_begun_at = None  # C3C only: sim time the Phase-2 relocation was issued
        phase2_gate_deadline = None  # set once attempt-0 execution begins (TRANSPORT_BEGIN)

        while time.monotonic() < deadline:
            rclpy.spin_once(monitor, timeout_sec=0.05)
            log = (evidence / 'm3_grasp.log').read_text(errors='replace')
            if not is_parity and spawned_at is None and 'M3 STAGE 4 TRANSPORT_BEGIN' in log:
                spawned_at = monitor.get_clock().now().nanoseconds * 1e-9
                spawn = cq.spawn_model(cq.OBSTACLE_MODEL_NAME, obstacle_sdf.read_text())
                (evidence / 'obstacle_spawn.log').write_text(spawn.stdout + spawn.stderr)
                if 'true' not in spawn.stdout.lower():
                    raise RuntimeError('Obstacle spawn failed')
                print(f'Obstacle spawned at sim {spawned_at:.6f}', flush=True)
                if is_second_trigger:
                    # Anchor the Phase-2 gate timeout to attempt-0 execution
                    # start, not the harness's own earlier pregrasp/descent/
                    # lift wall-clock time.
                    phase2_gate_deadline = time.monotonic() + C3C_PHASE2_GATE_TIMEOUT_S

            now_sim = monitor.get_clock().now().nanoseconds * 1e-9
            if not is_parity and spawned_at is not None and removed_at is None and is_transient:
                if now_sim - spawned_at >= active_window_s:
                    removal = session.run(['gz', 'service', '-s', '/world/empty/remove',
                        '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '5000', '--req', 'name: "dynamic_obstacle", type: MODEL'],
                        'obstacle_remove')
                    if 'true' not in removal.lower():
                        raise RuntimeError('Transient obstacle removal failed')
                    monitor.remove_obstacle_from_scene()
                    removed_at = now_sim
                    print(f'Transient obstacle removed at sim {removed_at:.6f}', flush=True)

            # C3C Phase-2 causal gate (design task section 5/17): relocate
            # dynamic_obstacle_0 ONLY after directly observing production
            # telemetry proving the replacement (attempt=1) FJT goal was
            # actually accepted -- never a hardcoded/derived sim timestamp,
            # never before this exact line has been seen in the log.
            if is_second_trigger and spawned_at is not None and phase2_begun_at is None:
                if 'M3 C3 FJT_GOAL_ACCEPTED attempt=1' in log:
                    phase2_begun_at = now_sim
                    result.setdefault('c3c_causal', {})['t_phase2_begin_sim'] = phase2_begun_at
                    print(f'Phase-2 gate satisfied (FJT_GOAL_ACCEPTED attempt=1) at sim '
                          f'{phase2_begun_at:.6f}; relocating dynamic_obstacle_0', flush=True)
                    # Measured safe barrier (probe-derived, see the block
                    # comment above despawn_gazebo_model_only()): despawn ->
                    # POLL until authoritatively absent -> same-name create
                    # -> require exactly one resulting entity -> wait for
                    # the pose topic to resume -> wait for the
                    # PlanningScene to converge on the Phase-2 region.
                    # Never a blind sleep; every wait below is bounded and
                    # measured. No /collision_object REMOVE is ever
                    # published, so the PlanningScene keeps
                    # dynamic_obstacle_0's identity across the whole gap.
                    despawn_gazebo_model_only(session, 'dynamic_obstacle', 'phase1_despawn')
                    respawn_gazebo_model_same_name(
                        session, 'dynamic_obstacle', phase2_sdf.read_text(), 'phase2_respawn',
                        evidence_pose_log=evidence / 'obstacle_phase2_spawn.log')
                    phase2_pose = wait_for_planning_scene_phase2_convergence(
                        monitor, y_threshold=C3C_PHASE2_Y_MIN - 0.01)
                    result['c3c_causal']['phase2_planning_scene_pose'] = dict(
                        x=phase2_pose.position.x, y=phase2_pose.position.y, z=phase2_pose.position.z)
                    print(f'Phase-2 transition complete: PlanningScene pose y={phase2_pose.position.y:.4f} '
                          '(converged)', flush=True)
                elif time.monotonic() >= phase2_gate_deadline:
                    raise RuntimeError(
                        'Phase-2 gate timeout: FJT_GOAL_ACCEPTED attempt=1 never observed '
                        f'within {C3C_PHASE2_GATE_TIMEOUT_S}s of attempt-0 collision stop')

            if 'RUN SUMMARY:' in log and (is_parity or spawned_at is not None):
                break
            if m3.poll() is not None:
                raise RuntimeError('Manipulation launch exited before run summary')
            if time.monotonic() >= next_validity:
                v = monitor.query_state_validity(timeout_sec=1)
                validity_samples.append(dict(sim_time=now_sim, valid=v.valid if v else None,
                    dynamic_pairs=[(c.contact_body_1, c.contact_body_2) for c in v.contacts
                        if c1b.OBSTACLE_ID in (c.contact_body_1, c.contact_body_2)] if v else None))
                next_validity = time.monotonic() + 1.0
            time.sleep(0.1)
        else:
            raise RuntimeError('Manipulation timed out; no retry')

        log = (evidence / 'm3_grasp.log').read_text(errors='replace')
        result['run_summary'] = next((l for l in log.splitlines() if 'RUN SUMMARY:' in l), '')
        result['scenario'].update(spawn_sim_time=spawned_at, remove_sim_time=removed_at)
        post_scene = monitor.query_planning_scene()
        if post_scene is None:
            raise RuntimeError('Post-run PlanningScene readback failed')
        save(evidence / 'post_stop_planning_scene.json', message_to_ordereddict(post_scene))
        result['target_attached'] = any(a.object.id == 'pick_target'
            for a in post_scene.robot_state.attached_collision_objects)
        result['moveit_current_state_samples'] = validity_samples
        result['c1'] = c1a.parse_c1_monitor_telemetry(log)
        result['c3'] = analyze_c3(log)
        c3 = result['c3']
        result['flow'] = {key: marker in log for key, marker in dict(
            place='M3 STAGE 5 PLACE_DESCEND_BEGIN', release='M3 STAGE 6 RELEASE_BEGIN',
            detach='DETACH_VERIFIED:', retreat='M3 STAGE 7 RETREAT_BEGIN').items()}

        expected = ('SUCCESS' if (is_parity or is_transient) else 'TRANSPORT_REPLAN_LIMIT_REACHED')
        row = list(csv.DictReader((evidence / 'm3_grasp.csv').open()))[-1]
        result['transport_result'] = row['transport_result']
        result['result'] = row['result']

        if is_parity or is_transient:
            session.run(['bash', '-c', 'source scripts/lib/gz_settle.sh; '
                'gz_settle_pose_windowed /world/empty/pose/info 0.0005 20 0.15 1.0 ' + f1.OBJ_NAME],
                'final_target_settle', timeout=35)
            save(evidence / 'final_settled_pose.json', f1.instantaneous_object_pose())
            cfg = yaml.safe_load((REPO / 'config/scene.yaml').read_text())
            result['manipulation_metrics'] = analyzer.analyze_case(evidence, configured_yaw_deg=0,
                target_place_xyz=[cfg['object']['place_pose'][k] for k in ('x', 'y', 'z')],
                target_place_yaw_deg=0, require_perceived_yaw=False,
                use_axial_placement_yaw=True, require_translation_decoupling=False)

        save(evidence / 'before_liveness_results.json', result)
        if contact_observer.poll() is not None:
            raise RuntimeError('Contact observer exited; physical evidence unqualified')
        print('Running same-observer post-window contact liveness probe', flush=True)
        if is_parity:
            blocker_xyz = (0.70, 0.35, 0.85)
        elif is_second_trigger:
            # Positive-control blocker placed against the obstacle's ACTUAL
            # live Gazebo pose, queried authoritatively at probe time --
            # never against an assumed phase.
            #
            # The previous version assumed Phase 2 was necessarily the last
            # active region by the time this probe runs. That assumption is
            # false whenever the run terminates before the Phase-2 causal
            # gate fires (e.g. the candidate-validation rejection in
            # evidence/stage3c_c3c_20260909_014313/): the obstacle was still
            # sweeping Phase 1's Y=[-0.05,+0.05] while the blocker was
            # staged at Phase 2's Y=0.24 -- a 140 mm gap the obstacle could
            # never close, so the positive control recorded 0 contacts and
            # the run's own zero-contact result was left unfalsifiable.
            # Which phase the obstacle is actually in is known deterministically
            # from this harness's own Phase-2 gate bookkeeping -- it relocates
            # the obstacle itself and records when. That is the primary signal;
            # a live `gz model` pose query is recorded only as corroboration,
            # never depended upon (its /gazebo/worlds lookup was directly
            # observed timing out against a healthy, running simulator).
            phase2_active = result.get('c3c_causal', {}).get('t_phase2_begin_sim') is not None
            if phase2_active:
                blocker_xyz = (C3C_PHASE2_CENTER_X, C3C_PHASE2_Y_MAX - 0.01, C3C_PHASE2_CENTER_Z)
            else:
                blocker_xyz = (C3B_CENTER_X, C3B_Y_MAX - 0.01, C3B_CENTER_Z)
            live_pose = cq.model_pose(cq.OBSTACLE_MODEL_NAME)
            result.setdefault('c3c_causal', {}).update(
                liveness_blocker_phase='phase2' if phase2_active else 'phase1',
                liveness_blocker_xyz=list(blocker_xyz),
                liveness_obstacle_live_pose=list(live_pose) if live_pose else None)
        else:
            blocker_xyz = (C3B_CENTER_X, C3B_Y_MAX - 0.01, C3B_CENTER_Z)
        if is_parity:
            respawn_sdf = None
        elif is_second_trigger:
            # Must match the phase the blocker was placed for, so a respawn
            # (only needed if the entity is genuinely absent) restores an
            # obstacle that can actually reach that blocker.
            respawn_sdf = phase2_sdf if phase2_active else obstacle_sdf
        else:
            respawn_sdf = obstacle_sdf
        liveness = cq.run_liveness_probe(evidence, contact_csv,
            blocker_xyz=blocker_xyz, respawn_obstacle_sdf=respawn_sdf,
            blocker_size=0.10 if is_parity else 0.05)
        save(evidence / 'contact_observer_liveness.json', liveness)
        contacts = cq.summarize_contact_csv(contact_csv, filter_substrings=[cq.OBSTACLE_MODEL_NAME],
                                           wall_ns_max=liveness['probe_start_wall_ns'])
        result['gazebo_physical_contacts'] = contacts
        result['contact_liveness'] = liveness
        ticks = result['c1']['ticks']
        future_invalid = sum(t.get('future_path_valid') == '0' for t in ticks)
        result['future_invalid_ticks'] = future_invalid

        gates = dict(
            typed_result=row['result'] == expected and row['transport_result'] == expected,
            monitor_active=bool(result['c1']['start_line']) and bool(result['c1']['stop_line']) and len(ticks) > 0,
            physical_contacts_zero=contacts['matched_pair_rows'] == 0,
            contact_observer_live=liveness['liveness_proven'],
            no_crash=not re.search(r'Segmentation fault|exit code -|terminate called', log)
        )

        if is_parity:
            gates.update(
                full_cycle=result['manipulation_metrics']['verdict'] == 'PASS',
                future_valid=future_invalid == 0,
                no_trigger=c3['trigger_count'] == 0,
                no_cancel=c3['cancel_count'] == 0,
                no_replan=c3['replan_count'] == 0,
                fjt_succeeded='terminal_action_status=SUCCEEDED' in log,
                full_flow=all(result['flow'].values()),
                detached=not result['target_attached']
            )
        elif is_transient:
            gates.update(
                one_trigger=c3['trigger_count'] == 1,
                one_cancel=c3['cancel_count'] == 1,
                replan_count=c3['replan_count'] == 1,
                scene_a_acquired=bool(c3['scene_a']),
                replan_plan_ok=bool(c3['replan_plan']),
                scene_b_acquired=bool(c3['scene_b']),
                replacement_send_ready=bool(c3['replacement_send']),
                replacement_success=c3['replacement_success'],
                full_flow=all(result['flow'].values()),
                detached=not result['target_attached'],
                state_e=len(c3['state_es']) == 1 and c3['state_es'][0].get('captured') == '1',
                full_cycle=result['manipulation_metrics']['verdict'] == 'PASS'
            )
        else:  # c3c
            # Second trigger budget limit:
            # 2 triggers, 2 cancels, cancel_exact for both, 2 physical settles, State E2 captured,
            # no third trajectory, no post-stop flow, target attached
            cancel_exact_1 = False
            cancel_exact_2 = False
            if len(c3['cancels']) >= 2 and len(c3['responses']) >= 2:
                cancel_exact_1 = (c3['responses'][0].get('this_goal_confirmed') == '1' and
                                  c3['responses'][0].get('goal_uuid') == c3['cancels'][0].get('goal_uuid'))
                cancel_exact_2 = (c3['responses'][1].get('this_goal_confirmed') == '1' and
                                  c3['responses'][1].get('goal_uuid') == c3['cancels'][1].get('goal_uuid'))

            # Attempt-aware State E / State E2, keyed by the coordinator's
            # own explicit attempt= field (transport_coordinator.cpp
            # SETTLED_STATE telemetry) rather than chronological list
            # position -- avoids relying on log-line ordering alone for
            # the E-vs-E2 distinction.
            state_e_by_attempt = c3['state_e_by_attempt']
            state_e_ok = state_e_by_attempt.get('0', {}).get('captured') == '1'
            state_e2_ok = state_e_by_attempt.get('1', {}).get('captured') == '1'

            # Budget-exhaustion telemetry: exactly one REPLAN_BUDGET_EXHAUSTED
            # line, with the fixed contract (replan_count=1, max_replans=1,
            # attempt=1, result=TRANSPORT_REPLAN_LIMIT_REACHED).
            budget_events = c3['budget_exhausted']
            budget_exhausted_ok = (
                len(budget_events) == 1 and
                budget_events[0].get('replan_count') == '1' and
                budget_events[0].get('max_replans') == '1' and
                budget_events[0].get('attempt') == '1' and
                budget_events[0].get('result') == 'TRANSPORT_REPLAN_LIMIT_REACHED')

            # No-third-goal / exactly-one-per-attempt proof (design task
            # section 8/19): mirrors ur5e_pick_place::goal_acceptance_count_ok()'s
            # own contract, evaluated here from the harness's independent
            # FJT_GOAL_ACCEPTED attempt=<n> tally rather than trusting only
            # the static source proof.
            goal_count_ok = (
                c3['attempt0_goal_accepted_count'] == 1 and
                c3['attempt1_goal_accepted_count'] == 1 and
                c3['attempt2_plus_goal_accepted_count'] == 0)

            # Monitor summary telemetry present for both attempts (design
            # task section 10) -- qualification-critical for attempt 1
            # specifically, since that is where the second trigger must be
            # observed.
            monitor_summary_by_attempt = {e.get('attempt'): e for e in c3['monitor_summaries']}
            monitor_summary_ok = '0' in monitor_summary_by_attempt and '1' in monitor_summary_by_attempt

            gates.update(
                two_triggers=c3['trigger_count'] == 2,
                two_cancels=c3['cancel_count'] == 2,
                cancel_exact_1=cancel_exact_1,
                cancel_exact_2=cancel_exact_2,
                replan_count=c3['replan_count'] == 1,
                second_trigger_limit=c3['second_trigger_limit'],
                no_post_stop_flow=not any(result['flow'].values()),
                still_attached=result['target_attached'],
                state_e=state_e_ok,
                state_e2=state_e2_ok,
                # --- C3C-specific attempt-aware / goal-count / causal proofs ---
                goal_acceptance_count_ok=goal_count_ok,
                goal_uuids_distinct=c3['goal_uuids_distinct'],
                budget_exhausted_telemetry_ok=budget_exhausted_ok,
                monitor_summary_present_both_attempts=monitor_summary_ok,
                candidate_validation_before_attempt1_accepted=c3['causal_order_ok'],
                phase2_began_after_attempt1_accepted=(
                    result.get('c3c_causal', {}).get('t_phase2_begin_sim') is not None),
            )

        result['gates'] = gates
        result['verdict'] = 'PASS' if all(gates.values()) else 'NEEDS_CORRECTION'
    except Exception as exc:
        result['anomaly'] = repr(exc)
        if isinstance(exc, PerceptionReadinessError):
            result['perception_readiness_failure_code'] = exc.code
        print(f'STOP: {exc}', flush=True)
    finally:
        if gate_server is not None:
            result['pre_replan_gate_server'] = gate_server.telemetry
            gate_server.shutdown()
        if monitor is not None:
            monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if session.procs:
            result['cleanup_clean'] = session.cleanup()
        else:
            result['cleanup_clean'] = not strays()
        if not result['cleanup_clean']:
            result['verdict'] = 'NEEDS_CORRECTION'
        save(evidence / 'qualification_results.json', result)
        print(json.dumps({k: result.get(k) for k in ('mode', 'verdict', 'gates', 'anomaly', 'cleanup_clean')}, indent=2), flush=True)
    return 0 if result['verdict'] == 'PASS' else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['c3a', 'c3b', 'c3c'])
    parser.add_argument('--prerequisite', help='Previous mode qualification_results.json (must PASS)')
    args = parser.parse_args()
    if args.mode != 'c3a' and not args.prerequisite:
        parser.error('--prerequisite is required for sequential qualification')
    sys.exit(run(args.mode, args.prerequisite))

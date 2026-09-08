#!/usr/bin/env python3
"""One-shot Stage-3C C2 qualification; run modes sequentially, never auto-retry.

C2A uses production B1 motion with C1's sensor-only SDF addition. C2B
imports the exact published C1B SDF generator and 1.2 s active window.
The watchdog case uses B1 with the existing watchdog parameters overridden
(scaling 0.3, margin 0.0). No scenario parameter tuning is exposed.

Contact evidence uses the published C1 DART observer and same-instance
post-window positive control. MoveIt validity is reported separately.
All long-lived subprocesses have owned process groups. Cleanup tracks PID
start times, observes shutdown, and checks a final process census.
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
            time.sleep(0.1)  # bounded process-exit observation, never robot settle
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
        # Owned PID/PGID cleanup implements the requested C2 teardown boundary.
        # Do not call legacy kill_sim here: its residual 'spawner' substring
        # also matches gvfsd-trash --spawner on this native desktop (recorded
        # prelaunch failure stage3c_c2a_20260908_103509).
        remaining = strays()
        save(self.evidence / 'cleanup_after.json', remaining)
        return not remaining


ANSI_ESCAPE = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')


def fields(line):
    # ros2 launch frames each process line with terminal colour escapes. The
    # reset at end-of-line otherwise becomes part of the final unquoted value
    # (for example this_goal_confirmed parses as "1\x1b[0m"), producing false
    # qualification failures even though the raw telemetry says 1.
    clean = ANSI_ESCAPE.sub('', line)
    return {k: quoted if quoted else plain for k, quoted, plain in
            re.findall(r'(\w+)=(?:"([^"]*)"|(\S+))', clean)}


def analyze_c2(log):
    events = {}
    for line in log.splitlines():
        match = re.search(r'M3 C2 (\w+) ', line)
        if match:
            events.setdefault(match.group(1), []).append(fields(line))
    def first(key):
        return events.get(key, [{}])[0]
    return dict(events=events, trigger=first('COLLISION_TRIGGER'),
        cancel=first('CANCEL_REQUEST'), response=first('CANCEL_RESPONSE'),
        terminal=first('TERMINAL'), settle=first('PHYSICAL_SETTLE_CONFIRMED'),
        state_e=first('STATE_E'), joints=events.get('STATE_E_JOINT', []),
        trigger_count=len(events.get('COLLISION_TRIGGER', [])),
        cancel_count=sum(e.get('cause') == 'COLLISION_STOP' for e in events.get('CANCEL_REQUEST', [])),
        watchdog_count=sum(e.get('cause') == 'WATCHDOG_CLEANUP' for e in events.get('CANCEL_REQUEST', [])),
        stop_count=sum(e.get('result') == 'TRANSPORT_COLLISION_STOPPED'
                       for e in events.get('STOP_RESULT', [])),
        replan_count=0 if not re.search(r'REPLAN_BEGIN|REPLAN_REQUEST', log) else 1)


def run(mode, prerequisite):
    if mode != 'c2a':
        previous = json.loads(Path(prerequisite).read_text())
        expected = 'c2a' if mode == 'c2b' else 'c2b'
        if previous.get('mode') != expected or previous.get('verdict') != 'PASS':
            raise RuntimeError(f'{mode} requires a passing {expected} evidence result')
    evidence = REPO / 'evidence' / ('stage3c_' + mode + '_' + time.strftime('%Y%m%d_%H%M%S'))
    evidence.mkdir(parents=True, exist_ok=False)
    print(f'Evidence directory: {evidence}', flush=True)
    session = Session(evidence)
    result = dict(mode=mode, verdict='NEEDS_CORRECTION', manipulation_started=False)
    monitor = None
    try:
        existing = strays()
        save(evidence / 'prelaunch_process_census.json', existing)
        if existing:
            raise RuntimeError('Pre-existing runtime processes; no launch attempted')
        # The published helper's broad 'spawner' match catches this unrelated
        # native desktop executable. Keep every other row and the helper's
        # original assertion intact; preserve raw and filtered census evidence.
        session.run(['ps', '-eo', 'pid,cmd'], 'raw_process_census')
        session.run(['bash', '-c', "source scripts/lib/gz_settle.sh; "
            "ps() { command ps \"$@\" | awk '$2 != \"/usr/libexec/gvfsd-trash\"'; }; "
            "gz_assert_clean_slate"], 'clean_slate')
        transient = mode == 'c2b'
        obstacle_sdf = evidence / 'qualification_obstacle.sdf'
        if transient:
            c1b.create_c1b_obstacle_sdf(obstacle_sdf)
            result['scenario'] = dict(center_x=c1b.C1B_CENTER_X, center_z=c1b.C1B_CENTER_Z,
                y_min=c1b.C1B_Y_MIN, y_max=c1b.C1B_Y_MAX, period_s=c1b.C1B_PERIOD_S,
                active_window_s=c1b.C1B_ACTIVE_WINDOW_S)
        else:
            result['scenario'] = dict(center_x=0.70, center_z=0.85,
                y_min=0.25, y_max=0.45, period_s=4.0)
            save(evidence / 'obstacle_sdf_provenance.json', cq.derive_contact_instrumented_sdf(
                REPO / 'ur5e_robotiq_description/models/dynamic_obstacle/model.sdf', obstacle_sdf))
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
        # Native parallel-jaw units: opening and a small 0.005 m free-air stroke,
        # then verified open. The historical 0.1 rad linkage helper is inapplicable.
        for i, position in enumerate((0.0, 0.005, 0.0)):
            output = session.run(['ros2', 'action', 'send_goal',
                '/parallel_jaw_gripper_controller/gripper_cmd', 'control_msgs/action/GripperCommand',
                '{command: {position: ' + str(position) + ', max_effort: 5.0}}'],
                f'gripper_precondition_{i}', timeout=15)
            if 'reached_goal: true' not in output or 'stalled: true' in output:
                raise RuntimeError('Parallel-jaw free-air responsiveness precondition failed')
        session.run(['bash', '-c', 'source scripts/lib/gz_settle.sh; '
            'gz_assert_joint /world/empty/model/ur5e_robotiq/joint_state gripper_jaw_joint 0.0 0.0005 C2_OPEN'],
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
        if not transient:
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
            # Measure at least one full sim-time motion period before proceeding.
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
        for node in ('object_detector', 'object_position_world'):
            session.launch(node, ['ros2', 'run', 'ur5e_pick_place', node,
                                 '--ros-args', '-p', 'use_sim_time:=true'])
        output = session.run(['ros2', 'topic', 'echo', '/object_detector/position_world', '--once'],
                             'perception_ready', timeout=30)
        if 'x:' not in output:
            raise RuntimeError('Perception sample absent')
        args = ['ros2', 'launch', 'ur5e_pick_place', 'm3_grasp.launch.py',
                'gripper_model:=parallel_jaw', 'use_perceived_position:=true',
                'require_perception:=true', 'csv_path:=' + str(evidence / 'm3_grasp.csv'),
                'marker_file_prefix:=' + str(evidence / 'stage')]
        if mode == 'watchdog':
            args += ['transport_execution_duration_scaling:=0.3', 'transport_goal_duration_margin_s:=0.0']
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
        while time.monotonic() < deadline:
            rclpy.spin_once(monitor, timeout_sec=0.05)
            log = (evidence / 'm3_grasp.log').read_text(errors='replace')
            if transient and spawned_at is None and 'M3 STAGE 4 TRANSPORT_BEGIN' in log:
                spawned_at = monitor.get_clock().now().nanoseconds * 1e-9
                spawn = cq.spawn_model(cq.OBSTACLE_MODEL_NAME, obstacle_sdf.read_text())
                (evidence / 'obstacle_spawn.log').write_text(spawn.stdout + spawn.stderr)
                if 'true' not in spawn.stdout.lower():
                    raise RuntimeError('Transient obstacle spawn failed')
                print(f'Transient obstacle spawned at sim {spawned_at:.6f}', flush=True)
            now_sim = monitor.get_clock().now().nanoseconds * 1e-9
            if transient and spawned_at is not None and removed_at is None:
                if now_sim - spawned_at >= c1b.C1B_ACTIVE_WINDOW_S:
                    removal = session.run(['gz', 'service', '-s', '/world/empty/remove',
                        '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '5000', '--req', 'name: "dynamic_obstacle", type: MODEL'],
                        'obstacle_remove')
                    if 'true' not in removal.lower():
                        raise RuntimeError('Transient obstacle removal failed')
                    monitor.remove_obstacle_from_scene()
                    removed_at = now_sim
                    print(f'Transient obstacle removed at sim {removed_at:.6f}', flush=True)
            if 'RUN SUMMARY:' in log and (not transient or spawned_at is None or removed_at is not None):
                break
            if m3.poll() is not None:
                raise RuntimeError('Manipulation launch exited before run summary')
            if time.monotonic() >= next_validity:
                v = monitor.query_state_validity(timeout_sec=1)
                validity_samples.append(dict(sim_time=now_sim, valid=v.valid if v else None,
                    dynamic_pairs=[(c.contact_body_1, c.contact_body_2) for c in v.contacts
                        if c1b.OBSTACLE_ID in (c.contact_body_1, c.contact_body_2)] if v else None))
                next_validity = time.monotonic() + 1.0
            time.sleep(0.1)  # C1B harness observation cadence, not a robot-settle wait
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
        result['c2'] = analyze_c2(log)
        c2 = result['c2']
        result['flow'] = {key: marker in log for key, marker in dict(
            place='M3 STAGE 5 PLACE_DESCEND_BEGIN', release='M3 STAGE 6 RELEASE_BEGIN',
            detach='DETACH_VERIFIED:', retreat='M3 STAGE 7 RETREAT_BEGIN').items()}
        expected = ('SUCCESS' if mode == 'c2a' else 'TRANSPORT_COLLISION_STOPPED'
                    if mode == 'c2b' else 'TRANSPORT_EXECUTION_WATCHDOG_TIMEOUT')
        row = list(csv.DictReader((evidence / 'm3_grasp.csv').open()))[-1]
        result['transport_result'] = row['transport_result']
        result['result'] = row['result']
        if mode == 'c2a':
            session.run(['bash', '-c', 'source scripts/lib/gz_settle.sh; '
                'gz_settle_pose_windowed /world/empty/pose/info 0.0005 20 0.15 1.0 ' + f1.OBJ_NAME],
                'final_target_settle', timeout=35)
            save(evidence / 'final_settled_pose.json', f1.instantaneous_object_pose())
            cfg = yaml.safe_load((REPO / 'config/scene.yaml').read_text())
            result['manipulation_metrics'] = analyzer.analyze_case(evidence, configured_yaw_deg=0,
                target_place_xyz=[cfg['object']['place_pose'][k] for k in ('x', 'y', 'z')],
                target_place_yaw_deg=0, require_perceived_yaw=False,
                use_axial_placement_yaw=True, require_translation_decoupling=False)
        # All stop/attachment evidence above is frozen before the deliberate contact probe.
        save(evidence / 'before_liveness_results.json', result)
        if contact_observer.poll() is not None:
            raise RuntimeError('Contact observer exited; physical evidence unqualified')
        print('Running same-observer post-window contact liveness proof', flush=True)
        liveness = cq.run_liveness_probe(evidence, contact_csv,
            blocker_xyz=(0.45, 0.34, 0.86) if transient else (0.70, 0.35, 0.85),
            respawn_obstacle_sdf=obstacle_sdf if transient else None,
            blocker_size=0.05 if transient else 0.10)
        save(evidence / 'contact_observer_liveness.json', liveness)
        contacts = cq.summarize_contact_csv(contact_csv, filter_substrings=[cq.OBSTACLE_MODEL_NAME],
                                           wall_ns_max=liveness['probe_start_wall_ns'])
        result['gazebo_physical_contacts'] = contacts
        result['contact_liveness'] = liveness
        ticks = result['c1']['ticks']
        future_invalid = sum(t.get('future_path_valid') == '0' for t in ticks)
        result['future_invalid_ticks'] = future_invalid
        gates = dict(typed_result=row['result'] == expected and row['transport_result'] == expected,
            monitor_active=bool(result['c1']['start_line']) and bool(result['c1']['stop_line']) and len(ticks) > 0,
            physical_contacts_zero=contacts['matched_pair_rows'] == 0,
            contact_observer_live=liveness['liveness_proven'],
            no_replan=c2['replan_count'] == 0,
            no_crash=not re.search(r'Segmentation fault|exit code -|terminate called', log))
        if mode == 'c2a':
            gates.update(full_cycle=result['manipulation_metrics']['verdict'] == 'PASS',
                future_valid=future_invalid == 0, no_trigger=c2['trigger_count'] == 0,
                no_cancel=c2['cancel_count'] == 0, no_stop=c2['stop_count'] == 0,
                no_watchdog=c2['watchdog_count'] == 0,
                fjt_succeeded='terminal_action_status=SUCCEEDED' in log,
                full_flow=all(result['flow'].values()), detached=not result['target_attached'])
        else:
            gates.update(no_post_stop_flow=not any(result['flow'].values()),
                still_attached=result['target_attached'],
                cancel_exact=c2['response'].get('return_code') == '0'
                    and c2['response'].get('this_goal_confirmed') == '1'
                    and bool(c2['cancel'].get('goal_uuid'))
                    and c2['response'].get('goal_uuid') == c2['cancel'].get('goal_uuid'),
                fjt_canceled=c2['terminal'].get('terminal_action_status') == 'CANCELED',
                physical_settle=c2['settle'].get('distinct_samples') == '6',
                target_not_reached=float(c2['state_e'].get('original_target_max_error_rad', 'nan')) > 0.01,
                state_e=c2['state_e'].get('captured') == '1' and len(c2['joints']) == 6)
            if mode == 'c2b':
                gates.update(one_trigger=c2['trigger_count'] == 1, one_cancel=c2['cancel_count'] == 1,
                    stopped=c2['stop_count'] == 1, no_watchdog=c2['watchdog_count'] == 0,
                    trigger_current_valid=c2['trigger'].get('current_state_valid') == '1',
                    trigger_future_invalid=c2['trigger'].get('future_path_valid') == '0',
                    trigger_fresh=0 <= float(c2['trigger'].get('scene_age_ms', 'nan')) <= 250,
                    expected_pair='dynamic_obstacle_0' in c2['trigger'].get('collision_pairs', ''),
                    measurable_state_delta=float(c2['state_e'].get('immediate_to_e_max_delta_rad', 'nan')) > 1e-6)
            else:
                gates.update(one_watchdog=c2['watchdog_count'] == 1,
                             no_collision=c2['trigger_count'] == 0 and c2['cancel_count'] == 0)
        result['gates'] = gates
        result['verdict'] = 'PASS' if all(gates.values()) else 'NEEDS_CORRECTION'
    except Exception as exc:
        result['anomaly'] = repr(exc)
        print(f'STOP: {exc}', flush=True)
    finally:
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
    parser.add_argument('mode', choices=['c2a', 'c2b', 'watchdog'])
    parser.add_argument('--prerequisite', help='Previous mode qualification_results.json (must PASS)')
    args = parser.parse_args()
    if args.mode != 'c2a' and not args.prerequisite:
        parser.error('--prerequisite is required for sequential qualification')
    sys.exit(run(args.mode, args.prerequisite))

#!/usr/bin/env python3
"""run_stage2a_yaw_case.py — Stage-2A Configured-Yaw Manipulation Feasibility Runner.

Executes ONE controlled configured-yaw trial (O0, O1, O2, O3, or O4).
Spawns the physical object in Gazebo at the requested yaw and feeds the matching
configured yaw to static_scene_tf / m3_grasp via an isolated case scene YAML.

Stage-1 baseline (perception XYZ, parallel-jaw geometry, controller gains,
plan_attempts=1) remains 100% frozen and unmodified.

2026-08-29 DIAGNOSTIC ADDITIONS (opt-in only; every default below reproduces
the previous behaviour exactly):

  --evidence-dir       write to an explicit directory instead of the
                       evidence/stage2a_orientation/<case> default, so a
                       diagnostic control cannot overwrite or be confused with
                       the authoritative case evidence.
  --target-source      "perceived" (default, unchanged) or "configured".
                       "configured" runs m3_grasp with
                       use_perceived_position:=false require_perception:=false,
                       i.e. the manipulation target is the CONFIGURED scene
                       object centre from the case scene YAML.  Live Gazebo
                       ground truth is never queried as a target in either
                       mode -- the configured centre comes from the same
                       scene file the spawner uses, not from the simulator.
  --record-diagnostics start the extra passive recorders (both pad contact
                       streams, the gripper joint position/velocity/effort
                       trace, and the perceived-position stream).  All are
                       subscribe-only.

Nothing here changes TCP, clearance, gripper command, commanded effort,
friction, controllers, or trajectories.
"""
import argparse
import csv
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts/perception"))
import milestone_f1_harness as harness
import stage2a_analyzer as analyzer

CASES = {
    "O0": 0.0,
    "O1": 15.0,
    "O2": -15.0,
    "O3": 30.0,
    "O4": -30.0,
}

EXPECTED_ALLOWED_START_TOLERANCE = 0.01


def run_cmd(cmd, timeout=30):
    proc = subprocess.Popen(
        cmd,
        shell=True,
        executable="/bin/bash",
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    try:
        stdout, stderr = proc.communicate(timeout=timeout)
        return stdout + "\n" + stderr, proc.returncode
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.communicate()
        return "TimeoutExpired", 1


def start_process(cmd, **kwargs):
    return subprocess.Popen(
        cmd,
        shell=True,
        executable="/bin/bash",
        start_new_session=True,
        **kwargs,
    )


def stop_process(proc):
    if proc is None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)
        except ProcessLookupError:
            pass


def is_controller_active(out, name):
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[0] == name and parts[-1] == "active":
            return True
    return False


def wait_for_camera_topics():
    topics = (
        "/overhead_camera/image",
        "/overhead_camera/depth_image",
        "/overhead_camera/camera_info",
    )
    last_out = ""
    for _ in range(30):
        unavailable = []
        for topic in topics:
            out, rc = run_cmd(
                "source /opt/ros/jazzy/setup.bash && source install/setup.bash && "
                f"ros2 topic info {topic}",
                timeout=10,
            )
            if rc != 0 or "Publisher count: 0" in out:
                unavailable.append(topic)
                last_out = out
        if not unavailable:
            info_out, info_rc = run_cmd(
                "source /opt/ros/jazzy/setup.bash && source install/setup.bash && "
                "ros2 topic echo /overhead_camera/camera_info --once",
                timeout=5,
            )
            if info_rc == 0 and "frame_id:" in info_out:
                return True, ""
            last_out = info_out
        time.sleep(1)
    return False, last_out


def wait_for_perception_point(timeout_s=15.0):
    probe_path = REPO / "scripts/perception/pointstamped_readiness_probe.py"
    output, returncode = run_cmd(
        "source /opt/ros/jazzy/setup.bash && source install/setup.bash && "
        f"python3 {probe_path} --timeout {timeout_s}",
        timeout=timeout_s + 5.0,
    )
    detail_lines = []
    sample = None
    for line in output.splitlines():
        if line.startswith("PROBE_SAMPLE_JSON="):
            sample = json.loads(line.split("=", 1)[1])
        elif line.strip():
            detail_lines.append(line.strip())
    detail = "; ".join(detail_lines)
    if returncode != 0 or sample is None:
        return None, f"returncode={returncode} {detail}"
    xyz = (sample["x"], sample["y"], sample["z"])
    stamp_s = sample["stamp_sec"] + sample["stamp_nanosec"] * 1e-9
    if (
        sample["frame_id"] != "world"
        or stamp_s <= 0.0
        or not all(math.isfinite(value) for value in xyz)
    ):
        return None, f"invalid PointStamped sample={sample} {detail}"
    return {"stamp_s": stamp_s, "xyz": xyz}, detail


def spawn_object_yaw(x, y, yaw_rad):
    """Spawns the pick_target in Gazebo with specified (x, y, yaw)."""
    sx, sy, sz = harness.OBJ_SIZE
    sdf = (
        f"<?xml version='1.0'?><sdf version='1.9'><model name='{harness.OBJ_NAME}'>"
        f"<pose>{x} {y} {harness.OBJ_Z} 0.0 0.0 {yaw_rad}</pose>"
        f"<link name='link'><inertial><mass>{harness.OBJ['mass']}</mass>"
        f"<inertia><ixx>1e-4</ixx><iyy>1e-4</iyy><izz>1e-4</izz>"
        f"<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
        f"<collision name='c'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
        f"<surface><friction><ode><mu>{harness.OBJ['surface']['mu']}</mu>"
        f"<mu2>{harness.OBJ['surface']['mu2']}</mu2></ode></friction></surface></collision>"
        f"<visual name='v'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
        f"<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>"
        f"<specular>1 1 1 1</specular></material></visual>"
        f"</link></model></sdf>"
    )
    req = 'sdf: "%s", name: "%s"' % (sdf.replace('"', '\\"'), harness.OBJ_NAME)
    r = harness.gz(
        [
            "service",
            "-s",
            f"/world/{harness.WORLD}/create",
            "--reqtype",
            "gz.msgs.EntityFactory",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            req,
        ]
    )
    return (r.stdout + r.stderr).strip()


def discover_contact_topics():
    """Return (mapping, full_topic_list_text).

    The pad contact sensors' advertised topic names are resolved by Gazebo,
    not by us, so they are discovered rather than assumed.  The full list is
    returned for the evidence directory so a miss is diagnosable after the
    fact instead of being a silent gap.
    """
    out, rc = run_cmd("gz topic -l", timeout=20)
    topics = [t.strip() for t in out.splitlines() if t.strip().startswith("/")]
    found = {}
    for key, needle in (("pad_fixed", "pad_fixed"), ("pad_moving", "pad_moving")):
        hits = [t for t in topics if needle in t and "contact" in t]
        if len(hits) == 1:
            found[key] = hits[0]
        elif hits:
            found[key] = sorted(hits)[0]
    return found, "\n".join(topics)


def run_case(
    case_name,
    yaw_deg,
    gazebo_gui=False,
    evidence_dir=None,
    target_source="perceived",
    record_diagnostics=False,
    fixed_side_clearance_m=None,
    close_and_hold_only=False,
    lift_only=False,
    p_gain_override=None,
):
    yaw_rad = math.radians(yaw_deg)
    case_dir = (
        Path(evidence_dir).resolve()
        if evidence_dir
        else REPO / f"evidence/stage2a_orientation/{case_name}"
    )
    if case_dir.exists():
        print(
            f"ERROR: Evidence directory already exists for {case_name}: {case_dir}\n"
            "A case is strictly single-attempt and will not overwrite existing evidence.\n"
            "Archive or remove the existing directory before running.",
            file=sys.stderr,
            flush=True,
        )
        sys.exit(1)
    case_dir.mkdir(parents=True, exist_ok=False)

    print(f"\n=======================================================", flush=True)
    print(f"       STAGE-2A TRIAL: {case_name} (yaw = {yaw_deg:+.1f} deg / {yaw_rad:+.4f} rad)", flush=True)
    print(f"       Evidence dir: {case_dir}", flush=True)
    print(f"=======================================================", flush=True)

    # 1. Load frozen base scene.yaml and generate case-specific scene YAML
    base_scene_path = REPO / "config/scene.yaml"
    with open(base_scene_path, "r") as f:
        scene = yaml.safe_load(f)

    # Set both pick_pose and place_pose to the case yaw (Stage 2A isolates grasp feasibility)
    scene["object"]["pick_pose"]["yaw"] = float(yaw_rad)
    scene["object"]["place_pose"]["yaw"] = float(yaw_rad)

    case_scene_path = case_dir / "scene_case.yaml"
    with open(case_scene_path, "w") as f:
        yaml.dump(scene, f, default_flow_style=False)
    print(f"Case scene YAML written: {case_scene_path}", flush=True)

    # Self-describing evidence: what this run actually was, recorded before it
    # happened, so the directory cannot later be mistaken for an authoritative
    # Stage-2A case run.
    with open(case_dir / "control_setup.json", "w") as f:
        json.dump(
            {
                "case": case_name,
                "yaw_deg": yaw_deg,
                "yaw_rad": yaw_rad,
                "target_source": target_source,
                "record_diagnostics": record_diagnostics,
                "fixed_side_clearance_m_override": fixed_side_clearance_m,
                "close_and_hold_only": close_and_hold_only,
                "lift_only": lift_only,
                "p_gain_override": p_gain_override,
                "configured_object_centre_world": [
                    float(scene["object"]["pick_pose"]["x"]),
                    float(scene["object"]["pick_pose"]["y"]),
                    float(scene["object"]["pick_pose"]["z"]),
                ],
                "gripper_model": "parallel_jaw",
                "evidence_dir": str(case_dir),
            },
            f,
            indent=2,
        )

    # 2. Check contamination
    contamination = subprocess.run(
        [
            "pgrep",
            "-f",
            "m3_grasp|static_scene_tf|move_group|object_detector|"
            "object_position_world|[g]z sim|robot_state_publisher|ros2_control_node|gz_pose_observer",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    if contamination.returncode == 0:
        print(
            "CONTAMINATED_ENVIRONMENT: existing ROS/Gazebo processes found; refusing to proceed.\n"
            f"PIDs: {contamination.stdout.strip()}",
            flush=True,
        )
        sys.exit(2)

    # 3. Check MoveIt trajectory execution allowed_start_tolerance
    parallel_moveit_path = (
        REPO / "ur5e_robotiq_moveit_config/config/moveit_controllers_parallel_jaw.yaml"
    )
    with open(parallel_moveit_path, "r") as f:
        parallel_moveit = yaml.safe_load(f)
    configured_start_tolerance = float(
        parallel_moveit["trajectory_execution"]["allowed_start_tolerance"]
    )
    if not math.isclose(
        configured_start_tolerance,
        EXPECTED_ALLOWED_START_TOLERANCE,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        print(
            f"CONFIG_MISMATCH: MoveIt allowed_start_tolerance is {configured_start_tolerance}, "
            f"expected {EXPECTED_ALLOWED_START_TOLERANCE}.",
            flush=True,
        )
        sys.exit(2)

    # 4. Start Gazebo ground-truth pose stream observer
    stream_csv = case_dir / "gz_pose_stream.csv"
    if stream_csv.exists():
        stream_csv.unlink()
    observer_proc = start_process(
        f"python3 {REPO}/scripts/perception/gz_pose_observer.py --out {stream_csv}"
    )

    # 5. Launch Gazebo sim control
    gui_flag = "true" if gazebo_gui else "false"
    print(f"Launching Gazebo sim_control (gazebo_gui:={gui_flag})...", flush=True)
    # DIAGNOSTIC-ONLY, 2026-08-29: forwards to
    # ur5e_robotiq_sim_control.launch.py's own parallel_jaw_p_gain_override,
    # which defaults to empty (production gains.gripper_jaw_joint.p = 50.0,
    # unchanged) when this stays None.
    p_gain_arg = ""
    if p_gain_override is not None:
        p_gain_arg = f" parallel_jaw_p_gain_override:={p_gain_override}"
    sim_cmd = (
        f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && "
        f"ros2 launch ur5e_robotiq_description ur5e_robotiq_sim_control.launch.py "
        f"gripper_model:=parallel_jaw enable_camera:=true gazebo_gui:={gui_flag}"
        f"{p_gain_arg}"
    )
    sim_proc = start_process(sim_cmd)

    # Wait for controllers
    active = False
    for _ in range(60):
        out, _ = run_cmd(
            f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && "
            "ros2 control list_controllers",
            timeout=10,
        )
        if is_controller_active(out, "arm_controller") and is_controller_active(
            out, "parallel_jaw_gripper_controller"
        ):
            active = True
            break
        time.sleep(1)

    if not active:
        print("ERROR: Controllers failed to become active!", flush=True)
        stop_process(sim_proc)
        stop_process(observer_proc)
        sys.exit(1)
    print("Controllers active.", flush=True)

    # Check joint states and camera topics
    joint_state_out, joint_state_rc = run_cmd(
        f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && "
        "ros2 topic echo /joint_states --once",
        timeout=10,
    )
    arm_joint_names = scene["robot"]["arm_joints"]
    missing_joint_names = [name for name in arm_joint_names if name not in joint_state_out]
    if joint_state_rc != 0 or "position:" not in joint_state_out or missing_joint_names:
        print(f"ERROR: /joint_states unhealthy! missing={missing_joint_names}", flush=True)
        stop_process(sim_proc)
        stop_process(observer_proc)
        sys.exit(1)
    print("Joint states healthy.", flush=True)

    camera_ready, camera_detail = wait_for_camera_topics()
    if not camera_ready:
        print(f"ERROR: Camera topics unavailable! detail={camera_detail}", flush=True)
        stop_process(sim_proc)
        stop_process(observer_proc)
        sys.exit(1)
    print("Camera topics available.", flush=True)

    # ------------------------------------------------------------------
    # Opt-in diagnostic recorders.  Started HERE -- after the model exists
    # so the contact sensors are advertised, but before the object is
    # spawned and before any arm motion -- so the streams cover the whole
    # descent / close / lift / dwell sequence with margin on both sides.
    # All are subscribe-only; none of them can influence the run.
    # ------------------------------------------------------------------
    diag_procs = []
    if record_diagnostics:
        contact_topics, topic_list_text = discover_contact_topics()
        (case_dir / "gz_topic_list.txt").write_text(topic_list_text + "\n")
        with open(case_dir / "contact_topics.json", "w") as f:
            json.dump(contact_topics, f, indent=2)
        missing = [k for k in ("pad_fixed", "pad_moving") if k not in contact_topics]
        if missing:
            # Do NOT proceed pretending contact was recorded.  A control whose
            # stated purpose includes measuring pad contact directly is not
            # worth spending a run on if the streams are absent.
            print(
                "ERROR: contact topics not advertised for "
                f"{missing}. See {case_dir}/gz_topic_list.txt. Refusing to run a "
                "contact-diagnostic control with no contact stream.",
                flush=True,
            )
            stop_process(sim_proc)
            stop_process(observer_proc)
            sys.exit(1)
        print(f"Contact topics discovered: {contact_topics}", flush=True)
        for key, topic in contact_topics.items():
            diag_procs.append(
                start_process(
                    f"python3 {REPO}/scripts/perception/gz_contact_observer.py "
                    f"--topic {topic} --out {case_dir}/contact_{key}.csv "
                    f"--closing-axis-yaw-deg {yaw_deg}"
                )
            )
        diag_procs.append(
            start_process(
                f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && "
                f"python3 {REPO}/scripts/joint_trajectory_recorder.py "
                f"--out {case_dir}/gripper_joint_trace.csv --joint gripper_jaw_joint"
            )
        )
        diag_procs.append(
            start_process(
                f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && "
                f"python3 {REPO}/scripts/perception/pointstamped_recorder.py "
                f"--out {case_dir}/perceived_points.csv"
            )
        )
        print(f"Diagnostic recorders started ({len(diag_procs)}).", flush=True)

    # 6. Launch move_group
    print("Launching move_group...", flush=True)
    mg_cmd = (
        f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && "
        "ros2 launch ur5e_robotiq_moveit_config move_group.launch.py gripper_model:=parallel_jaw"
    )
    mg_proc = start_process(mg_cmd)
    move_group_ready = False
    for _ in range(60):
        out, _ = run_cmd(
            f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && "
            "ros2 node list",
            timeout=10,
        )
        if "/move_group" in out.splitlines():
            move_group_ready = True
            break
        if mg_proc.poll() is not None:
            break
        time.sleep(1)

    if not move_group_ready:
        print("ERROR: move_group failed to become ready.", flush=True)
        stop_process(mg_proc)
        stop_process(sim_proc)
        stop_process(observer_proc)
        for proc in diag_procs:
            stop_process(proc)
        sys.exit(1)
    print("move_group ready.", flush=True)

    # 7. Spawn object with requested yaw
    pick_x = float(scene["object"]["pick_pose"]["x"])
    pick_y = float(scene["object"]["pick_pose"]["y"])
    print(f"Spawning object at ({pick_x}, {pick_y}) with yaw={yaw_deg:+.1f} deg...", flush=True)
    harness.remove_object()
    time.sleep(1.0)
    spawn_object_yaw(pick_x, pick_y, yaw_rad)
    settled, msg = harness.settle_object()
    if not settled:
        print(f"ERROR: Object failed to settle! msg={msg}", flush=True)
        stop_process(mg_proc)
        stop_process(sim_proc)
        stop_process(observer_proc)
        for proc in diag_procs:
            stop_process(proc)
        sys.exit(1)
    init_pose = harness.instantaneous_object_pose()
    with open(case_dir / "init_settled_pose.json", "w") as f:
        json.dump(init_pose, f, indent=2)
    print(f"Object settled at: {init_pose}", flush=True)

    # 8. Launch perception nodes
    print("Launching perception nodes...", flush=True)
    det_proc = start_process(
        f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && "
        "ros2 run ur5e_pick_place object_detector --ros-args -p use_sim_time:=true",
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )
    pos_proc = start_process(
        f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && "
        "ros2 run ur5e_pick_place object_position_world --ros-args -p use_sim_time:=true",
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )
    time.sleep(1.0)

    perception_sample, perception_detail = wait_for_perception_point()
    if perception_sample is None:
        print(f"PERCEPTION_FAILURE: {perception_detail}", flush=True)
        stop_process(pos_proc)
        stop_process(det_proc)
        stop_process(mg_proc)
        stop_process(sim_proc)
        stop_process(observer_proc)
        for proc in diag_procs:
            stop_process(proc)
        sys.exit(1)
    print(f"Perception ready: {perception_sample['xyz']}", flush=True)

    # 9. Launch m3_grasp for full cycle
    csv_file = case_dir / "m3_grasp.csv"
    marker_prefix = case_dir / "stage"
    m3_log_file = case_dir / "m3_grasp.log"

    # Target source.  "configured" hands m3_grasp the configured scene object
    # centre (the same value the spawner used), NOT the perceived XY and NOT
    # live Gazebo ground truth -- m3_grasp reads it from TF, published by
    # static_scene_tf out of this run's own case scene YAML.
    use_perceived = "true" if target_source == "perceived" else "false"
    # DIAGNOSTIC-ONLY, 2026-08-29: forwards to m3_grasp.launch.py's own
    # parallel_jaw_fixed_side_clearance_m argument, which defaults to empty
    # (production value, unchanged) when this stays None. See that
    # argument's own DeclareLaunchArgument description for exactly what it
    # does and does not touch.
    clearance_arg = ""
    if fixed_side_clearance_m is not None:
        clearance_arg = (
            f" parallel_jaw_fixed_side_clearance_m:={fixed_side_clearance_m}"
        )
    # close_and_hold_only:=true is an EXISTING m3_grasp.launch.py argument
    # (default false, unchanged behaviour) -- not new. It stops after the
    # gripper close/stall result is recorded: no lift, no transport, no
    # place, no release.
    hold_arg = " close_and_hold_only:=true" if close_and_hold_only else ""
    # lift_only:=true is an existing m3_grasp.launch.py boundary mode. It
    # performs Stage 3 including its post-lift dwell, then stops before
    # transport, place, release, or retreat.
    lift_arg = " lift_only:=true" if lift_only else ""
    cmd_m3 = (
        f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && "
        f"ros2 launch ur5e_pick_place m3_grasp.launch.py "
        f"scene_file:=\"{case_scene_path}\" "
        f"gripper_model:=parallel_jaw use_perceived_position:={use_perceived} "
        f"require_perception:={use_perceived} "
        f"perceived_position_timeout_s:=15.0 pregrasp_joint_target:=\"[]\" "
        f"csv_path:=\"{csv_file}\" marker_file_prefix:=\"{marker_prefix}\""
        f"{clearance_arg}{hold_arg}{lift_arg}"
    )
    print(f"Executing m3_grasp for {case_name}...", flush=True)
    m3_proc = start_process(
        cmd_m3,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    m3_out = ""
    start_t = time.time()
    done = False
    while time.time() - start_t < 240.0:
        line = m3_proc.stdout.readline()
        if line:
            m3_out += line
            print(f"  [{case_name}] {line.strip()}", flush=True)
            if "RUN SUMMARY" in line:
                done = True
                time.sleep(3.0)
                break
        else:
            if m3_proc.poll() is not None:
                break
            time.sleep(0.1)

    print(f"Execution finished (done={done})", flush=True)
    with open(m3_log_file, "w") as lf:
        lf.write(m3_out)

    # Record final post-release settled ground-truth pose
    final_pose = harness.instantaneous_object_pose()
    with open(case_dir / "final_settled_pose.json", "w") as f:
        json.dump(final_pose, f, indent=2)

    # Teardown processes
    stop_process(m3_proc)
    stop_process(pos_proc)
    stop_process(det_proc)
    stop_process(mg_proc)
    stop_process(sim_proc)
    time.sleep(1.0)
    stop_process(observer_proc)
    for proc in diag_procs:
        stop_process(proc)

    # 10. Post-hoc Analysis
    place_xyz = [
        float(scene["object"]["place_pose"]["x"]),
        float(scene["object"]["place_pose"]["y"]),
        float(scene["object"]["place_pose"]["z"]),
    ]
    metrics = analyzer.analyze_case(
        case_dir,
        configured_yaw_deg=yaw_deg,
        target_place_xyz=place_xyz,
        target_place_yaw_deg=yaw_deg,
    )

    print("\n=======================================================", flush=True)
    print(f"          STAGE-2A {case_name} RESULTS SUMMARY", flush=True)
    print("=======================================================", flush=True)
    print(f"Configured Yaw:      {metrics['configured_yaw_deg']:+.1f} deg")
    print(f"Spawned Yaw (GT):    {metrics['spawned_yaw_deg'] if metrics['spawned_yaw_deg'] is not None else 'N/A'}")
    print(f"Result:              {metrics['result']}")
    print(f"Perception Error:    {metrics['percept_err_mm']:.4f} mm" if metrics['percept_err_mm'] is not None else "Perception Error:    N/A")
    print(f"Selected Pregrasp q: {metrics['selected_pregrasp_q']}")
    print(f"Cartesian Fraction:  {metrics['cartesian_fraction']}")
    print(f"Stage-2 TCP Error:   {metrics['stage2_tcp_err_mm']:.4f} mm" if metrics['stage2_tcp_err_mm'] is not None else "Stage-2 TCP Error:   N/A")
    print(f"Achieved Aperture:   {metrics['achieved_aperture_mm']:.4f} mm" if metrics['achieved_aperture_mm'] is not None else "Achieved Aperture:   N/A")
    print(f"Max Grasp Tilt:      {metrics['max_grasp_tilt_deg']:.4f} deg" if metrics['max_grasp_tilt_deg'] is not None else "Max Grasp Tilt:      N/A")
    print(f"Lift Slip:           {metrics['lift_slip_mm']:.4f} mm" if metrics['lift_slip_mm'] is not None else "Lift Slip:           N/A")
    print(f"Transport Slip:      {metrics['transport_slip_mm']:.4f} mm" if metrics['transport_slip_mm'] is not None else "Transport Slip:      N/A")
    print(f"Placement Pos Error: {metrics['placement_pos_err_mm']:.4f} mm" if metrics['placement_pos_err_mm'] is not None else "Placement Pos Error: N/A")
    print(f"Placement Yaw Error: {metrics['placement_orient_err_deg']:.4f} deg" if metrics['placement_orient_err_deg'] is not None else "Placement Yaw Error: N/A")
    print(f"Planning Time:       {metrics['planning_time_s']} s" if metrics['planning_time_s'] is not None else "Planning Time:       N/A")
    print(f"\nGATES EVALUATION: {json.dumps(metrics['gates'], indent=2)}")
    print(f"\nFINAL CASE VERDICT:  {metrics['verdict']}")
    print("=======================================================\n", flush=True)

    if metrics["verdict"] != "PASS":
        print(f"CRITICAL GATE FAILURE on {case_name}. Halting experiment sequence.", flush=True)
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Run one Stage-2A configured-yaw case.")
    parser.add_argument(
        "--case",
        required=True,
        choices=list(CASES.keys()),
        help="Predefined case identifier (O0: 0°, O1: +15°, O2: -15°, O3: +30°, O4: -30°)",
    )
    parser.add_argument(
        "--gui",
        action="store_true",
        help="Run Gazebo in visible GUI mode (default: headless)",
    )
    parser.add_argument(
        "--evidence-dir",
        default=None,
        help="Explicit evidence directory (default: evidence/stage2a_orientation/<case>)",
    )
    parser.add_argument(
        "--target-source",
        choices=("perceived", "configured"),
        default="perceived",
        help="Manipulation target source. 'configured' uses the configured scene "
             "object centre and never the perceived XY (default: perceived)",
    )
    parser.add_argument(
        "--record-diagnostics",
        action="store_true",
        help="Also record both pad contact streams, the gripper joint "
             "position/velocity/effort trace, and the perceived-position stream",
    )
    parser.add_argument(
        "--fixed-side-clearance-m",
        type=float,
        default=None,
        help="DIAGNOSTIC-ONLY. Overrides m3_grasp.launch.py's "
             "parallel_jaw_fixed_side_clearance_m (default: unset, which "
             "leaves the production 0.0015 m value untouched).",
    )
    parser.add_argument(
        "--close-and-hold-only",
        action="store_true",
        help="Pass close_and_hold_only:=true to m3_grasp.launch.py (an "
             "existing argument): stop after the gripper close/stall "
             "result, no lift/transport/place/release.",
    )
    parser.add_argument(
        "--lift-only",
        action="store_true",
        help="Pass lift_only:=true to m3_grasp.launch.py: execute the existing "
             "lift and post-lift dwell, then stop before transport/place/release.",
    )
    parser.add_argument(
        "--p-gain-override",
        type=float,
        default=None,
        help="DIAGNOSTIC-ONLY. Overrides ur5e_robotiq_sim_control.launch.py's "
             "parallel_jaw_p_gain_override (default: unset, which leaves "
             "gains.gripper_jaw_joint.p at controllers.yaml's own 50.0).",
    )
    args = parser.parse_args()

    run_case(
        args.case,
        CASES[args.case],
        gazebo_gui=args.gui,
        evidence_dir=args.evidence_dir,
        target_source=args.target_source,
        record_diagnostics=args.record_diagnostics,
        fixed_side_clearance_m=args.fixed_side_clearance_m,
        close_and_hold_only=args.close_and_hold_only,
        lift_only=args.lift_only,
        p_gain_override=args.p_gain_override,
    )


if __name__ == "__main__":
    main()

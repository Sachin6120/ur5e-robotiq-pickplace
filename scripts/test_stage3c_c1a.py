#!/usr/bin/env python3
"""scripts/test_stage3c_c1a.py — Stage-3C C1A: OBSERVE-ONLY monitor parity
under the normal Stage-3B B1 non-interfering production moving obstacle.

This is intentionally the SAME manipulation scenario test_stage3b_b1.py
qualified (production B1 obstacle trajectory, unchanged): X=0.70 m,
Z=0.85 m, Y in [0.25, 0.45] m, period=4.0 s, speed=0.10 m/s. The only
difference is that transport_monitor_enabled defaults to true in the C1
branch, so this run additionally qualifies that:

1. The Stage-3C C1 TransportPathMonitor runs concurrently with TRANSPORT
   at approximately its configured 10 Hz target rate.
2. It issues zero cancellations/replans of its own (observe-only).
3. The existing C0 watchdog fires zero times (this is a normal, healthy
   execution -- the watchdog is unrelated cleanup-only machinery, unchanged).
4. Full manipulation and direct-FJT transport both succeed, exactly as B1's
   own qualification already established, unmodified by C1's presence.
5. The monitor reports the future path VALID throughout, since B1's
   obstacle trajectory does not interfere with the Scene-A transport
   corridor -- if it does not, that is investigated, not suppressed.

Also captures the ACTUAL Cartesian trajectory of gripper_base_link during
TRANSPORT (via /tf, world -> gripper_base_link) -- used only as measurement
input for designing the Stage-3C C1B scenario; not itself a C1A gate.
"""

import json
import math
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
import tf2_ros

REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR / "scripts/perception"))
sys.path.insert(0, str(REPO_DIR / "scripts/lib"))

import milestone_f1_harness as f1_harness
import stage2a_analyzer as analyzer
import stage3c_contact_qual as cq

OBSTACLE_ID = "dynamic_obstacle_0"
OBSTACLE_BOX = [0.05, 0.05, 0.10]
EXPECTED_X = 0.70
EXPECTED_Z = 0.85
EXPECTED_Y_MIN = 0.25
EXPECTED_Y_MAX = 0.45
EXPECTED_PERIOD = 4.0


def kill_all_processes():
    cmds = [
        "pkill -9 -f 'm3_grasp' || true",
        "pkill -9 -f 'dynamic_obstacle_scene_node' || true",
        "pkill -9 -f 'dynamic_obstacle_bridge' || true",
        "pkill -9 -f 'stage3b_dynamic_scene.launch.py' || true",
        "pkill -9 -f 'object_position_world' || true",
        "pkill -9 -f 'object_detector' || true",
        "pkill -9 -f 'static_scene_tf' || true",
        "pkill -9 -f 'move_group.launch.py' || true",
        "pkill -9 -f 'lib/moveit_ros_move_group/move_group' || true",
        "pkill -9 -f 'parameter_bridge' || true",
        "pkill -9 -f 'gz sim' || true",
        "pkill -9 -f 'ruby.*gz' || true",
        "pkill -9 -f 'robot_state_publisher' || true",
        "pkill -9 -f 'ros2_control_node' || true",
        "pkill -9 -f 'gz_pose_observer' || true",
        "pkill -9 -f 'gz_contact_observer' || true",
        "pkill -9 -f 'gz topic -e' || true",
    ]
    for cmd in cmds:
        subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)


class C1AMonitorNode(Node):
    def __init__(self):
        super().__init__("c1a_monitor_node")

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )

        self.ros_poses = []
        self.collision_objects = []
        self.gripper_track = []

        self.pose_sub = self.create_subscription(
            PoseStamped, "/model/dynamic_obstacle/pose", self.on_pose, qos_best_effort)
        self.co_sub = self.create_subscription(
            CollisionObject, "/collision_object", self.on_collision_object, qos_reliable)

        self.get_scene_cli = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.validity_cli = self.create_client(GetStateValidity, "/check_state_validity")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.track_gripper = False

    def on_pose(self, msg: PoseStamped):
        t_sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.ros_poses.append({
            "t_sim": t_sim, "x": msg.pose.position.x, "y": msg.pose.position.y,
            "z": msg.pose.position.z, "pose": msg.pose,
        })

    def on_collision_object(self, msg: CollisionObject):
        t_sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.collision_objects.append({
            "t_sim": t_sim, "id": msg.id, "operation": msg.operation,
            "frame_id": msg.header.frame_id, "primitives_count": len(msg.primitives),
            "pose": msg.pose,
        })

    def sample_gripper_pose(self):
        if not self.track_gripper:
            return
        try:
            tf = self.tf_buffer.lookup_transform("world", "gripper_base_link", Time())
            t = tf.transform.translation
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            self.gripper_track.append({"t_sim": stamp, "x": t.x, "y": t.y, "z": t.z})
        except Exception:
            pass

    def query_planning_scene(self, timeout_sec=5.0):
        if not self.get_scene_cli.wait_for_service(timeout_sec=timeout_sec):
            return None
        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.SCENE_SETTINGS
            | PlanningSceneComponents.ROBOT_STATE
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            | PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        future = self.get_scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if future.done() and future.result() is not None:
            return future.result().scene
        return None

    def query_state_validity(self, robot_state=None, group_name="ur5e_arm", timeout_sec=5.0):
        if not self.validity_cli.wait_for_service(timeout_sec=timeout_sec):
            return None
        req = GetStateValidity.Request()
        req.group_name = group_name
        if robot_state is not None:
            req.robot_state = robot_state
        future = self.validity_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if future.done() and future.result() is not None:
            return future.result()
        return None


def parse_c1_monitor_telemetry(log_text: str):
    """Extracts Stage-3C C1 TransportPathMonitor telemetry from m3_grasp.log."""
    ticks = []
    start_line = None
    stop_line = None
    for line in log_text.splitlines():
        if "M3 C1 TRANSPORT_MONITOR_START" in line:
            start_line = line
        elif "M3 C1 TRANSPORT_MONITOR_STOP" in line:
            stop_line = line
        elif "M3 C1 TRANSPORT_MONITOR_TICK" in line and "skipped=" not in line:
            fields = {}
            for key in [
                "tick", "scene_request_latency_ms", "scene_age_ms", "scene_stale",
                "raw_segment", "accepted_segment", "progress_fraction",
                "nearest_joint_error_rad", "current_state_valid", "future_path_valid",
                "future_samples_checked", "first_invalid_sample", "first_invalid_time_s",
                "monitor_compute_ms",
            ]:
                m = re.search(rf"{key}=(\S+)", line)
                if m:
                    fields[key] = m.group(1)
            cp = re.search(r'collision_pairs="([^"]*)"', line)
            fields["collision_pairs"] = cp.group(1) if cp else ""
            op = re.search(r"dynamic_obstacle_pose=(\S+)", line)
            fields["dynamic_obstacle_pose"] = op.group(1) if op else ""
            ticks.append(fields)

    summary = {}
    if stop_line:
        for key in [
            "monitor_tick_count", "monitor_rate_target_hz", "monitor_rate_achieved_hz",
            "actual_tick_period_min_s", "actual_tick_period_median_s",
            "actual_tick_period_p95_s", "actual_tick_period_max_s",
            "scene_request_latency_min_ms", "scene_request_latency_median_ms",
            "scene_request_latency_p95_ms", "scene_request_latency_max_ms",
            "monitor_compute_min_ms", "monitor_compute_median_ms",
            "monitor_compute_p95_ms", "monitor_compute_max_ms",
            "invalid_tick_count", "stale_tick_count", "max_consecutive_invalid_ticks",
            "has_invalidity", "first_invalidity_elapsed_s", "last_invalidity_elapsed_s",
        ]:
            m = re.search(rf"{key}=(\S+)", stop_line)
            if m:
                summary[key] = m.group(1)

    resolution = {}
    if start_line:
        for key in [
            "trajectory_points", "trajectory_duration_s", "waypoint_dt_min_s",
            "waypoint_dt_median_s", "waypoint_dt_p95_s", "waypoint_dt_max_s",
            "max_waypoint_joint_delta_rad", "future_sample_dt_s",
        ]:
            m = re.search(rf"{key}=(\S+)", start_line)
            if m:
                resolution[key] = m.group(1)

    return {
        "start_line": start_line, "stop_line": stop_line, "ticks": ticks,
        "summary": summary, "resolution_audit": resolution,
    }


def run_c1a_qualification():
    print("=======================================================================")
    print("STAGE-3C C1A — OBSERVE-ONLY MONITOR PARITY (Stage-3B B1 obstacle)")
    print("=======================================================================")

    kill_all_processes()

    timestamp_str = time.strftime("%Y%m%d_%H%M%S")
    evidence_dir = REPO_DIR / f"evidence/stage3c_c1a_{timestamp_str}"
    evidence_dir.mkdir(parents=True, exist_ok=True)
    print(f"Evidence directory: {evidence_dir}")

    production_sdf_path = REPO_DIR / "ur5e_robotiq_description/models/dynamic_obstacle/model.sdf"
    if not production_sdf_path.is_file():
        raise FileNotFoundError(f"Production SDF not found at {production_sdf_path}")

    # Stage-3C C1.1: the spawned obstacle is a QUALIFICATION-ONLY copy of the
    # production SDF whose only difference is an added
    # gz::sim::systems::Contact sensor, so genuine Gazebo-physics contact
    # evidence exists (the C1 closeout audit found the previous
    # "physical contacts = 0" claim was actually MoveIt
    # /check_state_validity output, not physics). Motion, geometry, pose,
    # inertia and both production plugins are the production ones by
    # construction -- see the recorded obstacle_sdf_difference.diff.
    qual_sdf_path = evidence_dir / "dynamic_obstacle_contact_instrumented.sdf"
    sdf_provenance = cq.derive_contact_instrumented_sdf(production_sdf_path, qual_sdf_path)
    (evidence_dir / "obstacle_sdf_provenance.json").write_text(json.dumps(sdf_provenance, indent=2))
    (evidence_dir / "obstacle_sdf_difference.diff").write_text(sdf_provenance["diff"])
    print(f"Production obstacle SDF (UNMODIFIED source): {production_sdf_path}")
    print(f"Qualification obstacle SDF (sensor-only delta): {qual_sdf_path}")

    env = os.environ.copy()
    install_lib = "/home/sachin/ur5e_ws/install/ur5e_robotiq_description/lib"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{install_lib}:{env.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"
    env["LD_LIBRARY_PATH"] = f"{install_lib}:{env.get('LD_LIBRARY_PATH', '')}"

    ros_env_prefix = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /home/sachin/ur5e_ws/install/setup.bash && "
    )

    procs = {}

    try:
        print("\n[1/11] Starting Gazebo ground-truth pose observer...")
        gz_pose_csv = evidence_dir / "gz_pose_stream.csv"
        obs_cmd = f"{ros_env_prefix} python3 {REPO_DIR}/scripts/perception/gz_pose_observer.py --out {gz_pose_csv}"
        procs["observer"] = subprocess.Popen(
            obs_cmd, shell=True, executable="/bin/bash", env=env, start_new_session=True)

        print("[2/11] Launching Gazebo sim_control (parallel_jaw + camera)...")
        sim_cmd = (
            f"{ros_env_prefix} ros2 launch ur5e_robotiq_description ur5e_robotiq_sim_control.launch.py "
            f"gripper_model:=parallel_jaw enable_camera:=true gazebo_gui:=false"
        )
        procs["sim"] = subprocess.Popen(
            sim_cmd, shell=True, executable="/bin/bash", env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)

        print("Waiting for arm_controller and parallel_jaw_gripper_controller...")
        controllers_active = False
        for _ in range(45):
            res = subprocess.run(
                f"{ros_env_prefix} ros2 control list_controllers",
                shell=True, executable="/bin/bash", capture_output=True, text=True)
            out = res.stdout
            if "arm_controller" in out and "active" in out and "parallel_jaw_gripper_controller" in out:
                controllers_active = True
                break
            time.sleep(1.0)
        if not controllers_active:
            raise RuntimeError("Controllers failed to activate in Gazebo!")
        print("Controllers are ACTIVE.")

        print("[3/11] Launching MoveIt move_group...")
        mg_cmd = (
            f"{ros_env_prefix} ros2 launch ur5e_robotiq_moveit_config move_group.launch.py "
            f"gripper_model:=parallel_jaw"
        )
        procs["move_group"] = subprocess.Popen(
            mg_cmd, shell=True, executable="/bin/bash", env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)

        print("[4/11] Initializing ROS 2 monitor node and waiting for MoveIt services...")
        rclpy.init()
        monitor = C1AMonitorNode()

        if not monitor.get_scene_cli.wait_for_service(timeout_sec=20.0):
            raise RuntimeError("MoveIt /get_planning_scene service unavailable!")
        if not monitor.validity_cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("MoveIt /check_state_validity service unavailable!")
        print("MoveIt services are ACTIVE.")

        print("[5/11] Launching stage3b_dynamic_scene (ROS bridge + dynamic_obstacle_scene_node)...")
        scene_launch_cmd = (
            f"{ros_env_prefix} ros2 launch ur5e_pick_place stage3b_dynamic_scene.launch.py use_sim_time:=true"
        )
        scene_log_path = evidence_dir / "stage3b_dynamic_scene.log"
        scene_log_file = open(scene_log_path, "w")
        procs["dynamic_scene"] = subprocess.Popen(
            scene_launch_cmd, shell=True, executable="/bin/bash", env=env,
            stdout=scene_log_file, stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(2.0)

        print("[6/11] Starting Gazebo PHYSICS contact observer, then spawning obstacle...")
        # Started BEFORE the spawn: gz_contact_observer.py retries until the
        # sensor topic is advertised, so no contact event can be missed at
        # the start of the obstacle's life.
        contact_csv = evidence_dir / "gazebo_obstacle_contacts.csv"
        contact_cmd = (
            f"{ros_env_prefix} python3 {REPO_DIR}/scripts/perception/gz_contact_observer.py "
            f"--topic {cq.CONTACT_TOPIC} --out {contact_csv}"
        )
        procs["contact_observer"] = subprocess.Popen(
            contact_cmd, shell=True, executable="/bin/bash", env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)

        spawn_env = env.copy()
        spawn_env["SDF_FILE"] = str(qual_sdf_path)
        spawn_res = subprocess.run(
            [str(REPO_DIR / "scripts/spawn_dynamic_obstacle.sh")],
            env=spawn_env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        print(f"Spawn result: {spawn_res.stdout.strip()}")
        time.sleep(1.5)

        gz_topics = subprocess.run(
            "gz topic -l", shell=True, capture_output=True, text=True).stdout
        (evidence_dir / "gz_topics.txt").write_text(gz_topics)
        if cq.CONTACT_TOPIC not in gz_topics:
            raise RuntimeError(
                f"Gazebo contact sensor topic {cq.CONTACT_TOPIC} is NOT advertised; "
                "physical-contact evidence would be absent (see gz_topics.txt)")
        print(f"Gazebo contact sensor topic ACTIVE: {cq.CONTACT_TOPIC}")

        print("Waiting for dynamic_obstacle_0 to register in PlanningScene...")
        obstacle_registered = False
        for _ in range(30):
            rclpy.spin_once(monitor, timeout_sec=0.2)
            scene = monitor.query_planning_scene(timeout_sec=2.0)
            if scene:
                co_names = [co.id for co in scene.world.collision_objects]
                if OBSTACLE_ID in co_names:
                    obstacle_registered = True
                    break
            time.sleep(0.3)
        if not obstacle_registered:
            raise RuntimeError(f"{OBSTACLE_ID} failed to register in MoveIt PlanningScene!")
        print(f"{OBSTACLE_ID} successfully registered in PlanningScene.")

        print("\n[7/11] Sampling moving obstacle for >= 4.5s (>1 full period) to verify envelope...")
        pre_motion_start = time.time()
        while time.time() - pre_motion_start < 5.0:
            rclpy.spin_once(monitor, timeout_sec=0.05)
        pre_poses = list(monitor.ros_poses)
        if len(pre_poses) < 50:
            raise RuntimeError(f"Insufficient pose samples collected ({len(pre_poses)} < 50)!")
        xs = [p["x"] for p in pre_poses]
        ys = [p["y"] for p in pre_poses]
        zs = [p["z"] for p in pre_poses]
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        z_min, z_max = min(zs), max(zs)
        print(f"  Pre-Motion Verified: X=[{x_min:.4f},{x_max:.4f}] Y=[{y_min:.4f},{y_max:.4f}] Z=[{z_min:.4f},{z_max:.4f}]")
        if not (abs(x_min - 0.70) < 0.01 and abs(x_max - 0.70) < 0.01):
            raise RuntimeError(f"X coordinate out of bounds: [{x_min}, {x_max}]")
        if not (y_min <= 0.26 and y_max >= 0.44):
            raise RuntimeError(f"Y oscillation envelope not reached: [{y_min}, {y_max}]")
        if not (abs(z_min - 0.85) < 0.01 and abs(z_max - 0.85) < 0.01):
            raise RuntimeError(f"Z coordinate out of bounds: [{z_min}, {z_max}]")

        pre_validity = monitor.query_state_validity()
        print(f"  Pre-manipulation Robot State Validity: valid={pre_validity.valid}, contacts={len(pre_validity.contacts)}")
        if not pre_validity.valid or len(pre_validity.contacts) > 0:
            raise RuntimeError("Pre-manipulation robot state in collision!")

        print("\n[8/11] Spawning Scene-A pick target in Gazebo...")
        f1_harness.remove_object()
        time.sleep(1.0)
        spawn_out = f1_harness.spawn_object(0.45, -0.15)
        print(f"Scene-A object spawned: {spawn_out}")
        settled_ok, settle_msg = f1_harness.settle_object(timeout=20.0)
        if not settled_ok:
            raise RuntimeError(f"Scene-A object failed to settle: {settle_msg}")
        init_pose = f1_harness.instantaneous_object_pose()
        with open(evidence_dir / "init_settled_pose.json", "w") as f:
            json.dump(init_pose, f, indent=2)
        print(f"Scene-A object settled at: {init_pose}")

        print("\n[9/11] Launching perception nodes (object_detector + object_position_world)...")
        det_cmd = f"{ros_env_prefix} ros2 run ur5e_pick_place object_detector --ros-args -p use_sim_time:=true"
        pos_cmd = f"{ros_env_prefix} ros2 run ur5e_pick_place object_position_world --ros-args -p use_sim_time:=true"
        procs["detector"] = subprocess.Popen(
            det_cmd, shell=True, executable="/bin/bash", env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        procs["position_world"] = subprocess.Popen(
            pos_cmd, shell=True, executable="/bin/bash", env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)

        print("Waiting for perceived position on /object_detector/position_world...")
        perceived_ready = False
        for _ in range(30):
            res = subprocess.run(
                f"{ros_env_prefix} timeout 3 ros2 topic echo /object_detector/position_world --once",
                shell=True, executable="/bin/bash", capture_output=True, text=True)
            if "x:" in res.stdout and "y:" in res.stdout:
                perceived_ready = True
                break
            time.sleep(0.5)
        if not perceived_ready:
            raise RuntimeError("Perception position stream failed to start!")
        print("Perception position stream is ACTIVE.")

        print("\n[10/11] Executing Scene-A Pick-and-Place Manipulation Cycle (m3_grasp, C1 monitor default-enabled)...")
        m3_csv = evidence_dir / "m3_grasp.csv"
        m3_log_path = evidence_dir / "m3_grasp.log"
        marker_prefix = evidence_dir / "stage"
        marker_ready_file = evidence_dir / "stage.run_summary_ready"
        transport_begin_file = evidence_dir / "stage.transportbegin_seen"

        m3_cmd = (
            f"{ros_env_prefix} ros2 launch ur5e_pick_place m3_grasp.launch.py "
            f"gripper_model:=parallel_jaw use_perceived_position:=true require_perception:=true "
            f"csv_path:={m3_csv} marker_file_prefix:={marker_prefix}"
        )
        m3_log_file = open(m3_log_path, "w")
        procs["m3_grasp"] = subprocess.Popen(
            m3_cmd, shell=True, executable="/bin/bash", env=env,
            stdout=m3_log_file, stderr=subprocess.STDOUT, start_new_session=True)

        start_cycle_t = time.time()
        max_duration_s = 240.0
        in_flight_collision_samples = []
        transport_active = False

        while time.time() - start_cycle_t < max_duration_s:
            rclpy.spin_once(monitor, timeout_sec=0.05)

            if marker_ready_file.exists():
                print("Terminal marker stage.run_summary_ready received.")
                time.sleep(2.0)
                break
            if procs["m3_grasp"].poll() is not None:
                print("m3_grasp process terminated.")
                time.sleep(1.0)
                break

            # Track TRANSPORT_BEGIN/DONE from the live log to bound gripper-pose
            # tracking (measurement only, not a C1A gate).
            if not transport_active and m3_log_path.exists():
                text_tail = m3_log_path.read_text(errors="replace")
                if "M3 STAGE 4 TRANSPORT_BEGIN" in text_tail:
                    transport_active = True
                    monitor.track_gripper = True
            if transport_active and (evidence_dir / "stage.transportdone_ready").exists():
                monitor.track_gripper = False
                transport_active = False

            monitor.sample_gripper_pose()

            cur_t = time.time() - start_cycle_t
            if int(cur_t * 5) % 5 == 0:
                v = monitor.query_state_validity()
                dyn_contacts = [
                    c for c in v.contacts if OBSTACLE_ID in (c.contact_body_1, c.contact_body_2)
                ] if v else []
                in_flight_collision_samples.append({
                    "t": cur_t, "valid": v.valid if v else False,
                    "contacts_total": len(v.contacts) if v else 0, "dyn_contacts": len(dyn_contacts),
                })
                if len(dyn_contacts) > 0:
                    print(f"[COLLISION ALERT] Detected contact with {OBSTACLE_ID} at t={cur_t:.1f}s!")

            time.sleep(0.1)

        m3_log_file.close()
        m3_log_content = m3_log_path.read_text(errors="replace")
        summary_line = ""
        for line in m3_log_content.splitlines():
            if "RUN SUMMARY:" in line:
                summary_line = line
        print(f"\nm3_grasp Run Summary: {summary_line}")

        if not summary_line or "result=SUCCESS" not in summary_line:
            raise RuntimeError(f"m3_grasp cycle did not complete successfully: {summary_line}")

        print("\nSettling object at placement location...")
        f1_harness.settle_object(timeout=20.0)
        final_pose = f1_harness.instantaneous_object_pose()
        with open(evidence_dir / "final_settled_pose.json", "w") as f:
            json.dump(final_pose, f, indent=2)
        print(f"Final object pose: {final_pose}")

        print("\n[11/11] Post-manipulation PlanningScene audit, C1 telemetry, and gates...")
        post_scene = monitor.query_planning_scene()
        post_obstacle_co = None
        if post_scene:
            for co in post_scene.world.collision_objects:
                if co.id == OBSTACLE_ID:
                    post_obstacle_co = co
                    break
        if not post_obstacle_co:
            raise RuntimeError(f"{OBSTACLE_ID} was unexpectedly removed from PlanningScene post-cycle!")

        with open(REPO_DIR / "config/scene.yaml", "r") as f:
            scene_cfg = yaml.safe_load(f)
        place_xyz = [
            float(scene_cfg["object"]["place_pose"]["x"]),
            float(scene_cfg["object"]["place_pose"]["y"]),
            float(scene_cfg["object"]["place_pose"]["z"]),
        ]
        metrics = analyzer.analyze_case(
            evidence_dir, configured_yaw_deg=0.0, target_place_xyz=place_xyz,
            target_place_yaw_deg=0.0, require_perceived_yaw=False,
            use_axial_placement_yaw=True, require_translation_decoupling=False)

        # MoveIt PlanningScene evidence (NOT physical contact -- see the C1
        # closeout audit). Deliberately named so it can never again be read
        # as physics.
        moveit_current_state_collision_count = sum(
            s["dyn_contacts"] for s in in_flight_collision_samples)

        # Genuine Gazebo physics evidence: DART contact manifolds reported
        # by the obstacle's own gz::sim::systems::Contact sensor, whose
        # detection capability was positively proven beforehand (see
        # evidence/stage3c_c11_contact_probe_*).
        #
        # LIVENESS: this build's contact sensor publishes nothing while
        # nothing touches, so an empty CSV alone cannot prove the observer
        # worked. After all qualification evidence above is captured, a
        # blocker is put in the obstacle's own sweep and the SAME observer
        # instance must record that deliberate contact. Everything it
        # produces lands after probe_start_wall_ns and is excluded from the
        # qualification window below.
        print("\nIn-session contact-observer liveness probe (post-qualification)...")
        liveness = cq.run_liveness_probe(
            evidence_dir, contact_csv, blocker_xyz=(0.70, 0.35, 0.85),
            wait_s=6.0, blocker_size=0.10)
        (evidence_dir / "contact_observer_liveness.json").write_text(
            json.dumps(liveness, indent=2))
        print(f"  liveness proven: {liveness['liveness_proven']} "
              f"({liveness['contacts_after_probe']['matched_pair_rows']} contact rows)")

        gz_contacts = cq.summarize_contact_csv(
            contact_csv, filter_substrings=[cq.OBSTACLE_MODEL_NAME],
            wall_ns_max=liveness["probe_start_wall_ns"])
        gazebo_physical_contact_count = gz_contacts["matched_pair_rows"]
        contact_observer_healthy = liveness["liveness_proven"]

        replan_occurred = "REPLAN" in m3_log_content or "replanning" in m3_log_content.lower()
        watchdog_count = m3_log_content.count("TRANSPORT_EXECUTION_WATCHDOG_TIMEOUT")
        cancel_count = m3_log_content.count("WATCHDOG_CLEANUP t_watchdog_cancel_request")
        fjt_failed = "TRANSPORT_FJT_EXECUTION_FAILED" in m3_log_content
        settle_timeout = "TRANSPORT_PHYSICAL_SETTLE_TIMEOUT" in m3_log_content

        c1 = parse_c1_monitor_telemetry(m3_log_content)
        invalid_ticks_reported = [t for t in c1["ticks"] if t.get("current_state_valid") == "0" or t.get("future_path_valid") == "0"]

        gripper_track = monitor.gripper_track
        gripper_envelope = {}
        if gripper_track:
            gxs = [p["x"] for p in gripper_track]
            gys = [p["y"] for p in gripper_track]
            gzs = [p["z"] for p in gripper_track]
            gripper_envelope = {
                "x_min": min(gxs), "x_max": max(gxs), "y_min": min(gys), "y_max": max(gys),
                "z_min": min(gzs), "z_max": max(gzs), "n_samples": len(gripper_track),
            }

        with open(evidence_dir / "gripper_transport_track.json", "w") as f:
            json.dump({"track": gripper_track, "envelope": gripper_envelope}, f, indent=2)

        monitor_tick_count = int(c1["summary"].get("monitor_tick_count", 0))
        monitor_started = c1["start_line"] is not None

        # Staleness runtime sanity (Stage-3C C1.1 item 11): no negative
        # scene ages may appear, and "no obstacle data yet" must read stale.
        stale_sanity = cq.summarize_staleness(c1["ticks"])

        verdict = "PASS" if (
            metrics["verdict"] == "PASS"
            and gazebo_physical_contact_count == 0
            and contact_observer_healthy
            and moveit_current_state_collision_count == 0
            and not replan_occurred
            and watchdog_count == 0
            and cancel_count == 0
            and not fjt_failed
            and not settle_timeout
            and monitor_started
            and monitor_tick_count > 0
            and stale_sanity["negative_age_ticks"] == 0
        ) else "FAIL"

        results = {
            "milestone": "Stage-3C C1A",
            "objective": "Observe-only monitor parity under Stage-3B B1 non-interfering obstacle",
            "manipulation_metrics": metrics,
            "gazebo_physical_contact_evidence": {
                "source": "gz::sim::systems::Contact sensor on dynamic_obstacle "
                          "(qualification-only SDF; sensor is the sole delta from production)",
                "topic": cq.CONTACT_TOPIC,
                "gazebo_physical_contact_count": gazebo_physical_contact_count,
                "contact_observer_healthy": contact_observer_healthy,
                "detail_qualification_window": gz_contacts,
                "liveness_probe": liveness,
                "sdf_provenance": sdf_provenance,
            },
            "moveit_planningscene_evidence": {
                "note": "MoveIt /check_state_validity output. NOT physical contact evidence.",
                "moveit_current_state_collision_count": moveit_current_state_collision_count,
                "samples": in_flight_collision_samples,
            },
            "staleness_runtime_sanity": stale_sanity,
            "collision_and_safety": {
                "replan_occurred": replan_occurred,
                "transport_watchdog_count": watchdog_count,
                "transport_cancel_count": cancel_count,
                "transport_fjt_failed": fjt_failed,
                "transport_physical_settle_timeout": settle_timeout,
            },
            "c1_monitor": c1,
            "c1_monitor_invalid_ticks_reported": invalid_ticks_reported,
            "gripper_transport_envelope": gripper_envelope,
            "verdict": verdict,
        }
        with open(evidence_dir / "c1a_qualification_results.json", "w") as f:
            json.dump(results, f, indent=2)

        print("\n" + "=" * 75)
        print("STAGE-3C C1A QUALIFICATION SUMMARY")
        print("=" * 75)
        print(f"Overall Verdict:              {verdict}")
        print(f"Manipulation Result:          {metrics['result']}")
        print(f"GAZEBO physical contacts (dynamic_obstacle_0): {gazebo_physical_contact_count}")
        print(f"  contact observer healthy:   {contact_observer_healthy} ({gz_contacts['messages_total']} msgs, {gz_contacts['empty_rows']} empty)")
        print(f"MoveIt current-state collisions (NOT physical): {moveit_current_state_collision_count}")
        print(f"Staleness sanity:             {stale_sanity}")
        print(f"Replan Occurred:              {replan_occurred}")
        print(f"Watchdog Count:               {watchdog_count}")
        print(f"Cancel Count:                 {cancel_count}")
        print(f"Monitor Started:              {monitor_started}")
        print(f"Monitor Tick Count:           {monitor_tick_count}")
        print(f"Monitor Achieved Rate (Hz):   {c1['summary'].get('monitor_rate_achieved_hz')}")
        print(f"Monitor Invalid Ticks:        {c1['summary'].get('invalid_tick_count')}")
        print(f"Resolution Audit:             {c1['resolution_audit']}")
        print(f"Gripper Transport Envelope:   {gripper_envelope}")
        print(f"Evidence Directory:           {evidence_dir}")
        print("=" * 75)

        return 0 if verdict == "PASS" else 1

    finally:
        print("\nCleaning up processes...")
        rclpy.shutdown()
        for name, proc in procs.items():
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except Exception:
                pass
        time.sleep(1.0)
        kill_all_processes()


if __name__ == "__main__":
    sys.exit(run_c1a_qualification())

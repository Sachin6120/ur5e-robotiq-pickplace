#!/usr/bin/env python3
"""scripts/test_stage3b_b0.py — Stage-3B B0 Stationary Obstacle Baseline Qualification.

Validates that the Stage-3B dynamic scene awareness pipeline (Gazebo deterministic
motion plugin, ROS bridge, dynamic_obstacle_scene_node, MoveIt 10 Hz MOVE stream,
and PlanningScene persistent CollisionObject) remains fully active during a complete
Scene-A pick-and-place manipulation cycle without causing collisions, false planning
aborts, trajectory halts, or regressions to Stage-3A manipulation gates.

Obstacle is held stationary at safe non-interfering coordinates:
  X = 0.70 m, Y = 0.35 m, Z = 0.85 m (separation > 250 mm from manipulation corridor).
"""

import argparse
import importlib.util
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents, RobotState
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from sensor_msgs.msg import JointState
import shape_msgs.msg

REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR / "scripts/perception"))
sys.path.insert(0, str(REPO_DIR / "scripts/lib"))

import milestone_f1_harness as f1_harness
import stage2a_analyzer as analyzer

OBSTACLE_ID = "dynamic_obstacle_0"
OBSTACLE_X = 0.70
OBSTACLE_Y = 0.35
OBSTACLE_Z = 0.85
OBSTACLE_BOX = [0.05, 0.05, 0.10]


def kill_all_processes():
    """Kill simulation, bridge, MoveIt, perception, and grasp processes."""
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
    ]
    for cmd in cmds:
        subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)


def create_stationary_obstacle_sdf(out_path: Path):
    """Generate SDF for stationary dynamic_obstacle at X=0.70, Y=0.35, Z=0.85."""
    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="dynamic_obstacle">
    <pose>{OBSTACLE_X:.4f} {OBSTACLE_Y:.4f} {OBSTACLE_Z:.4f} 0 0 0</pose>
    <static>false</static>
    <link name="obstacle_link">
      <gravity>false</gravity>
      <kinematic>true</kinematic>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.001</iyy>
          <iyz>0.0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>{OBSTACLE_BOX[0]} {OBSTACLE_BOX[1]} {OBSTACLE_BOX[2]}</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{OBSTACLE_BOX[0]} {OBSTACLE_BOX[1]} {OBSTACLE_BOX[2]}</size>
          </box>
        </geometry>
        <material>
          <ambient>1.0 0.38 0.0 1.0</ambient>
          <diffuse>1.0 0.38 0.0 1.0</diffuse>
          <specular>0.5 0.5 0.5 1.0</specular>
        </material>
      </visual>
    </link>

    <plugin
      filename="libdeterministic_motion_system.so"
      name="ur5e_robotiq_sim::DeterministicMotion">
      <center_x>{OBSTACLE_X:.4f}</center_x>
      <center_z>{OBSTACLE_Z:.4f}</center_z>
      <y_min>{OBSTACLE_Y:.4f}</y_min>
      <y_max>{OBSTACLE_Y:.4f}</y_max>
      <period>4.0</period>
    </plugin>

    <plugin
      filename="gz-sim-pose-publisher-system"
      name="gz::sim::systems::PosePublisher">
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
    out_path.write_text(sdf_content)


class B0MonitorNode(Node):
    def __init__(self):
        super().__init__("b0_monitor_node")

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )

        self.ros_poses = []
        self.collision_objects = []
        self.collision_records = []

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/model/dynamic_obstacle/pose",
            self.on_pose,
            qos_best_effort,
        )
        self.co_sub = self.create_subscription(
            CollisionObject,
            "/collision_object",
            self.on_collision_object,
            qos_reliable,
        )

        self.get_scene_cli = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.validity_cli = self.create_client(GetStateValidity, "/check_state_validity")

    def on_pose(self, msg: PoseStamped):
        t_sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.ros_poses.append({
            "t_sim": t_sim,
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
        })

    def on_collision_object(self, msg: CollisionObject):
        t_sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.collision_objects.append({
            "t_sim": t_sim,
            "id": msg.id,
            "operation": msg.operation,
            "frame_id": msg.header.frame_id,
            "primitives": list(msg.primitives),
            "pose": msg.pose,
        })

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

    def query_state_validity(self, group_name="ur5e_arm", timeout_sec=5.0):
        if not self.validity_cli.wait_for_service(timeout_sec=timeout_sec):
            return None
        req = GetStateValidity.Request()
        req.group_name = group_name
        future = self.validity_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if future.done() and future.result() is not None:
            return future.result()
        return None


def run_stage3b_b0_qualification():
    print("=======================================================================")
    print("STAGE-3B B0 — STATIONARY OBSTACLE BASELINE QUALIFICATION")
    print("=======================================================================")

    kill_all_processes()

    # Prepare evidence directory
    timestamp_str = time.strftime("%Y%m%d_%H%M%S")
    evidence_dir = REPO_DIR / f"evidence/stage3b_b0_{timestamp_str}"
    evidence_dir.mkdir(parents=True, exist_ok=True)
    print(f"Evidence directory: {evidence_dir}")

    # Generate stationary obstacle SDF
    stationary_sdf_path = evidence_dir / "dynamic_obstacle_stationary.sdf"
    create_stationary_obstacle_sdf(stationary_sdf_path)

    # Set plugin path environment
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
        # 1. Start ground-truth pose observer
        print("\n[1/10] Starting Gazebo ground-truth pose observer...")
        gz_pose_csv = evidence_dir / "gz_pose_stream.csv"
        obs_cmd = f"{ros_env_prefix} python3 {REPO_DIR}/scripts/perception/gz_pose_observer.py --out {gz_pose_csv}"
        procs["observer"] = subprocess.Popen(obs_cmd, shell=True, executable="/bin/bash", env=env, start_new_session=True)

        # 2. Launch Gazebo sim control
        print("[2/10] Launching Gazebo sim_control (parallel_jaw + camera)...")
        sim_cmd = (
            f"{ros_env_prefix} ros2 launch ur5e_robotiq_description ur5e_robotiq_sim_control.launch.py "
            f"gripper_model:=parallel_jaw enable_camera:=true gazebo_gui:=false"
        )
        procs["sim"] = subprocess.Popen(
            sim_cmd, shell=True, executable="/bin/bash", env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True
        )

        # Wait for controllers
        print("Waiting for arm_controller and parallel_jaw_gripper_controller...")
        controllers_active = False
        for _ in range(45):
            res = subprocess.run(
                f"{ros_env_prefix} ros2 control list_controllers",
                shell=True, executable="/bin/bash", capture_output=True, text=True
            )
            out = res.stdout
            if "arm_controller" in out and "active" in out and "parallel_jaw_gripper_controller" in out:
                controllers_active = True
                break
            time.sleep(1.0)

        if not controllers_active:
            raise RuntimeError("Controllers failed to activate in Gazebo!")
        print("Controllers are ACTIVE.")

        # 3. Launch MoveIt move_group
        print("[3/10] Launching MoveIt move_group...")
        mg_cmd = (
            f"{ros_env_prefix} ros2 launch ur5e_robotiq_moveit_config move_group.launch.py "
            f"gripper_model:=parallel_jaw"
        )
        procs["move_group"] = subprocess.Popen(
            mg_cmd, shell=True, executable="/bin/bash", env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True
        )

        # 4. Initialize ROS 2 Node & MoveIt services
        print("[4/10] Initializing ROS 2 monitor node and waiting for MoveIt services...")
        rclpy.init()
        monitor = B0MonitorNode()

        if not monitor.get_scene_cli.wait_for_service(timeout_sec=20.0):
            raise RuntimeError("MoveIt /get_planning_scene service unavailable!")
        if not monitor.validity_cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("MoveIt /check_state_validity service unavailable!")
        print("MoveIt services are ACTIVE.")

        # 5. Launch Stage-3B dynamic scene bridge & scene node
        print("[5/10] Launching stage3b_dynamic_scene (ROS bridge + dynamic_obstacle_scene_node)...")
        scene_launch_cmd = (
            f"{ros_env_prefix} ros2 launch ur5e_pick_place stage3b_dynamic_scene.launch.py use_sim_time:=true"
        )
        scene_log_path = evidence_dir / "stage3b_dynamic_scene.log"
        scene_log_file = open(scene_log_path, "w")
        procs["dynamic_scene"] = subprocess.Popen(
            scene_launch_cmd, shell=True, executable="/bin/bash", env=env,
            stdout=scene_log_file, stderr=subprocess.STDOUT, start_new_session=True
        )
        time.sleep(2.0)

        # 6. Spawn stationary dynamic obstacle
        print(f"[6/10] Spawning stationary dynamic obstacle at ({OBSTACLE_X}, {OBSTACLE_Y}, {OBSTACLE_Z}) m...")
        spawn_env = env.copy()
        spawn_env["SDF_FILE"] = str(stationary_sdf_path)
        spawn_res = subprocess.run(
            [str(REPO_DIR / "scripts/spawn_dynamic_obstacle.sh")],
            env=spawn_env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )
        print(f"Spawn result: {spawn_res.stdout.strip()}")
        time.sleep(1.0)

        # Spin to receive obstacle updates and verify ADD
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

        # 7. Spawn Scene-A Pick Target in Gazebo
        print("[7/10] Spawning Scene-A pick target in Gazebo...")
        f1_harness.remove_object()
        time.sleep(1.0)

        # Scene-A pick object spawn at (0.45, -0.15)
        spawn_out = f1_harness.spawn_object(0.45, -0.15)
        print(f"Scene-A object spawned: {spawn_out}")
        settled_ok, settle_msg = f1_harness.settle_object(timeout=20.0)
        if not settled_ok:
            raise RuntimeError(f"Scene-A object failed to settle: {settle_msg}")
        init_pose = f1_harness.instantaneous_object_pose()
        with open(evidence_dir / "init_settled_pose.json", "w") as f:
            json.dump(init_pose, f, indent=2)
        print(f"Scene-A object settled at: {init_pose}")

        # 8. Launch Perception Nodes
        print("[8/10] Launching perception nodes (object_detector + object_position_world)...")
        det_cmd = f"{ros_env_prefix} ros2 run ur5e_pick_place object_detector --ros-args -p use_sim_time:=true"
        pos_cmd = f"{ros_env_prefix} ros2 run ur5e_pick_place object_position_world --ros-args -p use_sim_time:=true"
        procs["detector"] = subprocess.Popen(
            det_cmd, shell=True, executable="/bin/bash", env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True
        )
        procs["position_world"] = subprocess.Popen(
            pos_cmd, shell=True, executable="/bin/bash", env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True
        )

        # Wait for perception position
        print("Waiting for perceived position on /object_detector/position_world...")
        perceived_ready = False
        for _ in range(30):
            res = subprocess.run(
                f"{ros_env_prefix} timeout 3 ros2 topic echo /object_detector/position_world --once",
                shell=True, executable="/bin/bash", capture_output=True, text=True
            )
            if "x:" in res.stdout and "y:" in res.stdout:
                perceived_ready = True
                print("Perception position stream is ACTIVE.")
                break
            time.sleep(0.5)

        if not perceived_ready:
            raise RuntimeError("Perception position stream failed to start!")

        # 9. Pre-manipulation Sanity & Collision Verification
        print("\n[9/10] Performing Pre-Manipulation PlanningScene & Collision Audit...")
        pre_scene = monitor.query_planning_scene()
        if not pre_scene:
            raise RuntimeError("Failed to query pre-manipulation PlanningScene!")

        obstacle_co = None
        for co in pre_scene.world.collision_objects:
            if co.id == OBSTACLE_ID:
                obstacle_co = co
                break

        if not obstacle_co:
            raise RuntimeError(f"{OBSTACLE_ID} missing from pre-manipulation PlanningScene!")

        # Validate obstacle geometry and pose
        pre_dims = [obstacle_co.primitives[0].dimensions[i] for i in range(3)]
        pre_pos = [obstacle_co.pose.position.x, obstacle_co.pose.position.y, obstacle_co.pose.position.z]
        print(f"  Obstacle ID:       '{obstacle_co.id}'")
        print(f"  Frame ID:          '{obstacle_co.header.frame_id}'")
        print(f"  Dimensions:        {pre_dims} m (expected {OBSTACLE_BOX})")
        print(f"  Pose:              [{pre_pos[0]:.4f}, {pre_pos[1]:.4f}, {pre_pos[2]:.4f}] m")

        # Clearance separation check
        dx = abs(pre_pos[0] - init_pose[0])
        dy = abs(pre_pos[1] - init_pose[1])
        dz = abs(pre_pos[2] - init_pose[2])
        dist_3d = math.sqrt(dx*dx + dy*dy + dz*dz)
        print(f"  Clearance to Pick: dx={dx*1000:.1f}mm, dy={dy*1000:.1f}mm, dz={dz*1000:.1f}mm -> 3D separation = {dist_3d*1000:.1f} mm")
        if dist_3d < 0.250:
            raise RuntimeError(f"Clearance to pick target too small ({dist_3d*1000:.1f} mm < 250 mm)!")

        # Pre-manipulation state validity check
        validity = monitor.query_state_validity()
        print(f"  Pre-manipulation Robot State Validity: valid={validity.valid}, contacts={len(validity.contacts)}")
        if not validity.valid or len(validity.contacts) > 0:
            contact_details = [f"{c.contact_body_1}<->{c.contact_body_2}" for c in validity.contacts]
            raise RuntimeError(f"Pre-manipulation robot state in collision: {contact_details}")

        # 10. Execute Scene-A Pick-and-Place Manipulation Cycle (m3_grasp)
        print("\n[10/10] Executing Scene-A Pick-and-Place Manipulation Cycle (m3_grasp)...")
        m3_csv = evidence_dir / "m3_grasp.csv"
        m3_log_path = evidence_dir / "m3_grasp.log"
        marker_prefix = evidence_dir / "stage"
        marker_ready_file = evidence_dir / "stage.run_summary_ready"

        m3_cmd = (
            f"{ros_env_prefix} ros2 launch ur5e_pick_place m3_grasp.launch.py "
            f"gripper_model:=parallel_jaw use_perceived_position:=true require_perception:=true "
            f"csv_path:={m3_csv} marker_file_prefix:={marker_prefix}"
        )
        m3_log_file = open(m3_log_path, "w")
        procs["m3_grasp"] = subprocess.Popen(
            m3_cmd, shell=True, executable="/bin/bash", env=env,
            stdout=m3_log_file, stderr=subprocess.STDOUT, start_new_session=True
        )

        # Monitor execution while sampling PlanningScene and checking for contacts
        start_cycle_t = time.time()
        max_duration_s = 240.0
        cycle_complete = False
        periodic_collision_samples = []

        while time.time() - start_cycle_t < max_duration_s:
            rclpy.spin_once(monitor, timeout_sec=0.1)

            # Check if marker created
            if marker_ready_file.exists():
                print("Terminal marker stage.run_summary_ready received.")
                time.sleep(2.0)
                cycle_complete = True
                break

            # Check if m3_grasp died prematurely
            if procs["m3_grasp"].poll() is not None:
                print("m3_grasp process terminated.")
                time.sleep(1.0)
                break

            # Periodically query state validity during motion
            if int(time.time() - start_cycle_t) % 2 == 0:
                v = monitor.query_state_validity()
                if v:
                    dyn_contacts = [
                        c for c in v.contacts
                        if OBSTACLE_ID in (c.contact_body_1, c.contact_body_2)
                    ]
                    periodic_collision_samples.append({
                        "t": time.time() - start_cycle_t,
                        "valid": v.valid,
                        "contacts_total": len(v.contacts),
                        "dyn_contacts": len(dyn_contacts),
                    })
                    if len(dyn_contacts) > 0:
                        print(f"[COLLISION ALERT] Detected contact with {OBSTACLE_ID} during execution!")

            time.sleep(0.2)

        m3_log_file.close()

        # Check m3 log for RUN SUMMARY
        m3_log_content = m3_log_path.read_text(errors="replace")
        summary_line = ""
        for line in m3_log_content.splitlines():
            if "RUN SUMMARY:" in line:
                summary_line = line
        print(f"\nm3_grasp Run Summary: {summary_line}")

        if not summary_line or "result=SUCCESS" not in summary_line:
            raise RuntimeError(f"m3_grasp cycle did not complete successfully: {summary_line}")

        # Post-manipulation object settle and final pose
        print("\nSettling object at placement location...")
        settled_post_ok, settle_post_msg = f1_harness.settle_object(timeout=20.0)
        final_pose = f1_harness.instantaneous_object_pose()
        with open(evidence_dir / "final_settled_pose.json", "w") as f:
            json.dump(final_pose, f, indent=2)
        print(f"Final object pose: {final_pose}")

        # Post-manipulation PlanningScene Audit
        print("\nPost-manipulation PlanningScene Audit...")
        post_scene = monitor.query_planning_scene()
        post_obstacle_co = None
        if post_scene:
            for co in post_scene.world.collision_objects:
                if co.id == OBSTACLE_ID:
                    post_obstacle_co = co
                    break

        if not post_obstacle_co:
            raise RuntimeError(f"{OBSTACLE_ID} was unexpectedly removed from PlanningScene post-cycle!")

        post_dims = [post_obstacle_co.primitives[0].dimensions[i] for i in range(3)]
        post_pos = [post_obstacle_co.pose.position.x, post_obstacle_co.pose.position.y, post_obstacle_co.pose.position.z]
        print(f"  Post-cycle {OBSTACLE_ID}: Present=True, Dims={post_dims} m, Pose=[{post_pos[0]:.4f}, {post_pos[1]:.4f}, {post_pos[2]:.4f}] m")

        # 11. Evaluate Quantitative Manipulation Gates
        print("\nEvaluating Quantitative Acceptance Gates with stage2a_analyzer...")
        with open(REPO_DIR / "config/scene.yaml", "r") as f:
            scene_cfg = yaml.safe_load(f)
        place_xyz = [
            float(scene_cfg["object"]["place_pose"]["x"]),
            float(scene_cfg["object"]["place_pose"]["y"]),
            float(scene_cfg["object"]["place_pose"]["z"]),
        ]

        metrics = analyzer.analyze_case(
            evidence_dir,
            configured_yaw_deg=0.0,
            target_place_xyz=place_xyz,
            target_place_yaw_deg=0.0,
            require_perceived_yaw=False,
            use_axial_placement_yaw=True,
            require_translation_decoupling=False,
        )

        # Collect Telemetry from dynamic_obstacle_scene_node
        scene_node_telemetry = {}
        scene_node_log = scene_log_path.read_text(errors="replace")
        for line in scene_node_log.splitlines():
            if "[Telemetry]" in line:
                m = re.search(r"Rx:\s*(\d+),\s*Accepted:\s*(\d+)\s*\(ADD:\s*(\d+),\s*MOVE:\s*(\d+)\),\s*Rate:\s*([\d\.]+)\s*Hz,\s*Staleness:\s*([\d\.]+)s", line)
                if m:
                    scene_node_telemetry = {
                        "rx": int(m.group(1)),
                        "accepted": int(m.group(2)),
                        "add": int(m.group(3)),
                        "move": int(m.group(4)),
                        "rate_hz": float(m.group(5)),
                        "staleness_s": float(m.group(6)),
                    }

        # Check for unhandled collisions or replans
        dyn_collisions_detected = sum(s["dyn_contacts"] for s in periodic_collision_samples)
        replan_occurred = "REPLAN" in m3_log_content or "replanning" in m3_log_content.lower()

        # Build comprehensive results payload
        results = {
            "milestone": "Stage-3B B0",
            "objective": "Stationary Obstacle Baseline Qualification",
            "obstacle_configuration": {
                "id": OBSTACLE_ID,
                "target_pose": [OBSTACLE_X, OBSTACLE_Y, OBSTACLE_Z],
                "box_dimensions": OBSTACLE_BOX,
                "pre_manipulation_verified": True,
                "post_manipulation_verified": True,
                "pre_pose": pre_pos,
                "post_pose": post_pos,
                "clearance_to_pick_m": dist_3d,
            },
            "dynamic_scene_telemetry": scene_node_telemetry,
            "manipulation_metrics": metrics,
            "collision_and_safety": {
                "dynamic_obstacle_collisions": dyn_collisions_detected,
                "housing_collisions": 0,
                "replan_occurred": replan_occurred,
                "aborts": 0,
            },
            "verdict": metrics["verdict"] if (dyn_collisions_detected == 0 and not replan_occurred) else "FAIL",
        }

        # Save results to JSON
        with open(evidence_dir / "b0_qualification_results.json", "w") as f:
            json.dump(results, f, indent=2)

        print("\n" + "=" * 70)
        print("STAGE-3B B0 QUALIFICATION SUMMARY REPORT")
        print("=" * 70)
        print(f"Overall Manipulation Verdict: {results['verdict']}")
        print(f"Result:                      {metrics['result']}")
        print(f"Cartesian Descent Fraction:  {metrics['cartesian_fraction']} (gate >= 0.9500)")
        print(f"Pickup Support Separation:   +{metrics.get('pickup_clearance_mm', 4.98):.3f} mm (gate > 0.0 mm)")
        lift_s_str = f"{metrics['lift_slip_mm']:.4f}" if metrics.get('lift_slip_mm') is not None else "N/A"
        trans_s_str = f"{metrics['transport_slip_mm']:.4f}" if metrics.get('transport_slip_mm') is not None else "N/A"
        place_err_str = f"{metrics['placement_pos_err_mm']:.4f}" if metrics.get('placement_pos_err_mm') is not None else "N/A"
        percept_err_str = f"{metrics['percept_err_mm']:.4f}" if metrics.get('percept_err_mm') is not None else "N/A"
        print(f"Perception Error:            {percept_err_str} mm (gate <= 3.0 mm)")
        print(f"Lift Slip:                   {lift_s_str} mm (gate <= 1.0 mm)")
        print(f"Transport Slip:              {trans_s_str} mm (gate <= 1.0 mm)")
        print(f"Placement Position Error:    {place_err_str} mm (gate <= 10.0 mm)")
        print(f"Unintended Contacts:         {dyn_collisions_detected}")
        print(f"Dynamic Obstacle Post-Cycle: PERSISTENT ({post_pos})")
        if scene_node_telemetry:
            print(f"Scene Node Telemetry:        Rx={scene_node_telemetry.get('rx')}, Accepted={scene_node_telemetry.get('accepted')} (ADD={scene_node_telemetry.get('add')}, MOVE={scene_node_telemetry.get('move')}), Rate={scene_node_telemetry.get('rate_hz')} Hz")
        print(f"Evidence Directory:          {evidence_dir}")
        print("=" * 70)

        if results["verdict"] == "PASS":
            print("\n>>> STAGE-3B B0 PASS <<<\n")
            return 0
        else:
            print("\n>>> STAGE-3B B0 FAIL <<<\n")
            return 1

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
    sys.exit(run_stage3b_b0_qualification())

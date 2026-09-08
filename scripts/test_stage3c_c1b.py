#!/usr/bin/env python3
"""scripts/test_stage3c_c1b.py — Stage-3C C1B: OBSERVE-ONLY future-path
invalidity observation, without forcing physical contact.

SCENARIO (qualification-only, does not touch the Stage-3B production
dynamic_obstacle model): dynamic_obstacle_0 does not exist in the world for
pregrasp/descent/grasp/lift at all. It is spawned dynamically the instant
"M3 STAGE 4 TRANSPORT_BEGIN" is observed in the live m3_grasp log, running a
case-specific DeterministicMotion sweep -- center_x=0.450 m (matching the
Scene-A transport corridor's X), center_z=0.860 m, Y in [0.15, 0.35] m,
period=1.0 s -- for exactly one measured sim-time period-and-change
(1.2 s) after spawn, then REMOVED (both the Gazebo model AND the MoveIt
PlanningScene CollisionObject, published directly by this harness, since
dynamic_obstacle_scene_node itself has no removal path -- see
PROJECT_STATE.md's Stage-3B "known limitation"; this harness's own removal
publish does not modify that node).

WHY THIS DOES NOT FORCE PHYSICAL CONTACT (stated before running, per the
Stage-3C C1 spec's requirement to justify this in advance, not after the
fact):
  1. The obstacle's Y range [0.15, 0.35] does not overlap the robot's actual
     starting transport Y (~-0.149, at the post-lift pick location) at all
     -- a 0.30 m gap at spawn time regardless of the obstacle's phase.
  2. The obstacle is REMOVED (Gazebo model deleted + PlanningScene
     CollisionObject REMOVE published) 1.2 s after spawn -- a fixed, sim-
     time-measured window -- while the measured Stage-3C C1A transport
     duration for this identical Scene-A cycle was 3.2584 s. The robot's
     actual Y only approaches the place-side region (~0.15-0.20) during the
     LATTER portion of that ~3.26 s window (MoveIt's iterative time
     parameterization is slowest at the start/end and fastest through the
     middle, so early real time corresponds to little real progress). By
     the time the robot could plausibly reach Y~0.15-0.20 physically, the
     obstacle has been gone for roughly two seconds.
  3. Within its own 1.2 s active window (more than one full 1.0 s period),
     the obstacle is GUARANTEED to visit its y_min=0.15 extreme at least
     once -- exactly the observation C1B needs -- entirely independent of
     spawn phase.
  4. Physical contact is therefore governed by Gazebo's own physics engine,
     which has the object entirely removed for roughly the back 60% of
     transport; the MoveIt-side "ghost" object this harness also clears at
     the same moment prevents even a stale collision-CHECK false positive
     for the remainder of the cycle.

PHYSICAL CONTACT EVIDENCE (Stage-3C C1.1)
  Physical contact is measured by a REAL Gazebo contact sensor on the
  obstacle (gz::sim::systems::Contact -> DART contact manifolds), recorded
  by scripts/perception/gz_contact_observer.py, whose ability to detect a
  deliberate contact for this exact kinematic obstacle body type was
  positively proven first (scripts/test_stage3c_contact_probe.py). MoveIt
  /check_state_validity is still recorded, but strictly under its own name
  (moveit_current_state_collision_count) -- it is a PlanningScene model
  check and is never presented as physical-contact evidence.
"""

import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetStateValidity

REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR / "scripts/perception"))
sys.path.insert(0, str(REPO_DIR / "scripts/lib"))

import milestone_f1_harness as f1_harness
import stage2a_analyzer as analyzer
import stage3c_contact_qual as cq

OBSTACLE_ID = "dynamic_obstacle_0"
OBSTACLE_BOX = [0.05, 0.05, 0.10]

# --- C1B qualification-only obstacle motion (NOT the Stage-3B B1 production
# model; see this file's header for why contact is not expected). ----------
C1B_CENTER_X = 0.450
C1B_CENTER_Z = 0.860
C1B_Y_MIN = 0.15
C1B_Y_MAX = 0.35
C1B_PERIOD_S = 1.0
C1B_ACTIVE_WINDOW_S = 1.2  # > one full period -- guarantees a y_min visit


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


def create_c1b_obstacle_sdf(out_path: Path):
    """Case-specific C1B obstacle SDF. Never writes to, or reads from, the
    production model path
    (ur5e_robotiq_description/models/dynamic_obstacle/model.sdf), which
    stays byte-for-byte untouched by this qualification."""
    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="dynamic_obstacle">
    <pose>{C1B_CENTER_X:.4f} {C1B_Y_MAX:.4f} {C1B_CENTER_Z:.4f} 0 0 0</pose>
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
        <geometry><box><size>{OBSTACLE_BOX[0]} {OBSTACLE_BOX[1]} {OBSTACLE_BOX[2]}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{OBSTACLE_BOX[0]} {OBSTACLE_BOX[1]} {OBSTACLE_BOX[2]}</size></box></geometry>
        <material>
          <ambient>1.0 0.05 0.85 1.0</ambient>
          <diffuse>1.0 0.05 0.85 1.0</diffuse>
          <specular>0.5 0.5 0.5 1.0</specular>
        </material>
      </visual>
    </link>
    <plugin filename="libdeterministic_motion_system.so" name="ur5e_robotiq_sim::DeterministicMotion">
      <center_x>{C1B_CENTER_X:.4f}</center_x>
      <center_z>{C1B_CENTER_Z:.4f}</center_z>
      <y_min>{C1B_Y_MIN:.4f}</y_min>
      <y_max>{C1B_Y_MAX:.4f}</y_max>
      <period>{C1B_PERIOD_S:.2f}</period>
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
    # Stage-3C C1.1: add the SAME qualification-only contact sensor used
    # for C1A, so this run produces genuine Gazebo-physics contact
    # evidence. Motion/geometry/pose/period are the already-qualified
    # C1B values, unchanged.
    out_path.write_text(cq.add_contact_sensor(sdf_content))


class C1BMonitorNode(Node):
    def __init__(self):
        # use_sim_time=True is REQUIRED here, not cosmetic: this node both
        # publishes a stamped message (the REMOVE below) and makes a timing
        # decision (the active-window elapsed check) from its own clock.
        # Without this, get_clock().now() returns wall-clock time while the
        # rest of the stack (m3_grasp, the C1 monitor, dynamic_obstacle_
        # scene_node) all run on sim time -- a clock-domain mismatch. This
        # was found during the Stage-3C C1 closeout audit: the REMOVE
        # message's wall-clock stamp made the C1 monitor's own
        # scene_age_ms computation go hugely negative for the rest of the
        # run (see transport_path_monitor.hpp's is_obstacle_data_stale()
        # audit-driven hardening, which independently ensures a negative
        # age is never read as "fresh" -- this fix addresses the root
        # cause in the harness rather than relying on that alone).
        super().__init__(
            "c1b_monitor_node",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=100)
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=200)

        self.ros_poses = []
        self.co_pub = self.create_publisher(CollisionObject, "/collision_object", qos_reliable)
        self.pose_sub = self.create_subscription(
            PoseStamped, "/model/dynamic_obstacle/pose", self.on_pose, qos_best_effort)
        self.get_scene_cli = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.validity_cli = self.create_client(GetStateValidity, "/check_state_validity")

    def on_pose(self, msg: PoseStamped):
        t_sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.ros_poses.append({"t_sim": t_sim, "y": msg.pose.position.y})

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

    def remove_obstacle_from_scene(self):
        co = CollisionObject()
        co.header.frame_id = "world"
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = OBSTACLE_ID
        co.operation = CollisionObject.REMOVE
        self.co_pub.publish(co)


def parse_c1_monitor_telemetry(log_text: str):
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
            "invalid_tick_count", "stale_tick_count", "max_consecutive_invalid_ticks",
            "has_invalidity", "first_invalidity_elapsed_s", "last_invalidity_elapsed_s",
        ]:
            m = re.search(rf"{key}=(\S+)", stop_line)
            if m:
                summary[key] = m.group(1)

    return {"start_line": start_line, "stop_line": stop_line, "ticks": ticks, "summary": summary}


def run_c1b_qualification():
    print("=======================================================================")
    print("STAGE-3C C1B — OBSERVE-ONLY FUTURE-PATH INVALIDITY (no forced contact)")
    print("=======================================================================")

    kill_all_processes()

    timestamp_str = time.strftime("%Y%m%d_%H%M%S")
    evidence_dir = REPO_DIR / f"evidence/stage3c_c1b_{timestamp_str}"
    evidence_dir.mkdir(parents=True, exist_ok=True)
    print(f"Evidence directory: {evidence_dir}")

    c1b_sdf_path = evidence_dir / "dynamic_obstacle_c1b.sdf"
    create_c1b_obstacle_sdf(c1b_sdf_path)
    print(f"C1B qualification-only obstacle SDF (production model.sdf untouched): {c1b_sdf_path}")
    print(
        f"  center_x={C1B_CENTER_X} center_z={C1B_CENTER_Z} "
        f"y=[{C1B_Y_MIN},{C1B_Y_MAX}] period={C1B_PERIOD_S}s active_window={C1B_ACTIVE_WINDOW_S}s")

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
        print("\n[1/9] Starting Gazebo ground-truth pose observer (for the obstacle, once spawned)...")
        gz_pose_csv = evidence_dir / "gz_pose_stream.csv"
        obs_cmd = f"{ros_env_prefix} python3 {REPO_DIR}/scripts/perception/gz_pose_observer.py --out {gz_pose_csv}"
        procs["observer"] = subprocess.Popen(
            obs_cmd, shell=True, executable="/bin/bash", env=env, start_new_session=True)

        print("[2/9] Launching Gazebo sim_control (parallel_jaw + camera)...")
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
            if "arm_controller" in res.stdout and "active" in res.stdout and "parallel_jaw_gripper_controller" in res.stdout:
                controllers_active = True
                break
            time.sleep(1.0)
        if not controllers_active:
            raise RuntimeError("Controllers failed to activate in Gazebo!")
        print("Controllers are ACTIVE.")

        print("[3/9] Launching MoveIt move_group...")
        mg_cmd = (
            f"{ros_env_prefix} ros2 launch ur5e_robotiq_moveit_config move_group.launch.py "
            f"gripper_model:=parallel_jaw"
        )
        procs["move_group"] = subprocess.Popen(
            mg_cmd, shell=True, executable="/bin/bash", env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)

        print("[4/9] Initializing ROS 2 monitor node and waiting for MoveIt services...")
        rclpy.init()
        monitor = C1BMonitorNode()
        if not monitor.get_scene_cli.wait_for_service(timeout_sec=20.0):
            raise RuntimeError("MoveIt /get_planning_scene service unavailable!")
        if not monitor.validity_cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("MoveIt /check_state_validity service unavailable!")
        print("MoveIt services are ACTIVE.")

        print("[5/9] Launching stage3b_dynamic_scene (ROS bridge + dynamic_obstacle_scene_node)...")
        scene_launch_cmd = (
            f"{ros_env_prefix} ros2 launch ur5e_pick_place stage3b_dynamic_scene.launch.py use_sim_time:=true"
        )
        scene_log_path = evidence_dir / "stage3b_dynamic_scene.log"
        scene_log_file = open(scene_log_path, "w")
        procs["dynamic_scene"] = subprocess.Popen(
            scene_launch_cmd, shell=True, executable="/bin/bash", env=env,
            stdout=scene_log_file, stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(2.0)
        print("(dynamic_obstacle_0 is NOT spawned yet -- deferred until TRANSPORT_BEGIN.)")

        print("\n[6/9] Spawning Scene-A pick target in Gazebo...")
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

        print("\n[7/9] Launching perception nodes (object_detector + object_position_world)...")
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

        # Stage-3C C1.1: start the GENUINE Gazebo-physics contact observer
        # before the cycle. The obstacle does not exist yet; the observer
        # retries until its sensor topic is advertised at spawn time, so the
        # obstacle's entire physical lifetime is covered.
        contact_csv = evidence_dir / "gazebo_obstacle_contacts.csv"
        contact_cmd = (
            f"{ros_env_prefix} python3 {REPO_DIR}/scripts/perception/gz_contact_observer.py "
            f"--topic {cq.CONTACT_TOPIC} --out {contact_csv}"
        )
        procs["contact_observer"] = subprocess.Popen(
            contact_cmd, shell=True, executable="/bin/bash", env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)

        print("\n[8/9] Executing Scene-A Pick-and-Place Manipulation Cycle (m3_grasp, C1 monitor)...")
        m3_csv = evidence_dir / "m3_grasp.csv"
        m3_log_path = evidence_dir / "m3_grasp.log"
        marker_prefix = evidence_dir / "stage"
        marker_ready_file = evidence_dir / "stage.run_summary_ready"
        transportdone_file = evidence_dir / "stage.transportdone_ready"

        # stdbuf -oL forces line buffering on m3_grasp's own stdout so this
        # harness's live log-tail detection of TRANSPORT_BEGIN is not
        # delayed by full (block) buffering under file redirection -- found
        # necessary during C1A's own gripper-pose reconnaissance capture.
        m3_cmd = (
            f"{ros_env_prefix} stdbuf -oL -eL ros2 launch ur5e_pick_place m3_grasp.launch.py "
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

        obstacle_spawned = False
        obstacle_removed = False
        obstacle_spawn_sim_t = None
        obstacle_registered_in_scene = False
        contact_topic_live = False

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

            if not obstacle_spawned and m3_log_path.exists():
                text_tail = m3_log_path.read_text(errors="replace")
                if "M3 STAGE 4 TRANSPORT_BEGIN" in text_tail:
                    obstacle_spawned = True
                    obstacle_spawn_sim_t = monitor.get_clock().now().nanoseconds * 1e-9
                    print(f"\n[TRANSPORT_BEGIN detected] Spawning C1B obstacle at sim_t~{obstacle_spawn_sim_t:.3f}s...")
                    spawn_env = env.copy()
                    spawn_env["SDF_FILE"] = str(c1b_sdf_path)
                    spawn_res = subprocess.run(
                        [str(REPO_DIR / "scripts/spawn_dynamic_obstacle.sh")],
                        env=spawn_env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                    print(f"Spawn result: {spawn_res.stdout.strip()}")
                    gz_topics_live = subprocess.run(
                        "gz topic -l", shell=True, capture_output=True, text=True).stdout
                    (evidence_dir / "gz_topics.txt").write_text(gz_topics_live)
                    contact_topic_live = cq.CONTACT_TOPIC in gz_topics_live
                    print(f"Gazebo contact sensor topic advertised: {contact_topic_live}")

            if obstacle_spawned and not obstacle_removed:
                if not obstacle_registered_in_scene:
                    scene = monitor.query_planning_scene(timeout_sec=1.0)
                    if scene and any(co.id == OBSTACLE_ID for co in scene.world.collision_objects):
                        obstacle_registered_in_scene = True
                        print("dynamic_obstacle_0 registered in PlanningScene.")
                now_sim_t = monitor.get_clock().now().nanoseconds * 1e-9
                if now_sim_t - obstacle_spawn_sim_t >= C1B_ACTIVE_WINDOW_S:
                    print(f"\n[Active window elapsed] Removing C1B obstacle at sim_t~{now_sim_t:.3f}s...")
                    remove_env = env.copy()
                    remove_env["REMOVE"] = "1"
                    subprocess.run(
                        [str(REPO_DIR / "scripts/spawn_dynamic_obstacle.sh")],
                        env=remove_env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                    monitor.remove_obstacle_from_scene()
                    obstacle_removed = True

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
                    print(f"[COLLISION ALERT] Detected ACTUAL contact with {OBSTACLE_ID} at t={cur_t:.2f}s!")

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

        print("\n[9/9] Post-manipulation audit, C1 telemetry, and gates...")
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

        # MoveIt PlanningScene evidence -- explicitly NOT physical contact.
        moveit_current_state_collision_count = sum(
            s["dyn_contacts"] for s in in_flight_collision_samples)

        # Genuine Gazebo physics evidence (positively proven detectable for
        # this exact kinematic obstacle body type beforehand -- see
        # evidence/stage3c_c11_contact_probe_*).
        # LIVENESS (same rationale as C1A): the C1B obstacle was REMOVED
        # mid-run, so the probe re-spawns that same instrumented obstacle and
        # puts a blocker in its sweep. Placed at y=0.34, clear of both the
        # placed object (y <= ~0.222) and the robot's parked column at
        # y ~ 0.20. Everything it produces is excluded from the
        # qualification window by wall_ns_max.
        print("\nIn-session contact-observer liveness probe (post-qualification)...")
        liveness = cq.run_liveness_probe(
            evidence_dir, contact_csv, blocker_xyz=(C1B_CENTER_X, 0.34, C1B_CENTER_Z),
            wait_s=6.0, respawn_obstacle_sdf=c1b_sdf_path, blocker_size=0.05)
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
        fjt_succeeded = "terminal_action_status=SUCCEEDED" in m3_log_content

        c1 = parse_c1_monitor_telemetry(m3_log_content)
        invalid_ticks = [
            t for t in c1["ticks"]
            if t.get("current_state_valid") == "0" or t.get("future_path_valid") == "0"
        ]
        future_invalid_ticks = [t for t in c1["ticks"] if t.get("future_path_valid") == "0"]
        current_invalid_ticks = [t for t in c1["ticks"] if t.get("current_state_valid") == "0"]

        has_invalidity = c1["summary"].get("has_invalidity") == "1"
        max_consecutive = int(c1["summary"].get("max_consecutive_invalid_ticks", 0))
        first_invalidity_elapsed_s = c1["summary"].get("first_invalidity_elapsed_s")
        last_invalidity_elapsed_s = c1["summary"].get("last_invalidity_elapsed_s")

        # eventual valid-again observation: at least one VALID tick after
        # the last recorded invalid tick.
        last_invalid_idx = -1
        for i, t in enumerate(c1["ticks"]):
            if t.get("current_state_valid") == "0" or t.get("future_path_valid") == "0":
                last_invalid_idx = i
        eventually_valid_again = any(
            t.get("current_state_valid") == "1" and t.get("future_path_valid") == "1"
            for t in c1["ticks"][last_invalid_idx + 1:]
        ) if last_invalid_idx >= 0 and last_invalid_idx + 1 < len(c1["ticks"]) else False

        collision_pairs_seen = sorted({
            t["collision_pairs"] for t in invalid_ticks if t.get("collision_pairs")
        })

        stale_sanity = cq.summarize_staleness(c1["ticks"])

        verdict = "PASS" if (
            metrics["verdict"] == "PASS"
            and gazebo_physical_contact_count == 0
            and contact_observer_healthy
            and moveit_current_state_collision_count == 0
            and stale_sanity["negative_age_ticks"] == 0
            and len(current_invalid_ticks) == 0
            and not replan_occurred
            and watchdog_count == 0
            and cancel_count == 0
            and not fjt_failed
            and not settle_timeout
            and fjt_succeeded
            and has_invalidity
            and len(future_invalid_ticks) >= 1
            and eventually_valid_again
        ) else "FAIL"

        results = {
            "milestone": "Stage-3C C1B",
            "objective": "Observe-only future-path invalidity without forced physical contact",
            "scenario": {
                "center_x": C1B_CENTER_X, "center_z": C1B_CENTER_Z,
                "y_min": C1B_Y_MIN, "y_max": C1B_Y_MAX, "period_s": C1B_PERIOD_S,
                "active_window_s": C1B_ACTIVE_WINDOW_S,
                "obstacle_spawn_sim_t": obstacle_spawn_sim_t,
                "obstacle_registered_in_scene": obstacle_registered_in_scene,
                "obstacle_removed": obstacle_removed,
            },
            "manipulation_metrics": metrics,
            "gazebo_physical_contact_evidence": {
                "source": "gz::sim::systems::Contact sensor on dynamic_obstacle "
                          "(qualification-only SDF; sensor is the sole telemetry delta)",
                "topic": cq.CONTACT_TOPIC,
                "gazebo_physical_contact_count": gazebo_physical_contact_count,
                "contact_observer_healthy": contact_observer_healthy,
                "contact_topic_advertised_at_spawn": contact_topic_live,
                "detail_qualification_window": gz_contacts,
                "liveness_probe": liveness,
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
                "transport_fjt_succeeded": fjt_succeeded,
                "transport_physical_settle_timeout": settle_timeout,
            },
            "c1_monitor": c1,
            "c1_monitor_invalid_ticks": invalid_ticks,
            "c1_monitor_future_invalid_ticks": future_invalid_ticks,
            "c1_monitor_current_invalid_ticks": current_invalid_ticks,
            "c1_monitor_collision_pairs_seen": collision_pairs_seen,
            "c1_monitor_eventually_valid_again": eventually_valid_again,
            "verdict": verdict,
        }
        with open(evidence_dir / "c1b_qualification_results.json", "w") as f:
            json.dump(results, f, indent=2)

        print("\n" + "=" * 75)
        print("STAGE-3C C1B QUALIFICATION SUMMARY")
        print("=" * 75)
        print(f"Overall Verdict:              {verdict}")
        print(f"Manipulation Result:          {metrics['result']}")
        print(f"FJT Succeeded:                {fjt_succeeded}")
        print(f"GAZEBO physical contacts (dynamic_obstacle_0): {gazebo_physical_contact_count}")
        print(f"  contact observer healthy:   {contact_observer_healthy} ({gz_contacts['messages_total']} msgs, {gz_contacts['empty_rows']} empty)")
        print(f"MoveIt current-state collisions (NOT physical): {moveit_current_state_collision_count}")
        print(f"Staleness sanity:             {stale_sanity}")
        print(f"Replan/Watchdog/Cancel:       {replan_occurred}/{watchdog_count}/{cancel_count}")
        print(f"Monitor Has Invalidity:       {has_invalidity}")
        print(f"Future-Invalid Ticks:         {len(future_invalid_ticks)}")
        print(f"Current-Invalid Ticks:        {len(current_invalid_ticks)}")
        print(f"Max Consecutive Invalid:      {max_consecutive}")
        print(f"First/Last Invalidity (s):    {first_invalidity_elapsed_s} / {last_invalidity_elapsed_s}")
        print(f"Eventually Valid Again:       {eventually_valid_again}")
        print(f"Collision Pairs Seen:         {collision_pairs_seen}")
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
    sys.exit(run_c1b_qualification())

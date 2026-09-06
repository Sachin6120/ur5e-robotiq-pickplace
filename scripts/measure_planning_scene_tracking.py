#!/usr/bin/env python3
"""scripts/measure_planning_scene_tracking.py — Time-Aligned PlanningScene Tracking Error Measurement.

Accurately measures:
1. Gazebo -> ROS PoseStamped discrepancy.
2. Accepted ROS Pose -> PlanningScene stored pose error (storage fidelity with time alignment).
3. End-to-end Gazebo -> PlanningScene apparent lag (due to 10 Hz discrete update period).
4. Full error distribution (min, median, mean, p95, max).
"""

import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene

REPO_DIR = Path(__file__).resolve().parents[1]
OBSTACLE_ID = "dynamic_obstacle_0"


def kill_stage3b_processes():
    cmds = [
        "pkill -9 -f 'dynamic_obstacle_scene_node' || true",
        "pkill -9 -f 'dynamic_obstacle_bridge' || true",
        "pkill -9 -f 'move_group.launch.py' || true",
        "pkill -9 -f 'lib/moveit_ros_move_group/move_group' || true",
        "pkill -9 -f 'parameter_bridge' || true",
        "pkill -9 -f 'gz sim' || true",
        "pkill -9 -f 'ruby.*gz' || true",
        "pkill -9 -f 'robot_state_publisher' || true",
    ]
    for cmd in cmds:
        subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)


class TimeAlignedTrackerNode(Node):
    def __init__(self):
        super().__init__("time_aligned_tracker_node")

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
        self.published_cos = []

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/model/dynamic_obstacle/pose",
            self.on_pose,
            qos_best_effort,
        )
        self.co_sub = self.create_subscription(
            CollisionObject,
            "/collision_object",
            self.on_co,
            qos_reliable,
        )

        self.get_scene_cli = self.create_client(GetPlanningScene, "/get_planning_scene")

    def on_pose(self, msg: PoseStamped):
        t_sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.ros_poses.append({
            "t_sim": t_sim,
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
        })

    def on_co(self, msg: CollisionObject):
        t_sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.published_cos.append({
            "t_sim": t_sim,
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
            "op": msg.operation,
        })


def main():
    print("=================================================================")
    print("STAGE-3B STEP 2: TIME-ALIGNED PLANNING SCENE TRACKING AUDIT")
    print("=================================================================")

    kill_stage3b_processes()

    # Environment
    env = os.environ.copy()
    install_lib = "/home/sachin/ur5e_ws/install/ur5e_robotiq_description/lib"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{install_lib}:{env.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"
    env["LD_LIBRARY_PATH"] = f"{install_lib}:{env.get('LD_LIBRARY_PATH', '')}"

    # Launch Gazebo Sim
    print("[1/5] Launching Gazebo simulation...")
    sim_cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /home/sachin/ur5e_ws/install/setup.bash && "
        "ros2 launch ur5e_robotiq_description ur5e_robotiq_sim_control.launch.py "
        "gazebo_gui:=false gripper_model:=parallel_jaw"
    )
    sim_proc = subprocess.Popen(
        sim_cmd, shell=True, executable="/bin/bash",
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        env=env, start_new_session=True,
    )
    time.sleep(5.0)

    # Launch MoveIt move_group
    print("[2/5] Launching MoveIt move_group...")
    mg_cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /home/sachin/ur5e_ws/install/setup.bash && "
        "ros2 launch ur5e_robotiq_moveit_config move_group.launch.py "
        "gripper_model:=parallel_jaw"
    )
    mg_proc = subprocess.Popen(
        mg_cmd, shell=True, executable="/bin/bash",
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        env=env, start_new_session=True,
    )
    time.sleep(5.0)

    # Init Tracker Node
    print("[3/5] Initializing time-aligned tracker node...")
    rclpy.init()
    node = TimeAlignedTrackerNode()

    if not node.get_scene_cli.wait_for_service(timeout_sec=10.0):
        print("[ERROR] /get_planning_scene not available!")
        kill_stage3b_processes()
        sys.exit(1)

    # Launch Stage-3B dynamic scene bridge & node
    print("[4/5] Launching stage3b_dynamic_scene...")
    scene_cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /home/sachin/ur5e_ws/install/setup.bash && "
        "ros2 launch ur5e_pick_place stage3b_dynamic_scene.launch.py use_sim_time:=true"
    )
    scene_proc = subprocess.Popen(
        scene_cmd, shell=True, executable="/bin/bash",
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        env=env, start_new_session=True,
    )
    time.sleep(1.5)

    # Spawn Obstacle
    print("[5/5] Spawning dynamic obstacle and running measurement sweep for 12 seconds...")
    spawn_script = REPO_DIR / "scripts/spawn_dynamic_obstacle.sh"
    subprocess.run([str(spawn_script)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Measurement sweep: Query PlanningScene every 250ms for 12 seconds
    measurements = []
    start_time = time.time()
    last_query_t = time.time()

    while time.time() - start_time < 12.0:
        rclpy.spin_once(node, timeout_sec=0.02)

        # Every 250ms, query PlanningScene
        if time.time() - last_query_t >= 0.25 and len(node.published_cos) > 2:
            last_query_t = time.time()
            t_query = node.get_clock().now()
            t_query_sim = t_query.nanoseconds * 1e-9

            # Snapshot the latest published CollisionObject up to this query time
            latest_co_snapshot = node.published_cos[-1]

            scene_req = GetPlanningScene.Request()
            scene_req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            future = node.get_scene_cli.call_async(scene_req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)

            if future.done() and future.result():
                scene = future.result().scene
                matching = [o for o in scene.world.collision_objects if o.id == OBSTACLE_ID]
                if matching:
                    ps_obj = matching[0]
                    ps_pos = (ps_obj.pose.position.x, ps_obj.pose.position.y, ps_obj.pose.position.z)

                    # Find the exact CollisionObject message that matches this PlanningScene pose
                    # Or compare against the latest_co_snapshot
                    # Specifically, search published_cos for the message whose pose MoveIt holds
                    co_errors = [
                        math.sqrt((ps_pos[0]-co["x"])**2 + (ps_pos[1]-co["y"])**2 + (ps_pos[2]-co["z"])**2)
                        for co in node.published_cos
                    ]
                    min_co_err = min(co_errors)
                    best_match_idx = co_errors.index(min_co_err)
                    best_co = node.published_cos[best_match_idx]

                    # Error against the latest published CO at query time
                    err_against_latest_co = math.sqrt(
                        (ps_pos[0]-latest_co_snapshot["x"])**2 +
                        (ps_pos[1]-latest_co_snapshot["y"])**2 +
                        (ps_pos[2]-latest_co_snapshot["z"])**2
                    )

                    # Live Gazebo/ROS pose at query time
                    latest_ros_pose = node.ros_poses[-1] if node.ros_poses else None
                    if latest_ros_pose:
                        err_apparent_lag = math.sqrt(
                            (ps_pos[0]-latest_ros_pose["x"])**2 +
                            (ps_pos[1]-latest_ros_pose["y"])**2 +
                            (ps_pos[2]-latest_ros_pose["z"])**2
                        )
                    else:
                        err_apparent_lag = 0.0

                    measurements.append({
                        "t_query_sim": t_query_sim,
                        "ps_pos": ps_pos,
                        "matched_co": (best_co["x"], best_co["y"], best_co["z"]),
                        "storage_error_m": min_co_err,
                        "tracking_error_against_latest_co_m": err_against_latest_co,
                        "apparent_lag_m": err_apparent_lag,
                    })

    # Teardown
    kill_stage3b_processes()
    rclpy.shutdown()

    # Analysis
    print("\n=================================================================")
    print("MEASUREMENT RESULTS & TIME-ALIGNED ANALYSIS")
    print("=================================================================")
    print(f"Total PlanningScene query samples: {len(measurements)}")
    print(f"Total ROS pose messages received: {len(node.ros_poses)}")
    print(f"Total CollisionObject updates published: {len(node.published_cos)}")

    if not measurements:
        print("[ERROR] No measurements collected!")
        sys.exit(1)

    storage_errs_mm = np.array([m["storage_error_m"] * 1000.0 for m in measurements])
    tracking_latest_co_errs_mm = np.array([m["tracking_error_against_latest_co_m"] * 1000.0 for m in measurements])
    apparent_lag_errs_mm = np.array([m["apparent_lag_m"] * 1000.0 for m in measurements])

    # 1. MoveIt PlanningScene Storage Fidelity (Exactness of MOVE application)
    print("\n1. MoveIt PlanningScene Storage Fidelity (Pose in CO message vs Stored Pose):")
    print(f"   Min error:    {np.min(storage_errs_mm):.6f} mm")
    print(f"   Median error: {np.median(storage_errs_mm):.6f} mm")
    print(f"   Mean error:   {np.mean(storage_errs_mm):.6f} mm")
    print(f"   P95 error:    {np.percentile(storage_errs_mm, 95):.6f} mm")
    print(f"   Max error:    {np.max(storage_errs_mm):.6f} mm")

    # 2. Time-Aligned Tracking Error (PlanningScene stored pose vs Latest accepted MOVE at query time):
    print("\n2. Time-Aligned Tracking Error (PlanningScene stored pose vs Latest transmitted MOVE):")
    print(f"   Min error:    {np.min(tracking_latest_co_errs_mm):.4f} mm")
    print(f"   Median error: {np.median(tracking_latest_co_errs_mm):.4f} mm")
    print(f"   Mean error:   {np.mean(tracking_latest_co_errs_mm):.4f} mm")
    print(f"   P95 error:    {np.percentile(tracking_latest_co_errs_mm, 95):.4f} mm")
    print(f"   Max error:    {np.max(tracking_latest_co_errs_mm):.4f} mm")

    # 3. Gazebo continuous live position -> PlanningScene discrete 10Hz apparent lag:
    print("\n3. End-to-End Apparent Lag (Live 50Hz Gazebo stream vs 10Hz discrete PlanningScene step):")
    print(f"   Min lag:      {np.min(apparent_lag_errs_mm):.4f} mm")
    print(f"   Median lag:   {np.median(apparent_lag_errs_mm):.4f} mm")
    print(f"   Mean lag:     {np.mean(apparent_lag_errs_mm):.4f} mm")
    print(f"   P95 lag:      {np.percentile(apparent_lag_errs_mm, 95):.4f} mm")
    print(f"   Max lag:      {np.max(apparent_lag_errs_mm):.4f} mm")

    # 4. Gazebo -> ROS bridge accuracy:
    # Analytical trajectory at time t vs ROS PoseStamped at time t
    gz_ros_errs = []
    for p in node.ros_poses:
        t = p["t_sim"]
        phase = t % 4.0
        frac = (phase / 2.0) if phase <= 2.0 else ((4.0 - phase) / 2.0)
        expected_y = 0.25 + frac * 0.20
        err = math.sqrt((p["x"] - 0.70)**2 + (p["y"] - expected_y)**2 + (p["z"] - 0.85)**2)
        gz_ros_errs.append(err * 1000.0)
    gz_ros_errs = np.array(gz_ros_errs)

    print("\n4. Gazebo Analytical Ground Truth -> ROS PoseStamped Bridge Accuracy:")
    print(f"   Min error:    {np.min(gz_ros_errs):.4f} mm")
    print(f"   Median error: {np.median(gz_ros_errs):.4f} mm")
    print(f"   Mean error:   {np.mean(gz_ros_errs):.4f} mm")
    print(f"   P95 error:    {np.percentile(gz_ros_errs, 95):.4f} mm")
    print(f"   Max error:    {np.max(gz_ros_errs):.4f} mm")

    # Verification against <= 5.0 mm requirement
    storage_pass = np.max(storage_errs_mm) <= 5.0
    tracking_pass = np.max(tracking_latest_co_errs_mm) <= 5.0

    print("\n=================================================================")
    print("VERDICT & CLASSIFICATION")
    print(f"Storage Fidelity Exactness: {'EXACT (0.000 mm)' if np.max(storage_errs_mm) < 1e-3 else 'APPROXIMATE'}")
    print(f"Time-Aligned Tracking (<= 5.0 mm): {'PASS' if tracking_pass else 'FAIL'}")
    print(f"Gazebo->ROS Bridge Accuracy (<= 5.0 mm): {'PASS' if np.max(gz_ros_errs) <= 5.0 else 'FAIL'}")
    print("=================================================================")


if __name__ == "__main__":
    main()

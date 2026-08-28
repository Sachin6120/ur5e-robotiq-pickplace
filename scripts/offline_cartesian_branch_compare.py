#!/usr/bin/env python3
"""Compare two fixed pregrasp IK branches without sending any motion command.

This client calls only MoveIt's FK, state-validity, and Cartesian-path
services.  It never creates an action client and never calls execute().
"""

import json
import math
import pathlib
import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

# Exact IK branches recovered against the frozen Scene-A tool0 pregrasp pose.
BRANCHES = {
    "A": [-0.571183951, -1.268026458, 2.127412001,
          -2.430181870, -1.570796327, -2.141980278],
    "B": [-0.571183951, -0.907168565, 1.521906541,
          0.956058351, 1.570796327, 0.999612376],
}

PREGRASP = (0.477965, -0.148707, 0.921500)
GRASP = (0.477965, -0.148707, 0.821500)
# The project logs this quaternion in geometry_msgs x/y/z/w order.
ORIENTATION = (1.0, 0.0, 0.0, 0.0)


def robot_state(q):
    state = RobotState()
    state.is_diff = False
    state.joint_state = JointState()
    state.joint_state.name = JOINTS
    state.joint_state.position = q
    return state


def seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


def pose_error(pose, target):
    dx = pose.position.x - target[0]
    dy = pose.position.y - target[1]
    dz = pose.position.z - target[2]
    position_m = math.sqrt(dx * dx + dy * dy + dz * dz)
    dot = abs(
        pose.orientation.x * ORIENTATION[0] + pose.orientation.y * ORIENTATION[1]
        + pose.orientation.z * ORIENTATION[2] + pose.orientation.w * ORIENTATION[3]
    )
    orientation_rad = 2.0 * math.acos(min(1.0, max(-1.0, dot)))
    return position_m, orientation_rad


class Compare(Node):
    def __init__(self):
        super().__init__("offline_cartesian_branch_compare")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.validity = self.create_client(GetStateValidity, "/check_state_validity")
        self.cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path")

    def call(self, client, request, name):
        if not client.wait_for_service(timeout_sec=20.0):
            raise RuntimeError(f"{name} service unavailable")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"{name} service did not return")
        return future.result()

    def fk_pose(self, q):
        request = GetPositionFK.Request()
        request.header.frame_id = "world"
        request.fk_link_names = ["tool0"]
        request.robot_state = robot_state(q)
        response = self.call(self.fk, request, "compute_fk")
        if response.error_code.val != 1 or len(response.pose_stamped) != 1:
            raise RuntimeError(f"compute_fk failed: {response.error_code.val}")
        return response.pose_stamped[0].pose

    def state_validity(self, q):
        request = GetStateValidity.Request()
        request.robot_state = robot_state(q)
        request.group_name = "arm"
        response = self.call(self.validity, request, "check_state_validity")
        return {
            "valid": bool(response.valid),
            "contacts": [
                {"body_1": c.contact_body_1, "body_2": c.contact_body_2}
                for c in response.contacts
            ],
        }

    def cartesian_path(self, q):
        waypoint = Pose()
        waypoint.position.x, waypoint.position.y, waypoint.position.z = GRASP
        waypoint.orientation.x, waypoint.orientation.y, waypoint.orientation.z, waypoint.orientation.w = ORIENTATION
        request = GetCartesianPath.Request()
        request.header.frame_id = "world"
        request.start_state = robot_state(q)
        request.group_name = "arm"
        request.link_name = "tool0"
        request.waypoints = [waypoint]
        request.max_step = 0.01
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1
        response = self.call(self.cartesian, request, "compute_cartesian_path")
        trajectory = response.solution.joint_trajectory
        if trajectory.joint_names != JOINTS:
            raise RuntimeError(f"unexpected Cartesian joint order: {trajectory.joint_names}")
        return response, trajectory


def analyse_branch(node, name, q):
    pose = node.fk_pose(q)
    pregrasp_error_m, pregrasp_orientation_rad = pose_error(pose, PREGRASP)
    start_validity = node.state_validity(q)
    response, trajectory = node.cartesian_path(q)
    points = trajectory.points
    if not points:
        raise RuntimeError(f"branch {name}: Cartesian service returned no points")

    point_validity = [node.state_validity(list(point.positions)) for point in points]
    collision_free = start_validity["valid"] and all(v["valid"] for v in point_validity)
    scale_points = []
    for point in points:
        scale_points.append({
            "time_from_start_ns": (
                point.time_from_start.sec * 1_000_000_000
                + point.time_from_start.nanosec
            ) * 2,
            "positions": list(point.positions),
            "velocities": [v * 0.5 for v in point.velocities],
            "accelerations": [a * 0.25 for a in point.accelerations],
            "effort": list(point.effort),
        })

    shoulder = [p["positions"][1] for p in scale_points]
    shoulder_velocities = [abs(p["velocities"][1]) for p in scale_points]
    shoulder_accelerations = [abs(p["accelerations"][1]) for p in scale_points]
    end_q = scale_points[-1]["positions"]
    deltas = [b - a for a, b in zip(q, end_q)]
    return {
        "branch": name,
        "pregrasp_joint_vector": q,
        "fk_tool0": {
            "position": [pose.position.x, pose.position.y, pose.position.z],
            "orientation_xyzw": [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
            "position_error_m": pregrasp_error_m,
            "orientation_error_rad": pregrasp_orientation_rad,
        },
        "start_state_validity": start_validity,
        "cartesian_fraction": response.fraction,
        "cartesian_error_code": response.error_code.val,
        "cartesian_error_message": response.error_code.message,
        "collision_free_all_trajectory_points": collision_free,
        "invalid_trajectory_point_indices": [i for i, v in enumerate(point_validity) if not v["valid"]],
        "final_joint_vector": end_q,
        "joint_delta": deltas,
        "shoulder_lift_displacement_rad": abs(end_q[1] - q[1]),
        "total_joint_displacement_l1_rad": sum(abs(d) for d in deltas),
        "total_joint_displacement_l2_rad": math.sqrt(sum(d * d for d in deltas)),
        "peak_desired_shoulder_velocity_rad_s": max(shoulder_velocities) if shoulder_velocities else None,
        "peak_desired_shoulder_acceleration_rad_s2": max(shoulder_accelerations) if shoulder_accelerations else None,
        "fjt_point_count": len(scale_points),
        "scaled_duration_s": scale_points[-1]["time_from_start_ns"] * 1e-9,
        "outbound_scaled_trajectory": scale_points,
    }


def main():
    out = pathlib.Path(sys.argv[1] if len(sys.argv) == 2 else "branch_compare.json")
    if out.exists():
        raise RuntimeError(f"refusing to overwrite evidence file: {out}")
    out.parent.mkdir(parents=True, exist_ok=False) if not out.parent.exists() else None
    rclpy.init()
    node = Compare()
    try:
        result = {
            "frozen_pregrasp_tool0": {"position": PREGRASP, "orientation_xyzw": ORIENTATION},
            "frozen_grasp_tool0": {"position": GRASP, "orientation_xyzw": ORIENTATION},
            "time_scaling": {"time": 2.0, "velocity": 0.5, "acceleration": 0.25},
            "branches": [analyse_branch(node, name, q) for name, q in BRANCHES.items()],
        }
        out.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

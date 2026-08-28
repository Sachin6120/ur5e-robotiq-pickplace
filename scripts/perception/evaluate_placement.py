#!/usr/bin/env python3
"""evaluate_placement.py -- Evaluate ground-truth object placement error from Gazebo.

Reads target from config/scene.yaml (object.place_pose), samples instantaneous
ground-truth pose from Gazebo via milestone_f1_harness, and computes:
- Delta X, Delta Y, Delta Z
- Euclidean position error
- Orientation error (angle distance in degrees and radians)
"""

import math
import pathlib
import sys
import yaml

project_dir = pathlib.Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_dir))
sys.path.insert(0, str(project_dir / "scripts/perception"))

import milestone_f1_harness as harness


def quaternion_angle_error(q_actual, q_target):
    # q = [x, y, z, w]
    dot = abs(sum(a * b for a, b in zip(q_actual, q_target)))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


def rpy_to_quaternion(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * sp * sy - sr * sp * cy
    return [qx, qy, qz, qw]


def main():
    scene_path = project_dir / "config/scene.yaml"
    with open(scene_path, "r") as f:
        scene = yaml.safe_load(f)

    place_cfg = scene["object"]["place_pose"]
    target_xyz = [float(place_cfg["x"]), float(place_cfg["y"]), float(place_cfg["z"])]
    target_rpy = [float(place_cfg["roll"]), float(place_cfg["pitch"]), float(place_cfg["yaw"])]
    target_q = rpy_to_quaternion(*target_rpy)

    pose = harness.instantaneous_object_pose()
    if not pose or len(pose) != 7:
        print("ERROR: could not read instantaneous object pose from Gazebo.", file=sys.stderr)
        sys.exit(1)

    actual_xyz = pose[:3]
    actual_q = pose[3:]

    dx = actual_xyz[0] - target_xyz[0]
    dy = actual_xyz[1] - target_xyz[1]
    dz = actual_xyz[2] - target_xyz[2]
    euclidean_err = math.sqrt(dx * dx + dy * dy + dz * dz)

    angle_err_rad = quaternion_angle_error(actual_q, target_q)
    angle_err_deg = math.degrees(angle_err_rad)

    print("=== GROUND-TRUTH PLACEMENT EVALUATION ===")
    print("Target Pose (from scene.yaml):")
    print(f"  xyz: [{target_xyz[0]:.6f}, {target_xyz[1]:.6f}, {target_xyz[2]:.6f}] m")
    print(f"  rpy: [{target_rpy[0]:.4f}, {target_rpy[1]:.4f}, {target_rpy[2]:.4f}] rad")
    print("Actual Pose (from Gazebo ground truth):")
    print(f"  xyz: [{actual_xyz[0]:.6f}, {actual_xyz[1]:.6f}, {actual_xyz[2]:.6f}] m")
    print(f"  quat [x, y, z, w]: [{actual_q[0]:.6f}, {actual_q[1]:.6f}, {actual_q[2]:.6f}, {actual_q[3]:.6f}]")
    print("Component Errors:")
    print(f"  Delta X: {dx:+.6f} m ({dx*1000:+.3f} mm)")
    print(f"  Delta Y: {dy:+.6f} m ({dy*1000:+.3f} mm)")
    print(f"  Delta Z: {dz:+.6f} m ({dz*1000:+.3f} mm)")
    print(f"Euclidean Position Error: {euclidean_err:.6f} m ({euclidean_err*1000:.3f} mm)")
    print(f"Orientation Error: {angle_err_rad:.6f} rad ({angle_err_deg:.3f} deg)")
    print("=========================================")


if __name__ == "__main__":
    main()

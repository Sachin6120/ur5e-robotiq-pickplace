#!/usr/bin/env python3
"""run_stage2_perception_yaw_probe.py — Stage-2 Perception-Only Yaw Characterisation.

Executes ONE isolated perception probe (P0..P4). Spawns pick_target at a fixed
(x, y) with a requested yaw, lets it settle, starts ONLY the RGB-D perception
pipeline (object_detector + object_position_world), and measures the
perceived-vs-ground-truth error as a function of yaw.

This is NOT a manipulation experiment. It never starts MoveIt, never launches
m3_grasp, never commands the gripper, and never moves the arm. Ground truth
is used for measurement only and is never fed back into the perception
estimate or any runtime correction — matching milestone_f1_harness.py's own
"EVALUATION ONLY" boundary, reused here unmodified.

Produced to fill the evidence gap identified by the Stage-2A O1 forensic /
clearance-budget audits: the existing dataset has samples at yaw 0 (n=13) and
yaw +15 (n=1, Stage-2A O1) only, and O1 showed the perception error is
yaw-coupled (ex roughly doubled at +15 deg vs the yaw-0 population, with ey
unchanged). ±30 and -15 have zero samples. This harness measures them
directly instead of extrapolating.
"""
import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts/perception"))

import milestone_f1_harness as harness  # noqa: E402  (reused unmodified)

_yawcase_spec = importlib.util.spec_from_file_location(
    "run_stage2a_yaw_case", str(REPO / "scripts/perception/run_stage2a_yaw_case.py")
)
yawcase = importlib.util.module_from_spec(_yawcase_spec)
_yawcase_spec.loader.exec_module(yawcase)  # reused unmodified; only functions are called

CASES = {
    "P0": 0.0,
    "P1": 15.0,
    "P2": -15.0,
    "P3": 30.0,
    "P4": -30.0,
}

PROBE_X = 0.450
PROBE_Y = -0.150
N_PERCEPTION_SAMPLES = 5


def git_head():
    try:
        r = subprocess.run(
            ["git", "-C", str(REPO), "rev-parse", "HEAD"],
            capture_output=True, text=True, timeout=10, check=False,
        )
        return r.stdout.strip() if r.returncode == 0 else None
    except Exception:
        return None


def collect_perception_samples(n, per_sample_timeout_s=15.0):
    """Calls the validated single-shot probe n times; each call subscribes
    fresh and waits for a message published after that subscription, so
    samples are independent, not a burst read of one retained message."""
    samples = []
    for i in range(n):
        sample, detail = yawcase.wait_for_perception_point(timeout_s=per_sample_timeout_s)
        if sample is None:
            return samples, f"sample {i + 1}/{n} failed: {detail}"
        samples.append(sample)
    return samples, None


def run_probe(case_name, yaw_deg, gazebo_gui=False):
    yaw_rad = math.radians(yaw_deg)
    case_dir = REPO / f"evidence/stage2_perception_yaw/{case_name}"
    if case_dir.exists():
        print(
            f"ERROR: Evidence directory already exists for {case_name}: {case_dir}\n"
            "A case is strictly single-attempt and will not overwrite existing evidence.\n"
            "Archive or remove the existing directory before running.",
            file=sys.stderr, flush=True,
        )
        sys.exit(1)
    # Reserve the slot before anything else starts, identically to
    # run_stage2a_yaw_case.py's own guard.
    case_dir.mkdir(parents=True, exist_ok=False)

    print("=======================================================", flush=True)
    print(f"  STAGE-2 PERCEPTION-ONLY YAW PROBE: {case_name} "
          f"(yaw = {yaw_deg:+.1f} deg / {yaw_rad:+.6f} rad)", flush=True)
    print(f"  Evidence dir: {case_dir}", flush=True)
    print("  NO MoveIt. NO m3_grasp. NO gripper command. NO arm motion.", flush=True)
    print("=======================================================", flush=True)

    obj_z = harness.OBJ_Z
    obj_size = harness.OBJ_SIZE
    metadata = {
        "case": case_name,
        "configured_yaw_deg": yaw_deg,
        "configured_yaw_rad": yaw_rad,
        "x": PROBE_X,
        "y": PROBE_Y,
        "z": obj_z,
        "object_size_m": obj_size,
        "object_name": harness.OBJ_NAME,
        "n_perception_samples": N_PERCEPTION_SAMPLES,
        "git_head": git_head(),
        "script": str(Path(__file__).resolve()),
        "started_wall_time": time.time(),
    }
    with open(case_dir / "case_metadata.json", "w") as f:
        json.dump(metadata, f, indent=2)

    # 1. Contamination check -- refuse to run inside a dirty environment.
    #    Same pattern as run_stage2a_yaw_case.py's own guard, reused verbatim.
    contamination = subprocess.run(
        ["pgrep", "-f",
         "m3_grasp|static_scene_tf|move_group|object_detector|"
         "object_position_world|[g]z sim|robot_state_publisher|ros2_control_node|"
         "gz_pose_observer"],
        capture_output=True, text=True, check=False,
    )
    if contamination.returncode == 0:
        print(
            "CONTAMINATED_ENVIRONMENT: existing ROS/Gazebo processes found; refusing to proceed.\n"
            f"PIDs: {contamination.stdout.strip()}",
            flush=True,
        )
        sys.exit(2)

    # 2. Start Gazebo ground-truth pose stream observer (continuous evidence).
    stream_csv = case_dir / "gt_pose_stream.csv"
    observer_proc = yawcase.start_process(
        f"python3 {REPO}/scripts/perception/gz_pose_observer.py --out {stream_csv}"
    )

    # 3. Launch Gazebo sim control WITH camera, WITHOUT MoveIt.
    gui_flag = "true" if gazebo_gui else "false"
    print(f"Launching Gazebo sim_control (camera enabled, gazebo_gui:={gui_flag})...",
          flush=True)
    sim_log_path = case_dir / "sim.log"
    sim_cmd = (
        f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && "
        "ros2 launch ur5e_robotiq_description ur5e_robotiq_sim_control.launch.py "
        f"gripper_model:=parallel_jaw enable_camera:=true gazebo_gui:={gui_flag}"
    )
    sim_log_fh = open(sim_log_path, "w")
    sim_proc = yawcase.start_process(sim_cmd, stdout=sim_log_fh, stderr=subprocess.STDOUT)

    def abort(code, *procs_and_files):
        for item in procs_and_files:
            if hasattr(item, "poll"):
                yawcase.stop_process(item)
            elif hasattr(item, "close"):
                item.close()
        sys.exit(code)

    # 4. Wait for controllers healthy (readiness check only -- no goal sent).
    active = False
    for _ in range(60):
        out, _ = yawcase.run_cmd(
            f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && "
            "ros2 control list_controllers",
            timeout=10,
        )
        if yawcase.is_controller_active(out, "arm_controller") and \
                yawcase.is_controller_active(out, "parallel_jaw_gripper_controller"):
            active = True
            break
        time.sleep(1)
    if not active:
        print("ERROR: Controllers failed to become active!", flush=True)
        abort(1, sim_proc, observer_proc, sim_log_fh)
    print("Controllers active (readiness check only; nothing commanded).", flush=True)

    # 5. Camera topics ready.
    camera_ready, camera_detail = yawcase.wait_for_camera_topics()
    if not camera_ready:
        print(f"ERROR: Camera topics unavailable! detail={camera_detail}", flush=True)
        abort(1, sim_proc, observer_proc, sim_log_fh)
    print("Camera topics available.", flush=True)

    # 6. Spawn object at the requested yaw. No MoveIt, no m3_grasp launched at any point.
    print(f"Spawning object at ({PROBE_X}, {PROBE_Y}, {obj_z}) yaw={yaw_deg:+.1f} deg...",
          flush=True)
    harness.remove_object()
    time.sleep(1.0)
    yawcase.spawn_object_yaw(PROBE_X, PROBE_Y, yaw_rad)
    settled, msg = harness.settle_object()
    if not settled:
        print(f"ERROR: Object failed to settle! msg={msg}", flush=True)
        abort(1, sim_proc, observer_proc, sim_log_fh)

    gt_settled = harness.instantaneous_object_pose()
    with open(case_dir / "gt_settled_pose.json", "w") as f:
        json.dump(gt_settled, f, indent=2)
    print(f"Object settled (Gazebo ground truth) = {gt_settled}", flush=True)

    # 7. Launch ONLY the perception pipeline. No static_scene_tf, no move_group.
    print("Launching perception nodes (object_detector, object_position_world)...",
          flush=True)
    det_log = open(case_dir / "object_detector.log", "w")
    pos_log = open(case_dir / "object_position_world.log", "w")
    det_proc = yawcase.start_process(
        f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && "
        "ros2 run ur5e_pick_place object_detector --ros-args -p use_sim_time:=true",
        stdout=det_log, stderr=subprocess.STDOUT,
    )
    pos_proc = yawcase.start_process(
        f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && "
        "ros2 run ur5e_pick_place object_position_world --ros-args -p use_sim_time:=true",
        stdout=pos_log, stderr=subprocess.STDOUT,
    )
    time.sleep(1.0)

    # 8. Capture N independent fresh perception samples. No grasp target is
    #    ever built from these; they are recorded and differenced only.
    samples, err = collect_perception_samples(N_PERCEPTION_SAMPLES)
    with open(case_dir / "perceived_samples.json", "w") as f:
        json.dump(samples, f, indent=2)
    if err is not None:
        print(f"PERCEPTION_FAILURE: {err}", flush=True)
        abort(1, pos_proc, det_proc, sim_proc, observer_proc, det_log, pos_log, sim_log_fh)
    print(f"Captured {len(samples)} perception samples.", flush=True)

    # 9. Re-query ground truth after perception capture, proving the object
    #    never moved during the probe (nothing in this script can move it).
    gt_final = harness.instantaneous_object_pose()
    with open(case_dir / "gt_final_pose.json", "w") as f:
        json.dump(gt_final, f, indent=2)

    # Teardown -- nothing manipulation-related was ever started.
    yawcase.stop_process(pos_proc)
    yawcase.stop_process(det_proc)
    yawcase.stop_process(sim_proc)
    time.sleep(1.0)
    yawcase.stop_process(observer_proc)
    det_log.close()
    pos_log.close()
    sim_log_fh.close()

    # 10. Metrics -- ground truth used for measurement only, never fed back.
    perceived_xyz = [
        sum(s["xyz"][k] for s in samples) / len(samples) for k in range(3)
    ]
    spreads = [
        math.dist(a["xyz"], b["xyz"]) for a in samples for b in samples
    ]
    sample_spread_m = max(spreads) if spreads else 0.0

    gt_xyz = gt_settled[:3]
    gt_quat = gt_settled[3:7]
    measured_yaw_rad = math.atan2(
        2.0 * (gt_quat[3] * gt_quat[2] + gt_quat[0] * gt_quat[1]),
        1.0 - 2.0 * (gt_quat[1] ** 2 + gt_quat[2] ** 2),
    )

    ex_mm = (perceived_xyz[0] - gt_xyz[0]) * 1000.0
    ey_mm = (perceived_xyz[1] - gt_xyz[1]) * 1000.0
    ez_mm = (perceived_xyz[2] - gt_xyz[2]) * 1000.0
    euclidean_mm = math.sqrt(ex_mm ** 2 + ey_mm ** 2 + ez_mm ** 2)

    cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
    harmful_projection_mm = ex_mm * cos_y + ey_mm * sin_y
    orthogonal_projection_mm = -ex_mm * sin_y + ey_mm * cos_y

    gt_drift_m = math.dist(gt_final[:3], gt_settled[:3]) if gt_final and gt_settled else None

    metrics = {
        "case": case_name,
        "configured_yaw_deg": yaw_deg,
        "configured_yaw_rad": yaw_rad,
        "measured_yaw_deg_gt": math.degrees(measured_yaw_rad),
        "gt_centre_xyz_m": gt_xyz,
        "gt_quaternion_xyzw": gt_quat,
        "perceived_xyz_mean_m": perceived_xyz,
        "perceived_sample_spread_max_m": sample_spread_m,
        "n_perception_samples": len(samples),
        "ex_mm": ex_mm,
        "ey_mm": ey_mm,
        "ez_mm": ez_mm,
        "euclidean_error_mm": euclidean_mm,
        "closing_axis": [cos_y, sin_y],
        "harmful_projection_mm": harmful_projection_mm,
        "orthogonal_projection_mm": orthogonal_projection_mm,
        "gt_drift_during_probe_m": gt_drift_m,
        "note": "ground truth used for measurement only; never fed back into "
                "perception or any target. No manipulation node was started.",
    }
    with open(case_dir / "metrics.json", "w") as f:
        json.dump(metrics, f, indent=2)
    (case_dir / "stage.probe_done").touch()

    print("\n=======================================================", flush=True)
    print(f"  {case_name} PERCEPTION-ONLY YAW PROBE RESULTS", flush=True)
    print("=======================================================", flush=True)
    print(f"Configured yaw:        {yaw_deg:+.1f} deg")
    print(f"Measured yaw (GT):     {metrics['measured_yaw_deg_gt']:+.4f} deg")
    print(f"GT centre:             {gt_xyz}")
    print(f"Perceived (mean of {len(samples)}): {perceived_xyz}")
    print(f"Sample spread (max):   {sample_spread_m * 1000:.4f} mm")
    print(f"ex, ey, ez:            {ex_mm:+.4f}, {ey_mm:+.4f}, {ez_mm:+.4f} mm")
    print(f"Euclidean error:       {euclidean_mm:.4f} mm")
    print(f"Harmful projection:    {harmful_projection_mm:+.4f} mm")
    print(f"Orthogonal projection: {orthogonal_projection_mm:+.4f} mm")
    print(f"GT drift during probe: "
          f"{gt_drift_m * 1000:.4f} mm" if gt_drift_m is not None else "N/A")
    print("=======================================================\n", flush=True)


def main():
    parser = argparse.ArgumentParser(
        description="Run one Stage-2 perception-only yaw characterisation probe. "
                     "Perception measurement only -- no MoveIt, no m3_grasp, no gripper "
                     "command, no arm motion."
    )
    parser.add_argument(
        "--case", required=True, choices=list(CASES.keys()),
        help="Predefined probe identifier (P0: 0deg, P1: +15deg, P2: -15deg, "
             "P3: +30deg, P4: -30deg)",
    )
    parser.add_argument(
        "--gui", action="store_true",
        help="Run Gazebo in visible GUI mode (default: headless)",
    )
    args = parser.parse_args()
    run_probe(args.case, CASES[args.case], gazebo_gui=args.gui)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Frozen 960x720 paired holdout for the default-off pixel-centre shadow path.

The campaign is deliberately opt-in: ``--run-all`` is required, and the 48
case manifest is written before Gazebo is started.  This is perception-only:
it starts sim_control with the RGB-D camera plus object_detector and
object_position_world.  It never starts MoveIt, static_scene_tf, m3_grasp, or
any trajectory/gripper command.  Gazebo truth is recorded only after spawn
and never passed to the detector, TF transform, or any control target.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import random
import subprocess
import sys
import time


REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts/perception"))

import milestone_f1_harness as harness  # noqa: E402 -- established truth-only helper

_yawcase_spec = importlib.util.spec_from_file_location(
    "run_stage2a_yaw_case", str(REPO / "scripts/perception/run_stage2a_yaw_case.py")
)
yawcase = importlib.util.module_from_spec(_yawcase_spec)
_yawcase_spec.loader.exec_module(yawcase)  # noqa: E402 -- reuse process/spawn helpers


CAMPAIGN_NAME = "pixel_centre_shadow_holdout_960x720"
EVIDENCE_ROOT = REPO / "evidence" / CAMPAIGN_NAME
CAMERA_WIDTH = 960
CAMERA_HEIGHT = 720
PIXEL_PITCH_M = 0.001930075
PHASE_OFFSET_M = PIXEL_PITCH_M / 4.0
RANDOMIZATION_SEED = 20260831
SHADOW_PARAMETER = "enable_pixel_centre_shadow"

# This is the frozen, pre-truth configuration matrix.  Do not add runtime
# arguments to alter it: changing this source necessarily changes its manifest
# hash and is therefore visible before the campaign can begin.
BASE_CONFIGURATIONS = (
    ("G1_yaw0", 0.450, -0.100, 0.0),
    ("G2_yaw0", 0.450, -0.200, 0.0),
    ("G3_yaw0", 0.500, -0.150, 0.0),
    ("G4_yaw0", 0.400, -0.150, 0.0),
    ("G5_yaw0", 0.480, -0.120, 0.0),
    ("Centre_yaw0", 0.450, -0.150, 0.0),
    ("Centre_yawp30", 0.450, -0.150, 30.0),
    ("Centre_yawm30", 0.450, -0.150, -30.0),
    ("Centre_yawp45", 0.450, -0.150, 45.0),
    ("D1_yawp30", 0.480, -0.120, 30.0),
    ("D2_yawm30", 0.420, -0.180, -30.0),
    ("D3_yawp45", 0.480, -0.180, 45.0),
)

PHASES = (
    ("pp", +PHASE_OFFSET_M, +PHASE_OFFSET_M),
    ("pm", +PHASE_OFFSET_M, -PHASE_OFFSET_M),
    ("mp", -PHASE_OFFSET_M, +PHASE_OFFSET_M),
    ("mm", -PHASE_OFFSET_M, -PHASE_OFFSET_M),
)

# Generated once with random.Random(RANDOMIZATION_SEED).shuffle(canonical_ids)
# and stored as a literal so code review can inspect the actual execution
# order.  The campaign manifest persists this same order before capture.
FROZEN_EXECUTION_ORDER = (
    "Centre_yawp30__phase_pm",
    "G1_yaw0__phase_mp",
    "Centre_yawp45__phase_pm",
    "Centre_yawp30__phase_mm",
    "D2_yawm30__phase_mm",
    "G3_yaw0__phase_pm",
    "G4_yaw0__phase_mm",
    "D1_yawp30__phase_pm",
    "G4_yaw0__phase_pm",
    "Centre_yaw0__phase_mp",
    "Centre_yawp45__phase_mp",
    "G3_yaw0__phase_mm",
    "G5_yaw0__phase_pm",
    "Centre_yawp30__phase_pp",
    "Centre_yawm30__phase_mp",
    "Centre_yaw0__phase_pm",
    "G3_yaw0__phase_mp",
    "G2_yaw0__phase_mp",
    "G1_yaw0__phase_pp",
    "G2_yaw0__phase_mm",
    "G5_yaw0__phase_mm",
    "G1_yaw0__phase_mm",
    "D2_yawm30__phase_pm",
    "G1_yaw0__phase_pm",
    "D3_yawp45__phase_mp",
    "D3_yawp45__phase_pp",
    "D1_yawp30__phase_mm",
    "G3_yaw0__phase_pp",
    "G2_yaw0__phase_pp",
    "Centre_yawm30__phase_mm",
    "D2_yawm30__phase_pp",
    "G5_yaw0__phase_pp",
    "D1_yawp30__phase_mp",
    "G4_yaw0__phase_mp",
    "D3_yawp45__phase_pm",
    "Centre_yaw0__phase_pp",
    "Centre_yaw0__phase_mm",
    "D2_yawm30__phase_mp",
    "Centre_yawp30__phase_mp",
    "D1_yawp30__phase_pp",
    "Centre_yawp45__phase_mm",
    "Centre_yawm30__phase_pp",
    "G2_yaw0__phase_pm",
    "Centre_yawm30__phase_pm",
    "G5_yaw0__phase_mp",
    "Centre_yawp45__phase_pp",
    "D3_yawp45__phase_mm",
    "G4_yaw0__phase_pp",
)

REQUIRED_OBSERVATION_FIELDS = (
    "camera_info",
    "raw_centroid",
    "production_camera",
    "shadow_camera",
    "production_world",
    "shadow_world",
    "perceived_pose_world",
)

OUTPUT_CONTRACT = {
    "worst_shadow_xy_norm_mm": "maximum norm of the shadow XY residual",
    "Bxy_observed_mm": "maximum absolute shadow local closing/transverse residual",
    "worst_closing_axis_residual_mm": "signed extrema and maximum absolute local closing residual",
    "residual_sign_coverage_by_yaw_family": "positive/negative/zero counts for shadow local axes",
    "production_vs_shadow_paired_improvement": "per-case norm deltas and aggregate win/draw/loss",
    "detector_yaw_failures": "all missing/invalid detector, pose, or yaw records",
}


def canonical_json(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def sha256_json(value):
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()


def git_head():
    result = subprocess.run(
        ["git", "-C", str(REPO), "rev-parse", "HEAD"],
        capture_output=True, text=True, check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else None


def yaw_family(yaw_deg):
    return f"yaw_{yaw_deg:+.0f}".replace("+", "p").replace("-", "m")


def build_frozen_cases():
    """Return the 48 immutable cases with their persisted random index."""
    by_id = {}
    canonical_ids = []
    for base_id, base_x, base_y, yaw_deg in BASE_CONFIGURATIONS:
        for phase_id, dx, dy in PHASES:
            case_id = f"{base_id}__phase_{phase_id}"
            canonical_ids.append(case_id)
            by_id[case_id] = {
                "case_id": case_id,
                "base_id": base_id,
                "base_pose_xy_m": [base_x, base_y],
                "base_yaw_deg": yaw_deg,
                "phase_id": phase_id,
                "phase_offset_xy_m": [dx, dy],
                "requested_spawn_xy_m": [base_x + dx, base_y + dy],
                "requested_spawn_yaw_deg": yaw_deg,
                "yaw_family": yaw_family(yaw_deg),
            }

    generated_order = list(canonical_ids)
    random.Random(RANDOMIZATION_SEED).shuffle(generated_order)
    if tuple(generated_order) != FROZEN_EXECUTION_ORDER:
        raise RuntimeError("frozen execution-order literal does not match its declared seed")
    if len(by_id) != 48 or len(FROZEN_EXECUTION_ORDER) != 48 or set(by_id) != set(FROZEN_EXECUTION_ORDER):
        raise RuntimeError("frozen holdout matrix must contain exactly one entry for every 12x4 case")

    return [dict(by_id[case_id], random_execution_index=index)
            for index, case_id in enumerate(FROZEN_EXECUTION_ORDER, start=1)]


def campaign_manifest():
    cases = build_frozen_cases()
    static = {
        "campaign": CAMPAIGN_NAME,
        "camera_resolution_required": [CAMERA_WIDTH, CAMERA_HEIGHT],
        "pixel_pitch_m": PIXEL_PITCH_M,
        "phase_offset_m": PHASE_OFFSET_M,
        "randomization_seed": RANDOMIZATION_SEED,
        "frozen_execution_order": list(FROZEN_EXECUTION_ORDER),
        "cases": cases,
        "output_contract": OUTPUT_CONTRACT,
        "safety_contract": {
            "perception_only": True,
            "shadow_enabled": True,
            "moveit_started": False,
            "m3_grasp_started": False,
            "trajectory_commanded": False,
            "ground_truth_evaluation_only": True,
            "reject_non_960x720": True,
            "reject_missing_or_mismatched_camera_info": True,
            "reject_timestamp_mismatch": True,
            "reject_missing_ground_truth": True,
            "no_posthoc_matrix_change": True,
        },
    }
    return dict(static, frozen_manifest_sha256=sha256_json(static))


def write_frozen_manifest(root):
    """Write the manifest once, before any truth/camera observation exists."""
    root = Path(root)
    manifest_path = root / "campaign_manifest.json"
    if root.exists():
        raise RuntimeError(f"campaign evidence root already exists: {root}; refusing to overwrite")
    root.mkdir(parents=True, exist_ok=False)
    manifest = campaign_manifest()
    manifest.update({
        "git_head_at_campaign_start": git_head(),
        "created_wall_time": time.time(),
        "matrix_frozen_before_truth": True,
    })
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def _stamp_key(message):
    stamp = message.header.stamp
    return int(stamp.sec), int(stamp.nanosec)


def _stamp_dict(message):
    sec, nanosec = _stamp_key(message)
    return {
        "sec": sec,
        "nanosec": nanosec,
        "seconds": sec + nanosec * 1e-9,
        "frame_id": message.header.frame_id,
    }


def _point_dict(message):
    return {"stamp": _stamp_dict(message), "xyz": [message.point.x, message.point.y, message.point.z]}


def _pose_dict(message):
    q = message.pose.orientation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return {
        "stamp": _stamp_dict(message),
        "xyz": [message.pose.position.x, message.pose.position.y, message.pose.position.z],
        "quaternion_xyzw": [q.x, q.y, q.z, q.w],
        "yaw_rad": yaw,
        "yaw_deg": math.degrees(yaw),
    }


def _camera_info_dict(message):
    return {"stamp": _stamp_dict(message), "width": message.width, "height": message.height,
            "K": list(message.k)}


class PairedObservationCollector:
    """Collect one strictly same-stamp production/shadow observation.

    ROS imports are intentionally local so the frozen-matrix/offline tests can
    import this module without a launched ROS graph.
    """

    def __init__(self):
        self.messages = {field: {} for field in REQUIRED_OBSERVATION_FIELDS}
        self.failure = None

    def _store(self, field, message):
        if self.failure is not None:
            return
        self.messages[field][_stamp_key(message)] = message

    def _camera_info(self, message):
        if message.width != CAMERA_WIDTH or message.height != CAMERA_HEIGHT:
            self.failure = (f"RUNTIME_RESOLUTION_REJECTED: got {message.width}x{message.height}, "
                            f"required {CAMERA_WIDTH}x{CAMERA_HEIGHT}")
            return
        if len(message.k) != 9 or not all(math.isfinite(value) for value in message.k):
            self.failure = "CAMERAINFO_REJECTED: K must contain nine finite values"
            return
        if message.k[0] <= 0.0 or message.k[4] <= 0.0:
            self.failure = "CAMERAINFO_REJECTED: focal lengths must be positive"
            return
        self._store("camera_info", message)

    def _point_callback(self, field):
        def callback(message):
            if not all(math.isfinite(value) for value in
                       (message.point.x, message.point.y, message.point.z)):
                self.failure = f"{field}_REJECTED: non-finite PointStamped"
                return
            self._store(field, message)
        return callback

    def _pose_callback(self, message):
        q = message.pose.orientation
        values = (message.pose.position.x, message.pose.position.y, message.pose.position.z,
                  q.x, q.y, q.z, q.w)
        norm = math.sqrt(sum(value * value for value in (q.x, q.y, q.z, q.w)))
        if not all(math.isfinite(value) for value in values) or abs(norm - 1.0) > 1e-3:
            self.failure = "PERCEIVED_YAW_REJECTED: invalid pose_world quaternion or position"
            return
        self._store("perceived_pose_world", message)

    def _matching_key(self):
        common = None
        for field in REQUIRED_OBSERVATION_FIELDS:
            keys = set(self.messages[field])
            common = keys if common is None else common & keys
        return min(common) if common else None

    def capture(self, timeout_s=20.0):
        import rclpy
        from geometry_msgs.msg import PointStamped, PoseStamped
        from rclpy.node import Node
        from sensor_msgs.msg import CameraInfo

        rclpy.init(args=None)
        node = Node("pixel_centre_shadow_holdout_collector")
        subscriptions = [
            node.create_subscription(CameraInfo, "/overhead_camera/camera_info", self._camera_info, 20),
            node.create_subscription(PointStamped, "/object_detector/centroid",
                                     self._point_callback("raw_centroid"), 20),
            node.create_subscription(PointStamped, "/object_detector/position_camera",
                                     self._point_callback("production_camera"), 20),
            node.create_subscription(PointStamped, "/object_detector/position_camera_shadow",
                                     self._point_callback("shadow_camera"), 20),
            node.create_subscription(PointStamped, "/object_detector/position_world",
                                     self._point_callback("production_world"), 20),
            node.create_subscription(PointStamped, "/object_detector/position_world_shadow",
                                     self._point_callback("shadow_world"), 20),
            node.create_subscription(PoseStamped, "/object_detector/pose_world",
                                     self._pose_callback, 20),
        ]
        del subscriptions  # Retain through node ownership; avoid an unused-variable warning.
        deadline = time.monotonic() + timeout_s
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
                if self.failure is not None:
                    return None, self.failure
                key = self._matching_key()
                if key is not None:
                    messages = {field: self.messages[field][key] for field in REQUIRED_OBSERVATION_FIELDS}
                    observation = {
                        "camera_info": _camera_info_dict(messages["camera_info"]),
                        "raw_centroid": _point_dict(messages["raw_centroid"]),
                        "production_camera": _point_dict(messages["production_camera"]),
                        "shadow_camera": _point_dict(messages["shadow_camera"]),
                        "production_world": _point_dict(messages["production_world"]),
                        "shadow_world": _point_dict(messages["shadow_world"]),
                        "perceived_pose_world": _pose_dict(messages["perceived_pose_world"]),
                        "sample_key": {"sec": key[0], "nanosec": key[1]},
                    }
                    raw = observation["raw_centroid"]
                    observation["raw_centroid"]["uv"] = raw["xyz"][:2]
                    observation["corrected_centroid"] = {
                        "stamp": dict(raw["stamp"]),
                        "uv": [raw["xyz"][0] + 0.5, raw["xyz"][1] + 0.5],
                        "correction_px": [0.5, 0.5],
                    }
                    observation["d10_depth_m"] = {
                        "production": observation["production_camera"]["xyz"][2],
                        "shadow": observation["shadow_camera"]["xyz"][2],
                        "identical": observation["production_camera"]["xyz"][2] ==
                                     observation["shadow_camera"]["xyz"][2],
                    }
                    return validate_paired_observation(observation)
            observed = {field: len(values) for field, values in self.messages.items()}
            return None, f"PAIRED_SAMPLE_TIMEOUT: no all-topic same-stamp sample; observed={observed}"
        finally:
            node.destroy_node()
            rclpy.shutdown()


def validate_paired_observation(observation):
    """Pure evidence gate used by the collector and offline tests."""
    try:
        camera = observation["camera_info"]
        if (camera["width"], camera["height"]) != (CAMERA_WIDTH, CAMERA_HEIGHT):
            return None, "RUNTIME_RESOLUTION_REJECTED"
        if len(camera["K"]) != 9 or not all(math.isfinite(value) for value in camera["K"]):
            return None, "CAMERAINFO_REJECTED"
        if camera["K"][0] <= 0.0 or camera["K"][4] <= 0.0:
            return None, "CAMERAINFO_REJECTED"
        sample = observation["sample_key"]
        expected = (sample["sec"], sample["nanosec"])
        for field in REQUIRED_OBSERVATION_FIELDS:
            stamp = observation[field]["stamp"]
            if (stamp["sec"], stamp["nanosec"]) != expected:
                return None, f"TIMESTAMP_MISMATCH:{field}"
        if observation["production_camera"]["xyz"][2] != observation["shadow_camera"]["xyz"][2]:
            return None, "D10_MISMATCH"
        d10 = observation["d10_depth_m"]
        if (not d10["identical"] or d10["production"] != d10["shadow"] or
                d10["production"] != observation["production_camera"]["xyz"][2]):
            return None, "D10_MISMATCH"
        raw_uv = observation["raw_centroid"]["uv"]
        corrected_uv = observation["corrected_centroid"]["uv"]
        if corrected_uv != [raw_uv[0] + 0.5, raw_uv[1] + 0.5]:
            return None, "PIXEL_CENTRE_CORRECTION_MISMATCH"
        return observation, None
    except (KeyError, TypeError, IndexError):
        return None, "MISSING_OR_MALFORMED_PAIRED_OBSERVATION"


def local_residuals_mm(world_xyz, gt_xyz, yaw_deg):
    ex_mm = (world_xyz[0] - gt_xyz[0]) * 1000.0
    ey_mm = (world_xyz[1] - gt_xyz[1]) * 1000.0
    yaw_rad = math.radians(yaw_deg)
    closing = ex_mm * math.cos(yaw_rad) + ey_mm * math.sin(yaw_rad)
    transverse = -ex_mm * math.sin(yaw_rad) + ey_mm * math.cos(yaw_rad)
    return {
        "xy_residual_mm": [ex_mm, ey_mm],
        "xy_norm_mm": math.hypot(ex_mm, ey_mm),
        "closing_axis_residual_mm": closing,
        "transverse_axis_residual_mm": transverse,
        "closing_axis": [math.cos(yaw_rad), math.sin(yaw_rad)],
        "transverse_axis": [-math.sin(yaw_rad), math.cos(yaw_rad)],
        "axis_source": "requested_spawn_yaw_deg (not ground truth)",
    }


def case_metrics(case, observation, gt_settled):
    if gt_settled is None or len(gt_settled) < 7:
        raise ValueError("MISSING_GROUND_TRUTH_EVALUATION_RECORD")
    gt_xyz = gt_settled[:3]
    yaw_deg = case["requested_spawn_yaw_deg"]
    production = local_residuals_mm(observation["production_world"]["xyz"], gt_xyz, yaw_deg)
    shadow = local_residuals_mm(observation["shadow_world"]["xyz"], gt_xyz, yaw_deg)
    return {
        "ground_truth_evaluation_only": True,
        "gt_centre_xyz_m": gt_xyz,
        "gt_quaternion_xyzw": gt_settled[3:7],
        "requested_yaw_deg_for_local_axes": yaw_deg,
        "production": production,
        "shadow": shadow,
        "paired_shadow_minus_production_xy_norm_mm": shadow["xy_norm_mm"] - production["xy_norm_mm"],
        "paired_shadow_improvement_mm": production["xy_norm_mm"] - shadow["xy_norm_mm"],
    }


def _launch_commands(case_dir, gui):
    gui_flag = "true" if gui else "false"
    setup = f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && "
    return {
        "gazebo_truth_observer":
            f"python3 {REPO}/scripts/perception/gz_pose_observer.py --out {case_dir}/gt_pose_stream.csv",
        "sim_control": setup +
            "ros2 launch ur5e_robotiq_description ur5e_robotiq_sim_control.launch.py "
            f"gripper_model:=parallel_jaw enable_camera:=true gazebo_gui:={gui_flag} "
            f"camera_width:={CAMERA_WIDTH} camera_height:={CAMERA_HEIGHT}",
        "object_detector": setup +
            f"ros2 run ur5e_pick_place object_detector --ros-args -p use_sim_time:=true "
            f"-p {SHADOW_PARAMETER}:=true",
        "object_position_world": setup +
            f"ros2 run ur5e_pick_place object_position_world --ros-args -p use_sim_time:=true "
            f"-p {SHADOW_PARAMETER}:=true",
    }


def _assert_clean_environment():
    contamination = subprocess.run(
        ["pgrep", "-f",
         "m3_grasp|static_scene_tf|move_group|object_detector|object_position_world|"
         "[g]z sim|robot_state_publisher|ros2_control_node|gz_pose_observer"],
        capture_output=True, text=True, check=False,
    )
    if contamination.returncode == 0:
        raise RuntimeError(f"CONTAMINATED_ENVIRONMENT: PIDs {contamination.stdout.strip()}")


def _wait_for_controllers():
    for _ in range(60):
        out, _ = yawcase.run_cmd(
            f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && "
            "ros2 control list_controllers", timeout=10,
        )
        if (yawcase.is_controller_active(out, "arm_controller") and
                yawcase.is_controller_active(out, "parallel_jaw_gripper_controller")):
            return
        time.sleep(1.0)
    raise RuntimeError("CONTROLLER_READINESS_TIMEOUT")


def run_case(root, manifest, case, gui=False):
    """Run one predeclared case once; failures preserve evidence and abort."""
    case_dir = Path(root) / f"{case['random_execution_index']:02d}_{case['case_id']}"
    if case_dir.exists():
        raise RuntimeError(f"case evidence already exists: {case_dir}; refusing to overwrite")
    case_dir.mkdir(parents=False, exist_ok=False)
    commands = _launch_commands(case_dir, gui)
    request = {
        **case,
        "requested_spawn_z_m": harness.OBJ_Z,
        "object_name": harness.OBJ_NAME,
        "object_size_m": harness.OBJ_SIZE,
        "git_head": git_head(),
        "campaign_manifest_sha256": manifest["frozen_manifest_sha256"],
        "camera_resolution_requested": [CAMERA_WIDTH, CAMERA_HEIGHT],
        "enable_pixel_centre_shadow": True,
        "perception_only": True,
        "moveit_started": False,
        "m3_grasp_started": False,
        "trajectory_commanded": False,
        "ground_truth_evaluation_only": True,
        "exact_launched_commands": commands,
        "started_wall_time": time.time(),
    }
    (case_dir / "case_request.json").write_text(json.dumps(request, indent=2) + "\n")

    observer_proc = sim_proc = detector_proc = world_proc = None
    files = []
    try:
        _assert_clean_environment()
        observer_proc = yawcase.start_process(commands["gazebo_truth_observer"])
        sim_log = open(case_dir / "sim.log", "w")
        files.append(sim_log)
        sim_proc = yawcase.start_process(commands["sim_control"], stdout=sim_log, stderr=subprocess.STDOUT)
        _wait_for_controllers()
        ready, detail = yawcase.wait_for_camera_topics()
        if not ready:
            raise RuntimeError(f"CAMERA_TOPIC_READINESS_FAILURE: {detail}")

        harness.remove_object()
        time.sleep(1.0)
        spawn_x, spawn_y = case["requested_spawn_xy_m"]
        yawcase.spawn_object_yaw(spawn_x, spawn_y, math.radians(case["requested_spawn_yaw_deg"]))
        settled, settle_detail = harness.settle_object()
        if not settled:
            raise RuntimeError(f"GROUND_TRUTH_SETTLE_FAILURE: {settle_detail}")
        gt_settled = harness.instantaneous_object_pose()
        if gt_settled is None:
            raise RuntimeError("MISSING_GROUND_TRUTH_EVALUATION_RECORD")
        (case_dir / "gt_settled_pose.json").write_text(json.dumps({
            "evaluation_only": True, "pose_xyz_quaternion_xyzw": gt_settled,
        }, indent=2) + "\n")

        detector_log = open(case_dir / "object_detector.log", "w")
        world_log = open(case_dir / "object_position_world.log", "w")
        files.extend((detector_log, world_log))
        detector_proc = yawcase.start_process(
            commands["object_detector"], stdout=detector_log, stderr=subprocess.STDOUT)
        world_proc = yawcase.start_process(
            commands["object_position_world"], stdout=world_log, stderr=subprocess.STDOUT)

        observation, failure = PairedObservationCollector().capture()
        if failure is not None:
            raise RuntimeError(failure)
        (case_dir / "paired_observation.json").write_text(json.dumps(observation, indent=2) + "\n")

        # The truth record is intentionally read only here, after all sensor
        # messages have been captured.  It cannot influence detector/TF/control.
        metrics = case_metrics(case, observation, gt_settled)
        (case_dir / "case_metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
        gt_final = harness.instantaneous_object_pose()
        if gt_final is None:
            raise RuntimeError("MISSING_FINAL_GROUND_TRUTH_EVALUATION_RECORD")
        (case_dir / "gt_final_pose.json").write_text(json.dumps({
            "evaluation_only": True, "pose_xyz_quaternion_xyzw": gt_final,
        }, indent=2) + "\n")
        (case_dir / "stage.capture_done").touch()
        return metrics
    except Exception as error:
        (case_dir / "failure.json").write_text(json.dumps({
            "case_id": case["case_id"], "failure": str(error),
            "ground_truth_evaluation_only": True, "wall_time": time.time(),
        }, indent=2) + "\n")
        raise
    finally:
        for proc in (world_proc, detector_proc, sim_proc, observer_proc):
            yawcase.stop_process(proc)
        for file_handle in files:
            file_handle.close()


def _sign(value, epsilon=1e-12):
    return "positive" if value > epsilon else "negative" if value < -epsilon else "zero"


def summarize_campaign(root, manifest):
    """Create the predeclared aggregate only when all 48 completed cases exist."""
    records = []
    failures = []
    for case in manifest["cases"]:
        case_dir = Path(root) / f"{case['random_execution_index']:02d}_{case['case_id']}"
        metric_path = case_dir / "case_metrics.json"
        failure_path = case_dir / "failure.json"
        if failure_path.exists():
            failures.append(json.loads(failure_path.read_text()))
        elif metric_path.exists():
            records.append((case, json.loads(metric_path.read_text())))
        else:
            failures.append({"case_id": case["case_id"], "failure": "MISSING_CASE_METRICS"})
    if failures:
        return {"campaign_complete": False, "detector_yaw_failures": failures,
                "expected_cases": 48, "completed_cases": len(records)}

    shadow_norm_worst = max(records, key=lambda item: item[1]["shadow"]["xy_norm_mm"])
    max_local = max(
        ((case, metrics, axis, abs(metrics["shadow"][axis]))
         for case, metrics in records
         for axis in ("closing_axis_residual_mm", "transverse_axis_residual_mm")),
        key=lambda item: item[3],
    )
    closing_values = [(case, metrics["shadow"]["closing_axis_residual_mm"])
                      for case, metrics in records]
    coverage = {}
    paired = {"improved": 0, "unchanged": 0, "worsened": 0, "mean_improvement_mm": 0.0}
    improvements = []
    for case, metrics in records:
        family = coverage.setdefault(case["yaw_family"], {
            "closing": {"positive": 0, "negative": 0, "zero": 0},
            "transverse": {"positive": 0, "negative": 0, "zero": 0},
        })
        family["closing"][_sign(metrics["shadow"]["closing_axis_residual_mm"])] += 1
        family["transverse"][_sign(metrics["shadow"]["transverse_axis_residual_mm"])] += 1
        improvement = metrics["paired_shadow_improvement_mm"]
        improvements.append(improvement)
        paired["improved" if improvement > 1e-12 else "worsened" if improvement < -1e-12 else "unchanged"] += 1
    paired["mean_improvement_mm"] = sum(improvements) / len(improvements)
    return {
        "campaign_complete": True,
        "cases": len(records),
        "worst_shadow_xy_norm": {"case_id": shadow_norm_worst[0]["case_id"],
                                  "mm": shadow_norm_worst[1]["shadow"]["xy_norm_mm"]},
        "Bxy_observed": {"case_id": max_local[0]["case_id"], "axis": max_local[2], "mm": max_local[3]},
        "worst_closing_axis_residual": {
            "maximum_signed_mm": max(closing_values, key=lambda item: item[1])[1],
            "minimum_signed_mm": min(closing_values, key=lambda item: item[1])[1],
            "maximum_absolute_mm": max(abs(value) for _, value in closing_values),
        },
        "residual_sign_coverage_by_yaw_family": coverage,
        "production_vs_shadow_paired_improvement": paired,
        "detector_yaw_failures": [],
    }


def run_campaign(gui=False):
    manifest = write_frozen_manifest(EVIDENCE_ROOT)
    try:
        for case in manifest["cases"]:
            run_case(EVIDENCE_ROOT, manifest, case, gui=gui)
    finally:
        # The failure list is itself a predeclared campaign output.  It is
        # written even for an aborted run, without retrying or changing cases.
        summary = summarize_campaign(EVIDENCE_ROOT, manifest)
        (EVIDENCE_ROOT / "campaign_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    if not summary["campaign_complete"]:
        raise RuntimeError("campaign did not complete; preserved evidence must be inspected")


def main():
    parser = argparse.ArgumentParser(
        description="Run the frozen 48-case 960x720 pixel-centre-shadow holdout. "
                    "Perception only; no MoveIt or manipulation.")
    parser.add_argument("--run-all", action="store_true",
                        help="Required explicit authorization to start all 48 frozen cases")
    parser.add_argument("--gui", action="store_true", help="Use Gazebo GUI (default: headless)")
    parser.add_argument("--print-manifest", action="store_true",
                        help="Print the frozen matrix/order without creating evidence or launching anything")
    args = parser.parse_args()
    if args.print_manifest:
        print(json.dumps(campaign_manifest(), indent=2))
        return
    if not args.run_all:
        parser.error("refusing to launch: pass --run-all only when the full campaign is authorized")
    run_campaign(gui=args.gui)


if __name__ == "__main__":
    main()

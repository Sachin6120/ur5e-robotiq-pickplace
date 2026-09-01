#!/usr/bin/env python3
"""Focused, ROS-free regression tests for Stage-2D planar-pose evidence handling.

Stage-2D adds a decoupled planar spawn OFFSET (X and Y) on top of Stage-2C's
already-independent yaw decoupling. These tests cover the pure-Python pieces
only: offset resolution/arithmetic, control_setup.json metadata, the two new
analyzer evidence-integrity gates, and that none of this changes Stage-2A/2C
behaviour when offsets are left at their zero default. No ROS, no Gazebo.
"""

import csv
import json
import math
from pathlib import Path
import sys
import tempfile

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts/perception"))
import run_stage2a_yaw_case as harness
import run_stage2d_pose_case as wrapper
import stage2a_analyzer as analyzer

CONFIGURED_PICK_XYZ = None  # filled in main() from config/scene.yaml


def q_from_yaw_deg(yaw_deg):
    return analyzer.rpy_to_quaternion(0.0, 0.0, math.radians(yaw_deg))


def load_base_pick_xy():
    import yaml

    with open(REPO / "config/scene.yaml", "r") as f:
        scene = yaml.safe_load(f)
    return [float(scene["object"]["pick_pose"]["x"]), float(scene["object"]["pick_pose"]["y"])]


def write_case_scene(case_dir, pick_xy):
    (case_dir / "scene_case.yaml").write_text(
        f"object:\n  pick_pose:\n    x: {pick_xy[0]}\n    y: {pick_xy[1]}\n"
    )


def write_minimal_evidence(case_dir, *, spawn_xy, spawn_yaw_deg=0.0):
    (case_dir / "m3_grasp.log").write_text("")
    with open(case_dir / "init_settled_pose.json", "w") as f:
        json.dump([spawn_xy[0], spawn_xy[1], 0.7725, *q_from_yaw_deg(spawn_yaw_deg)], f)
    fields = [
        "result",
        "yaw_source",
        "configured_object_yaw_deg",
        "perceived_object_yaw_deg",
        "yaw_delta_deg",
        "commanded_grasp_yaw_deg",
    ]
    with open(case_dir / "m3_grasp.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerow(
            {
                "result": "SUCCESS",
                "yaw_source": "perceived",
                "configured_object_yaw_deg": "0",
                "perceived_object_yaw_deg": str(spawn_yaw_deg),
                "yaw_delta_deg": "0",
                "commanded_grasp_yaw_deg": "0",
            }
        )


def test_zero_offset_defaults_preserve_stage2a_2c_spawn_behaviour():
    # No offset requested -> spawn XY must equal configured pick XY exactly,
    # byte-for-byte with Stage-2A/2C's prior (pick_x, pick_y) spawn call.
    spawn_x, spawn_y = harness.compute_spawn_xy(0.450, -0.150, 0.0, 0.0)
    assert (spawn_x, spawn_y) == (0.450, -0.150)


def test_spawn_offset_resolution_validates_and_defaults():
    assert harness.resolve_spawn_offsets(0.0, 0.0) == (0.0, 0.0)
    assert harness.resolve_spawn_offsets(0.03, -0.03) == (0.03, -0.03)
    try:
        harness.resolve_spawn_offsets(float("nan"), 0.0)
        assert False, "expected ValueError for a non-finite spawn offset"
    except ValueError:
        pass


def test_requested_spawn_xy_is_independently_offset_from_configured():
    pick_x, pick_y = 0.450, -0.150
    spawn_x, spawn_y = harness.compute_spawn_xy(pick_x, pick_y, 0.030, 0.030)
    assert math.isclose(spawn_x, 0.480, abs_tol=1e-9)
    assert math.isclose(spawn_y, -0.120, abs_tol=1e-9)
    # Configured pick XY itself must be untouched by the offset computation.
    assert (pick_x, pick_y) == (0.450, -0.150)
    # An asymmetric/negative offset is independently honoured on each axis.
    spawn_x2, spawn_y2 = harness.compute_spawn_xy(pick_x, pick_y, -0.030, 0.0)
    assert math.isclose(spawn_x2, 0.420, abs_tol=1e-9)
    assert math.isclose(spawn_y2, -0.150, abs_tol=1e-9)


def test_control_setup_records_spawn_offset_metadata():
    configured_xy = [0.450, -0.150]
    spawn_offset_x_m, spawn_offset_y_m = 0.030, 0.030
    spawn_request_xy = list(
        harness.compute_spawn_xy(configured_xy[0], configured_xy[1], spawn_offset_x_m, spawn_offset_y_m)
    )
    metadata = harness.make_control_setup(
        case_name="D1",
        configured_pick_yaw_deg=0.0,
        spawned_yaw_deg=30.0,
        target_place_yaw_deg=0.0,
        use_perceived_position=True,
        use_perceived_yaw=True,
        target_source="perceived",
        record_diagnostics=False,
        fixed_side_clearance_m=None,
        close_and_hold_only=False,
        lift_only=False,
        p_gain_override=None,
        configured_object_centre_world=[*configured_xy, 0.7725],
        case_dir=Path("/tmp/D1"),
        configured_pick_xy_m=configured_xy,
        spawn_request_xy_m=spawn_request_xy,
        spawn_offset_request_mm=[spawn_offset_x_m * 1000.0, spawn_offset_y_m * 1000.0],
    )
    assert metadata["configured_pick_xy_m"] == [0.450, -0.150]
    assert metadata["spawn_request_xy_m"] == [0.480, -0.120]
    assert metadata["spawn_offset_request_mm"] == [30.0, 30.0]
    # Measured fields are unknown until the object actually settles in Gazebo.
    assert metadata["measured_spawned_xy_m"] is None
    assert metadata["measured_spawn_offset_from_configured_mm"] is None
    # Stage-2C's yaw metadata must be untouched by the Stage-2D XY additions.
    assert metadata["spawn_request_yaw_deg"] == 30.0
    assert metadata["configured_pick_yaw_deg"] == 0.0


def test_measured_spawn_offset_metadata_matches_ground_truth():
    configured_xy = [0.450, -0.150]
    measured_xy = [0.4796, -0.1205]  # a settled pose close to a +30/+30 mm request
    offset_mm = harness.compute_measured_spawn_offset_mm(measured_xy, configured_xy)
    assert math.isclose(offset_mm[0], (0.4796 - 0.450) * 1000.0, abs_tol=1e-9)
    assert math.isclose(offset_mm[1], (-0.1205 - (-0.150)) * 1000.0, abs_tol=1e-9)
    assert math.isclose(offset_mm[0], 29.6, abs_tol=1e-6)
    assert math.isclose(offset_mm[1], 29.5, abs_tol=1e-6)


def test_translation_decoupled_gate_at_or_above_20mm():
    base_xy = load_base_pick_xy()
    with tempfile.TemporaryDirectory() as temp_dir:
        case_dir = Path(temp_dir)
        write_case_scene(case_dir, base_xy)
        spawn_xy = [base_xy[0] + 0.030, base_xy[1] + 0.030]  # 42.4 mm combined offset
        write_minimal_evidence(case_dir, spawn_xy=spawn_xy, spawn_yaw_deg=30.0)
        metrics = analyzer.analyze_case(
            case_dir,
            configured_yaw_deg=0.0,
            target_place_xyz=[0.45, 0.2, 0.7725],
            target_place_yaw_deg=0.0,
            use_axial_placement_yaw=True,
            require_translation_decoupling=True,
        )
        assert metrics["translation_decoupled"] is True
        assert metrics["configured_pose_unchanged"] is True
        assert metrics["gates"]["translation_decoupled"] is True
        assert metrics["gates"]["configured_pose_unchanged"] is True
        offset = metrics["measured_spawn_offset_mm"]
        assert math.hypot(*offset) >= analyzer.GATES["translation_decoupled_mm_min"]


def test_translation_decoupled_gate_fails_below_20mm():
    base_xy = load_base_pick_xy()
    with tempfile.TemporaryDirectory() as temp_dir:
        case_dir = Path(temp_dir)
        write_case_scene(case_dir, base_xy)
        # A 5 mm settle jitter is not a real decoupled offset.
        spawn_xy = [base_xy[0] + 0.005, base_xy[1]]
        write_minimal_evidence(case_dir, spawn_xy=spawn_xy, spawn_yaw_deg=0.0)
        metrics = analyzer.analyze_case(
            case_dir,
            configured_yaw_deg=0.0,
            target_place_xyz=[0.45, 0.2, 0.7725],
            target_place_yaw_deg=0.0,
            use_axial_placement_yaw=True,
            require_translation_decoupling=True,
        )
        assert metrics["translation_decoupled"] is False
        assert metrics["gates"]["translation_decoupled"] is False
        assert metrics["verdict"] == "FAIL"


def test_configured_pose_unchanged_gate_detects_drift():
    base_xy = load_base_pick_xy()
    with tempfile.TemporaryDirectory() as temp_dir:
        case_dir = Path(temp_dir)
        # Simulate a case scene whose configured pick XY drifted from
        # config/scene.yaml -- exactly what this gate exists to catch.
        drifted_xy = [base_xy[0] + 0.100, base_xy[1]]
        write_case_scene(case_dir, drifted_xy)
        write_minimal_evidence(case_dir, spawn_xy=[drifted_xy[0] + 0.030, drifted_xy[1] + 0.030])
        metrics = analyzer.analyze_case(
            case_dir,
            configured_yaw_deg=0.0,
            target_place_xyz=[0.45, 0.2, 0.7725],
            target_place_yaw_deg=0.0,
            use_axial_placement_yaw=True,
            require_translation_decoupling=True,
        )
        assert metrics["configured_pose_unchanged"] is False
        assert metrics["gates"]["configured_pose_unchanged"] is False


def test_stage2c_scoring_unaffected_by_stage2d_gates():
    # Default require_translation_decoupling=False (Stage-2A/2C's own call
    # sites never pass it) must not add the new gate keys at all, and must
    # not change any existing gate's pass/fail outcome.
    base_xy = load_base_pick_xy()
    with tempfile.TemporaryDirectory() as temp_dir:
        case_dir = Path(temp_dir)
        write_case_scene(case_dir, base_xy)
        write_minimal_evidence(case_dir, spawn_xy=base_xy, spawn_yaw_deg=30.0)
        metrics = analyzer.analyze_case(
            case_dir,
            configured_yaw_deg=0.0,
            target_place_xyz=[0.45, 0.2, 0.7725],
            target_place_yaw_deg=0.0,
            require_perceived_yaw=True,
            use_axial_placement_yaw=True,
        )
        assert "translation_decoupled" not in metrics["gates"]
        assert "configured_pose_unchanged" not in metrics["gates"]
        # The evidence-integrity fields are still computed and reported
        # (evidence-only), just never gate the verdict unless requested.
        assert metrics["translation_decoupled"] is False
        assert metrics["configured_pose_unchanged"] is True


def test_combined_xy_and_yaw_inputs_remain_independent():
    # A large XY offset must not perturb yaw resolution, and a large yaw
    # request must not perturb the XY offset -- verified together since
    # Stage-2D's whole point is exercising both at once.
    configured, spawned, place = harness.resolve_yaw_inputs(0.0, 0.0, 45.0, 0.0)
    assert (configured, spawned, place) == (0.0, 45.0, 0.0)
    spawn_offset_x_m, spawn_offset_y_m = harness.resolve_spawn_offsets(0.030, -0.030)
    assert (spawn_offset_x_m, spawn_offset_y_m) == (0.030, -0.030)
    spawn_x, spawn_y = harness.compute_spawn_xy(0.450, -0.150, spawn_offset_x_m, spawn_offset_y_m)
    assert math.isclose(spawn_x, 0.480, abs_tol=1e-9)
    assert math.isclose(spawn_y, -0.180, abs_tol=1e-9)
    # Yaw values are untouched by the XY computation and vice versa.
    assert spawned == 45.0


def test_stage2d_wrapper_forwards_offsets_and_keeps_yaw_independent():
    captured = {}

    def fake_run_case(case_name, yaw_deg, **kwargs):
        captured["case_name"] = case_name
        captured["yaw_deg"] = yaw_deg
        captured.update(kwargs)

    original_run_case = wrapper.harness.run_case
    original_argv = sys.argv
    try:
        wrapper.harness.run_case = fake_run_case
        sys.argv = [
            "run_stage2d_pose_case.py",
            "--case", "D1",
            "--spawn-offset-x-m", "0.030",
            "--spawn-offset-y-m", "0.030",
            "--spawned-yaw-deg", "30.0",
        ]
        wrapper.main()
    finally:
        wrapper.harness.run_case = original_run_case
        sys.argv = original_argv

    assert captured["spawn_offset_x_m"] == 0.030
    assert captured["spawn_offset_y_m"] == 0.030
    assert captured["spawned_yaw_deg"] == 30.0
    assert captured["configured_pick_yaw_deg"] == 0.0
    assert captured["use_perceived_yaw"] is True
    assert captured["evidence_root"] == "evidence/stage2d_pose"
    assert captured["stage2c_mode"] is True
    assert captured["stage2d_mode"] is True


def test_stage2d_wrapper_configured_target_source_forces_yaw_off():
    captured = {}

    def fake_run_case(case_name, yaw_deg, **kwargs):
        captured.update(kwargs)

    original_run_case = wrapper.harness.run_case
    original_argv = sys.argv
    try:
        wrapper.harness.run_case = fake_run_case
        sys.argv = [
            "run_stage2d_pose_case.py",
            "--case", "D_control",
            "--target-source", "configured",
            "--use-perceived-yaw",
        ]
        wrapper.main()
    finally:
        wrapper.harness.run_case = original_run_case
        sys.argv = original_argv

    # use_perceived_yaw requires a perceived position (m3_grasp.cpp's
    # perceived_yaw_configuration_valid); the wrapper must not forward a
    # combination the node itself would reject as CONFIG_ERROR.
    assert captured["target_source"] == "configured"
    assert captured["use_perceived_yaw"] is False


def main():
    test_zero_offset_defaults_preserve_stage2a_2c_spawn_behaviour()
    test_spawn_offset_resolution_validates_and_defaults()
    test_requested_spawn_xy_is_independently_offset_from_configured()
    test_control_setup_records_spawn_offset_metadata()
    test_measured_spawn_offset_metadata_matches_ground_truth()
    test_translation_decoupled_gate_at_or_above_20mm()
    test_translation_decoupled_gate_fails_below_20mm()
    test_configured_pose_unchanged_gate_detects_drift()
    test_stage2c_scoring_unaffected_by_stage2d_gates()
    test_combined_xy_and_yaw_inputs_remain_independent()
    test_stage2d_wrapper_forwards_offsets_and_keeps_yaw_independent()
    test_stage2d_wrapper_configured_target_source_forces_yaw_off()
    print("Stage-2D pose evidence tests passed")


if __name__ == "__main__":
    main()

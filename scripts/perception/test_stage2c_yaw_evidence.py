#!/usr/bin/env python3
"""Focused, ROS-free regression tests for Stage-2C yaw evidence handling."""

import csv
import json
import math
from pathlib import Path
import sys
import tempfile

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts/perception"))
import run_stage2a_yaw_case as harness
import stage2a_analyzer as analyzer


def q_from_yaw_deg(yaw_deg):
    return analyzer.rpy_to_quaternion(0.0, 0.0, math.radians(yaw_deg))


def write_evidence(case_dir, *, spawned_yaw_deg, perceived_yaw_deg, final_yaw_deg):
    (case_dir / "m3_grasp.log").write_text("")
    with open(case_dir / "init_settled_pose.json", "w") as f:
        json.dump([0.4, 0.0, 0.75, *q_from_yaw_deg(spawned_yaw_deg)], f)
    with open(case_dir / "final_settled_pose.json", "w") as f:
        json.dump([0.45, 0.2, 0.7725, *q_from_yaw_deg(final_yaw_deg)], f)
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
                "perceived_object_yaw_deg": str(perceived_yaw_deg),
                "yaw_delta_deg": "30",
                "commanded_grasp_yaw_deg": "-60",
            }
        )


def test_decoupled_inputs_and_metadata():
    configured, spawned, place = harness.resolve_yaw_inputs(0.0, 0.0, 30.0, 0.0)
    assert (configured, spawned, place) == (0.0, 30.0, 0.0)
    metadata = harness.make_control_setup(
        case_name="C1",
        configured_pick_yaw_deg=configured,
        spawned_yaw_deg=spawned,
        target_place_yaw_deg=place,
        use_perceived_position=True,
        use_perceived_yaw=True,
        target_source="perceived",
        record_diagnostics=False,
        fixed_side_clearance_m=None,
        close_and_hold_only=False,
        lift_only=False,
        p_gain_override=None,
        configured_object_centre_world=[0.4, 0.0, 0.75],
        case_dir=Path("/tmp/C1"),
    )
    assert metadata["configured_pick_yaw_deg"] == 0.0
    assert metadata["spawn_request_yaw_deg"] == 30.0
    assert metadata["measured_spawned_yaw_deg"] is None
    assert metadata["target_place_yaw_deg"] == 0.0
    assert metadata["use_perceived_position"] is True
    assert metadata["use_perceived_yaw"] is True


def test_yaw_forwarding():
    command = harness.build_m3_command(
        case_scene_path=Path("/tmp/scene_case.yaml"),
        csv_file=Path("/tmp/m3.csv"),
        marker_prefix=Path("/tmp/stage"),
        use_perceived_position=True,
        use_perceived_yaw=True,
    )
    assert "use_perceived_position:=true" in command
    assert "use_perceived_yaw:=true" in command


def test_axial_equivalence():
    assert analyzer.axial_error_deg(math.radians(0.0), math.radians(180.0)) < 1e-9
    assert analyzer.axial_error_deg(math.radians(30.0), math.radians(210.0)) < 1e-9


def test_metrics_use_spawned_yaw_and_propagate_telemetry():
    with tempfile.TemporaryDirectory() as temp_dir:
        case_dir = Path(temp_dir)
        # Configured is deliberately 0 while physical GT and perception are +30.
        write_evidence(
            case_dir,
            spawned_yaw_deg=30.0,
            perceived_yaw_deg=30.0,
            final_yaw_deg=180.0,
        )
        metrics = analyzer.analyze_case(
            case_dir,
            configured_yaw_deg=0.0,
            target_place_xyz=[0.45, 0.2, 0.7725],
            target_place_yaw_deg=0.0,
            require_perceived_yaw=True,
            use_axial_placement_yaw=True,
        )
        # This must be zero against spawned +30, rather than 30 against config 0.
        assert metrics["perceived_yaw_err_deg"] is not None
        assert metrics["perceived_yaw_err_deg"] < 1e-9
        assert metrics["yaw_source"] == "perceived"
        assert metrics["configured_object_yaw_deg"] == 0.0
        assert metrics["perceived_object_yaw_deg"] == 30.0
        assert metrics["yaw_delta_deg"] == 30.0
        assert metrics["commanded_grasp_yaw_deg"] == -60.0
        # A 30 x 45 rectangle at yaw 180 is axially aligned with place yaw 0.
        assert metrics["placement_yaw_err_deg"] < 1e-9
        assert metrics["placement_orient_err_deg"] > 179.9
        assert metrics["final_upright_tilt_deg"] < 1e-9
        persisted = json.loads((case_dir / "cycle_metrics.json").read_text())
        for field in (
            "yaw_source",
            "configured_object_yaw_deg",
            "perceived_object_yaw_deg",
            "yaw_delta_deg",
            "commanded_grasp_yaw_deg",
        ):
            assert persisted[field] == metrics[field]


def main():
    test_decoupled_inputs_and_metadata()
    test_yaw_forwarding()
    test_axial_equivalence()
    test_metrics_use_spawned_yaw_and_propagate_telemetry()
    print("Stage-2C yaw evidence tests passed")


if __name__ == "__main__":
    main()

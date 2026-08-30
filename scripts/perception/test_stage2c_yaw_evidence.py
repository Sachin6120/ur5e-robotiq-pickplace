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


def test_upright_tilt_is_yaw_invariant():
    def tilt(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
        return analyzer.quaternion_upright_tilt_deg(
            analyzer.rpy_to_quaternion(
                math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
            )
        )

    assert tilt() < 1e-9
    assert tilt(yaw_deg=30.0) < 1e-9
    assert tilt(yaw_deg=90.0) < 1e-9
    assert abs(tilt(roll_deg=2.0) - 2.0) < 1e-9
    assert abs(tilt(pitch_deg=2.0) - 2.0) < 1e-9
    assert abs(tilt(roll_deg=2.0, yaw_deg=30.0) - 2.0) < 1e-9
    assert abs(tilt(pitch_deg=2.0, yaw_deg=90.0) - 2.0) < 1e-9


def test_stage2c_uses_upright_tilt_but_retains_full_orientation_diagnostic():
    with tempfile.TemporaryDirectory() as temp_dir:
        case_dir = Path(temp_dir)
        write_evidence(
            case_dir,
            spawned_yaw_deg=30.0,
            perceived_yaw_deg=30.0,
            final_yaw_deg=0.0,
        )
        fields = [
            "wall_ns",
            *(f"pick_target_{axis}" for axis in ("x", "y", "z", "qx", "qy", "qz", "qw")),
            *(f"wrist_3_link_{axis}" for axis in ("x", "y", "z", "qx", "qy", "qz", "qw")),
        ]
        with open(case_dir / "gz_pose_stream.csv", "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for index in range(53):
                t = 100.2 + index * 0.1
                sample_q = q_from_yaw_deg(0.0 if t < 101.0 else 30.0)
                writer.writerow(
                    {
                        "wall_ns": int(t * 1e9),
                        **dict(zip((f"pick_target_{a}" for a in ("x", "y", "z")), (0.4, 0.0, 0.75))),
                        **dict(zip((f"pick_target_{a}" for a in ("qx", "qy", "qz", "qw")), sample_q)),
                        **dict(zip((f"wrist_3_link_{a}" for a in ("x", "y", "z")), (0.0, 0.0, 0.0))),
                        **dict(zip((f"wrist_3_link_{a}" for a in ("qx", "qy", "qz", "qw")), (0.0, 0.0, 0.0, 1.0))),
                    }
        )
        (case_dir / "m3_grasp.log").write_text(
            "[0000000101.000] M3 STAGE 3 LIFT_BEGIN cycle=0 sim=0 t=1.000\n"
            "[0000000102.000] M3 STAGE 3 LIFT_DONE cycle=0 sim=0 t=2.000\n"
            "[0000000104.000] M3 STAGE 4 TRANSPORT_DONE cycle=0 sim=0 t=4.000\n"
        )
        metrics = analyzer.analyze_case(
            case_dir,
            configured_yaw_deg=0.0,
            target_place_xyz=[0.45, 0.2, 0.7725],
            target_place_yaw_deg=0.0,
            use_axial_placement_yaw=True,
        )
        assert metrics["max_grasp_tilt_deg"] < 1e-9
        assert metrics["max_grasp_orientation_change_deg"] > 28.0


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
    test_upright_tilt_is_yaw_invariant()
    test_stage2c_uses_upright_tilt_but_retains_full_orientation_diagnostic()
    test_metrics_use_spawned_yaw_and_propagate_telemetry()
    print("Stage-2C yaw evidence tests passed")


if __name__ == "__main__":
    main()

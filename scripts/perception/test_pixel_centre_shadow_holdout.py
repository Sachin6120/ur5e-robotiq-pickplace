#!/usr/bin/env python3
"""Offline guards for the frozen 960x720 pixel-centre-shadow holdout."""
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock


SCRIPT = Path(__file__).with_name("run_pixel_centre_shadow_holdout.py")
SPEC = importlib.util.spec_from_file_location("pixel_centre_shadow_holdout", SCRIPT)
holdout = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(holdout)


def valid_observation():
    stamp = {"sec": 42, "nanosec": 100, "seconds": 42.0000001, "frame_id": "world"}
    camera_stamp = dict(stamp, frame_id="camera_optical_frame")
    point = lambda frame: {"stamp": dict(stamp, frame_id=frame), "xyz": [0.1, -0.2, 1.605]}
    observation = {
        "camera_info": {
            "stamp": camera_stamp,
            "width": holdout.CAMERA_WIDTH,
            "height": holdout.CAMERA_HEIGHT,
            "K": [831.574069234, 0.0, 480.0, 0.0, 831.574069234, 360.0, 0.0, 0.0, 1.0],
        },
        "raw_centroid": point("camera_optical_frame"),
        "production_camera": point("camera_optical_frame"),
        "shadow_camera": point("camera_optical_frame"),
        "production_world": point("world"),
        "shadow_world": point("world"),
        "perceived_pose_world": {
            "stamp": dict(stamp), "xyz": [0.1, -0.2, 0.795],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0], "yaw_rad": 0.0, "yaw_deg": 0.0,
        },
        "sample_key": {"sec": 42, "nanosec": 100},
    }
    observation["raw_centroid"]["uv"] = [570.0, 359.5]
    observation["corrected_centroid"] = {
        "stamp": dict(camera_stamp), "uv": [570.5, 360.0], "correction_px": [0.5, 0.5],
    }
    observation["d10_depth_m"] = {"production": 1.605, "shadow": 1.605, "identical": True}
    return observation


class FrozenMatrixTest(unittest.TestCase):
    def test_frozen_matrix_has_exactly_the_requested_12_times_4_cases(self):
        cases = holdout.build_frozen_cases()
        self.assertEqual(len(cases), 48)
        self.assertEqual([case["random_execution_index"] for case in cases], list(range(1, 49)))
        self.assertEqual(len({case["case_id"] for case in cases}), 48)
        self.assertEqual({case["base_id"] for case in cases}, {base[0] for base in holdout.BASE_CONFIGURATIONS})
        self.assertEqual({case["phase_id"] for case in cases}, {"pp", "pm", "mp", "mm"})

    def test_phase_offsets_are_the_four_unseen_signed_quarter_pixel_offsets(self):
        expected = {
            (+holdout.PHASE_OFFSET_M, +holdout.PHASE_OFFSET_M),
            (+holdout.PHASE_OFFSET_M, -holdout.PHASE_OFFSET_M),
            (-holdout.PHASE_OFFSET_M, +holdout.PHASE_OFFSET_M),
            (-holdout.PHASE_OFFSET_M, -holdout.PHASE_OFFSET_M),
        }
        self.assertEqual({(dx, dy) for _, dx, dy in holdout.PHASES}, expected)
        self.assertEqual(holdout.PIXEL_PITCH_M, 0.001930075)
        self.assertAlmostEqual(holdout.PHASE_OFFSET_M * 1000.0, 0.48251875, places=8)

    def test_random_order_is_seeded_frozen_and_persisted_in_manifest(self):
        canonical = [f"{base[0]}__phase_{phase[0]}"
                     for base in holdout.BASE_CONFIGURATIONS for phase in holdout.PHASES]
        generated = list(canonical)
        holdout.random.Random(holdout.RANDOMIZATION_SEED).shuffle(generated)
        self.assertEqual(tuple(generated), holdout.FROZEN_EXECUTION_ORDER)
        manifest = holdout.campaign_manifest()
        self.assertEqual(manifest["campaign"], "pixel_centre_shadow_holdout_960x720_clean_slate_v2")
        self.assertEqual(manifest["lifecycle_contract_revision"], "post_case_clean_slate_v1")
        self.assertEqual(holdout.RANDOMIZATION_SEED, 20260831)
        self.assertEqual(manifest["frozen_execution_order"], list(holdout.FROZEN_EXECUTION_ORDER))
        self.assertTrue(manifest["safety_contract"]["no_posthoc_matrix_change"])
        self.assertTrue(manifest["safety_contract"]["post_case_clean_slate_required"])
        self.assertEqual(manifest["frozen_manifest_sha256"],
                         holdout.sha256_json({key: value for key, value in manifest.items()
                                              if key != "frozen_manifest_sha256"}))
        self.assertNotEqual(
            manifest["frozen_manifest_sha256"],
            "4163e16283fdc850df905a51249d3d56b446bd9218e780c577bd5a566f48f4cf",
        )


class EvidenceGateTest(unittest.TestCase):
    def test_valid_same_stamp_960_observation_is_accepted(self):
        observation, error = holdout.validate_paired_observation(valid_observation())
        self.assertIsNone(error)
        self.assertIsNotNone(observation)

    def test_resolution_camera_info_and_timestamp_gates_reject_invalid_data(self):
        bad_resolution = valid_observation()
        bad_resolution["camera_info"]["width"] = 2880
        _, error = holdout.validate_paired_observation(bad_resolution)
        self.assertEqual(error, "RUNTIME_RESOLUTION_REJECTED")

        bad_k = valid_observation()
        bad_k["camera_info"]["K"] = [math.nan] * 9
        _, error = holdout.validate_paired_observation(bad_k)
        self.assertEqual(error, "CAMERAINFO_REJECTED")

        bad_stamp = valid_observation()
        bad_stamp["shadow_world"]["stamp"]["nanosec"] += 1
        _, error = holdout.validate_paired_observation(bad_stamp)
        self.assertEqual(error, "TIMESTAMP_MISMATCH:shadow_world")

        bad_correction = valid_observation()
        bad_correction["corrected_centroid"]["uv"][0] += 0.1
        _, error = holdout.validate_paired_observation(bad_correction)
        self.assertEqual(error, "PIXEL_CENTRE_CORRECTION_MISMATCH")

        bad_d10 = valid_observation()
        bad_d10["d10_depth_m"]["shadow"] += 0.001
        bad_d10["d10_depth_m"]["identical"] = False
        _, error = holdout.validate_paired_observation(bad_d10)
        self.assertEqual(error, "D10_MISMATCH")

    def test_local_residuals_and_case_metrics_are_evaluation_only(self):
        residual = holdout.local_residuals_mm([0.451, -0.149, 0.795], [0.450, -0.150, 0.7725], 0.0)
        self.assertAlmostEqual(residual["closing_axis_residual_mm"], 1.0, places=9)
        self.assertAlmostEqual(residual["transverse_axis_residual_mm"], 1.0, places=9)
        case = holdout.build_frozen_cases()[0]
        metrics = holdout.case_metrics(case, valid_observation(), [0.1, -0.2, 0.7725, 0.0, 0.0, 0.0, 1.0])
        self.assertTrue(metrics["ground_truth_evaluation_only"])
        self.assertIn("production", metrics)
        self.assertIn("shadow", metrics)
        with self.assertRaisesRegex(ValueError, "MISSING_GROUND_TRUTH"):
            holdout.case_metrics(case, valid_observation(), None)


class FakeClock:
    def __init__(self):
        self.now = 0.0
        self.sleeps = []

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.now += seconds


class CleanSlateSynchronizationTest(unittest.TestCase):
    def _clock_patches(self, clock):
        return (
            mock.patch.object(holdout.time, "monotonic", side_effect=clock.monotonic),
            mock.patch.object(holdout.time, "sleep", side_effect=clock.sleep),
        )

    def test_transient_descendant_disappears_and_passes(self):
        clock = FakeClock()
        matcher = mock.Mock(side_effect=[["43277"], ["43277"], []])
        monotonic, sleep = self._clock_patches(clock)
        with monotonic, sleep, mock.patch.object(holdout, "_matching_contamination_pids", matcher):
            holdout._wait_for_clean_slate(timeout_s=1.0, poll_s=0.05)
        self.assertEqual(matcher.call_count, 3)
        self.assertEqual(clock.sleeps, [0.05, 0.05])

    def test_persistent_contamination_times_out_with_full_diagnostics(self):
        clock = FakeClock()
        diagnostic = [{
            "pid": 43277, "ppid": 43204, "pgid": 43204,
            "state": "Sl", "command_line": "robot_state_publisher --ros-args",
        }]
        monotonic, sleep = self._clock_patches(clock)
        with (
            monotonic,
            sleep,
            mock.patch.object(holdout, "_matching_contamination_pids", return_value=["43277"]),
            mock.patch.object(holdout, "_process_diagnostics", return_value=diagnostic),
        ):
            with self.assertRaises(holdout.CleanSlateTimeout) as raised:
                holdout._wait_for_clean_slate(timeout_s=0.10, poll_s=0.05)
        self.assertEqual(raised.exception.diagnostics, diagnostic)
        self.assertIn("CLEAN_SLATE_TIMEOUT: PIDs 43277", str(raised.exception))
        self.assertEqual(clock.sleeps, [0.05, 0.05])

    def test_already_clean_environment_passes_without_sleep(self):
        clock = FakeClock()
        matcher = mock.Mock(return_value=[])
        monotonic, sleep = self._clock_patches(clock)
        with monotonic, sleep, mock.patch.object(holdout, "_matching_contamination_pids", matcher):
            holdout._wait_for_clean_slate(timeout_s=1.0, poll_s=0.05)
        matcher.assert_called_once_with()
        self.assertEqual(clock.sleeps, [])

    def test_wait_never_kills_a_matched_or_unrelated_process(self):
        clock = FakeClock()
        monotonic, sleep = self._clock_patches(clock)
        with (
            monotonic,
            sleep,
            mock.patch.object(holdout, "_matching_contamination_pids", return_value=["99999"]),
            mock.patch.object(holdout, "_process_diagnostics", return_value=[]),
            mock.patch.object(holdout.os, "killpg", side_effect=AssertionError("must not kill")),
            mock.patch.object(holdout.yawcase, "stop_process", side_effect=AssertionError("must not stop")),
        ):
            with self.assertRaises(holdout.CleanSlateTimeout):
                holdout._wait_for_clean_slate(timeout_s=0.0, poll_s=0.05)

    def test_process_snapshot_records_requested_fields(self):
        completed = subprocess.CompletedProcess(
            args=["ps"], returncode=0,
            stdout="43277 43204 43204 Sl robot_state_publisher --ros-args\n",
            stderr="",
        )
        with mock.patch.object(holdout.subprocess, "run", return_value=completed) as run:
            diagnostics = holdout._process_diagnostics(["43277"])
        self.assertEqual(diagnostics, [{
            "pid": 43277, "ppid": 43204, "pgid": 43204,
            "state": "Sl", "command_line": "robot_state_publisher --ros-args",
        }])
        self.assertEqual(run.call_args.args[0][:4], ["ps", "-ww", "-o", "pid=,ppid=,pgid=,stat=,args="])

    def test_timeout_failure_record_preserves_process_diagnostics(self):
        diagnostic = [{
            "pid": 43277, "ppid": 43204, "pgid": 43204,
            "state": "Sl", "command_line": "robot_state_publisher --ros-args",
        }]
        with tempfile.TemporaryDirectory() as temporary:
            case_dir = Path(temporary)
            holdout._write_failure(
                case_dir,
                "case-id",
                holdout.CleanSlateTimeout(diagnostic),
                lifecycle_contract_revision=holdout.LIFECYCLE_CONTRACT_REVISION,
                clean_slate_timeout_s=holdout.CLEAN_SLATE_TIMEOUT_S,
                contamination_processes=diagnostic,
            )
            record = json.loads((case_dir / "failure.json").read_text())
        self.assertEqual(record["contamination_processes"], diagnostic)
        self.assertEqual(record["contamination_processes"][0]["command_line"],
                         "robot_state_publisher --ros-args")


if __name__ == "__main__":
    unittest.main()

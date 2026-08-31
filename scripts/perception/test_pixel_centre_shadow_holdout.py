#!/usr/bin/env python3
"""Offline guards for the frozen 960x720 pixel-centre-shadow holdout."""
import importlib.util
import math
from pathlib import Path
import unittest


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
        self.assertAlmostEqual(holdout.PHASE_OFFSET_M * 1000.0, 0.48251875, places=8)

    def test_random_order_is_seeded_frozen_and_persisted_in_manifest(self):
        canonical = [f"{base[0]}__phase_{phase[0]}"
                     for base in holdout.BASE_CONFIGURATIONS for phase in holdout.PHASES]
        generated = list(canonical)
        holdout.random.Random(holdout.RANDOMIZATION_SEED).shuffle(generated)
        self.assertEqual(tuple(generated), holdout.FROZEN_EXECUTION_ORDER)
        manifest = holdout.campaign_manifest()
        self.assertEqual(manifest["frozen_execution_order"], list(holdout.FROZEN_EXECUTION_ORDER))
        self.assertTrue(manifest["safety_contract"]["no_posthoc_matrix_change"])
        self.assertEqual(manifest["frozen_manifest_sha256"],
                         holdout.sha256_json({key: value for key, value in manifest.items()
                                              if key != "frozen_manifest_sha256"}))


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


if __name__ == "__main__":
    unittest.main()

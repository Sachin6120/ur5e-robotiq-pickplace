#!/usr/bin/env python3
"""Run one Stage-2C perceived-yaw evidence case.

The pick TF yaw, physical spawn yaw, and placement yaw are intentionally
separate controls.  This wrapper never modifies config/scene.yaml: it asks the
shared runner to write an isolated case scene under the evidence directory.
"""

import argparse
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts/perception"))
import run_stage2a_yaw_case as harness


def main():
    parser = argparse.ArgumentParser(description="Run one Stage-2C yaw evidence case.")
    parser.add_argument("--case", required=True, help="Evidence case identifier")
    parser.add_argument(
        "--configured-pick-yaw-deg",
        type=float,
        default=0.0,
        help="Configured object-frame yaw written to the generated case scene (default: 0)",
    )
    parser.add_argument(
        "--spawned-yaw-deg",
        type=float,
        default=30.0,
        help="Physical object spawn yaw only (default: +30)",
    )
    parser.add_argument(
        "--target-place-yaw-deg",
        type=float,
        default=0.0,
        help="Configured placement target yaw written to the generated case scene (default: 0)",
    )
    yaw_group = parser.add_mutually_exclusive_group()
    yaw_group.add_argument(
        "--use-perceived-yaw",
        dest="use_perceived_yaw",
        action="store_true",
        help="Require fresh pose_world yaw and forward use_perceived_yaw:=true (default)",
    )
    yaw_group.add_argument(
        "--no-use-perceived-yaw",
        dest="use_perceived_yaw",
        action="store_false",
        help="Comparison control: keep perceived XYZ but disable yaw consumption",
    )
    parser.set_defaults(use_perceived_yaw=True)
    parser.add_argument("--gui", action="store_true", help="Run Gazebo with GUI (default: headless)")
    parser.add_argument(
        "--evidence-dir",
        default=None,
        help="Explicit evidence directory (default: evidence/stage2c_orientation/<case>)",
    )
    args = parser.parse_args()

    harness.run_case(
        args.case,
        # Preserve the shared runner's legacy positional input for compatibility;
        # all Stage-2C yaw behavior below comes from explicit named inputs.
        args.configured_pick_yaw_deg,
        gazebo_gui=args.gui,
        evidence_dir=args.evidence_dir,
        target_source="perceived",
        configured_pick_yaw_deg=args.configured_pick_yaw_deg,
        spawned_yaw_deg=args.spawned_yaw_deg,
        target_place_yaw_deg=args.target_place_yaw_deg,
        use_perceived_yaw=args.use_perceived_yaw,
        evidence_root="evidence/stage2c_orientation",
        stage2c_mode=True,
    )


if __name__ == "__main__":
    main()

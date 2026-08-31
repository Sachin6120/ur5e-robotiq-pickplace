#!/usr/bin/env python3
"""Run one Stage-2D combined planar-pose (XY offset + yaw) evidence case.

Stage-2D extends Stage-2C: the physical spawn XY and yaw are BOTH
independently decoupled from the configured pick pose, while manipulation
still consumes perceived XYZ and perceived yaw together. Configured pick
pose and the placement target remain fixed/configured, exactly as in
Stage-2C. This wrapper never modifies config/scene.yaml: it asks the shared
runner to write an isolated case scene under the evidence directory, and it
retains Stage-2C's axial (mod-180) placement-yaw scoring semantics.
"""

import argparse
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts/perception"))
import run_stage2a_yaw_case as harness


def main():
    parser = argparse.ArgumentParser(
        description="Run one Stage-2D combined XY-offset + yaw evidence case."
    )
    parser.add_argument("--case", required=True, help="Evidence case identifier")
    parser.add_argument(
        "--spawn-offset-x-m",
        type=float,
        default=0.0,
        help="Physical spawn X offset from the configured pick X (default: 0.0 m)",
    )
    parser.add_argument(
        "--spawn-offset-y-m",
        type=float,
        default=0.0,
        help="Physical spawn Y offset from the configured pick Y (default: 0.0 m)",
    )
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
    position_group = parser.add_mutually_exclusive_group()
    position_group.add_argument(
        "--target-source",
        choices=("perceived", "configured"),
        default="perceived",
        help="Manipulation position source (default: perceived). 'configured' is a "
             "comparison control only -- it also forces use_perceived_yaw off, since "
             "perceived yaw requires a perceived position (see m3_grasp.cpp's "
             "perceived_yaw_configuration_valid check).",
    )
    parser.add_argument("--gui", action="store_true", help="Run Gazebo with GUI (default: headless)")
    parser.add_argument(
        "--evidence-dir",
        default=None,
        help="Explicit evidence directory (default: evidence/stage2d_pose/<case>)",
    )
    parser.add_argument(
        "--record-diagnostics",
        action="store_true",
        help="Also record both pad contact streams, the gripper joint "
             "position/velocity/effort trace, and the perceived-position stream",
    )
    parser.add_argument(
        "--fixed-side-clearance-m",
        type=float,
        default=None,
        help="DIAGNOSTIC-ONLY. Overrides m3_grasp.launch.py's "
             "parallel_jaw_fixed_side_clearance_m (default: unset, which "
             "leaves the production 0.0020 m value untouched). Forwarded "
             "unchanged from run_stage2a_yaw_case.py's own diagnostic "
             "override; nothing else about pre-close aperture, final-close "
             "target, controllers, or trajectories changes.",
    )
    args = parser.parse_args()

    use_perceived_yaw = args.use_perceived_yaw and args.target_source == "perceived"

    harness.run_case(
        args.case,
        # Preserve the shared runner's legacy positional input for compatibility;
        # all Stage-2D yaw/position behavior below comes from explicit named inputs.
        args.configured_pick_yaw_deg,
        gazebo_gui=args.gui,
        evidence_dir=args.evidence_dir,
        target_source=args.target_source,
        record_diagnostics=args.record_diagnostics,
        fixed_side_clearance_m=args.fixed_side_clearance_m,
        configured_pick_yaw_deg=args.configured_pick_yaw_deg,
        spawned_yaw_deg=args.spawned_yaw_deg,
        target_place_yaw_deg=args.target_place_yaw_deg,
        use_perceived_yaw=use_perceived_yaw,
        evidence_root="evidence/stage2d_pose",
        stage2c_mode=True,
        spawn_offset_x_m=args.spawn_offset_x_m,
        spawn_offset_y_m=args.spawn_offset_y_m,
        stage2d_mode=True,
    )


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""gz_pose_observer.py — Gazebo ground-truth pose stream recorder.

Records Gazebo ground-truth link poses continuously to a CSV file for
quiescent-window slip and tilt analysis (using scripts/lib/slip.py and sample_pose.py).
"""
import argparse
import importlib.util
from pathlib import Path
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts/lib"))
import sample_pose as sp

ENTITIES = [
    "pick_target",
    "wrist_3_link",
    "pad_fixed_link",
    "pad_moving_link",
    "jaw_moving_link",
]


def main():
    parser = argparse.ArgumentParser(description="Record Gazebo ground-truth pose stream.")
    parser.add_argument("--out", required=True, help="Output CSV path")
    parser.add_argument("--topic", default="/world/empty/pose/info", help="Gazebo pose topic")
    args = parser.parse_args()

    cols = []
    for e in ENTITIES:
        cols += [f"{e}_{c}" for c in ("x", "y", "z", "qx", "qy", "qz", "qw")]

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out = open(out_path, "w", buffering=1)
    out.write("wall_ns," + ",".join(cols) + "\n")

    seen_any = False
    while True:
        proc = subprocess.Popen(
            ["gz", "topic", "-e", "-t", args.topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        try:
            for lines in sp.stream(proc.stdout):
                poses = sp.parse_pose_v(lines)
                row = []
                got = False
                for e in ENTITIES:
                    hit = [n for n in poses if n == e or n.endswith("::" + e)]
                    if hit:
                        row += [f"{v:.9f}" for v in poses[hit[0]]]
                        got = True
                    else:
                        row += [""] * 7
                if got:
                    seen_any = True
                    out.write(f"{time.time_ns()}," + ",".join(row) + "\n")
        except KeyboardInterrupt:
            break
        finally:
            try:
                proc.kill()
            except Exception:
                pass
        if not seen_any:
            time.sleep(1.0)
            continue
        time.sleep(0.2)


if __name__ == "__main__":
    main()

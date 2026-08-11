#!/usr/bin/env python3
"""sample_pose.py — take a SETTLED ground-truth pose, or fail loudly.

WHY THIS EXISTS RATHER THAN A SLEEP

    The first blocker this project recorded was a 6 mm sampling error caused by
    reading a pose while the gripper was still settling, after a fixed sleep.
    It was caught only because one value in five looked wrong and got retested
    by hand. M3 runs 20 cycles unattended; nobody retests individual samples,
    and 6 mm of noise is larger than the 5 mm criterion it would be judged
    against.

    So this never samples on a timer. It watches a window, requires the spread
    across that window to be under a tolerance, and EXITS NON-ZERO if it is
    not. A caller that cannot get a settled pose gets an error, not a number.

    gz_settle_pose's original non-windowed form also produced false quiescence
    -- a single below-threshold sample is not evidence of rest. Hence a window
    with a required minimum sample count, not a first-crossing test.

WHAT IT REPORTS
    One line per requested entity:

        <name> x y z qx qy qz qw

    Positions are the MEAN across the settled window, not the last sample.
    Averaging a quiescent window costs nothing and removes single-sample jitter
    from a number that feeds a 5 mm pass/fail.

GROUND TRUTH ONLY
    Reads Gazebo's own pose topic. Never TF. TF reports what the system
    believes, and a grasp that has silently failed still has a perfectly
    consistent TF tree -- MoveIt's attachObject is collision bookkeeping and
    nothing more. That distinction is the entire reason this file exists.

USAGE
    python3 scripts/lib/sample_pose.py \\
        --topic /world/empty/pose/info \\
        --entities tool0 pick_target \\
        --window-s 1.0 --tol-m 0.0005

EXIT 0 = every requested entity was settled; poses printed
EXIT 1 = at least one was still moving, or was never seen. Nothing printed.
"""

import argparse
import math
import subprocess
import sys
import time


def parse_pose_v(lines):
    """{name: (x,y,z,qx,qy,qz,qw)} from one gz.msgs.Pose_V text message.

    position{} and orientation{} both carry x/y/z, so the enclosing block has
    to be tracked -- reading them positionally is how a quaternion ends up in a
    position slot and a 5 mm criterion starts reading in radians.

    CORRECTNESS NOTE, found live against a real dump before this was ever
    pointed at the harness: protobuf's text format OMITS scalar fields that
    equal their default (0.0), so an axis-aligned pose like pick_target's
    (orientation identity: x=y=z=0, w=1) prints only "orientation { w: 1 }" --
    x/y/z never appear at all. An earlier version of this function required
    len(pos)==3 and len(ori)==4 before considering an entity complete, which
    silently dropped EVERY entity with any zero-valued field -- including
    pick_target itself (identity orientation) and table/ground_plane (mostly-
    zero pose). That would have made every harness cycle read NO_SAMPLE.
    Completion is now signalled by having CLOSED the position{}/orientation{}
    blocks (have_pos/have_ori), not by how many keys happened to be non-zero;
    missing keys default to 0.0, which is what their omission means.
    """
    out = {}
    name = None
    pos = {}
    ori = {}
    block = None
    have_pos = False
    have_ori = False
    for raw in lines:
        s = raw.strip()
        if s.startswith("pose {"):
            name, pos, ori, block = None, {}, {}, None
            have_pos = have_ori = False
        elif s.startswith("name:"):
            q1, q2 = s.find('"'), s.rfind('"')
            if q1 != -1 and q2 > q1:
                name = s[q1 + 1:q2]
        elif s.startswith("position {"):
            block = "p"
        elif s.startswith("orientation {"):
            block = "o"
        elif s == "}":
            if block == "p":
                have_pos = True
                block = None
            elif block == "o":
                have_ori = True
                block = None
            elif name and have_pos and have_ori:
                out[name] = (
                    pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0),
                    ori.get("x", 0.0), ori.get("y", 0.0), ori.get("z", 0.0),
                    ori.get("w", 0.0),
                )
                name = None
        elif block and ":" in s:
            k, v = s.split(":", 1)
            k = k.strip()
            if k in ("x", "y", "z", "w"):
                (pos if block == "p" else ori)[k] = float(v)
    return out


def stream(fh):
    """Yield line-buffers, one per top-level message, by brace depth."""
    buf, depth = [], 0
    for line in fh:
        if not buf and not line.strip():
            continue
        buf.append(line)
        depth += line.count("{") - line.count("}")
        if depth <= 0 and buf:
            yield buf
            buf, depth = [], 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/world/empty/pose/info")
    ap.add_argument("--entities", nargs="+", required=True,
                    help="matched by SUFFIX, so prefixed link names work")
    ap.add_argument("--window-s", type=float, default=1.0)
    ap.add_argument("--min-samples", type=int, default=10,
                    help="A window with too few samples is not evidence of "
                         "rest, it is evidence of a slow topic.")
    ap.add_argument("--tol-m", type=float, default=0.0005,
                    help="Max position spread across the window. Default is "
                         "1/10 of M3's 5 mm criterion, so sampling noise "
                         "cannot account for a pass or a fail.")
    ap.add_argument("--timeout-s", type=float, default=15.0)
    args = ap.parse_args()

    proc = subprocess.Popen(
        ["gz", "topic", "-e", "-t", args.topic],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)

    history = {e: [] for e in args.entities}
    t_start = time.time()
    settled = {}

    try:
        for lines in stream(proc.stdout):
            poses = parse_pose_v(lines)
            now = time.time()

            for want in args.entities:
                hit = [n for n in poses if n == want or n.endswith(want)]
                if hit:
                    history[want].append((now, poses[hit[0]]))
                    # Keep only the trailing window.
                    history[want] = [
                        h for h in history[want] if now - h[0] <= args.window_s
                    ]

            # Every entity settled across a full, densely-sampled window?
            ok_all = True
            for want in args.entities:
                h = history[want]
                if len(h) < args.min_samples or (now - h[0][0]) < args.window_s * 0.9:
                    ok_all = False
                    break
                spread = max(
                    math.dist(a[1][:3], b[1][:3]) for a in h for b in h
                )
                if spread > args.tol_m:
                    ok_all = False
                    break

            if ok_all:
                for want in args.entities:
                    h = history[want]
                    n = len(h)
                    mean = [sum(s[1][i] for s in h) / n for i in range(7)]
                    settled[want] = mean
                break

            if now - t_start > args.timeout_s:
                break
    finally:
        proc.terminate()

    if len(settled) != len(args.entities):
        missing = [e for e in args.entities if e not in settled]
        print(
            f"NOT SETTLED after {args.timeout_s:.1f}s: {missing}. "
            "Refusing to sample a moving pose — a number taken here would "
            "enter the CSV as slip. Check the entity names exist on "
            f"{args.topic}, and that the dwell is longer than --window-s.",
            file=sys.stderr,
        )
        return 1

    for want in args.entities:
        v = settled[want]
        print(f"{want} " + " ".join(f"{x:.9f}" for x in v))
    return 0


if __name__ == "__main__":
    sys.exit(main())

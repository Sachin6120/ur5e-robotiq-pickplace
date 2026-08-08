#!/usr/bin/env python3
# gz_settle.py — poll a Gazebo ground-truth topic until motion has stopped,
# instead of guessing a fixed sleep duration before sampling.
#
# WHY THIS EXISTS
#   05_measure_gripper_geometry.sh's sweep sampled fingertip pose on a fixed
#   `sleep 1.5` after each gripper_cmd goal. One sample (0.2 rad) landed 6mm
#   off truth (0.1218 vs 0.1158) because the gripper was still settling when
#   the sample was taken — caught only because it looked wrong and was
#   hand-retested. M3 runs 20 unattended cycles; nobody is there to notice one
#   bad sample, so it lands in the CSV indistinguishable from real slip.
#
#   Replace "sleep N, then sample" with "poll until motion stops, then
#   sample". If it never settles, FAIL LOUDLY (nonzero exit, message on
#   stderr) rather than sampling a moving target and calling it ground truth.
#
# GROUND TRUTH, NOT /joint_states
#   Same discipline as the rest of this project: reads Gazebo's own
#   /world/<world>/model/<model>/joint_state and /world/<world>/pose/info via
#   `gz topic -e`, never ros2 topics, which report what ros2_control believes.
#
# USAGE
#   gz_settle.py joint --topic <gz_js_topic> --eps 0.02 --timeout 5 --poll 0.15 \
#       JOINT_NAME [JOINT_NAME ...]
#   gz_settle.py pose  --topic <gz_pose_topic> --eps 0.001 --timeout 5 --poll 0.15 \
#       LINK_NAME [LINK_NAME ...]
#
#   joint mode watches |velocity| for the named joints.
#   pose mode watches the position delta between consecutive polls for the
#   named links (needs at least 2 samples before it can evaluate anything).
#
#   Both require every named quantity to be under its threshold for two
#   consecutive polls (guards against a single lucky near-zero-crossing
#   sample) before returning 0. Link/joint names must match the `name:`
#
#   PRECONDITION THIS DOES NOT AND CANNOT CHECK: the target must already be
#   in motion, or have already finished moving, when polling starts -- "not
#   moving" and "hasn't started moving yet" are indistinguishable from here,
#   and a target that hasn't started yet settles falsely, immediately (two
#   near-zero polls in, ~2*poll_dt seconds). Every current caller in this
#   project is safe because it only calls settle right after a SYNCHRONOUS,
#   already-returned `ros2 action send_goal` (which blocks until the goal
#   finishes) -- by the time settle starts polling, the command is known
#   complete. Do NOT call this right after firing an async/backgrounded
#   command with no other synchronization; confirmed to false-positive that
#   way while validating scripts/07_check_gripper_spawn_state.sh (an
#   ExecuteProcess launched from a launch file, polled from an unrelated
#   process with nothing to wait on) -- see docs/HANDOFF_M3.md.
#   field in the topic exactly — 04_mimic_contact_probe.sh already learned
#   the hard way that substring matching pulls in sub-frames like
#   "..._visual" and silently corrupts the result.
#
#   Exit 0  = settled; final sample already taken, caller should re-read the
#             topic once more (cheap) to get the settled values.
#   Exit 1  = timed out without settling, or a requested name never appeared
#             in the topic at all. Caller must treat this as a hard stop for
#             the sample, not paper over it with `|| true`.

import argparse
import math
import re
import subprocess
import sys
import time


def _gz_echo(topic, echo_timeout=3.0):
    # `gz topic -e` blocks until a message arrives; if the topic has no
    # active publisher (wrong name, sim not up, world paused) it hangs
    # indefinitely. Bound it and treat a timeout as "no sample this poll" —
    # the caller's own settle-loop timeout is what turns persistent silence
    # into a loud [STOP], not an uncaught exception here.
    try:
        out = subprocess.run(
            ["gz", "topic", "-e", "-t", topic, "-n", "1"],
            capture_output=True, text=True, timeout=echo_timeout,
        )
        return out.stdout
    except subprocess.TimeoutExpired:
        return ""


def _parse_joint_velocities(txt):
    joints = {}
    for blk in re.findall(r'joint\s*\{(.*?)\n\}', txt, re.S):
        n = re.search(r'name:\s*"([^"]+)"', blk)
        v = re.search(r'velocity:\s*(-?[\d.eE+-]+)', blk)
        if n and v:
            joints[n.group(1)] = float(v.group(1))
    return joints


def _parse_joint_positions(txt):
    joints = {}
    for blk in re.findall(r'joint\s*\{(.*?)\n\}', txt, re.S):
        n = re.search(r'name:\s*"([^"]+)"', blk)
        p = re.search(r'position:\s*(-?[\d.eE+-]+)', blk)
        if n and p:
            joints[n.group(1)] = float(p.group(1))
    return joints


def _parse_link_positions(txt):
    links = {}
    for blk in re.findall(r'pose\s*\{(.*?)\n\}', txt, re.S):
        n = re.search(r'name:\s*"([^"]+)"', blk)
        if not n:
            continue
        pos = re.search(r'position\s*\{(.*?)\}', blk, re.S)
        if not pos:
            continue
        d = dict(re.findall(r'([xyz]):\s*(-?[\d.eE+-]+)', pos.group(1)))
        links[n.group(1)] = tuple(float(d.get(k, 0.0)) for k in 'xyz')
    return links


def settle_joints(topic, names, eps, timeout, poll_dt, need_streak=2):
    t0 = time.time()
    streak = 0
    last_worst = float('nan')
    ever_seen = False
    while time.time() - t0 < timeout:
        joints = _parse_joint_velocities(_gz_echo(topic))
        missing = [n for n in names if n not in joints]
        if missing:
            streak = 0
            time.sleep(poll_dt)
            continue
        ever_seen = True
        last_worst = max(abs(joints[n]) for n in names)
        if last_worst < eps:
            streak += 1
            if streak >= need_streak:
                print(f"[settle] joints settled after {time.time()-t0:.2f}s "
                      f"(max|vel|={last_worst:.5f} rad/s)", file=sys.stderr)
                return 0
        else:
            streak = 0
        time.sleep(poll_dt)
    if not ever_seen:
        print(f"[STOP] joint(s) {names} never appeared on {topic} within "
              f"{timeout}s — check the topic/joint name, or is the sim up?",
              file=sys.stderr)
    else:
        print(f"[STOP] joints {names} did not settle within {timeout}s "
              f"(last max|vel|={last_worst:.5f} rad/s, threshold={eps}) — "
              "refusing to sample a moving target", file=sys.stderr)
    return 1


def settle_poses(topic, names, eps, timeout, poll_dt, need_streak=2, window_s=None):
    # window_s: compare the current sample against the sample from AT LEAST
    # window_s seconds ago, not just the immediately preceding poll. Defaults
    # to poll_dt, i.e. the original consecutive-poll behaviour, unchanged for
    # every existing caller.
    #
    # WHY THIS EXISTS, on top of need_streak: consecutive-poll comparison
    # cannot distinguish "at rest" from "creeping at a small constant rate."
    # A 0.05 mm/s creep produces ~0.0075mm per 0.15s poll -- comfortably under
    # eps=0.0005m regardless of how many consecutive polls are required,
    # because each individual poll-to-poll delta is genuinely small even
    # while the process is still very much in motion on a longer timescale.
    # Confirmed directly, 2026-08-08: a box
    # settling against the gripper after `gripper_close_and_hold` was declared
    # "settled" by the old consecutive-poll check at t=0.58s, then measured
    # (dedicated diagnostic, not this function) still sinking ~9.4mm over the
    # next 30s and not fully leveling off until ~t=120s. See
    # docs/HANDOFF_M3.md, "box-settle false-quiescence" for the full trace —
    # same shape of bug as stall_velocity_threshold's noise-floor problem, but
    # not fixable by raising need_streak alone: the fast-poll deltas were
    # never above eps in the first place, there was no spike to filter out.
    # Comparing against a sample far enough back in time makes a slow,
    # sustained creep visible again (0.05mm/s * 10s = 0.5mm, no longer
    # invisible at eps=0.0005m), which raising the consecutive-poll count at a
    # fixed 0.15s cadence cannot do.
    if window_s is None:
        window_s = poll_dt
    t0 = time.time()
    streak = 0
    history = []  # list of (timestamp, {name: (x,y,z)})
    last_worst = float('nan')
    ever_seen = False
    while time.time() - t0 < timeout:
        now = time.time()
        cur_all = _parse_link_positions(_gz_echo(topic))
        missing = [n for n in names if n not in cur_all]
        if missing:
            streak = 0
            time.sleep(poll_dt)
            continue
        ever_seen = True
        cur = {n: cur_all[n] for n in names}
        # oldest history entry that is at least window_s old
        baseline = None
        for ts, snap in history:
            if now - ts >= window_s:
                baseline = snap
            else:
                break
        if baseline is not None:
            last_worst = max(
                math.sqrt(sum((cur[n][i] - baseline[n][i]) ** 2 for i in range(3)))
                for n in names
            )
            if last_worst < eps:
                streak += 1
                if streak >= need_streak:
                    print(f"[settle] poses settled after {time.time()-t0:.2f}s "
                          f"(max|delta over {window_s}s|={last_worst:.6f} m)",
                          file=sys.stderr)
                    return 0
            else:
                streak = 0
        history.append((now, cur))
        # trim history older than window_s plus one poll of slack
        cutoff = now - window_s - poll_dt
        history = [(ts, snap) for ts, snap in history if ts >= cutoff]
        time.sleep(poll_dt)
    if not ever_seen:
        print(f"[STOP] link(s) {names} never appeared on {topic} within "
              f"{timeout}s — check the topic/link name, or is the sim up?",
              file=sys.stderr)
    else:
        print(f"[STOP] links {names} did not settle within {timeout}s "
              f"(last max|delta over {window_s}s|={last_worst:.6f} m, "
              f"threshold={eps}) — refusing to sample a moving target",
              file=sys.stderr)
    return 1


def assert_joint_position(topic, name, expected, tol, label=None):
    # PRECONDITION ASSERTION, not a settle check. Every measurement script
    # this project has written assumed "gripper OPEN at start" in a comment,
    # never checked it, and it was false on every single fresh launch this
    # session (confirmed: 5/5 fresh launches spawned near-closed, ~0.767rad,
    # not open ~0rad — see docs/HANDOFF_M3.md's spawn-state investigation).
    # That precondition failure was invisible for a full session specifically
    # because nothing ever read the actual state and compared it. Read it,
    # compare it, name the joint and the values in the failure — do not let
    # a script proceed on an assumed starting condition again.
    label = label or name
    pos = _parse_joint_positions(_gz_echo(topic)).get(name)
    if pos is None:
        print(f"[STOP] PRECONDITION_UNREADABLE: {label} ({name}) not found on {topic}",
              file=sys.stderr)
        return 1
    if abs(pos - expected) > tol:
        print(f"[STOP] PRECONDITION_FAILED: {label} ({name}) = {pos:.4f} rad, "
              f"expected {expected:.4f} +/- {tol} rad — refusing to proceed on "
              "an unmet starting condition", file=sys.stderr)
        return 1
    print(f"[ok] {label} ({name}) = {pos:.4f} rad, within {expected:.4f} +/- {tol}",
          file=sys.stderr)
    return 0


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="mode", required=True)

    jp = sub.add_parser("joint")
    jp.add_argument("--topic", required=True)
    jp.add_argument("--eps", type=float, default=0.02)
    jp.add_argument("--timeout", type=float, default=5.0)
    jp.add_argument("--poll", type=float, default=0.15)
    jp.add_argument("names", nargs="+")

    pp = sub.add_parser("pose")
    pp.add_argument("--topic", required=True)
    pp.add_argument("--eps", type=float, default=0.0005)
    pp.add_argument("--timeout", type=float, default=5.0)
    pp.add_argument("--poll", type=float, default=0.15)
    pp.add_argument("--window-s", type=float, default=None,
                     help="compare against a sample this many seconds back, "
                          "not just the previous poll (default: poll interval, "
                          "i.e. unchanged consecutive-poll behavior). Use a "
                          "larger value for contact/compliance settling, "
                          "which creeps slower than a fixed short window can "
                          "detect -- see settle_poses' docstring.")
    pp.add_argument("names", nargs="+")

    ap_ = sub.add_parser("assert-joint")
    ap_.add_argument("--topic", required=True)
    ap_.add_argument("--expected", type=float, required=True)
    ap_.add_argument("--tol", type=float, required=True)
    ap_.add_argument("--label", default=None)
    ap_.add_argument("name")

    args = ap.parse_args()
    if args.mode == "joint":
        rc = settle_joints(args.topic, args.names, args.eps, args.timeout, args.poll)
    elif args.mode == "pose":
        rc = settle_poses(args.topic, args.names, args.eps, args.timeout, args.poll,
                           window_s=args.window_s)
    else:
        rc = assert_joint_position(args.topic, args.name, args.expected, args.tol, args.label)
    sys.exit(rc)


if __name__ == "__main__":
    main()

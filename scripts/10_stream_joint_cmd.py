#!/usr/bin/env python3
"""10_stream_joint_cmd.py — reconstruct the POSITION command from the velocity
trace, at Gazebo's own publish rate, from ONE long-lived subscriber.

WHAT IT REPLACES AND WHY

    The previous poller spawned `gz topic -e -n 1` per sample: ~150-200 ms
    between samples against a 500 Hz control loop. Every sample in both
    excursion traces came back railed at +/-0.1, because the turnover the
    analysis needs happens 40-100x faster than that. The measurement was not
    wrong, it was undersampled, and undersampled in a way that hid exactly the
    quantity of interest.

    `gz topic -e -t <topic>` without -n streams from a single process. No
    per-sample spawn, no gap, sampling at whatever rate Gazebo publishes.

THE IDENTITY THIS EXPLOITS

    gz_ros2_control's POSITION branch (gz_system.cpp, write()) is deterministic:

        vel_cmd = clamp( gain * (cmd - pos),  +/- rail )
        gain    = position_proportional_gain * update_rate = 0.1 * 500 = 50
        rail    = the joint's URDF velocity limit = 0.1

    So a RAILED sample only bounds the command:

        vel = +rail  ->  cmd >= pos + rail/gain   (= pos + 0.002)
        vel = -rail  ->  cmd <= pos - rail/gain

    and an UNRAILED sample gives it exactly:

        cmd = pos + vel/gain

    Both traces so far contain only railed samples, which is why the command
    could be bounded (enough to falsify a flat-hold reading) but never read.

THE DISCRIMINATOR, STATED BEFORE THE RUN

    Two readings of the hold-gap excursion survive:

      #1  something is executing a smooth trajectory on the gripper's command
          interface during the gap
      #3  two writers are fighting over that interface

    They differ in how cmd behaves near the turnover, and that is observable
    without catching the crossing itself:

      The unrailed band is |cmd - pos| < rail/gain = 0.002 rad. At 0.1 rad/s a
      joint crosses 0.002 rad in 20 ms, so a SMOOTH cmd cannot get from +rail
      to -rail without spending ~20 ms inside the band -- roughly 20 samples at
      1 kHz, and it is geometrically impossible for it to spend none, because
      cmd - pos must pass continuously through zero.

      A DISCONTINUOUS cmd jumps across the band. The joint slams rail to rail
      with zero, one or two samples in between.

    So: many unrailed samples at the turnover -> #1.
        Zero to two -> #3.

    This decides it from one capture and does not require the crossing sample.
    Counting is the measurement; the exact cmd value is a bonus.

BACKPRESSURE, AND WHY IT DOES NOT CORRUPT THE RESULT

    At 1 kHz with ~25 joints this is a lot of text for Python to chew. If the
    parser falls behind, the pipe fills and `gz topic -e` blocks -- it does not
    drop. Data is delayed, never lost.

    And every timestamp in the CSV comes from the message's own header, not
    from arrival, so lag cannot shift the reconstructed timeline. The one thing
    to watch is whether blocking the publisher perturbs the sim itself; the
    lines/sec figure in the summary is there to make that visible rather than
    assumed.

USAGE
    gz topic -e -t /world/empty/model/ur5e_robotiq/joint_state \\
      | python3 scripts/10_stream_joint_cmd.py \\
          --joint robotiq_85_left_knuckle_joint \\
          --out runs/cmd_trace_$(date +%s).csv

    Start it before the trial, kill it after. Then look at the summary block:
    it prints every unrailed run it found, longest first. The turnover is the
    one bracketed by opposite rails.

    --gain and --rail are NOT guesses. gain is the plugin's logged
    position_proportional_gain (0.1, confirmed 6/6 in tonight's sim logs)
    times controllers.yaml's update_rate (500). rail is the URDF velocity
    limit. If either changes, pass it -- do not let this script keep asserting
    a stale calibration.
"""

import argparse
import csv
import os
import signal
import sys
import time


def stream_messages(fh):
    """Yield complete protobuf-text messages from `gz topic -e` output.

    Blank-line delimited. Brace-depth was tried first and is wrong for this
    message type: gz.msgs.Model's text dump has no enclosing top-level braces
    -- header {}, name:, pose {}, and each joint {} are flat siblings, so
    depth returns to zero after every one of them, not just at the true
    message boundary. That fragmented one ~25-joint message into ~25 pieces,
    and every fragment containing the target joint's axis1 { xyz {...}
    limit_upper: ... position: ... velocity: ... } compounded it further (see
    parse_joint). Confirmed empirically: a captured 2-message dump has a
    blank line at exactly line 330 (end of message 1, 330 lines) and line 660
    (end of message 2), none elsewhere -- it is the CLI's actual delimiter,
    not an unreliable formatting detail.
    """
    buf = []
    for line in fh:
        if not line.strip():
            if buf:
                yield buf
                buf = []
            continue
        buf.append(line)
    if buf:
        yield buf


def parse_joint(lines, target):
    """Pull (sim_t, pos, vel) for `target` out of one gz.msgs.Model message.

    Hand-rolled rather than regex: this runs on every line of a 1 kHz stream
    and the difference is the difference between keeping up and applying
    backpressure to the simulator.
    """
    sim_sec = sim_nsec = 0
    in_joint = False
    is_target = False
    in_axis = False
    axis_depth = 0
    pos = vel = None
    seen_stamp = False

    for raw in lines:
        s = raw.strip()

        if not seen_stamp:
            if s.startswith("sec:"):
                sim_sec = int(s[4:])
                continue
            if s.startswith("nsec:"):
                sim_nsec = int(s[5:])
                seen_stamp = True
                continue

        if s.startswith("joint {"):
            in_joint = True
            is_target = False
            in_axis = False
            axis_depth = 0
            continue

        if in_joint and s.startswith('name:'):
            # name: "prefix_joint_name"
            q1 = s.find('"')
            q2 = s.rfind('"')
            if q1 != -1 and q2 > q1:
                is_target = s[q1 + 1:q2].endswith(target)
            continue

        if is_target:
            if s.startswith("axis1 {"):
                in_axis = True
                axis_depth = 1
                continue
            if in_axis:
                # axis1 wraps a nested `xyz { ... }` submessage that closes
                # before position:/velocity:. Depth-track rather than treat
                # the first "}" as axis1's own close, or those two fields
                # are never reached.
                axis_depth += s.count("{") - s.count("}")
                if s.startswith("position:"):
                    pos = float(s[9:])
                elif s.startswith("velocity:"):
                    vel = float(s[9:])
                elif axis_depth <= 0:
                    in_axis = False
                    # Found everything for the target joint; the rest of the
                    # message is other joints and can be skipped entirely.
                    if pos is not None and vel is not None:
                        break

    if pos is None or vel is None:
        return None
    return (sim_sec + sim_nsec * 1e-9, pos, vel)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", required=True, help="suffix of the actuated joint name")
    ap.add_argument("--out", default="cmd_trace.csv")
    ap.add_argument(
        "--gain",
        type=float,
        default=50.0,
        help="position_proportional_gain * update_rate. 0.1 * 500 = 50.",
    )
    ap.add_argument(
        "--rail",
        type=float,
        default=0.1,
        help="URDF velocity limit for this joint.",
    )
    ap.add_argument(
        "--rail-frac",
        type=float,
        default=0.90,
        help="A sample counts as unrailed only below this fraction of rail. "
        "Samples just under the rail (0.0994 vs 0.1) bound cmd to within "
        "0.00001 rad of the railed bound and carry no extra information, so "
        "counting them as unrailed would inflate the discriminator.",
    )
    args = ap.parse_args()

    band = args.rail / args.gain
    informative = args.rail * args.rail_frac

    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
    fh = open(args.out, "w", newline="", buffering=1)
    w = csv.writer(fh)
    w.writerow(["sim_t", "pos", "vel", "state", "cmd_exact", "cmd_lo", "cmd_hi"])

    print(
        f"gain={args.gain}  rail={args.rail}  unrailed band |cmd-pos| < {band:.5f} rad\n"
        f"a smooth traversal at {args.rail} rad/s spends "
        f"{2 * band / args.rail * 1000:.0f} ms in that band\n"
        f"joint suffix: {args.joint}\n",
        file=sys.stderr,
    )

    n = 0
    n_unrailed = 0
    over_rail = []     # (sim_t, vel) — samples the identity cannot represent
    runs = []          # (start_t, end_t, count, prev_rail, next_rail)
    cur = None
    prev_state = None
    t0_wall = time.time()

    def summarise(signum=None, frame=None):
        fh.flush()
        fh.close()
        dt = time.time() - t0_wall
        print(f"\n--- {n} samples in {dt:.1f}s ({n / dt:.0f}/s) ---", file=sys.stderr)
        print(f"unrailed samples: {n_unrailed}", file=sys.stderr)
        if over_rail:
            worst = max(over_rail, key=lambda x: abs(x[1]))
            print(
                f"over-rail samples: {len(over_rail)}  "
                f"(worst {worst[1]:+.4f} rad/s at sim {worst[0]:.4f} = "
                f"{abs(worst[1]) / args.rail:.1f}x rail)  "
                "— joint moved by external force; identity invalid there",
                file=sys.stderr,
            )

        turnovers = [r for r in runs if r[3] and r[4] and r[3] != r[4]]
        if turnovers:
            print("\nTURNOVERS (rail sign change), longest first:", file=sys.stderr)
            for st, en, c, a, b in sorted(turnovers, key=lambda r: -r[2])[:10]:
                print(
                    f"  sim {st:.4f} -> {en:.4f}  {c} unrailed samples  "
                    f"({a} -> {b})",
                    file=sys.stderr,
                )
            best_run = max(turnovers, key=lambda r: r[2])
            best = best_run[2]
            print("", file=sys.stderr)

            # Void the verdict if the joint was under external force anywhere
            # near this turnover. The whole method rests on vel being the
            # COMMANDED velocity; contact breaks that, and a contaminated
            # window produces a confident answer to a question it cannot see.
            lo_t, hi_t = best_run[0] - 0.5, best_run[1] + 0.5
            near = [(t, v) for t, v in over_rail if lo_t <= t <= hi_t]
            if near:
                worst = max(near, key=lambda x: abs(x[1]))
                print(
                    f"  VERDICT VOID — {len(near)} over-rail samples within "
                    f"0.5 s of this turnover (worst {worst[1]:+.4f} rad/s at "
                    f"sim {worst[0]:.4f}, {abs(worst[1]) / args.rail:.1f}x the "
                    f"{args.rail} rail).\n"
                    "  clamp() bounds a commanded velocity to the rail under "
                    "ANY command source, so these samples are external force — "
                    "contact or linkage — not a command. The identity does not "
                    "hold here and neither does the rail count that depends on "
                    "it.\n"
                    "  This method is only valid during free, unloaded motion. "
                    "Find a turnover with no over-rail samples nearby, or "
                    "capture a window before first contact.",
                    file=sys.stderr,
                )
            elif best >= 5:
                print(
                    f"  {best} samples inside the band at the turnover. cmd - pos "
                    "traversed zero continuously, which a discontinuous command "
                    "cannot do. Reads as #1: something is executing a smooth "
                    "trajectory on the command interface.",
                    file=sys.stderr,
                )
            else:
                print(
                    f"  only {best} sample(s) inside the band at the turnover, "
                    f"against ~{2 * band / args.rail * 1000:.0f} ms of expected "
                    "dwell. cmd jumped across the band rather than crossing it. "
                    "Reads as #3: two writers.",
                    file=sys.stderr,
                )
                print(
                    "  Before accepting that: confirm the sample rate above is "
                    "actually dense enough. At this rate the band should hold "
                    f"~{(2 * band / args.rail) * (n / dt):.0f} samples. If that "
                    "number is itself small, the capture is still undersampled "
                    "and this says nothing.",
                    file=sys.stderr,
                )
        else:
            print(
                "\nNo rail sign change captured. Either the excursion did not "
                "happen in this window, or the capture missed it — check the CSV "
                "for whether the joint moved at all before concluding anything.",
                file=sys.stderr,
            )
        sys.exit(0)

    signal.signal(signal.SIGINT, summarise)
    signal.signal(signal.SIGTERM, summarise)

    for lines in stream_messages(sys.stdin):
        rec = parse_joint(lines, args.joint)
        if rec is None:
            continue
        t, pos, vel = rec
        n += 1

        # A sample beyond the rail CANNOT be a commanded velocity. clamp() makes
        # |vel_cmd| <= rail by construction, under any command source, smooth or
        # discontinuous. So an over-rail sample means external force moved the
        # joint — contact, or the linkage — and cmd = pos + vel/gain does not
        # hold for it. It also contaminates the railed samples around it, since
        # a rail count can no longer be attributed to a command. Recorded so it
        # can VOID a verdict rather than being silently absorbed into rail_pos.
        if abs(vel) > args.rail * 1.01:
            over_rail.append((t, vel))

        if vel >= informative:
            state, cmd, lo, hi = "rail_pos", "", f"{pos + band:.6f}", ""
        elif vel <= -informative:
            state, cmd, lo, hi = "rail_neg", "", "", f"{pos - band:.6f}"
        else:
            state = "unrailed"
            cmd, lo, hi = f"{pos + vel / args.gain:.6f}", "", ""
            n_unrailed += 1

        w.writerow([f"{t:.6f}", f"{pos:.6f}", f"{vel:.6f}", state, cmd, lo, hi])

        if state == "unrailed":
            if cur is None:
                cur = [t, t, 0, prev_state, None]
            cur[1] = t
            cur[2] += 1
        elif cur is not None:
            cur[4] = state
            runs.append(tuple(cur))
            cur = None

        if state != "unrailed":
            prev_state = state

    summarise()


if __name__ == "__main__":
    main()

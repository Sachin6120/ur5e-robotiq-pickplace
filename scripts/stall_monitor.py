#!/usr/bin/env python3
"""stall_monitor.py — record controller-loop cadence for the whole life of a run.

WHY THIS EXISTS
    The pre-close overshoot mechanism is closed: gz_ros2_control's POSITION
    branch converts a position command into a JointVelocityCmd every cycle
    (gz_system.cpp, GazeboSimSystem::write). Gazebo holds a *velocity*, not a
    position. So when the controller loop stops running, the last velocity
    stays in effect and the joint keeps moving open-loop:

        overshoot_rad = commanded_velocity * sim_seconds_the_loop_was_stalled

    With the knuckle joint saturated at its URDF velocity limit of 0.5 rad/s,
    the two observed overshoots (0.456 and 0.416 rad past target) require
    roughly 0.91 s and 0.83 s of stalled loop.

    What is NOT known is what makes a stall that long. Directly measured stalls
    so far are 333-474 ms. Dedicated tests to reproduce the long ones have
    failed twice, for two different reasons, because the event is intermittent
    and each sim start yields one sample before degradation sets in.

    So stop hunting it. Run this alongside everything and let the samples
    accumulate. Twenty M3 cycles is twenty samples for free.

TWO MEASUREMENT DECISIONS, BOTH DELIBERATE

    1. Gaps come from the message HEADER STAMP, not from arrival time.
       joint_state_broadcaster stamps at publish time from inside the
       controller_manager loop, so header-stamp gaps are the loop's own
       cadence. Arrival-time gaps would also include this monitor's
       scheduling lag and would report the monitor's problems as the
       robot's. Arrival time is recorded too, in a separate column, purely
       so the two can be told apart after the fact.

       Using /joint_states here is not a departure from "ground truth comes
       from Gazebo's state topics, never /joint_states". That rule is about
       robot state. Here the loop's publish cadence IS the quantity under
       test, and /joint_states is the only place it is observable.

    2. The primary clock is SIM time. The joint travels 0.5 rad per sim
       second, so the overshoot is set by sim-time elapsed, not wall-time.
       A wall-clock gap during a real-time-factor dip is not the same event.
       Both are logged; sim is the one that enters the arithmetic.

WHAT IT WRITES
    A CSV row per gap above --min-gap-ms, plus a periodic heartbeat row so
    that "no stalls" is distinguishable from "monitor died". Columns:

        kind         gap | heartbeat | summary
        sim_t        sim time of the LATER message (s)
        wall_t       wall clock of the LATER message, epoch seconds
        sim_gap_ms   header-stamp delta -- THIS is the one that matters
        wall_gap_ms  arrival delta; differs from sim_gap_ms when RTF != 1
        rtf          sim_gap/wall_gap for this pair; ~1.0 means real time
        msg_count    messages seen so far

    Join offline against m3_grasp's own timestamped stage logs. No IPC, no
    coupling, nothing for this monitor to get wrong about phase.

WHAT TO LOOK FOR
    Two regimes produce overshoot and they are distinguishable in this data:

      A. one clean freeze  -> a single large sim_gap_ms
      B. sustained degradation -> a run of medium gaps back to back, where
         the loop is alive but writing velocities computed from stale
         positions. This produces MORE overshoot than any individual gap
         predicts, and it is the regime the single-gap arithmetic misses.

    Regime B is why every gap is logged rather than only the maximum, and
    why --min-gap-ms defaults low.

USAGE
    ros2 run ... no. Just:
        python3 scripts/stall_monitor.py --out runs/stalls_$(date +%s).csv &
    Start it before the sim work, leave it for the whole run. It subscribes
    only; it commands nothing and cannot perturb the thing it measures
    beyond one subscription's worth of load.
"""

import argparse
import csv
import os
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState


class StallMonitor(Node):
    def __init__(self, out_path, topic, min_gap_ms, heartbeat_s):
        super().__init__("stall_monitor")

        # This node timestamps nothing itself and must not be handed a clock
        # that disagrees with the stamps it reads. Header stamps arrive as sim
        # time because the publisher uses sim time; we never call self.get_clock()
        # for anything that enters the CSV, so use_sim_time is irrelevant here
        # and is deliberately not set. Wall time comes from the OS directly.
        self.min_gap_s = min_gap_ms / 1000.0
        self.heartbeat_s = heartbeat_s
        self.topic = topic

        self.prev_sim = None
        self.prev_wall = None
        self.count = 0
        self.gaps_logged = 0
        self.max_sim_gap = 0.0
        self.last_heartbeat_wall = None

        os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
        # Line-buffered: a killed sim must not take the last rows with it.
        self.fh = open(out_path, "w", newline="", buffering=1)
        self.csv = csv.writer(self.fh)
        self.csv.writerow(
            ["kind", "sim_t", "wall_t", "sim_gap_ms", "wall_gap_ms", "rtf", "msg_count"]
        )

        # Match joint_state_broadcaster's publisher QoS. A mismatch here means
        # no messages at all, which would look exactly like a dead sim -- so if
        # msg_count stays 0, suspect this before suspecting the robot.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            # Deep enough that a momentary hiccup in THIS process does not drop
            # messages and manufacture a gap in arrival time. Header-stamp gaps
            # are immune to that, but wall_gap_ms is not, and it is only useful
            # if it is honest.
            depth=200,
        )
        self.create_subscription(JointState, topic, self.on_js, qos)

        self.get_logger().info(
            f"stall_monitor: topic={topic} out={out_path} "
            f"min_gap={min_gap_ms:.0f}ms heartbeat={heartbeat_s:.0f}s"
        )
        self.get_logger().info(
            "gaps are HEADER-STAMP (sim time) deltas; wall column is arrival "
            "time and exists only to separate loop stalls from monitor lag"
        )

    @staticmethod
    def _now_wall():
        # time.time() and not the ROS clock: this must keep meaning something
        # when sim time freezes, which is precisely the case of interest.
        import time

        return time.time()

    def on_js(self, msg):
        sim_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wall_t = self._now_wall()
        self.count += 1

        if self.last_heartbeat_wall is None:
            self.last_heartbeat_wall = wall_t

        if self.prev_sim is not None:
            sim_gap = sim_t - self.prev_sim
            wall_gap = wall_t - self.prev_wall

            # A zero or negative sim gap means duplicate or reordered stamps,
            # which is a different fault from a stall. Log it rather than
            # silently dividing by it.
            rtf = (sim_gap / wall_gap) if wall_gap > 1e-9 else float("nan")

            if sim_gap > self.max_sim_gap:
                self.max_sim_gap = sim_gap

            if sim_gap >= self.min_gap_s or sim_gap <= 0.0:
                self.csv.writerow(
                    [
                        "gap",
                        f"{sim_t:.6f}",
                        f"{wall_t:.6f}",
                        f"{sim_gap * 1000.0:.2f}",
                        f"{wall_gap * 1000.0:.2f}",
                        f"{rtf:.3f}",
                        self.count,
                    ]
                )
                self.gaps_logged += 1
                # 0.5 rad/s is the knuckle joint's saturated velocity. Printing
                # the implied excursion makes a stall legible as the thing it
                # would do to the gripper, without asserting that it did.
                self.get_logger().warn(
                    f"loop gap {sim_gap * 1000.0:.0f} ms sim "
                    f"({wall_gap * 1000.0:.0f} ms wall, rtf {rtf:.2f}) "
                    f"-> up to {0.5 * sim_gap:.3f} rad of open-loop travel "
                    f"at saturated gripper velocity"
                )

        self.prev_sim = sim_t
        self.prev_wall = wall_t

        if wall_t - self.last_heartbeat_wall >= self.heartbeat_s:
            self.csv.writerow(
                [
                    "heartbeat",
                    f"{sim_t:.6f}",
                    f"{wall_t:.6f}",
                    f"{self.max_sim_gap * 1000.0:.2f}",
                    "",
                    "",
                    self.count,
                ]
            )
            self.last_heartbeat_wall = wall_t

    def finish(self):
        self.csv.writerow(
            [
                "summary",
                "",
                f"{self._now_wall():.6f}",
                f"{self.max_sim_gap * 1000.0:.2f}",
                "",
                "",
                self.count,
            ]
        )
        self.fh.flush()
        self.fh.close()
        # Printed to stderr so it survives even if the log is being grepped.
        print(
            f"[stall_monitor] messages={self.count} gaps_logged={self.gaps_logged} "
            f"max_sim_gap_ms={self.max_sim_gap * 1000.0:.1f} "
            f"implied_max_overshoot_rad={0.5 * self.max_sim_gap:.3f}",
            file=sys.stderr,
        )
        if self.count == 0:
            print(
                "[stall_monitor] WARNING: zero messages received. That is a QoS "
                "or topic-name problem in this monitor, NOT evidence about the "
                f"controller loop. Check that {self.topic} exists and is being "
                "published.",
                file=sys.stderr,
            )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="stalls.csv")
    ap.add_argument("--topic", default="/joint_states")
    ap.add_argument(
        "--min-gap-ms",
        type=float,
        default=20.0,
        help="Log any gap at least this large. Default is deliberately low: "
        "regime B (a run of medium gaps) produces more overshoot than any "
        "single gap in it, and a high threshold hides exactly that pattern. "
        "At 500 Hz the baseline is 2 ms, so 20 ms is already 10x nominal.",
    )
    ap.add_argument("--heartbeat-s", type=float, default=30.0)
    args = ap.parse_args()

    rclpy.init()
    node = StallMonitor(args.out, args.topic, args.min_gap_ms, args.heartbeat_s)

    def bye(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, bye)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

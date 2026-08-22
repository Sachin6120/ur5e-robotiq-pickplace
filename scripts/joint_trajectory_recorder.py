#!/usr/bin/env python3
"""joint_trajectory_recorder.py — full-resolution position/velocity log for
one /joint_states joint, for the whole life of a run.

WHY THIS EXISTS
    M6 trajectory-capture run 3 (branch m6-width-30mm) saw the gripper's
    master joint (robotiq_85_left_knuckle_joint) swing 0 -> 0.716 -> -0.588
    rad during the PRE-GRASP ARM MOVE, i.e. while the gripper was
    uncommanded and should have stayed pinned at its last commanded value.
    That capture's poller only sampled link poses at intervals; it did not
    keep a full joint trajectory, so the excursion is documented but not
    reproducible-on-paper. This script is the fix: it logs EVERY
    /joint_states message for a given joint, unfiltered, sim_t + wall_t +
    position + velocity, so a spurious excursion shows up directly as a
    departure from the near-zero baseline before the real, sustained
    closing ramp -- no cross-referencing against stage-marker timestamps
    required.

    stall_monitor.py (same directory) answers a different question --
    controller-LOOP CADENCE -- and only logs gaps above a threshold. It
    would not have caught this: a smooth but wrong joint value doesn't
    produce a publish-cadence gap. This script and that one are
    complementary, not redundant; run both if loop-stall context also
    matters for the run in question.

USAGE
    python3 scripts/joint_trajectory_recorder.py --out runs/foo.csv \
        --joint robotiq_85_left_knuckle_joint &
    Start before the sim work begins, leave running for the whole cycle,
    SIGTERM to stop (writes a summary row and closes cleanly). Subscribes
    only; commands nothing.
"""

import argparse
import csv
import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState


class JointTrajectoryRecorder(Node):
    def __init__(self, out_path, topic, joint_name):
        super().__init__("joint_trajectory_recorder")

        self.topic = topic
        self.joint_name = joint_name
        self.count = 0
        self.matched = 0
        self.min_pos = None
        self.max_pos = None

        os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
        # Line-buffered: a killed sim must not take the last rows with it.
        self.fh = open(out_path, "w", newline="", buffering=1)
        self.csv = csv.writer(self.fh)
        self.csv.writerow(["sim_t", "wall_t", "position", "velocity", "effort", "msg_count"])

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )
        self.create_subscription(JointState, topic, self.on_js, qos)

        self.get_logger().info(
            f"joint_trajectory_recorder: topic={topic} joint={joint_name} out={out_path}"
        )

    def on_js(self, msg):
        wall_t = time.time()
        sim_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.count += 1

        try:
            idx = msg.name.index(self.joint_name)
        except ValueError:
            return

        pos = msg.position[idx] if idx < len(msg.position) else float("nan")
        vel = msg.velocity[idx] if idx < len(msg.velocity) else float("nan")
        eff = msg.effort[idx] if idx < len(msg.effort) else float("nan")
        self.matched += 1

        if self.min_pos is None or pos < self.min_pos:
            self.min_pos = pos
        if self.max_pos is None or pos > self.max_pos:
            self.max_pos = pos

        self.csv.writerow(
            [f"{sim_t:.6f}", f"{wall_t:.6f}", f"{pos:.6f}", f"{vel:.6f}", f"{eff:.6f}", self.count]
        )

    def finish(self):
        self.fh.flush()
        self.fh.close()
        print(
            f"[joint_trajectory_recorder] msgs={self.count} matched={self.matched} "
            f"pos_range=[{self.min_pos},{self.max_pos}]",
            file=sys.stderr,
        )
        if self.matched == 0:
            print(
                "[joint_trajectory_recorder] WARNING: zero matched messages. "
                f"Check that '{self.joint_name}' appears on {self.topic}.",
                file=sys.stderr,
            )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="joint_trajectory.csv")
    ap.add_argument("--topic", default="/joint_states")
    ap.add_argument("--joint", default="robotiq_85_left_knuckle_joint")
    args = ap.parse_args()

    rclpy.init()
    node = JointTrajectoryRecorder(args.out, args.topic, args.joint)

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

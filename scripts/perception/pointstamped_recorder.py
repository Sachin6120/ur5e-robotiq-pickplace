#!/usr/bin/env python3
"""pointstamped_recorder.py — log every PointStamped on a topic to CSV.

WHY THIS EXISTS

    The Stage-2A O1 configured-center diagnostic control deliberately does
    NOT feed perception into the manipulation target: the target comes from
    the configured scene centre instead.  Perception still needs to be
    OBSERVED though, because the whole decision question is whether residual
    perceived-XY misregistration is what initiates the lift-onset roll.  With
    perception out of the control loop, m3_grasp logs nothing about it, so
    this recorder is the only place the perceived position for this run is
    preserved.

    It subscribes and nothing else.  It cannot influence the target.

USAGE
    python3 scripts/perception/pointstamped_recorder.py \
        --topic /object_detector/position_world --out <dir>/perceived_points.csv
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
from geometry_msgs.msg import PointStamped


class PointStampedRecorder(Node):
    def __init__(self, out_path, topic):
        super().__init__("pointstamped_recorder")
        self.count = 0
        os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
        self.fh = open(out_path, "w", newline="", buffering=1)
        self.csv = csv.writer(self.fh)
        self.csv.writerow(["wall_t", "stamp_s", "frame_id", "x", "y", "z", "msg_index"])
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.create_subscription(PointStamped, topic, self.on_point, qos)
        self.get_logger().info(f"pointstamped_recorder: topic={topic} out={out_path}")

    def on_point(self, msg):
        self.count += 1
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.csv.writerow([
            f"{time.time():.6f}", f"{stamp_s:.9f}", msg.header.frame_id,
            f"{msg.point.x:.9f}", f"{msg.point.y:.9f}", f"{msg.point.z:.9f}",
            self.count,
        ])

    def finish(self):
        self.fh.flush()
        self.fh.close()
        print(f"[pointstamped_recorder] msgs={self.count}", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--topic", default="/object_detector/position_world")
    args = ap.parse_args()

    rclpy.init()
    node = PointStampedRecorder(args.out, args.topic)

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

#!/usr/bin/env python3
"""Print the first freshly received object world PoseStamped and exit."""

import argparse
import json
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init(args=None)
    node = Node("stage2c_perception_pose_probe")
    topic = "/object_detector/pose_world"
    discovery_deadline = time.monotonic() + min(5.0, args.timeout)
    publishers = []
    while time.monotonic() < discovery_deadline and not publishers:
        publishers = node.get_publishers_info_by_topic(topic)
        if not publishers:
            time.sleep(0.1)
    if not publishers:
        print("POSE_PROBE_FAILURE=no publisher discovered", flush=True)
        node.destroy_node()
        rclpy.shutdown()
        return 2

    endpoint = publishers[0]
    print(
        "POSE_PROBE_PUBLISHER "
        f"node={endpoint.node_namespace}/{endpoint.node_name} "
        f"type={endpoint.topic_type} qos={endpoint.qos_profile}",
        flush=True,
    )
    # A volatile subscriber receives only a message published after this
    # probe subscribes, which makes this a real readiness check.
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    sample = None

    def on_pose(msg):
        nonlocal sample
        sample = {
            "stamp_sec": msg.header.stamp.sec,
            "stamp_nanosec": msg.header.stamp.nanosec,
            "frame_id": msg.header.frame_id,
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
            "qx": msg.pose.orientation.x,
            "qy": msg.pose.orientation.y,
            "qz": msg.pose.orientation.z,
            "qw": msg.pose.orientation.w,
        }

    subscription = node.create_subscription(PoseStamped, topic, on_pose, qos)
    deadline = time.monotonic() + args.timeout
    while sample is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()
    if sample is None:
        print("POSE_PROBE_FAILURE=no PoseStamped received", flush=True)
        return 2
    print(f"POSE_PROBE_SAMPLE_JSON={json.dumps(sample, sort_keys=True)}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

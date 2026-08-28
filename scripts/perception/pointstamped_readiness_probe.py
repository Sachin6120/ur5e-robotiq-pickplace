#!/usr/bin/env python3
"""Print the first live object world PointStamped and exit."""

import argparse
import json
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init(args=None)
    node = Node("repeatability_perception_probe")
    topic = "/object_detector/position_world"
    discovery_deadline = time.monotonic() + min(5.0, args.timeout)
    publishers = []
    while time.monotonic() < discovery_deadline and not publishers:
        publishers = node.get_publishers_info_by_topic(topic)
        if not publishers:
            time.sleep(0.1)
    if not publishers:
        print("PROBE_FAILURE=no publisher discovered", flush=True)
        node.destroy_node()
        rclpy.shutdown()
        return 2

    endpoint = publishers[0]
    print(
        "PROBE_PUBLISHER "
        f"node={endpoint.node_namespace}/{endpoint.node_name} "
        f"type={endpoint.topic_type} qos={endpoint.qos_profile}",
        flush=True,
    )

    # BEST_EFFORT/VOLATILE is compatible with the discovered RELIABLE/VOLATILE
    # publisher and guarantees that only a message published after this probe
    # subscribed can satisfy readiness.
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    sample = None

    def on_point(msg):
        nonlocal sample
        sample = {
            "stamp_sec": msg.header.stamp.sec,
            "stamp_nanosec": msg.header.stamp.nanosec,
            "frame_id": msg.header.frame_id,
            "x": msg.point.x,
            "y": msg.point.y,
            "z": msg.point.z,
        }

    subscription = node.create_subscription(PointStamped, topic, on_point, qos)
    deadline = time.monotonic() + args.timeout
    while sample is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()
    if sample is None:
        print("PROBE_FAILURE=no PointStamped received", flush=True)
        return 2
    print(f"PROBE_SAMPLE_JSON={json.dumps(sample, sort_keys=True)}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
# m3_capture_joint_effort.py -- answers the question the achieved_grip_angle
# check alone cannot: was the actuated gripper joint actually squeezing
# during the post-close hold (substantial, sustained effort), or did it sit
# at essentially zero effort the whole time (never gripping, just resting at
# first contact)? Logged from real hardware-interface state via
# /joint_states (gz_ros2_control's JointTransmittedWrench-backed effort
# interface), not inferred from achieved_grip_angle, which cannot
# distinguish a 6-micron-per-side rest from a real squeeze.
#
# Runs for the whole grasp cycle (started before m3_grasp launches, killed
# by the caller after teardown) so the CSV covers close, hold, lift, and
# transport in one continuous trace -- no separate before/after capture to
# drift across an interactive gap.
#
# Usage: m3_capture_joint_effort.py <joint_name> <out_csv>
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class EffortLogger(Node):
    def __init__(self, joint_name, out_csv):
        super().__init__("joint_effort_logger")
        self.joint_name = joint_name
        self.out_fh = open(out_csv, "w")
        self.out_fh.write("wall_time,stamp_sec,position_rad,velocity,effort\n")
        self.count = 0
        self.sub = self.create_subscription(JointState, "/joint_states", self.cb, 200)
        self.get_logger().info(f"logging joint '{joint_name}' -> {out_csv}")

    def cb(self, msg):
        try:
            idx = msg.name.index(self.joint_name)
        except ValueError:
            return
        position = msg.position[idx] if idx < len(msg.position) else float("nan")
        velocity = msg.velocity[idx] if idx < len(msg.velocity) else float("nan")
        effort = msg.effort[idx] if idx < len(msg.effort) else float("nan")
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.out_fh.write(f"{time.time():.6f},{stamp_sec:.6f},{position:.6f},{velocity:.6f},{effort:.6f}\n")
        self.out_fh.flush()
        self.count += 1


def main():
    if len(sys.argv) != 3:
        print("usage: m3_capture_joint_effort.py <joint_name> <out_csv>", file=sys.stderr)
        sys.exit(2)
    joint_name, out_csv = sys.argv[1:3]

    rclpy.init()
    node = EffortLogger(joint_name, out_csv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"wrote {node.count} rows to {out_csv}", flush=True)
        node.out_fh.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

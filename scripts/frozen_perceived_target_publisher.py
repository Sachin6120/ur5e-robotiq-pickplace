#!/usr/bin/env python3
"""Publish the already-captured Scene-A perceived top position with fresh sim stamps."""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PointStamped


class FrozenTarget(Node):
    def __init__(self):
        super().__init__("frozen_scene_a_target")
        self.set_parameters([Parameter("use_sim_time", value=True)])
        self.pub = self.create_publisher(PointStamped, "/object_detector/position_world", 10)
        self.timer = self.create_timer(0.1, self.publish)

    def publish(self):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.point.x = 0.450965
        msg.point.y = -0.148707
        msg.point.z = 0.795000
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FrozenTarget()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

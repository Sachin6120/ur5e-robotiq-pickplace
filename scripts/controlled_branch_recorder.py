#!/usr/bin/env python3
"""Record arm controller/reference and actual joint state for one M3 branch test."""

import argparse
import csv
import math
import pathlib
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState


JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]


class Recorder(Node):
    def __init__(self):
        super().__init__("controlled_branch_recorder")
        self.set_parameters([Parameter("use_sim_time", value=True)])
        self.joint_rows = []
        self.controller_rows = []
        self.create_subscription(JointState, "/joint_states", self.on_joint, 1000)
        self.create_subscription(
            JointTrajectoryControllerState, "/arm_controller/controller_state",
            self.on_controller, 1000,
        )

    def sim_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_joint(self, msg):
        if not all(name in msg.name for name in JOINTS):
            return
        idx = [msg.name.index(name) for name in JOINTS]
        q = [msg.position[i] for i in idx]
        v = [msg.velocity[i] if i < len(msg.velocity) else math.nan for i in idx]
        self.joint_rows.append([self.sim_time(), time.time(), *q, *v])

    def on_controller(self, msg):
        if msg.joint_names != JOINTS:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.controller_rows.append([
            stamp, time.time(),
            msg.reference.positions[1], msg.reference.velocities[1],
            msg.reference.accelerations[1] if len(msg.reference.accelerations) > 1 else math.nan,
            msg.output.positions[1] if len(msg.output.positions) > 1 else math.nan,
            msg.feedback.positions[1], msg.feedback.velocities[1], msg.error.positions[1],
        ])

    def write(self, out):
        out = pathlib.Path(out)
        out.mkdir(parents=True, exist_ok=True)
        with (out / "joint_states_arm.csv").open("w", newline="") as f:
            csv.writer(f).writerows([
                ["sim_t", "wall_t", *["q_" + x for x in JOINTS], *["v_" + x for x in JOINTS]],
                *self.joint_rows,
            ])
        with (out / "controller_shoulder.csv").open("w", newline="") as f:
            csv.writer(f).writerows([
                ["sim_t", "wall_t", "reference_position", "reference_velocity",
                 "reference_acceleration", "output_position", "actual_position",
                 "actual_velocity", "error_position"],
                *self.controller_rows,
            ])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=80.0)
    args = ap.parse_args()
    rclpy.init()
    node = Recorder()
    try:
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.write(args.out)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

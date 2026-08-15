#!/usr/bin/env python3
# m3_send_gripper_once.py — minimal one-shot gripper command, for the
# MoveIt-executed version of the context-overshoot test
# (docs/m3_run_context_overshoot_test_v2.sh). Split out from
# m3_test_context_overshoot.py because that test's arm motion is now a real
# `m2_cartesian_approach` process running externally (needs move_group,
# which this script does not touch), not something this process can await
# directly -- the orchestrating bash script polls m2's own log for
# "execution reported SUCCESS" and fires this immediately on that match, to
# match the live m3_grasp trial's own timing as closely as possible.
#
# Usage: m3_send_gripper_once.py <target> <label> <markers_file> [--reset-first]
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts", "lib"))
import gz_settle  # noqa: E402

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import GripperCommand

GZ_JS = "/world/empty/model/ur5e_robotiq/joint_state"
GRIPPER_JOINT = "robotiq_85_left_knuckle_joint"


class OneShot(Node):
    def __init__(self, markers_path):
        super().__init__("gripper_one_shot")
        self.markers = open(markers_path, "a")
        self.client = ActionClient(self, GripperCommand, "/gripper_controller/gripper_cmd")

    def mark(self, label):
        line = f"{label} t={time.time():.6f}"
        print(line)
        self.markers.write(line + "\n")
        self.markers.flush()

    def send(self, target, label):
        self.client.wait_for_server(timeout_sec=10.0)
        goal = GripperCommand.Goal()
        goal.command.position = target
        goal.command.max_effort = 50.0
        self.mark(f"{label}_send target={target}")
        send_fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=10.0)
        goal_handle = send_fut.result()
        result_fut = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_fut, timeout_sec=10.0)
        result = result_fut.result()
        self.mark(f"{label}_done")
        if result is not None:
            r = result.result
            print(f"    result: position={r.position:.4f} stalled={r.stalled} "
                  f"reached_goal={r.reached_goal}")


def main():
    target = float(sys.argv[1])
    label = sys.argv[2]
    markers_path = sys.argv[3]
    reset_first = "--reset-first" in sys.argv[4:]

    rclpy.init()
    node = OneShot(markers_path)

    if reset_first:
        node.send(0.0, f"{label}_RESET")
        gz_settle.settle_joints(GZ_JS, [GRIPPER_JOINT], 0.01, 5.0, 0.1)

    node.send(target, label)

    node.markers.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

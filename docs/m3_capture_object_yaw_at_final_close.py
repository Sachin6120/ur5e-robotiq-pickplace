#!/usr/bin/env python3
# m3_capture_object_yaw_at_final_close.py — one-shot: watch
# /<gripper_ctrl>/gripper_cmd/_action/status for the 3rd distinct goal id
# (gripper_close_and_hold() sends exactly 2 goals per call — close-toward-
# target, then hold-at-achieved — and it's called twice: once for pre-close,
# once for the main close. Order is [preclose-close, preclose-hold,
# MAIN-close, main-hold]. The 3rd is the main close -- "the final close" --
# confirmed against ur5e_pick_place/src/m3_grasp.cpp's gripper_close_and_hold()
# and the previous trial's own GOAL trace: id=120944ee was #3, 1.2s duration,
# ending in the STALLED result.
#
# On that goal's EXECUTING transition, immediately shells out to
# `gz topic -e -t /world/<world>/pose/info -n 1` (same ground-truth query
# m3_grasp.cpp itself uses for link poses) and pulls pick_target's pose,
# converts the quaternion to yaw, and prints it. Then exits -- one-shot, not
# a continuous poller.
#
# Usage: m3_capture_object_yaw_at_final_close.py <gripper_ctrl> <gz_world> <object_model_name>
import math
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray

STATUS_NAME = {
    0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING",
    4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED",
}


def gz_echo_pose(world, model_name, timeout_s=2.0):
    cmd = ["gz", "topic", "-e", "-t", f"/world/{world}/pose/info", "-n", "1"]
    try:
        out = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout_s
        ).stdout
    except subprocess.TimeoutExpired:
        return None
    return out


def parse_model_pose(dump, model_name):
    needle = f'name: "{model_name}"'
    name_pos = dump.find(needle)
    if name_pos == -1:
        return None
    pos_start = dump.find("position {", name_pos)
    ori_start = dump.find("orientation {", name_pos)
    # bound the search so we don't run past this entity into the next
    next_name = dump.find('name: "', name_pos + len(needle))
    end = next_name if next_name != -1 else len(dump)
    if pos_start == -1 or pos_start > end:
        pos_start = None
    if ori_start == -1 or ori_start > end:
        return None
    ori_block_end = dump.find("}", ori_start)
    ori_block = dump[ori_start:ori_block_end]

    def field(block, key, default=0.0):
        k = key + ":"
        i = block.find(k)
        if i == -1:
            return default
        return float(block[i + len(k):].split()[0])

    qx = field(ori_block, "x")
    qy = field(ori_block, "y")
    qz = field(ori_block, "z")
    qw = field(ori_block, "w", 1.0)

    x = y = z = None
    if pos_start is not None:
        pos_block_end = dump.find("}", pos_start)
        pos_block = dump[pos_start:pos_block_end]
        x = field(pos_block, "x")
        y = field(pos_block, "y")
        z = field(pos_block, "z")

    # yaw (Z rotation) from quaternion, standard formula
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return {"x": x, "y": y, "z": z, "qx": qx, "qy": qy, "qz": qz, "qw": qw, "yaw_rad": yaw, "yaw_deg": math.degrees(yaw)}


class FinalCloseWatcher(Node):
    def __init__(self, action_ns, world, object_model):
        super().__init__("final_close_yaw_watcher")
        self.world = world
        self.object_model = object_model
        self.seen_status = {}
        self.goal_order = []  # goal ids in order first observed EXECUTING
        self.done = False
        topic = f"/{action_ns}/gripper_cmd/_action/status"
        self.sub = self.create_subscription(GoalStatusArray, topic, self.cb, 50)
        self.get_logger().info(f"watching {topic} for the 3rd EXECUTING goal (the final close)")

    def cb(self, msg):
        if self.done:
            return
        now = time.time()
        for entry in msg.status_list:
            gid = bytes(entry.goal_info.goal_id.uuid).hex()[:8]
            status = entry.status
            if self.seen_status.get(gid) == status:
                continue
            self.seen_status[gid] = status
            self.get_logger().info(f"GOAL {now:.6f} id={gid} status={STATUS_NAME.get(status, status)}")
            if status == 2 and gid not in self.goal_order:  # EXECUTING
                self.goal_order.append(gid)
                if len(self.goal_order) == 3:
                    self.get_logger().info(
                        f"goal #3 ({gid}) EXECUTING -- this is the final close. "
                        "querying pick_target ground truth NOW."
                    )
                    dump = gz_echo_pose(self.world, self.object_model)
                    if dump is None:
                        print("RESULT: gz pose query timed out")
                    else:
                        pose = parse_model_pose(dump, self.object_model)
                        if pose is None:
                            print(f"RESULT: could not find model '{self.object_model}' in pose/info dump")
                        else:
                            print(
                                f"RESULT: pick_target pose at final-close trigger: "
                                f"x={pose['x']:.6f} y={pose['y']:.6f} z={pose['z']:.6f}  "
                                f"quat=({pose['qx']:.6f},{pose['qy']:.6f},{pose['qz']:.6f},{pose['qw']:.6f})  "
                                f"yaw={pose['yaw_rad']:.6f} rad = {pose['yaw_deg']:.2f} deg"
                            )
                    self.done = True


def main():
    if len(sys.argv) != 4:
        print("usage: m3_capture_object_yaw_at_final_close.py <gripper_ctrl> <gz_world> <object_model_name>", file=sys.stderr)
        sys.exit(2)
    action_ns, world, object_model = sys.argv[1], sys.argv[2], sys.argv[3]

    rclpy.init()
    node = FinalCloseWatcher(action_ns, world, object_model)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if node.done else 1)


if __name__ == "__main__":
    main()

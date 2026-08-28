#!/usr/bin/env python3
"""One-shot direct FJT replay of a captured Cartesian arm trajectory.

This is deliberately independent of MoveIt and perception.  It first restores
the captured first point with a direct controller goal, verifies a settled
start state, then sends the captured goal unchanged and records controller and
ROS joint-state streams with both simulation and wall-clock timestamps.
"""

import argparse
import csv
import json
import math
import pathlib
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]


def duration(ns):
    from builtin_interfaces.msg import Duration
    value = Duration()
    value.sec = ns // 1_000_000_000
    value.nanosec = ns % 1_000_000_000
    return value


def parse_capture(path):
    rows = pathlib.Path(path).read_text().splitlines()
    points = []
    for line in rows:
        if not line.startswith("point "):
            continue
        fields = line.split()
        t = int(fields[fields.index("time_ns") + 1])
        def values(name):
            i = fields.index(name) + 1
            n = int(fields[i])
            return [float(x) for x in fields[i + 1:i + 1 + n]]
        p = JointTrajectoryPoint()
        p.positions = values("positions")
        p.velocities = values("velocities")
        p.accelerations = values("accelerations")
        p.time_from_start = duration(t)
        points.append(p)
    if len(points) != 20 or any(len(p.positions) != 6 for p in points):
        raise RuntimeError(f"expected exactly 20 six-joint points, got {len(points)}")
    return points


class Replay(Node):
    def __init__(self, points, out):
        super().__init__("direct_cartesian_replay")
        self.set_parameters([Parameter("use_sim_time", value=True)])
        self.points = points
        self.out = pathlib.Path(out)
        self.out.mkdir(parents=True, exist_ok=False)
        self.last_joint = None
        self.controller_rows = []
        self.joint_rows = []
        self.replay_active = False
        self.action = ActionClient(self, FollowJointTrajectory,
                                   "/arm_controller/follow_joint_trajectory")
        self.create_subscription(JointState, "/joint_states", self.on_joint, 1000)
        self.create_subscription(JointTrajectoryControllerState,
                                 "/arm_controller/controller_state",
                                 self.on_controller, 1000)

    def sim_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_joint(self, msg):
        if not all(j in msg.name for j in JOINTS):
            return
        index = [msg.name.index(j) for j in JOINTS]
        q = [msg.position[i] for i in index]
        v = [msg.velocity[i] if i < len(msg.velocity) else math.nan for i in index]
        self.last_joint = (q, v, self.sim_time(), time.time())
        if self.replay_active:
            self.joint_rows.append([self.sim_time(), time.time(), *q, *v])

    def on_controller(self, msg):
        if not self.replay_active or msg.joint_names != JOINTS:
            return
        s = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.controller_rows.append([
            s, time.time(),
            msg.reference.positions[1], msg.reference.velocities[1],
            msg.output.positions[1] if len(msg.output.positions) > 1 else math.nan,
            msg.feedback.positions[1], msg.feedback.velocities[1],
            msg.error.positions[1],
        ])

    def send(self, points):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = JOINTS
        goal.trajectory.points = points
        future = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle or not handle.accepted:
            raise RuntimeError("FJT goal was not accepted")
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        wrapped = result.result()
        return {"status": wrapped.status, "error_code": wrapped.result.error_code,
                "error_string": wrapped.result.error_string}

    def wait_settled(self, target, position_eps=0.002, velocity_eps=0.002,
                     consecutive=20, timeout=20.0):
        deadline = time.monotonic() + timeout
        good = 0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_joint is None:
                continue
            q, v, _, _ = self.last_joint
            qerr = max(abs(a - b) for a, b in zip(q, target))
            vmax = max(abs(x) for x in v)
            if qerr <= position_eps and vmax <= velocity_eps:
                good += 1
                if good >= consecutive:
                    return qerr, vmax
            else:
                good = 0
        raise RuntimeError("start state did not settle within limits")

    def run(self):
        if not self.action.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("arm_controller FJT action unavailable")
        if self.last_joint is None:
            deadline = time.monotonic() + 15.0
            while self.last_joint is None and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
        if self.last_joint is None:
            raise RuntimeError("no /joint_states received")

        restore = JointTrajectoryPoint()
        restore.positions = self.points[0].positions
        restore.velocities = [0.0] * 6
        restore.accelerations = [0.0] * 6
        restore.time_from_start = duration(8_000_000_000)
        restore_result = self.send([restore])
        start_qerr, start_vmax = self.wait_settled(self.points[0].positions)
        q, v, start_sim, start_wall = self.last_joint

        outbound = {
            "joint_names": JOINTS,
            "points": [{"time_from_start_ns": p.time_from_start.sec * 1_000_000_000 + p.time_from_start.nanosec,
                        "positions": list(p.positions), "velocities": list(p.velocities),
                        "accelerations": list(p.accelerations), "effort": list(p.effort)} for p in self.points],
        }
        (self.out / "outbound_fjt_goal.json").write_text(json.dumps(outbound, indent=2) + "\n")
        preflight = {"restore_result": restore_result, "settled_start_positions": q,
                     "settled_start_velocities": v, "max_start_position_error": start_qerr,
                     "max_start_velocity": start_vmax, "start_sim_time": start_sim,
                     "start_wall_time": start_wall}
        (self.out / "preflight.json").write_text(json.dumps(preflight, indent=2) + "\n")

        self.replay_active = True
        replay_sim_start, replay_wall_start = self.sim_time(), time.time()
        result = self.send(self.points)
        replay_sim_end, replay_wall_end = self.sim_time(), time.time()
        self.replay_active = False
        result.update({"sim_start": replay_sim_start, "sim_end": replay_sim_end,
                       "sim_duration": replay_sim_end - replay_sim_start,
                       "wall_start": replay_wall_start, "wall_end": replay_wall_end,
                       "wall_duration": replay_wall_end - replay_wall_start})
        (self.out / "fjt_result.json").write_text(json.dumps(result, indent=2) + "\n")
        with (self.out / "controller_shoulder.csv").open("w", newline="") as f:
            csv.writer(f).writerows([["sim_t", "wall_t", "reference_position", "reference_velocity", "output_position", "actual_position", "actual_velocity", "error_position"], *self.controller_rows])
        with (self.out / "joint_states_arm.csv").open("w", newline="") as f:
            csv.writer(f).writerows([["sim_t", "wall_t", *["q_" + j for j in JOINTS], *["v_" + j for j in JOINTS]], *self.joint_rows])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Replay(parse_capture(args.capture), args.out)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

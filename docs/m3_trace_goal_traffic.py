#!/usr/bin/env python3
# m3_trace_goal_traffic.py — built 2026-08-10 to test three hypotheses for
# the 0.0960 -> 0.0912 rad m3_grasp.cpp log entry against a screenshot pair
# that showed a large, cube-displacing aperture change: (a) sim-degradation
# drift (too small in this log, ~0.005rad, to match a visible change), (b) a
# goal m3_grasp.cpp itself sent that the source read missed, (c) something
# OTHER than m3_grasp commanding the gripper -- every probe/cleanup script in
# this project reopens the gripper when it finishes, and a leftover process
# or late-delivered goal would show up nowhere in m3_grasp's own log.
#
# Two independent evidence streams, one file, one wall-clock timebase so
# they can be laid on top of each other after the fact:
#
#   GZ   <t> left_knuckle:pos=..,vel=.. right_knuckle:pos=..,vel=.. ...
#                               Gazebo's OWN joint_state topic (gz topic -e),
#                               NOT ros2_control's /joint_states -- same
#                               ground-truth discipline as gz_settle.py
#                               ("Evidence comes from Gazebo's own state
#                               topics, never /joint_states and never TF").
#                               Reuses gz_settle.py's own parser rather than
#                               re-deriving the regex. At the time this was
#                               written (2026-08-10), ALL SIX linkage joints
#                               were logged per sample (master + 5 mimic
#                               followers), not just the master -- see
#                               GzJointPoller's own 2026-08-10 update note.
#                               NOTE: the TENTH OVERRIDE (2026-08-12, after
#                               this script was written) changed both
#                               fingertip joints from continuous+mimic to
#                               type="fixed" (1 master + 3 mimic followers
#                               remain). Whether the two fixed joints still
#                               appear as independently reported joints in
#                               this topic, or are folded into their parent
#                               rigid body instead, has not been re-verified
#                               against this script's current output.
#   GOAL <t> id=.. status=..   the gripper action's ROS status topic
#                               (/<ctrl>/gripper_cmd/_action/status). This
#                               one is legitimately ROS-level, not
#                               gz-level -- Gazebo has no concept of a ROS
#                               action goal, so this is the only place "who
#                               sent this" can be observed at all. Every
#                               (goal_id, status) transition is logged once,
#                               on change, not every status-array tick.
#
# A process census (`ps -eo pid,lstart,cmd`, filtered to ros2/gz/python
# processes) is written once at the top, before the sim is even touched by
# this trial -- catches a leftover process BEFORE blaming a mechanism.
#
# UPDATE 2026-08-10: added a third stream for the loop-stall hypothesis (the
# overshoot magnitude implies ~0.85-0.91s of uncorrected motion at the
# joint's 0.5rad/s velocity limit -- consistent with the control loop
# stalling, the last-written command persisting, and the joint coasting
# open-loop until the loop resumes and corrects):
#
#   ROS_JS <t>                 arrival time of each /joint_states message.
#                               Deliberately ros2_control's OWN topic, not
#                               Gazebo's -- here that's correct, not a
#                               violation of the gz-truth discipline above:
#                               this stream diagnoses ROS's OWN control-loop
#                               cadence, not physical truth. Gaps much larger
#                               than the configured update_rate (500Hz ->
#                               ~2ms) are the signature of a stalled loop.
#
# Usage: m3_trace_goal_traffic.py <outfile> <duration_s> [gripper_ctrl] [gz_js_topic]
import os
import re
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts", "lib"))
import gz_settle  # noqa: E402  (reuse _gz_echo / _parse_joint_* — no re-derivation)

import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray
from sensor_msgs.msg import JointState

STATUS_NAME = {
    0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING",
    4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED",
}


def process_census():
    out = subprocess.run(
        ["ps", "-eo", "pid,lstart,cmd"], capture_output=True, text=True
    ).stdout
    keep = [
        line for line in out.splitlines()
        if re.search(r"ros2|gz sim|gz_settle|m3_grasp|probe_gripper|rclpy|move_group", line)
        and "grep" not in line
    ]
    return keep


class GzJointPoller(threading.Thread):
    # Polls Gazebo's own joint_state topic on a fixed cadence via
    # `gz topic -e -n 1` (same subprocess-per-sample pattern gz_settle.py
    # uses elsewhere in this project) rather than opening a raw gz-transport
    # subscription — consistent with every other script here, at the cost of
    # per-sample process-spawn overhead. Fine for a ~60s bracketed trial.
    #
    # UPDATE 2026-08-10: logs ALL SIX linkage joints per sample, not just the
    # master. Every trace before this watched the joint being dragged and
    # never the joints doing the dragging -- the hold-gap finding (master
    # moved 0.0956->0.2546rad with zero active goal, one sample at 3.5x its
    # own clamp) makes the follower velocities the direct evidence for
    # whether the 500/s mimic-correction branch in gz_ros2_control is what's
    # doing it, not just an inference from the master's trace alone. One
    # `gz topic -e` dump already contains every joint's state, so this costs
    # nothing extra per sample -- just parsing more fields out of data
    # already being fetched.
    LINKAGE = [
        "robotiq_85_left_knuckle_joint",
        "robotiq_85_right_knuckle_joint",
        "robotiq_85_left_inner_knuckle_joint",
        "robotiq_85_right_inner_knuckle_joint",
        "robotiq_85_left_finger_tip_joint",
        "robotiq_85_right_finger_tip_joint",
    ]

    def __init__(self, out, lock, t0, duration_s, topic, joint, poll_dt=0.08):
        super().__init__(daemon=True)
        self.out, self.lock, self.t0 = out, lock, t0
        self.duration_s, self.topic, self.joint, self.poll_dt = (
            duration_s, topic, joint, poll_dt,
        )
        self.samples = 0
        self._stop_evt = threading.Event()  # not `_stop` -- collides with threading.Thread's own private method

    def run(self):
        while not self._stop_evt.is_set() and (time.time() - self.t0) < self.duration_s:
            now = time.time()
            txt = gz_settle._gz_echo(self.topic, echo_timeout=1.0)
            positions = gz_settle._parse_joint_positions(txt)
            velocities = gz_settle._parse_joint_velocities(txt)
            if self.joint in positions:
                fields = []
                for name in self.LINKAGE:
                    p = positions.get(name)
                    v = velocities.get(name)
                    short = name.replace("robotiq_85_", "").replace("_joint", "")
                    fields.append(
                        f"{short}:pos={p:.6f},vel={v:.6f}"
                        if p is not None and v is not None
                        else f"{short}:pos=nan,vel=nan"
                    )
                with self.lock:
                    self.out.write(f"GZ {now:.6f} " + " ".join(fields) + "\n")
                    self.out.flush()
                self.samples += 1
            time.sleep(self.poll_dt)

    def stop(self):
        self._stop_evt.set()


class GoalStatusTracer(Node):
    def __init__(self, out, lock, t0, action_ns):
        super().__init__("goal_traffic_tracer")
        self.out, self.lock, self.t0 = out, lock, t0
        self.seen = {}
        self.topic = f"/{action_ns}/gripper_cmd/_action/status"
        self.sub = self.create_subscription(GoalStatusArray, self.topic, self.cb, 50)
        self.js_sub = self.create_subscription(JointState, "/joint_states", self.js_cb, 200)
        self.js_count = 0

    def cb(self, msg):
        now = time.time()
        for entry in msg.status_list:
            gid = bytes(entry.goal_info.goal_id.uuid).hex()[:8]
            status = entry.status
            if self.seen.get(gid) != status:
                self.seen[gid] = status
                with self.lock:
                    self.out.write(
                        f"GOAL {now:.6f} id={gid} status={STATUS_NAME.get(status, status)}\n"
                    )
                    self.out.flush()

    def js_cb(self, msg):
        now = time.time()
        with self.lock:
            self.out.write(f"ROS_JS {now:.6f}\n")
        self.js_count += 1


def main():
    outfile = sys.argv[1]
    duration_s = float(sys.argv[2])
    gripper_ctrl = sys.argv[3] if len(sys.argv) > 3 else "gripper_controller"
    gz_js_topic = (
        sys.argv[4] if len(sys.argv) > 4
        else "/world/empty/model/ur5e_robotiq/joint_state"
    )
    joint = "robotiq_85_left_knuckle_joint"

    out = open(outfile, "w")
    lock = threading.Lock()
    t0 = time.time()

    out.write(f"# t0={t0:.6f} joint={joint} gz_js_topic={gz_js_topic} "
              f"gripper_ctrl={gripper_ctrl} duration_s={duration_s}\n")
    out.write("# --- process census at trace start ---\n")
    for line in process_census():
        out.write(f"CENSUS {line}\n")
    out.write("# --- trace begins ---\n")
    out.flush()

    rclpy.init()
    node = GoalStatusTracer(out, lock, t0, gripper_ctrl)
    poller = GzJointPoller(out, lock, t0, duration_s, gz_js_topic, joint)
    poller.start()

    end = t0 + duration_s + 1.0
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    poller.stop()
    poller.join(timeout=2.0)
    print(
        f"gz samples={poller.samples} distinct goal ids={len(node.seen)} "
        f"ros joint_states samples={node.js_count}",
        file=sys.stderr,
    )
    node.destroy_node()
    rclpy.shutdown()
    out.close()


if __name__ == "__main__":
    main()

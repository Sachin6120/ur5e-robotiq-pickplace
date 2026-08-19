#!/usr/bin/env python3
# m3_capture_full_cycle_screenshots.py — visual answer to "what actually
# happened in the ejected cycle": is the object between the pads right after
# the close, and is it still there (or already gone) after the lift+dwell.
#
# Cycle 3 of the 3-cycle harness run showed pick_target's position/orientation
# byte-identical between the LIFT_DONE and TRANSPORT_DONE pose samples --
# sitting exactly at its spawn height the whole time, while wrist_3_link
# lifted and transported normally. The grasp reported SUCCESS at essentially
# the same achieved_grip_angle as the two passing cycles (0.4033 vs
# 0.4030/0.4033) and Stage 2's positioning check passed. That data says the
# object was never actually captured; it does not say WHY. This is the look.
#
# FOUR SCREENSHOTS, TWO TRIGGER SOURCES
#   BEFORE_CLOSE / CLOSE_RESULT: goal #3 (the main close) on
#   gripper_cmd/_action/status, same counting as m3_capture_stall_screenshots.py
#   (gripper_close_and_hold() sends exactly 2 goals per call, called once for
#   pre-close, once for the main close -- goal #3 EXECUTING is the main close
#   starting). FIXED here vs that script: this project's actual outcome on
#   every live trial so far is TIMED_OUT_HELD, whose goal transitions to
#   CANCELED, not SUCCEEDED -- the earlier script only watched for SUCCEEDED
#   and missed its own trigger once already (caught live, screenshot taken by
#   hand into the open window). This one fires on ANY terminal status
#   (SUCCEEDED, CANCELED, or ABORTED).
#
#   LIFT_DONE / TRANSPORT_DONE: tailed from the live grasp log (the same
#   marker text transport.cpp emits, watched the same way
#   scripts/11_m3_cycles.sh's own watcher does) rather than a second ROS
#   subscription -- lift/transport are MoveGroup planning events with no
#   client-observable action topic already in hand, and the log is already
#   being teed live for exactly this kind of watching.
#
# Usage: m3_capture_full_cycle_screenshots.py <gripper_ctrl> <object_model> <grasp_log_path> <out_dir>
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray

STATUS_NAME = {
    0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING",
    4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED",
}
TERMINAL = {4, 5, 6}


def gz_service_call(service, reqtype, reptype, req, timeout_s=3.0):
    cmd = [
        "gz", "service", "-s", service,
        "--reqtype", reqtype, "--reptype", reptype,
        "--timeout", str(int(timeout_s * 1000)), "--req", req,
    ]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s + 2)
        return out.returncode == 0, out.stdout, out.stderr
    except subprocess.TimeoutExpired:
        return False, "", "timeout"


def move_to(entity_name):
    return gz_service_call(
        "/gui/move_to", "gz.msgs.StringMsg", "gz.msgs.Boolean",
        f'data: "{entity_name}"',
    )


def screenshot(out_dir, label):
    ok, out, err = gz_service_call(
        "/gui/screenshot", "gz.msgs.StringMsg", "gz.msgs.Boolean",
        f'data: "{out_dir}"',
    )
    print(f"{label} screenshot: ok={ok} out={out.strip()} err={err.strip()}", flush=True)


class FullCycleWatcher(Node):
    def __init__(self, action_ns, object_model, out_dir):
        super().__init__("full_cycle_screenshot_watcher")
        self.object_model = object_model
        self.out_dir = out_dir
        self.seen_status = {}
        self.goal_order = []
        self.before_done = False
        self.close_result_done = False
        topic = f"/{action_ns}/gripper_cmd/_action/status"
        self.sub = self.create_subscription(GoalStatusArray, topic, self.cb, 50)
        self.get_logger().info(
            f"watching {topic}; will screenshot at goal #3 EXECUTING and its "
            "first terminal status (SUCCEEDED/CANCELED/ABORTED)"
        )
        ok, out, err = move_to(self.object_model)
        self.get_logger().info(f"move_to({self.object_model}) ok={ok} out={out.strip()} err={err.strip()}")

    def cb(self, msg):
        if self.close_result_done:
            return
        for entry in msg.status_list:
            gid = bytes(entry.goal_info.goal_id.uuid).hex()[:8]
            status = entry.status
            if self.seen_status.get(gid) == status:
                continue
            self.seen_status[gid] = status
            self.get_logger().info(f"GOAL id={gid} status={STATUS_NAME.get(status, status)}")

            if status == 2 and gid not in self.goal_order:  # EXECUTING
                self.goal_order.append(gid)
                if len(self.goal_order) == 3 and not self.before_done:
                    self.get_logger().info(f"goal #3 ({gid}) EXECUTING -- BEFORE_CLOSE screenshot")
                    screenshot(self.out_dir, "BEFORE_CLOSE")
                    self.before_done = True

            if (status in TERMINAL and len(self.goal_order) >= 3
                    and gid == self.goal_order[2] and not self.close_result_done):
                self.get_logger().info(
                    f"goal #3 ({gid}) reached terminal status "
                    f"{STATUS_NAME.get(status, status)} -- CLOSE_RESULT screenshot"
                )
                screenshot(self.out_dir, "CLOSE_RESULT")
                self.close_result_done = True


def tail_log_for_markers(log_path, out_dir, stop_flags):
    """Poll-tail the live grasp log (same file scripts/11_m3_cycles.sh's own
    watcher reads) for M3 STAGE 3 LIFT_DONE / STAGE 4 TRANSPORT_DONE, firing
    one screenshot each. Runs in its own thread -- rclpy's spin owns the main
    thread for the action-status side.
    """
    # Wait for the file to exist -- it's created by the trial script's `tee`
    # only once the m3_grasp launch actually starts.
    fh = None
    for _ in range(300):
        try:
            fh = open(log_path, "r")
            break
        except FileNotFoundError:
            time.sleep(0.1)
    if fh is None:
        print(f"LOG_WATCH: {log_path} never appeared", flush=True)
        return

    lift_done = transport_done = False
    with fh:
        while not stop_flags["stop"]:
            line = fh.readline()
            if not line:
                if lift_done and transport_done:
                    return
                time.sleep(0.05)
                continue
            if not lift_done and "M3 STAGE 3 LIFT_DONE" in line:
                screenshot(out_dir, "LIFT_DONE")
                lift_done = True
            elif not transport_done and "M3 STAGE 4 TRANSPORT_DONE" in line:
                screenshot(out_dir, "TRANSPORT_DONE")
                transport_done = True
                return


def main():
    if len(sys.argv) != 5:
        print(
            "usage: m3_capture_full_cycle_screenshots.py <gripper_ctrl> "
            "<object_model> <grasp_log_path> <out_dir>",
            file=sys.stderr,
        )
        sys.exit(2)
    action_ns, object_model, log_path, out_dir = sys.argv[1:5]

    rclpy.init()
    node = FullCycleWatcher(action_ns, object_model, out_dir)

    stop_flags = {"stop": False}
    log_thread = threading.Thread(
        target=tail_log_for_markers, args=(log_path, out_dir, stop_flags), daemon=True
    )
    log_thread.start()

    try:
        # Runs until both the close-result shot and the log thread have
        # finished, or SIGINT/SIGTERM from the caller.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            if node.close_result_done and not log_thread.is_alive():
                break
    except KeyboardInterrupt:
        pass
    finally:
        stop_flags["stop"] = True
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if node.close_result_done else 1)


if __name__ == "__main__":
    main()

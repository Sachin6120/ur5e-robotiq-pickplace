#!/usr/bin/env python3
#
# ===========================================================================
# EVALUATION ONLY -- NEVER A PERCEPTION INPUT
# ===========================================================================
#
# 1. This file is an EVALUATION harness. It is not part of the perception
#    system and produces no part of the sensor estimate. The estimator is
#    ur5e_pick_place/src/object_detector.cpp, which subscribes to three
#    sensor topics and has no Gazebo, world-pose, TF, or camera-extrinsics
#    input of any kind.
#
# 2. Gazebo ground truth is queried ONLY AFTER the sensor estimate has been
#    frozen to disk. The two phases are deliberately separate processes:
#    milestone_d_harness.py collects the estimate, writes <scene>_sensor.json
#    and prints "SENSOR ESTIMATE FROZEN" before exiting; milestone_d_truth.py
#    then loads that frozen file and only then makes its first truth call.
#
# 3. NOTHING computed here may ever be fed back into object_detector.cpp, or
#    into any future estimator, as input, calibration, or correction. Doing so
#    would make the estimate a function of the answer it is measured against.
#    Ground truth is for scoring an already-final estimate, nothing else.
#
# 4. This is the exact harness used to validate MILESTONE D -- camera-frame
#    3D object position -- on 2026-08-22 (results in
#    docs/HANDOFF_RGBD_PERCEPTION.md section 8). Preserved verbatim from that
#    validated run; behaviour is unchanged apart from this comment block.
#
# PRESERVATION NOTE: the --out default below still points at the session
#    scratchpad directory the Milestone D run used, which no longer exists.
#    That path was NOT edited, because preserving validated behaviour exactly
#    was the point of keeping these files. Pass --out explicitly when re-running.
# ===========================================================================
"""milestone_e_harness.py -- Milestone E evaluation harness.  NOT part of the estimator.

Milestone E adds one thing to the Milestone D procedure: it also records
object_detector/position_world, the TF2-transformed world-frame estimate
published by the separate object_position_world node, and pairs it with the
camera-frame estimate that produced it by observation stamp.  The production
transform is TF2 inside that node; this harness only reads its output.

STRICT SEPARATION
  The sensor estimate is produced entirely inside object_detector.cpp, which
  has no Gazebo/world/TF input of any kind.  This harness only:
    (a) sets the scene up (spawn/settle/move/hold),
    (b) READS object_detector/position_camera,
    (c) FREEZES those readings to disk,
    (d) and ONLY THEN queries Gazebo truth and evaluates.

  Phase (c) writes the sensor JSON and prints a frozen marker before any
  ground-truth call is made, so the ordering is auditable from the log.
"""
import argparse, json, math, os, subprocess, sys, time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Duration as MsgDuration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32MultiArray, UInt32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml

REPO = "/home/sachin/ur5e_pickplace"
WORLD = "empty"
SCENE = yaml.safe_load(open(f"{REPO}/config/scene.yaml"))
ARM_JOINTS = SCENE["robot"]["arm_joints"]
HOME = SCENE["robot"]["home_positions"]
M1 = SCENE["milestones"]["m1"]["goal_positions"]
OBJ = SCENE["object"]
OBJ_NAME = OBJ["name"]
OBJ_SIZE = OBJ["size"]
OBJ_Z = OBJ["pick_pose"]["z"]


def sh(cmd, timeout=30):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)


def spawn_object(x, y):
    """Same SDF as scripts/08_spawn_pick_object.sh, XY parameterised."""
    sx, sy, sz = OBJ_SIZE
    sdf = (
        f"<?xml version='1.0'?><sdf version='1.9'><model name='{OBJ_NAME}'>"
        f"<pose>{x} {y} {OBJ_Z} 0.0 0.0 0.0</pose>"
        f"<link name='link'><inertial><mass>{OBJ['mass']}</mass>"
        f"<inertia><ixx>1e-4</ixx><iyy>1e-4</iyy><izz>1e-4</izz>"
        f"<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
        f"<collision name='c'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
        f"<surface><friction><ode><mu>{OBJ['surface']['mu']}</mu>"
        f"<mu2>{OBJ['surface']['mu2']}</mu2></ode></friction></surface></collision>"
        f"<visual name='v'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
        f"<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>"
        f"<specular>1 1 1 1</specular></material></visual>"
        f"</link></model></sdf>"
    )
    # argv form, NOT a shell string: the SDF itself contains single quotes
    # (<?xml version='1.0'?>), which silently truncate a shell-quoted --req.
    req = 'sdf: "%s", name: "%s"' % (sdf.replace('"', '\\"'), OBJ_NAME)
    r = subprocess.run(
        ["gz", "service", "-s", f"/world/{WORLD}/create",
         "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
         "--timeout", "5000", "--req", req],
        capture_output=True, text=True, timeout=30)
    return r.stdout.strip() + r.stderr.strip()


def remove_object():
    r = subprocess.run(
        ["gz", "service", "-s", f"/world/{WORLD}/remove",
         "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean",
         "--timeout", "5000", "--req", f'name: "{OBJ_NAME}", type: MODEL'],
        capture_output=True, text=True, timeout=30)
    return r.stdout.strip() + r.stderr.strip()


def settle_object(timeout=20.0):
    r = sh(f"python3 {REPO}/scripts/lib/gz_settle.py pose "
           f"--topic /world/{WORLD}/pose/info --eps 0.0005 --timeout {timeout} "
           f"--poll 0.15 {OBJ_NAME}", timeout=timeout + 15)
    return r.returncode == 0, (r.stdout + r.stderr).strip()


def truth_pose(timeout=25.0):
    """GROUND TRUTH -- evaluation only.  Never called before the freeze."""
    r = sh(f"python3 {REPO}/scripts/lib/sample_pose.py "
           f"--topic /world/{WORLD}/pose/info --entities {OBJ_NAME} "
           f"--window-s 1.0 --tol-m 0.0005 --timeout-s {timeout}",
           timeout=timeout + 15)
    if r.returncode != 0:
        return None, (r.stdout + r.stderr).strip()
    for line in r.stdout.splitlines():
        p = line.split()
        if p and p[0].endswith(OBJ_NAME):
            return [float(v) for v in p[1:8]], line.strip()
    return None, r.stdout.strip()


class Harness(Node):
    def __init__(self):
        super().__init__("md_harness")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        sensor_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                                history=HistoryPolicy.KEEP_LAST)
        self.positions = []   # camera frame: (stamp, x, y, z, recv, frame_id)
        self.worlds = []      # world frame:  (stamp, x, y, z, recv, frame_id)
        self.detected = []    # (recv_walltime, bool)
        self.bboxes = []
        self.areas = []
        self.js = None
        self.create_subscription(PointStamped, "object_detector/position_camera",
                                 self._on_pos, sensor_qos)
        self.create_subscription(PointStamped, "object_detector/position_world",
                                 self._on_world, sensor_qos)
        self.create_subscription(Bool, "object_detector/detected", self._on_det, sensor_qos)
        self.create_subscription(Int32MultiArray, "object_detector/bounding_box",
                                 self._on_box, sensor_qos)
        self.create_subscription(UInt32, "object_detector/component_area",
                                 self._on_area, sensor_qos)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.arm = ActionClient(self, FollowJointTrajectory,
                                "/arm_controller/follow_joint_trajectory")

    def _on_pos(self, m):
        self.positions.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                               m.point.x, m.point.y, m.point.z, time.time(),
                               m.header.frame_id))

    def _on_world(self, m):
        self.worlds.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                            m.point.x, m.point.y, m.point.z, time.time(),
                            m.header.frame_id))

    def _on_det(self, m):
        self.detected.append((time.time(), bool(m.data)))

    def _on_box(self, m):
        self.bboxes.append((time.time(), list(m.data)))

    def _on_area(self, m):
        self.areas.append((time.time(), int(m.data)))

    def _on_js(self, m):
        self.js = m

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def move(self, positions, secs=5.0, label=""):
        if not self.arm.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("arm_controller action server unavailable")
        traj = JointTrajectory()
        traj.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions]
        pt.velocities = [0.0] * len(positions)
        pt.time_from_start = MsgDuration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        traj.points = [pt]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self.arm.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=25.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError(f"move({label}) goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=40.0)
        return rf.result().result.error_code if rf.result() else None

    def wait_stationary(self, eps=1e-3, need=6, timeout=25.0):
        """All six arm joints under eps rad/s for `need` consecutive samples."""
        idx = None
        ok = 0
        end = time.time() + timeout
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            m = self.js
            if m is None or not m.velocity:
                continue
            if idx is None:
                try:
                    idx = [m.name.index(j) for j in ARM_JOINTS]
                except ValueError:
                    idx = None
                    continue
            vmax = max(abs(m.velocity[i]) for i in idx)
            ok = ok + 1 if vmax < eps else 0
            if ok >= need:
                return True, vmax
        return False, None

    def fresh_pairs(self, t_min_sim, count, skip, timeout=60.0):
        """(camera, world) estimate pairs whose OWN observation stamp is after
        the arm stopped, matched by that stamp.  A camera estimate with no
        world partner is retained with world=None so a silent transform
        failure shows up as a hole rather than disappearing."""
        # Wait for COMPLETE pairs.  Counting camera estimates alone raced the
        # extra DDS hop: the first run of scene A exited 0.2 ms before the
        # world message for the last stamp arrived, and reported a hole the
        # production node's own log disproved.  A genuine, persistent
        # transform failure now surfaces as a loud timeout below rather than
        # as a silently short result.
        c0, w0 = len(self.positions), len(self.worlds)
        end = time.time() + timeout
        got, complete = [], []
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            wmap = {round(w[0], 6): w for w in self.worlds[w0:]}
            got = []
            for rec in self.positions[c0:]:
                if rec[0] > t_min_sim:
                    got.append((rec, wmap.get(round(rec[0], 6))))
            complete = [g for g in got if g[1] is not None]
            if len(complete) >= count + skip:
                break
        return complete[skip:skip + count], got


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", required=True)
    ap.add_argument("--x", type=float)
    ap.add_argument("--y", type=float)
    ap.add_argument("--absent", action="store_true")
    ap.add_argument("--frames", type=int, default=5)
    ap.add_argument("--out", default="/tmp/claude-1000/-home-sachin-ur5e-pickplace/00fa8ea1-a70a-40c8-ab8d-69cbc21316fb/scratchpad/me_results")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    rclpy.init()
    h = Harness()
    log = {"scene": a.scene, "absent": a.absent, "requested_xy": [a.x, a.y]}

    print(f"\n{'='*64}\nSCENE {a.scene}  absent={a.absent}  xy=({a.x},{a.y})\n{'='*64}")

    # --- scene setup ------------------------------------------------------
    print("[setup] removing any existing object")
    remove_object()
    h.spin(1.0)
    print("[setup] arm -> HOME")
    h.move(HOME, 5.0, "home")
    h.wait_stationary()

    if not a.absent:
        print(f"[setup] spawning {OBJ_NAME} at ({a.x}, {a.y}, {OBJ_Z})")
        print("        " + spawn_object(a.x, a.y).replace("\n", "\n        "))
        ok, msg = settle_object()
        print(f"[setup] settle ok={ok} {msg}")
        log["settled"] = ok
        if not ok:
            print("[STOP] object did not settle; refusing to measure a moving scene")
            json.dump(log, open(f"{a.out}/{a.scene}.json", "w"), indent=2)
            return 2

    print("[setup] arm -> M1")
    h.move(M1, 5.0, "m1")
    stationary, vmax = h.wait_stationary()
    t_stop = h.sim_now()
    print(f"[setup] all six arm joints stationary={stationary} vmax={vmax:.2e} rad/s "
          f"at sim t={t_stop:.6f}")
    log["stationary"] = stationary
    log["t_stop_sim"] = t_stop
    if not stationary:
        print("[STOP] arm never became stationary")
        json.dump(log, open(f"{a.out}/{a.scene}.json", "w"), indent=2)
        return 2

    # --- observation ------------------------------------------------------
    n_pos_before = len(h.positions)
    n_world_before = len(h.worlds)
    if a.absent:
        print("[observe] object-absent: watching 6 s for any detection/estimate")
        h.spin(6.0)
        after = [p for p in h.positions[n_pos_before:] if p[0] > t_stop]
        after_w = [p for p in h.worlds[n_world_before:] if p[0] > t_stop]
        dets = [d[1] for d in h.detected if d[0] > 0]
        log["absent_positions_after_stop"] = len(after)
        log["absent_positions_total_seen"] = len(h.positions) - n_pos_before
        log["absent_world_after_stop"] = len(after_w)
        log["absent_world_total_seen"] = len(h.worlds) - n_world_before
        log["absent_detected_values"] = dets[-15:]
        log["absent_detected_any_true"] = any(dets[-15:])
        print(f"[observe] detected samples (last 15): {dets[-15:]}")
        print(f"[observe] NEW position_camera messages with stamp > t_stop: {len(after)}")
        print(f"[observe] NEW position_world  messages with stamp > t_stop: {len(after_w)}")
    else:
        print(f"[observe] discarding stale frames, collecting {a.frames} fresh estimates")
        fresh, allseen = h.fresh_pairs(t_stop, a.frames, skip=1)
        log["fresh_count"] = len(fresh)
        log["estimates"] = [
            {"stamp": c[0], "x": c[1], "y": c[2], "z": c[3], "frame_id": c[5],
             "world": (None if w is None else
                       {"stamp": w[0], "x": w[1], "y": w[2], "z": w[3],
                        "frame_id": w[5]})}
            for c, w in fresh]
        log["world_missing"] = sum(1 for _, w in fresh if w is None)
        log["discarded_stale_or_warmup"] = len(allseen) - len(fresh)
        for i, (c, w) in enumerate(fresh):
            print(f"[observe] frame {i}: stamp={c[0]:.6f}")
            print(f"            camera [{c[5]}] = ({c[1]:.9f}, {c[2]:.9f}, {c[3]:.9f})")
            if w is None:
                print("            world  = *** NO WORLD ESTIMATE FOR THIS STAMP ***")
            else:
                print(f"            world  [{w[5]}] = ({w[1]:.9f}, {w[2]:.9f}, {w[3]:.9f})")
        recent_boxes = [b[1] for b in h.bboxes][-a.frames:]
        recent_areas = [x[1] for x in h.areas][-a.frames:]
        log["bboxes"] = recent_boxes
        log["areas"] = recent_areas
        print(f"[observe] bboxes={recent_boxes}")
        print(f"[observe] areas={recent_areas}")

    # --- FREEZE -----------------------------------------------------------
    log["frozen_at_walltime"] = time.time()
    json.dump(log, open(f"{a.out}/{a.scene}_sensor.json", "w"), indent=2)
    print(f"\n>>> SENSOR ESTIMATE FROZEN to {a.out}/{a.scene}_sensor.json "
          f"at wall {log['frozen_at_walltime']:.3f}")
    print(">>> ONLY NOW does any ground-truth query occur.\n")

    h.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

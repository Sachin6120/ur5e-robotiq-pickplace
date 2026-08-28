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
"""milestone_e_truth.py -- Milestone E GROUND-TRUTH EVALUATION ONLY.

Separate process, run only after milestone_e_harness.py has frozen the WORLD
estimate to disk.  Compares the TF2-produced world-frame sensor estimate
against the top-surface truth built from the runtime Gazebo pose.

The analytic transform in here is a DIAGNOSTIC CROSS-CHECK of TF only.  The
production path is TF2 inside object_position_world; nothing computed in this
file is or can be substituted for it.
"""
import argparse, json, math, subprocess, sys, time

import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
import yaml

REPO = "/home/sachin/ur5e_pickplace"
WORLD = "empty"
SCENE = yaml.safe_load(open(f"{REPO}/config/scene.yaml"))
OBJ = SCENE["object"]
OBJ_NAME = OBJ["name"]
SIZE = OBJ["size"]


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def analytic_optical_to_world(p_opt):
    """DIAGNOSTIC ONLY.  URDF mount constants, never used in production."""
    t = np.array([0.450, 0.025, 2.400])
    R = rpy_matrix(0.0, math.pi / 2, 0.0) @ rpy_matrix(-math.pi / 2, 0.0, -math.pi / 2)
    return R @ np.array(p_opt) + t


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def truth_pose(timeout=25.0):
    r = subprocess.run(
        f"python3 {REPO}/scripts/lib/sample_pose.py --topic /world/{WORLD}/pose/info "
        f"--entities {OBJ_NAME} --window-s 1.0 --tol-m 0.0005 --timeout-s {timeout}",
        shell=True, capture_output=True, text=True, timeout=timeout + 15)
    if r.returncode != 0:
        return None, (r.stdout + r.stderr).strip()
    for line in r.stdout.splitlines():
        p = line.split()
        if p and p[0].endswith(OBJ_NAME):
            return [float(v) for v in p[1:8]], line.strip()
    return None, r.stdout.strip()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", required=True)
    ap.add_argument("--out", default="/tmp/claude-1000/-home-sachin-ur5e-pickplace/00fa8ea1-a70a-40c8-ab8d-69cbc21316fb/scratchpad/me_results")
    a = ap.parse_args()

    sensor = json.load(open(f"{a.out}/{a.scene}_sensor.json"))
    print(f"[truth] loaded FROZEN sensor file written at wall "
          f"{sensor['frozen_at_walltime']:.3f}; now {time.time():.3f} "
          f"(+{time.time() - sensor['frozen_at_walltime']:.1f}s)")

    rclpy.init()
    node = Node("milestone_e_truth")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    end, tr = time.time() + 10.0, None
    while time.time() < end and tr is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            tr = buf.lookup_transform("world", "camera_optical_frame", rclpy.time.Time())
        except Exception:
            pass
    if tr is None:
        print("[STOP] could not look up world <- camera_optical_frame")
        return 2

    t = tr.transform.translation
    q = tr.transform.rotation
    R_tf = quat_to_R(q.x, q.y, q.z, q.w)
    print("\n--- TF CONSISTENCY CHECK: world <- camera_optical_frame ---")
    print(f"  translation = [{t.x:.9f}, {t.y:.9f}, {t.z:.9f}]")
    print(f"  quaternion  = [x={q.x:.9f}, y={q.y:.9f}, z={q.z:.9f}, w={q.w:.9f}]")
    print(f"  rotation matrix =\n{R_tf}")
    for i, ax in enumerate("XYZ"):
        v = R_tf[:, i]
        print(f"  optical +{ax} -> world [{v[0]:+.6f}, {v[1]:+.6f}, {v[2]:+.6f}]")

    # DIAGNOSTIC cross-check on one frozen camera point.
    est0 = sensor["estimates"][0]
    cam0 = [est0["x"], est0["y"], est0["z"]]
    ana0 = analytic_optical_to_world(cam0)
    prod0 = np.array([est0["world"]["x"], est0["world"]["y"], est0["world"]["z"]])
    print("\n--- TF2 vs ANALYTIC (diagnostic only; production stays TF2) ---")
    print(f"  frozen camera point   = {cam0}")
    print(f"  production TF2 world  = {prod0}")
    print(f"  analytic URDF world   = {ana0}")
    print(f"  disagreement          = {np.linalg.norm(prod0 - ana0) * 1000:.9f} mm")

    truth, raw = truth_pose()
    if truth is None:
        print(f"[STOP] could not sample settled truth: {raw}")
        return 2
    centre = np.array(truth[:3])
    qx, qy, qz, qw = truth[3:7]
    ident = abs(qx) < 1e-9 and abs(qy) < 1e-9 and abs(qz) < 1e-9 and abs(abs(qw) - 1) < 1e-9
    print(f"\n[truth] gz settled pose: {raw}")
    print(f"[truth] runtime orientation identity (upright, zero rotation)? {ident}")
    if not ident:
        print("[STOP] object is not axis-aligned; the +size_z/2 top-surface "
              "construction below assumes it is.  Refusing to guess.")
        return 2
    half_h = SIZE[2] / 2.0
    top = centre + np.array([0.0, 0.0, half_h])
    print(f"[truth] object centre (world)      = {centre}")
    print(f"[truth] object height (scene.yaml size[2]) = {SIZE[2]:.6f} m, half = {half_h:.6f} m")
    print(f"[truth] TOP-SURFACE centre (world) = {top}")

    out = {"scene": a.scene, "truth_centre_world": centre.tolist(),
           "truth_top_world": top.tolist(),
           "tf_translation": [t.x, t.y, t.z],
           "tf_quaternion": [q.x, q.y, q.z, q.w],
           "tf_vs_analytic_mm": float(np.linalg.norm(prod0 - ana0) * 1000),
           "results": []}

    print(f"\n{'frame':>5} {'world X':>13} {'world Y':>13} {'world Z':>13} "
          f"{'dX mm':>9} {'dY mm':>9} {'dZ mm':>9} {'euclid mm':>10}")
    for i, e in enumerate(sensor["estimates"]):
        if e["world"] is None:
            print(f"{i:>5}   *** NO WORLD ESTIMATE ***")
            continue
        est = np.array([e["world"]["x"], e["world"]["y"], e["world"]["z"]])
        d = est - top
        eu = float(np.linalg.norm(d))
        out["results"].append({"frame": i, "world_est": est.tolist(),
                               "camera_est": [e["x"], e["y"], e["z"]],
                               "frame_id": e["world"]["frame_id"],
                               "d": d.tolist(), "euclid_m": eu})
        print(f"{i:>5} {est[0]:>13.9f} {est[1]:>13.9f} {est[2]:>13.9f} "
              f"{d[0]*1000:>9.4f} {d[1]*1000:>9.4f} {d[2]*1000:>9.4f} {eu*1000:>10.4f}")

    if out["results"]:
        E = np.array([r["world_est"] for r in out["results"]])
        mean = E.mean(axis=0)
        out["repeat_mean"] = mean.tolist()
        out["repeat_std"] = E.std(axis=0).tolist()
        out["repeat_max_abs_coord_dev_m"] = float(np.abs(E - mean).max())
        out["repeat_max_euclid_dev_m"] = float(np.linalg.norm(E - mean, axis=1).max())
        print(f"\n[repeat] world mean = {mean}")
        print(f"[repeat] world std  = {E.std(axis=0)}")
        print(f"[repeat] max |coord dev| = {out['repeat_max_abs_coord_dev_m']*1000:.9f} mm")
        print(f"[repeat] max euclid dev  = {out['repeat_max_euclid_dev_m']*1000:.9f} mm")
        print(f"[repeat] frame_id(s) = {sorted({r['frame_id'] for r in out['results']})}")

    json.dump(out, open(f"{a.out}/{a.scene}_truth.json", "w"), indent=2)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

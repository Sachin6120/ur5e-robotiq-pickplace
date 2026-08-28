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
# The --out default below is a portable path derived from this file's own
#    location, not the machine-specific one the original validated run used.
#    Pass --out explicitly to write elsewhere.
# ===========================================================================
"""md_truth.py -- Milestone D GROUND-TRUTH EVALUATION ONLY.

Runs as a SEPARATE PROCESS, after md_harness.py has already frozen the
sensor estimate to disk.  Nothing computed here can reach object_detector.cpp:
the estimator has no subscription, parameter, or file that this touches.

Builds the top-surface-centre truth (object centre + size_z/2 in world Z),
transforms it into camera_optical_frame, and compares.  No half-height term
is ever removed from or added to the SENSOR estimate.
"""
import argparse, json, math, subprocess, sys, time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
import yaml

REPO = str(Path(__file__).resolve().parents[2])
WORLD = "empty"
SCENE = yaml.safe_load(open(f"{REPO}/config/scene.yaml"))
OBJ = SCENE["object"]
OBJ_NAME = OBJ["name"]
SIZE_Z = OBJ["size"][2]


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def analytic_world_to_optical():
    """From the URDF constants alone -- an independent check on TF."""
    t = np.array([0.450, 0.025, 2.400])
    R_body = rpy_matrix(0.0, math.pi / 2, 0.0)
    R_opt_in_body = rpy_matrix(-math.pi / 2, 0.0, -math.pi / 2)
    R = R_body @ R_opt_in_body
    return R, t


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
    ap.add_argument("--out", default="/tmp/ur5e_pickplace/md_results")
    a = ap.parse_args()

    sensor = json.load(open(f"{a.out}/{a.scene}_sensor.json"))
    print(f"[truth] loaded FROZEN sensor file written at wall "
          f"{sensor['frozen_at_walltime']:.3f}; now {time.time():.3f} "
          f"(+{time.time() - sensor['frozen_at_walltime']:.1f}s)")

    rclpy.init()
    node = Node("md_truth")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    end = time.time() + 10.0
    tf_ok, tr = False, None
    while time.time() < end and not tf_ok:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            tr = buf.lookup_transform("camera_optical_frame", "world",
                                      rclpy.time.Time())
            tf_ok = True
        except Exception:
            pass

    R_a, t_a = analytic_world_to_optical()
    print(f"[truth] analytic R(world->optical columns) =\n{R_a}")
    print(f"[truth] analytic optical axes in world: "
          f"+X={R_a[:, 0]}  +Y={R_a[:, 1]}  +Z={R_a[:, 2]}")

    def world_to_optical_analytic(p):
        return R_a.T @ (np.array(p) - t_a)

    def world_to_optical_tf(p):
        q = tr.transform.rotation
        tt = tr.transform.translation
        w, x, y, z = q.w, q.x, q.y, q.z
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return R @ np.array(p) + np.array([tt.x, tt.y, tt.z])

    truth, raw = truth_pose()
    if truth is None:
        print(f"[STOP] could not sample settled truth: {raw}")
        return 2
    print(f"[truth] gz settled object pose: {raw}")
    centre = truth[:3]
    top = [centre[0], centre[1], centre[2] + SIZE_Z / 2.0]
    print(f"[truth] object centre (world)      = {centre}")
    print(f"[truth] TOP-SURFACE centre (world) = {top}   (+size_z/2 = {SIZE_Z/2:.6f})")

    top_opt_a = world_to_optical_analytic(top)
    print(f"[truth] top-surface centre in camera_optical_frame (analytic) = {top_opt_a}")
    if tf_ok:
        top_opt_tf = world_to_optical_tf(top)
        print(f"[truth] same, via runtime TF                                  = {top_opt_tf}")
        print(f"[truth] analytic-vs-TF agreement = "
              f"{np.linalg.norm(top_opt_a - top_opt_tf)*1000:.6f} mm")
    else:
        top_opt_tf = None
        print("[truth] WARNING: TF lookup failed; analytic transform only")

    ref = top_opt_tf if tf_ok else top_opt_a
    out = {"scene": a.scene, "truth_centre_world": centre,
           "truth_top_world": top,
           "truth_top_optical_analytic": top_opt_a.tolist(),
           "truth_top_optical_tf": (top_opt_tf.tolist() if tf_ok else None),
           "tf_ok": tf_ok, "results": []}

    print(f"\n{'frame':>5} {'est X':>13} {'est Y':>13} {'est Z':>13} "
          f"{'dX mm':>9} {'dY mm':>9} {'dZ mm':>9} {'euclid mm':>10}")
    for i, e in enumerate(sensor.get("estimates", [])):
        est = np.array([e["x"], e["y"], e["z"]])
        d = est - ref
        eu = float(np.linalg.norm(d))
        out["results"].append({"frame": i, "est": est.tolist(),
                               "d": d.tolist(), "euclid_m": eu})
        print(f"{i:>5} {est[0]:>13.9f} {est[1]:>13.9f} {est[2]:>13.9f} "
              f"{d[0]*1000:>9.4f} {d[1]*1000:>9.4f} {d[2]*1000:>9.4f} {eu*1000:>10.4f}")

    if out["results"]:
        E = np.array([r["est"] for r in out["results"]])
        mean = E.mean(axis=0)
        dev = np.linalg.norm(E - mean, axis=1)
        out["repeat_mean"] = mean.tolist()
        out["repeat_std"] = E.std(axis=0).tolist()
        out["repeat_max_abs_coord_dev_m"] = float(np.abs(E - mean).max())
        out["repeat_max_euclid_dev_m"] = float(dev.max())
        print(f"\n[repeat] mean XYZ = {mean}")
        print(f"[repeat] std  XYZ = {E.std(axis=0)}  "
              f"({E.std(axis=0)*1000} mm)")
        print(f"[repeat] max |coord dev| = {out['repeat_max_abs_coord_dev_m']*1000:.6f} mm")
        print(f"[repeat] max euclid dev  = {out['repeat_max_euclid_dev_m']*1000:.6f} mm")

    json.dump(out, open(f"{a.out}/{a.scene}_truth.json", "w"), indent=2)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

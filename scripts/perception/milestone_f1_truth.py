#!/usr/bin/env python3
#
# ===========================================================================
# EVALUATION ONLY -- NEVER A PERCEPTION INPUT
# ===========================================================================
# 1. Evaluation harness for Milestone F1. It produces no part of the sensor
#    estimate and no part of the motion target. Perception lives in
#    object_detector.cpp / object_position_world.cpp; the target is composed
#    inside m3_grasp.cpp from the perceived point and the configured geometry.
# 2. Gazebo truth is queried only by milestone_f1_truth.py, a separate
#    process, and only after this script has frozen the run to disk.
# 3. Nothing computed here may be fed back into perception or planning.
# 4. Used to validate MILESTONE F1 (perception-derived pre-grasp).
#
# PRECONDITION: the simulation stack is already up (sim + move_group, and for
# perceived runs the camera, object_detector and object_position_world).
# ===========================================================================
"""milestone_f1_truth.py -- Milestone F1/F2/F3 GROUND-TRUTH EVALUATION ONLY.

Builds the truth-derived pre-grasp / grasp target from the object's
PRE-MANIPULATION Gazebo pose and the SAME classical geometry the node used
(grasp_frame rotation from TF, grasp.standoff from scene.yaml), then compares
it with the perception-derived target the node actually commanded. Truth never
reaches the node: m3_grasp receives only launch flags, and its object position
comes solely from object_detector/position_world.

TIMING/ORDERING FIX, 2026-08-23 -- the defect the F2 evidence regeneration
documented (evidence/f2_0070_regeneration_20260823_114505/README.md, limitation
2). This file was written for `pregrasp_only`, where the object is never
touched, so a LIVE post-run Gazebo sample of the object is also its
pre-manipulation pose. Under `grasp_only` (F2) and `grasp_only` + lift (F3)
that is false, and the comparison was invalid in TWO independent ways:

  1. BASELINE. The live sample is taken after final closure has SEATED the
     object by the known ~21.6-22.3 mm. Scene A: truth_centre read
     [0.428656, -0.149986, 0.775523] against the spawn [0.450, -0.150,
     0.772500]. Every truth-derived target built from it was therefore
     displaced by the seating.
  2. STAGE. m3_grasp.cpp reassigns commanded_tcp at line ~1164: it holds the
     PRE-GRASP tcp only while the run stops at pregrasp_only; from the descent
     onward it holds the GRASP tcp. This file compared it against a
     truth-derived PRE-GRASP regardless, baking in the whole
     grasp.standoff = 0.100 m.

Together those produced scene A's ~105.4 mm "error", which is
0.1000 m (standoff, defect 2) + 0.0216 m (seating, defect 1) and no error
at all. The fix:

  * the truth BASELINE for any run that touched the object is the harness's
    frozen pre-manipulation `object_pose_before` (settled, sampled before
    m3_grasp was launched). It is never the live post-run sample.
  * the truth TARGET compared against is chosen to MATCH the stage the CSV's
    commanded_tcp actually holds, and the choice is recorded in the JSON as
    `compared_stage` so it cannot be misread again.
  * the live post-run object pose is still sampled and reported, but only as
    seating / drop / displacement evidence under its own key.

pregrasp_only behaviour is unchanged: no manipulation, so the frozen baseline
and the live sample are the same pose, and the compared stage is the pre-grasp.
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
SIZE_Z = OBJ["size"][2]
STANDOFF = float(SCENE["grasp"]["standoff"])


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# wrist_3_link -> flange -> tool0: the SAME fixed constants m3_grasp.cpp uses
# (kR_wrist3_to_flange / kR_flange_to_tool0). Reproduced here so the achieved
# ORIENTATION can be measured; m3_grasp's own pre-grasp check is translation
# only, matching its classical Stage-2 check.
def rpy_R(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


R_WRIST3_TO_FLANGE = rpy_R(0.0, -math.pi / 2, -math.pi / 2)
R_FLANGE_TO_TOOL0 = rpy_R(math.pi / 2, 0.0, math.pi / 2)


def achieved_tool0_orientation(timeout=25.0):
    """Measured tool0 orientation from Gazebo. A measurement of what the arm
    did, taken after the motion completed -- it cannot influence the motion."""
    r = subprocess.run(
        f"python3 {REPO}/scripts/lib/sample_pose.py --topic /world/{WORLD}/pose/info "
        f"--entities wrist_3_link --window-s 1.0 --tol-m 0.0005 --timeout-s {timeout}",
        shell=True, capture_output=True, text=True, timeout=timeout + 15)
    if r.returncode != 0:
        return None
    for line in r.stdout.splitlines():
        p = line.split()
        if p and p[0].endswith("wrist_3_link"):
            v = [float(x) for x in p[1:8]]
            R_w3 = quat_to_R(v[3], v[4], v[5], v[6])
            return R_w3 @ R_WRIST3_TO_FLANGE @ R_FLANGE_TO_TOOL0
    return None


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
    ap.add_argument("--out", default="/tmp/claude-1000/-home-sachin-ur5e-pickplace/00fa8ea1-a70a-40c8-ab8d-69cbc21316fb/scratchpad/f1_results")
    ap.add_argument(
        "--pose-csv", default=None,
        help="optional pose_recorder.py CSV for this run. Cross-checks the "
             "truth baseline against an independently recorded Gazebo pose "
             "stream. Reported only; never replaces the baseline.")
    a = ap.parse_args()
    s = json.load(open(f"{a.out}/{a.scene}_sensor.json"))
    print(f"[truth] loaded FROZEN evidence written at wall {s['frozen_at_walltime']:.3f}; "
          f"now {time.time():.3f} (+{time.time() - s['frozen_at_walltime']:.1f}s)")

    csv = s.get("csv") or {}
    print(f"[evidence] result          = {csv.get('result')}")
    print(f"[evidence] position_source = {csv.get('position_source')}")
    if csv.get("position_source") != "perceived":
        print("[NOTE] position_source is not 'perceived'; this run cannot count as F1 PASS.")

    # --- SENSOR-DERIVED quantities, already frozen ------------------------
    perceived_top = None
    if s.get("perception_used"):
        import re
        m = re.search(r"top_surface=\[([-\d.eE ]+)\]", s["perception_used"])
        if m:
            perceived_top = [float(v) for v in m.group(1).split()]
        m = re.search(r"object_centre=\[([-\d.eE ]+)\]", s["perception_used"])
        perceived_centre = [float(v) for v in m.group(1).split()] if m else None
    else:
        perceived_centre = None
    cmd = None
    if csv.get("commanded_x") is not None:
        cmd = np.array([float(csv["commanded_x"]), float(csv["commanded_y"]),
                        float(csv["commanded_z"])])
    ach = None
    if csv.get("achieved_x") is not None:
        ach = np.array([float(csv["achieved_x"]), float(csv["achieved_y"]),
                        float(csv["achieved_z"])])
    print(f"[SENSOR-DERIVED] perceived TOP world      = {perceived_top}")
    print(f"[SENSOR-DERIVED] converted object CENTRE  = {perceived_centre}")
    print(f"[SENSOR-DERIVED] commanded PRE-GRASP tcp  = {cmd}")
    print(f"[MEASURED]       achieved  PRE-GRASP tcp  = {ach}")
    if csv.get("tcp_error_m"):
        print(f"[MEASURED]       achieved-vs-commanded    = "
              f"{float(csv['tcp_error_m'])*1000:.4f} mm")

    # --- EVALUATION-ONLY TRUTH from here down -----------------------------
    rclpy.init()
    node = Node("milestone_f1_truth")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    end, tr = time.time() + 10.0, None
    while time.time() < end and tr is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            tr = buf.lookup_transform("world", "grasp_frame", rclpy.time.Time())
        except Exception:
            pass
    if tr is None:
        print("[STOP] could not look up world <- grasp_frame")
        return 2
    q = tr.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    print(f"\n[EVALUATION-ONLY TRUTH] configured grasp_frame rotation (from TF), "
          f"local Z in world = {R[:, 2]}")

    # --- BASELINE SELECTION (the 2026-08-23 timing/ordering fix) ----------
    # `manipulated` is decided from the run's OWN recorded flags, not from a
    # command-line switch, so no caller can select the wrong baseline. Any of
    # these means something physically touched or moved the object, and the
    # live post-run pose is therefore NOT its pre-manipulation pose.
    manipulated = any(
        str(csv.get(k, "0")) == "1"
        for k in ("gripper_close_attempted", "lift_attempted",
                  "transport_attempted", "place_release_attempted"))

    # The stage commanded_tcp actually holds. m3_grasp.cpp writes the PRE-GRASP
    # tcp at ~line 1055 and OVERWRITES it with the GRASP tcp at ~line 1164,
    # before the descent. So anything past pre-grasp is a grasp-stage compare.
    reached_grasp_stage = (str(csv.get("descent_attempted", "0")) == "1" or
                           str(csv.get("grasp_only", "0")) == "1")
    compared_stage = "grasp" if reached_grasp_stage else "pregrasp"

    live, raw = truth_pose()
    if live is None:
        print(f"[STOP] could not sample settled truth: {raw}")
        return 2
    live_centre = np.array(live[:3])
    print(f"[EVALUATION-ONLY TRUTH] gz LIVE post-run pose: {raw}")

    before = s.get("object_pose_before")
    if manipulated:
        if not before:
            print("[STOP] this run manipulated the object "
                  "(gripper_close/lift/transport/place attempted) but the frozen "
                  "evidence has no `object_pose_before`. The live post-run pose "
                  "is NOT a valid truth baseline for it -- it includes the "
                  "closure seating and any lift. Refusing to emit an invalid "
                  "comparison.")
            return 2
        truth_source = "frozen_pre_manipulation_object_pose_before"
        centre = np.array(before[:3])
        qx, qy, qz, qw = before[3:7]
    else:
        # pregrasp_only / no-contact runs: unchanged F1 behaviour. The object
        # was never touched, so the live settled sample IS the pre-manipulation
        # pose. Cross-checked against the frozen baseline when one exists.
        truth_source = "live_settled_post_run"
        centre = live_centre
        qx, qy, qz, qw = live[3:7]
        if before:
            drift = float(np.linalg.norm(np.array(before[:3]) - live_centre))
            print(f"[EVALUATION-ONLY TRUTH] untouched-run cross-check: live vs "
                  f"frozen object_pose_before = {drift*1000:.4f} mm")

    upright = abs(qx) < 1e-9 and abs(qy) < 1e-9 and abs(qz) < 1e-9 and abs(abs(qw) - 1) < 1e-9
    print(f"[EVALUATION-ONLY TRUTH] baseline source = {truth_source}")
    print(f"[EVALUATION-ONLY TRUTH] upright/zero-rotation: {upright}")
    top = centre + np.array([0.0, 0.0, SIZE_Z / 2.0])
    print(f"[EVALUATION-ONLY TRUTH] object centre = {centre}")
    print(f"[EVALUATION-ONLY TRUTH] top surface   = {top}")

    # Optional independent cross-check of the baseline against a separately
    # recorded Gazebo pose stream (pose_recorder.py's CSV), which carries the
    # object's world pose continuously with simulation timestamps. Uses the
    # EARLIEST recorded object sample, which precedes any manipulation.
    if a.pose_csv:
        try:
            import csv as _csv
            first = None
            with open(a.pose_csv) as fh:
                for row in _csv.DictReader(fh):
                    if row.get("entity") == OBJ_NAME:
                        first = row
                        break
            if first:
                indep = np.array([float(first["x"]), float(first["y"]),
                                  float(first["z"])])
                dv = float(np.linalg.norm(indep - centre))
                out_indep = {"pose_csv": a.pose_csv,
                             "earliest_object_centre": indep.tolist(),
                             "sim_t": first.get("sim_t"),
                             "delta_vs_baseline_m": dv}
                print(f"[EVALUATION-ONLY TRUTH] independent stream cross-check: "
                      f"earliest recorded object centre {indep} at sim_t="
                      f"{first.get('sim_t')} -> {dv*1000:.4f} mm from baseline")
            else:
                out_indep = {"pose_csv": a.pose_csv, "error": "no object rows"}
        except Exception as exc:                    # evidence tool: never fatal
            out_indep = {"pose_csv": a.pose_csv, "error": repr(exc)}
    else:
        out_indep = None

    # Truth-derived targets, SAME geometry the node used. grasp_frame's origin
    # IS the commanded grasp tcp (m3_grasp composes T_world_grasp from the
    # object centre and the configured rotation; the standoff and the
    # corrected_offset are applied after it, as pure translations).
    truth_grasp = centre.copy()
    truth_pregrasp = centre + R @ np.array([0.0, 0.0, -STANDOFF])
    print(f"[EVALUATION-ONLY TRUTH] truth-derived GRASP     tcp = {truth_grasp}")
    print(f"[EVALUATION-ONLY TRUTH] truth-derived PRE-GRASP tcp = {truth_pregrasp} "
          f"(standoff={STANDOFF})")
    print(f"[EVALUATION-ONLY TRUTH] commanded_tcp in this run holds the "
          f"{compared_stage.upper()} pose -> comparing against that one")

    seating = None
    if before:
        seating = float(np.linalg.norm(live_centre - np.array(before[:3])))
        print(f"[EVALUATION-ONLY TRUTH] object pre-manipulation -> post-run "
              f"displacement = {seating*1000:.4f} mm "
              f"(closure seating and anything after it; NOT a target error)")

    out = {"scene": a.scene, "position_source": csv.get("position_source"),
           "result": csv.get("result"),
           "manipulated": manipulated,
           "truth_source": truth_source,
           "compared_stage": compared_stage,
           "perceived_top": perceived_top, "perceived_centre": perceived_centre,
           "commanded_tcp": None if cmd is None else cmd.tolist(),
           "achieved_tcp": None if ach is None else ach.tolist(),
           # legacy key names, kept so existing readers do not break. They now
           # always describe `compared_stage`, not unconditionally pre-grasp.
           "commanded_pregrasp": None if cmd is None else cmd.tolist(),
           "achieved_pregrasp": None if ach is None else ach.tolist(),
           "truth_centre": centre.tolist(), "truth_top": top.tolist(),
           "truth_pregrasp": truth_pregrasp.tolist(),
           "truth_grasp": truth_grasp.tolist(),
           "object_centre_post_run": live_centre.tolist(),
           "object_pre_to_post_displacement_m": seating,
           "independent_pose_stream_check": out_indep,
           "object_displacement_m": s.get("object_displacement_m")}

    if cmd is not None:
        ref = truth_grasp if compared_stage == "grasp" else truth_pregrasp
        d = cmd - ref
        eu = float(np.linalg.norm(d))
        out["target_delta_m"] = d.tolist()
        out["target_euclid_m"] = eu
        print(f"\n--- PERCEPTION-DERIVED vs TRUTH-DERIVED "
              f"{compared_stage.upper()} ---")
        print(f"  dX = {d[0]*1000:+9.4f} mm")
        print(f"  dY = {d[1]*1000:+9.4f} mm")
        print(f"  dZ = {d[2]*1000:+9.4f} mm")
        print(f"  Euclidean = {eu:.9f} m = {eu*1000:.4f} mm")
    # --- orientation check (criterion 12) ---------------------------------
    # Commanded tool0 orientation IS grasp_frame's rotation: every transform
    # applied after it (-standoff, -corrected_offset) is a pure translation.
    R_ach = achieved_tool0_orientation()
    if R_ach is not None:
        R_err = R.T @ R_ach
        cos_a = max(-1.0, min(1.0, (np.trace(R_err) - 1.0) / 2.0))
        ang = math.degrees(math.acos(cos_a))
        out["orientation_error_deg"] = ang
        print(f"\n--- ORIENTATION (commanded = configured grasp_frame rotation) ---")
        print(f"  achieved tool0 local Z in world = {R_ach[:, 2]}")
        print(f"  commanded      local Z in world = {R[:, 2]}")
        print(f"  angular error = {ang:.6f} deg")
    else:
        print("\n  [warn] could not measure achieved tool0 orientation")

    if s.get("object_displacement_m") is not None:
        print(f"\n  object displacement during F1 = "
              f"{s['object_displacement_m']*1000:.4f} mm")

    json.dump(out, open(f"{a.out}/{a.scene}_truth.json", "w"), indent=2)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

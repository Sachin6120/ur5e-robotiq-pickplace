#!/usr/bin/env python3
"""stage2a_analyzer.py — Evaluates Stage-2A cycle evidence against Stage-1 criteria.

Extracts:
  - Configured vs ground-truth spawn pose & yaw
  - Perceived XYZ error
  - Deterministic pregrasp IK solution
  - Cartesian descent fraction & Stage-2 TCP error
  - Achieved grasp aperture
  - Quiescent-windowed lift slip, transport slip, and grasp-orientation evidence (via slip.py)
  - Placement position error and placement orientation error
  - Planning time
  - Authoritative PASS/FAIL evaluation
"""
import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path
import re
import sys

import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts/lib"))
import sample_pose as sp
import slip as slipmod

OBJ = "pick_target"
FLA = "wrist_3_link"

# Authoritative Stage-1 Acceptance Gates
GATES = {
    "percept_err_mm_max": 3.0,
    "cartesian_fraction_min": 0.9500,
    "stage2_tcp_err_mm_max": 2.0,
    "aperture_min_mm": 29.0,
    "aperture_max_mm": 31.0,
    "grasp_tilt_deg_max": 2.0,
    "lift_slip_mm_max": 1.0,
    "transport_slip_mm_max": 1.0,
    "placement_pos_err_mm_max": 10.0,
    "placement_orient_err_deg_max": 5.0,
    # Stage-2D evidence-integrity gates. Not physical acceptance thresholds --
    # they verify the case actually exercised planar-pose decoupling, not
    # that the manipulation was accurate.
    "translation_decoupled_mm_min": 20.0,
}

# Largest ground-truth sample gap boundary_pose() will reach across, seconds.
# The observed gz_pose_observer cadence is ~10-17 ms; a lift leg is ~1.9 s.
# 0.25 s therefore absorbs ordinary jitter and a dropped sample or two while
# still refusing to interpolate a baseline across any part of the lift itself.
# This is a stream-integrity bound, NOT a physical acceptance threshold.
BOUNDARY_MAX_GAP_S = 0.25


def quaternion_angle_error(q_actual, q_target):
    dot = abs(sum(a * b for a, b in zip(q_actual, q_target)))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


def rpy_to_quaternion(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return [qx, qy, qz, qw]


def quaternion_to_yaw(q):
    qx, qy, qz, qw = q
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def canonicalize_axial_angle(theta_rad):
    """Return the rectangle-axis representative in [-pi/2, +pi/2)."""
    wrapped = math.fmod(theta_rad, math.pi)
    if wrapped < 0.0:
        wrapped += math.pi
    if wrapped >= math.pi / 2.0:
        wrapped -= math.pi
    return wrapped


def axial_difference(a_rad, b_rad):
    """Shortest axial a-b difference; yaw and yaw+pi are equivalent."""
    return canonicalize_axial_angle(a_rad - b_rad)


def axial_error_deg(a_rad, b_rad):
    return abs(math.degrees(axial_difference(a_rad, b_rad)))


def quaternion_upright_tilt_deg(q):
    """Angle between object-local +Z and world +Z, independent of yaw."""
    qx, qy, _qz, _qw = q
    world_z_dot_local_z = 1.0 - 2.0 * (qx * qx + qy * qy)
    return math.degrees(math.acos(min(1.0, max(-1.0, world_z_dot_local_z))))


def finite_csv_float(csv_data, name):
    value = csv_data.get(name)
    if value in (None, "", "N/A"):
        return None
    try:
        parsed = float(value)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def load_pose_stream(path):
    obj, fla = [], []
    if not Path(path).is_file():
        return obj, fla
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        for r in reader:
            t = int(r["wall_ns"]) / 1e9
            if r.get(f"{OBJ}_x") and r[f"{OBJ}_x"] != "":
                obj.append(
                    (
                        t,
                        tuple(
                            float(r[f"{OBJ}_{c}"])
                            for c in ("x", "y", "z", "qx", "qy", "qz", "qw")
                        ),
                    )
                )
            if r.get(f"{FLA}_x") and r[f"{FLA}_x"] != "":
                fla.append(
                    (
                        t,
                        tuple(
                            float(r[f"{FLA}_{c}"])
                            for c in ("x", "y", "z", "qx", "qy", "qz", "qw")
                        ),
                    )
                )
    return obj, fla


def boundary_pose(series, t_boundary, max_gap_s=BOUNDARY_MAX_GAP_S):
    """Ground-truth pose AT a stage boundary. Returns (pose_or_None, method).

    WHY THIS EXISTS, AND WHY IT IS NOT A QUIESCENCE WINDOW

    The lift baseline must answer "where was the object, relative to the
    gripper, at the instant the lift started".  mean_window() answers a
    different question -- "where was the object while it was demonstrably at
    rest before the lift" -- and those coincide only when the object happens
    to be at rest.  Under the production P=200 parallel-jaw gain the gripper
    is still force-seating the object when LIFT_BEGIN fires (measured: the
    stall and LIFT_BEGIN are 0.3 ms apart in stage2d_pose/D2), so the object
    is genuinely still moving through the whole pre-lift window and the
    quiescence check correctly refuses to average it -- returning None, which
    then reads as a threshold FAIL rather than as "not measured".  That is a
    measurement artifact, not a grasp defect.

    The boundary pose has no such precondition: it is defined for any sample
    cadence that brackets the boundary, moving or not.

    DETERMINISM AND THE "HIDDEN SLIP" HAZARD

    Selection is a total function of (series, t_boundary), with no tolerance
    to tune and no choice between candidates:

      * A sample at or before the boundary is REQUIRED.  If the stream only
        starts after LIFT_BEGIN there is no valid baseline, and this returns
        None rather than reaching forward for a post-boundary sample -- such a
        sample already contains part of the lift, so using it as the reference
        would subtract real slip out of the answer and report a clean grasp.
      * When samples bracket the boundary, the pose is interpolated AT the
        boundary (position linearly, orientation by sign-aligned nlerp).  Over
        the observed ~10-17 ms cadence nlerp and slerp differ by far less than
        any threshold here, and the result is anchored by the pre-boundary
        sample, so this cannot drift into the lift.
      * max_gap_s bounds both the reach back to the previous sample and the
        bracketing span, so a ground-truth dropout fails explicitly instead of
        silently interpolating across the lift.
    """
    before = [(t, p) for t, p in series if t <= t_boundary]
    if not before:
        # Deliberately NOT falling forward to a post-boundary sample: see the
        # "hidden slip" hazard above.
        return None, "none:no_sample_at_or_before_boundary"
    t_prev, p_prev = before[-1]
    if t_boundary - t_prev > max_gap_s:
        return None, f"none:pre_boundary_gap_{t_boundary - t_prev:.4f}s"

    after = [(t, p) for t, p in series if t > t_boundary]
    if after:
        t_next, p_next = after[0]
        span = t_next - t_prev
        if span <= max_gap_s and span > 0.0:
            f = (t_boundary - t_prev) / span
            position = tuple(p_prev[i] + f * (p_next[i] - p_prev[i]) for i in range(3))
            qa = p_prev[3:7]
            qb = p_next[3:7]
            # Hemisphere-align before blending: q and -q are the same rotation,
            # and blending across the antipode would swing the long way round.
            if sum(a * b for a, b in zip(qa, qb)) < 0.0:
                qb = tuple(-b for b in qb)
            blended = tuple(qa[i] + f * (qb[i] - qa[i]) for i in range(4))
            norm = math.sqrt(sum(v * v for v in blended))
            if norm > 1e-12:
                quaternion = tuple(v / norm for v in blended)
                return position + quaternion, f"interpolated:dt={t_boundary - t_prev:.4f}s"
    return p_prev, f"nearest_before:dt={t_boundary - t_prev:.4f}s"


def upright_tilt_series(obj_stream, t0, t1):
    """Upright tilt samples over [t0, t1], one per ground-truth sample.

    Each value depends only on that sample's own quaternion (object-local +Z
    against world +Z), so this needs no pre-lift reference pose of any kind --
    which is the whole point: the historical code computed exactly this list
    but kept it behind a quiescent-pre-lift-window guard it never used, so a
    still-seating object suppressed a metric that was mathematically available.
    """
    return [
        quaternion_upright_tilt_deg(p[3:7]) for t, p in obj_stream if t0 <= t <= t1
    ]


def mean_window(series, t0, t1, tol_m=0.0005, min_samples=5):
    w = [p for t, p in series if t0 <= t <= t1]
    if len(w) < min_samples:
        return None, len(w), 0.0
    spread = max(math.dist(a[:3], b[:3]) for a in w for b in w)
    if spread > tol_m:
        return None, len(w), spread
    avg_pose = tuple(sum(p[k] for p in w) / len(w) for k in range(7))
    return avg_pose, len(w), spread


def parse_stage_timestamps(log_lines):
    ev = {}
    for l in log_lines:
        clean = re.sub(r"\x1b\[[0-9;]*m", "", l)
        m = re.search(r"\[(\d{10}\.\d+)\].*M3 STAGE \d+ ([A-Z_]+)", clean)
        if m and m.group(2) not in ev:
            ev[m.group(2)] = float(m.group(1))
    return ev


def analyze_case(
    case_dir,
    configured_yaw_deg,
    target_place_xyz,
    target_place_yaw_deg,
    require_perceived_yaw=False,
    use_axial_placement_yaw=False,
    require_translation_decoupling=False,
):
    case_path = Path(case_dir)
    log_file = case_path / "m3_grasp.log"
    csv_file = case_path / "m3_grasp.csv"
    stream_file = case_path / "gz_pose_stream.csv"
    init_pose_file = case_path / "init_settled_pose.json"
    final_pose_file = case_path / "final_settled_pose.json"
    case_scene_file = case_path / "scene_case.yaml"
    base_scene_file = REPO / "config/scene.yaml"

    log_lines = []
    if log_file.is_file():
        with open(log_file, "r", errors="ignore") as lf:
            log_lines = lf.readlines()

    csv_data = {}
    if csv_file.is_file():
        with open(csv_file, "r") as cf:
            rows = list(csv.DictReader(cf))
            if rows:
                csv_data = rows[0]

    # 1. Parse log fields
    sel_q = "N/A"
    d_descent = "N/A"
    d_transit = "N/A"
    plan_time_s = None
    perceived_top_world = None
    for line in log_lines:
        clean = re.sub(r"\x1b\[[0-9;]*m", "", line)
        if "DETERMINISTIC_PREGRASP_SELECTED" in clean:
            if "q=" in clean:
                sel_q = clean.split("q=")[1].split("]")[0] + "]"
            if "D_descent=" in clean:
                d_descent = clean.split("D_descent=")[1].split(" ")[0]
            if "D_transit=" in clean:
                d_transit = clean.split("D_transit=")[1].split(" ")[0]
        if "time taken to generate plan:" in clean:
            try:
                plan_time_s = float(clean.split("time taken to generate plan:")[1].split()[0])
            except (IndexError, ValueError):
                pass
        if "PERCEIVED_TOP_WORLD=[" in clean:
            try:
                coords = clean.split("PERCEIVED_TOP_WORLD=[")[1].split("]")[0].split()
                perceived_top_world = [float(c) for c in coords]
            except (IndexError, ValueError):
                pass

    # 2. Spawn and Initial Pose
    init_pose = None
    if init_pose_file.is_file():
        with open(init_pose_file, "r") as f:
            init_pose = json.load(f)

    spawned_yaw_deg = None
    percept_err_mm = None
    if init_pose and len(init_pose) >= 7:
        init_q = init_pose[3:7]
        spawned_yaw_deg = math.degrees(quaternion_to_yaw(init_q))
        if perceived_top_world:
            # Distance in XYZ between perceived top and actual ground-truth top surface
            # Ground truth top surface Z = center Z + height / 2 (height = 0.045)
            gt_top = [init_pose[0], init_pose[1], init_pose[2] + 0.045 / 2.0]
            percept_err_mm = math.dist(perceived_top_world, gt_top) * 1000.0

    # 2b. Stage-2D evidence integrity: planar translation decoupling.
    # measured_spawn_offset_mm compares the ground-truth settled spawn XY
    # against THIS CASE's configured pick XY (scene_case.yaml, i.e. what
    # static_scene_tf actually published as object_frame) -- not against
    # config/scene.yaml directly, so it is correct even if a future case
    # generator legitimately varies the configured pose. configured_pose_
    # unchanged separately confirms the case's configured pick XY never
    # drifted from config/scene.yaml's frozen value.
    measured_spawn_offset_mm = None
    translation_decoupled = None
    configured_pose_unchanged = None
    case_pick_xy_m = None
    base_pick_xy_m = None
    if case_scene_file.is_file():
        with open(case_scene_file, "r") as f:
            case_scene = yaml.safe_load(f)
        case_pick_xy_m = [
            float(case_scene["object"]["pick_pose"]["x"]),
            float(case_scene["object"]["pick_pose"]["y"]),
        ]
        if base_scene_file.is_file():
            with open(base_scene_file, "r") as f:
                base_scene = yaml.safe_load(f)
            base_pick_xy_m = [
                float(base_scene["object"]["pick_pose"]["x"]),
                float(base_scene["object"]["pick_pose"]["y"]),
            ]
            configured_pose_unchanged = math.isclose(
                case_pick_xy_m[0], base_pick_xy_m[0], rel_tol=0.0, abs_tol=1e-9
            ) and math.isclose(
                case_pick_xy_m[1], base_pick_xy_m[1], rel_tol=0.0, abs_tol=1e-9
            )
        if init_pose and len(init_pose) >= 2:
            measured_spawn_offset_mm = [
                (init_pose[0] - case_pick_xy_m[0]) * 1000.0,
                (init_pose[1] - case_pick_xy_m[1]) * 1000.0,
            ]
            translation_decoupled = (
                math.hypot(*measured_spawn_offset_mm)
                >= GATES["translation_decoupled_mm_min"]
            )

    # 3. CSV metrics
    m3_res = csv_data.get("result", "UNKNOWN")
    yaw_source = csv_data.get("yaw_source") or None
    configured_object_yaw_deg = finite_csv_float(csv_data, "configured_object_yaw_deg")
    perceived_object_yaw_deg = finite_csv_float(csv_data, "perceived_object_yaw_deg")
    yaw_delta_deg = finite_csv_float(csv_data, "yaw_delta_deg")
    commanded_grasp_yaw_deg = finite_csv_float(csv_data, "commanded_grasp_yaw_deg")

    perceived_yaw_err_deg = None
    if perceived_object_yaw_deg is not None and spawned_yaw_deg is not None:
        # Ground-truth spawned orientation is the evaluation reference. The
        # configured frame is intentionally absent from this calculation.
        perceived_yaw_err_deg = axial_error_deg(
            math.radians(perceived_object_yaw_deg), math.radians(spawned_yaw_deg)
        )

    cartesian_fraction = finite_csv_float(csv_data, "cartesian_fraction")
    tcp_error_m = finite_csv_float(csv_data, "tcp_error_m")
    stage2_tcp_err_mm = tcp_error_m * 1000.0 if tcp_error_m is not None else None

    achieved_q = finite_csv_float(csv_data, "achieved_q")
    achieved_aperture_mm = (0.085 - achieved_q) * 1000.0 if achieved_q is not None else None

    # 4. Stream analysis: Slip and Tilt
    obj_stream, fla_stream = load_pose_stream(stream_file)
    stage_stamps = parse_stage_timestamps(log_lines)

    lift_slip_mm = None
    transport_slip_mm = None
    max_grasp_tilt_deg = None
    max_grasp_orientation_change_deg = None
    # Boundary-referenced measurements (force-seating robust) and the
    # historical quiescent-window ones, both always recorded. Which pair is
    # authoritative is decided per stage at the bottom of this block.
    lift_slip_boundary_mm = None
    lift_slip_quiescent_window_mm = None
    max_upright_tilt_deg = None
    max_grasp_orientation_change_boundary_deg = None
    lift_baseline_method = None
    retained_interval = None

    if (
        obj_stream
        and fla_stream
        and "LIFT_BEGIN" in stage_stamps
        and "LIFT_DONE" in stage_stamps
    ):
        lb = stage_stamps["LIFT_BEGIN"]
        ld = stage_stamps["LIFT_DONE"]
        win_s = 0.8
        dwell_offset_s = 0.6

        base_o, _, _ = mean_window(obj_stream, lb - win_s, lb)
        base_f, _, _ = mean_window(fla_stream, lb - win_s, lb)
        post_o, _, _ = mean_window(obj_stream, ld + dwell_offset_s, ld + dwell_offset_s + win_s)
        post_f, _, _ = mean_window(fla_stream, ld + dwell_offset_s, ld + dwell_offset_s + win_s)

        # Historical baseline: the mean of a provably-quiescent pre-lift
        # window. Kept verbatim so its value stays on the record even when the
        # boundary method supersedes it.
        if base_o and base_f and post_o and post_f:
            lift_slip_quiescent_window_mm = (
                slipmod.slip_m(base_f, base_o, post_f, post_o) * 1000.0
            )

        # Boundary baseline: the pose AT LIFT_BEGIN, which does not require
        # the object to have finished seating.
        bnd_o, method_o = boundary_pose(obj_stream, lb)
        bnd_f, method_f = boundary_pose(fla_stream, lb)
        lift_baseline_method = {"object": method_o, "flange": method_f}
        if bnd_o and bnd_f and post_o and post_f:
            lift_slip_boundary_mm = (
                slipmod.slip_m(bnd_f, bnd_o, post_f, post_o) * 1000.0
            )

        if "TRANSPORT_DONE" in stage_stamps:
            td = stage_stamps["TRANSPORT_DONE"]
            tran_o, _, _ = mean_window(
                obj_stream, td + dwell_offset_s, td + dwell_offset_s + win_s
            )
            tran_f, _, _ = mean_window(
                fla_stream, td + dwell_offset_s, td + dwell_offset_s + win_s
            )
            if post_o and post_f and tran_o and tran_f:
                trans_slip = slipmod.slip_m(post_f, post_o, tran_f, tran_o)
                transport_slip_mm = trans_slip * 1000.0

        # Retained-object interval: LIFT_BEGIN until the end of the last dwell
        # the object is still held through. TRANSPORT_DONE when there was a
        # transport leg, LIFT_DONE otherwise -- the historical code derived
        # this bound from TRANSPORT_DONE only, so lift_only runs silently got
        # no tilt at all. Capped at RELEASE_BEGIN because "grasp tilt" is
        # meaningless once the object is no longer in the jaws (verified a
        # no-op across all existing evidence: no recorded dwell end ever falls
        # after its own RELEASE_BEGIN).
        interval_end = (
            stage_stamps["TRANSPORT_DONE"] if "TRANSPORT_DONE" in stage_stamps else ld
        ) + dwell_offset_s + win_s
        if "RELEASE_BEGIN" in stage_stamps:
            interval_end = min(interval_end, stage_stamps["RELEASE_BEGIN"])
        retained_interval = [lb, interval_end]

        # Upright tilt needs no reference pose, so it is computed
        # unconditionally over the retained interval.
        upright_tilt_samples = upright_tilt_series(obj_stream, lb, interval_end)
        if upright_tilt_samples:
            max_upright_tilt_deg = max(upright_tilt_samples)

        # Full SO(3) displacement from a pre-lift reference. This one genuinely
        # needs a reference orientation, so it is reported against both the
        # historical window mean and the boundary pose.
        def _max_orientation_change(reference_pose):
            if not reference_pose:
                return None
            reference_q = reference_pose[3:7]
            samples = [
                math.degrees(quaternion_angle_error(p[3:7], reference_q))
                for t, p in obj_stream
                if lb <= t <= interval_end
            ]
            return max(samples) if samples else None

        max_grasp_orientation_change_deg = _max_orientation_change(base_o)
        max_grasp_orientation_change_boundary_deg = _max_orientation_change(bnd_o)

        if use_axial_placement_yaw:
            # Stage-2C/2D: a rectangle may yaw about world +Z while remaining
            # upright. That yaw is not a physical grasp tilt, and the upright
            # measure is force-seating robust.
            max_grasp_tilt_deg = max_upright_tilt_deg
            lift_slip_mm = lift_slip_boundary_mm
        else:
            # Stage-2A keeps its historical quiescent-window semantics
            # unchanged; the boundary values above are recorded alongside as
            # diagnostics rather than silently redefining a closed campaign.
            max_grasp_tilt_deg = max_grasp_orientation_change_deg
            lift_slip_mm = lift_slip_quiescent_window_mm

    # 5. Final Placement
    final_pose = None
    if final_pose_file.is_file():
        with open(final_pose_file, "r") as f:
            final_pose = json.load(f)

    placement_pos_err_mm = None
    placement_orient_err_deg = None
    placement_yaw_err_deg = None
    final_upright_tilt_deg = None
    if final_pose and len(final_pose) >= 7:
        final_xyz = final_pose[:3]
        final_q = final_pose[3:7]
        target_q = rpy_to_quaternion(0.0, 0.0, math.radians(target_place_yaw_deg))
        placement_pos_err_mm = math.dist(final_xyz, target_place_xyz) * 1000.0
        # Diagnostic only in Stage-2C: this remains the historical full SO(3)
        # quaternion error, so a 180-degree axial equivalent reads as 180.
        placement_orient_err_deg = math.degrees(quaternion_angle_error(final_q, target_q))
        placement_yaw_err_deg = axial_error_deg(
            quaternion_to_yaw(final_q), math.radians(target_place_yaw_deg)
        )
        final_upright_tilt_deg = quaternion_upright_tilt_deg(final_q)

    # 6. Evaluation against Authoritative Gates
    gate_checks = {
        "result_success": (m3_res == "SUCCESS"),
        "percept_err": (
            percept_err_mm is not None and percept_err_mm < GATES["percept_err_mm_max"]
        ),
        "pregrasp_selected": (sel_q != "N/A"),
        "cartesian_fraction": (
            cartesian_fraction is not None
            and cartesian_fraction >= GATES["cartesian_fraction_min"]
        ),
        "stage2_tcp_err": (
            stage2_tcp_err_mm is not None
            and stage2_tcp_err_mm < GATES["stage2_tcp_err_mm_max"]
        ),
        "aperture": (
            achieved_aperture_mm is not None
            and GATES["aperture_min_mm"] <= achieved_aperture_mm <= GATES["aperture_max_mm"]
        ),
        "grasp_tilt": (
            max_grasp_tilt_deg is not None
            and max_grasp_tilt_deg < GATES["grasp_tilt_deg_max"]
        ),
        "lift_slip": (
            lift_slip_mm is not None and lift_slip_mm < GATES["lift_slip_mm_max"]
        ),
        "transport_slip": (
            transport_slip_mm is not None and transport_slip_mm < GATES["transport_slip_mm_max"]
        ),
        "placement_pos_err": (
            placement_pos_err_mm is not None
            and placement_pos_err_mm < GATES["placement_pos_err_mm_max"]
        ),
    }
    if require_perceived_yaw:
        gate_checks["perceived_yaw"] = (
            yaw_source == "perceived"
            and perceived_yaw_err_deg is not None
        )
    if use_axial_placement_yaw:
        # Keep the existing 5-degree placement-orientation threshold.  In
        # Stage-2C it applies independently to axial yaw and upright tilt.
        gate_checks["placement_yaw_err"] = (
            placement_yaw_err_deg is not None
            and placement_yaw_err_deg < GATES["placement_orient_err_deg_max"]
        )
        gate_checks["final_upright_tilt"] = (
            final_upright_tilt_deg is not None
            and final_upright_tilt_deg < GATES["placement_orient_err_deg_max"]
        )
    else:
        gate_checks["placement_orient_err"] = (
            placement_orient_err_deg is not None
            and placement_orient_err_deg < GATES["placement_orient_err_deg_max"]
        )
    if require_translation_decoupling:
        # Evidence-integrity gates only -- they confirm the case actually
        # exercised a decoupled planar spawn, not that manipulation was
        # accurate. Every other gate above is unchanged by this flag.
        gate_checks["translation_decoupled"] = bool(translation_decoupled)
        gate_checks["configured_pose_unchanged"] = bool(configured_pose_unchanged)

    all_passed = all(gate_checks.values())
    verdict = "PASS" if all_passed else "FAIL"

    metrics = {
        "configured_yaw_deg": configured_yaw_deg,
        "configured_yaw_rad": math.radians(configured_yaw_deg),
        "spawned_yaw_deg": spawned_yaw_deg,
        "yaw_source": yaw_source,
        "configured_object_yaw_deg": configured_object_yaw_deg,
        "perceived_object_yaw_deg": perceived_object_yaw_deg,
        "yaw_delta_deg": yaw_delta_deg,
        "commanded_grasp_yaw_deg": commanded_grasp_yaw_deg,
        "perceived_yaw_err_deg": perceived_yaw_err_deg,
        "result": m3_res,
        "percept_err_mm": percept_err_mm,
        "selected_pregrasp_q": sel_q,
        "d_descent": d_descent,
        "d_transit": d_transit,
        "cartesian_fraction": cartesian_fraction,
        "stage2_tcp_err_mm": stage2_tcp_err_mm,
        "achieved_q": achieved_q,
        "achieved_aperture_mm": achieved_aperture_mm,
        "max_grasp_tilt_deg": max_grasp_tilt_deg,
        "max_grasp_orientation_change_deg": max_grasp_orientation_change_deg,
        "lift_slip_mm": lift_slip_mm,
        # Both measurement methods, always recorded. lift_slip_mm and
        # max_grasp_tilt_deg above select between them per stage; these keep
        # the superseded quiescent-window values on the record rather than
        # deleting the evidence that the old method produced no number.
        "lift_slip_boundary_mm": lift_slip_boundary_mm,
        "lift_slip_quiescent_window_mm": lift_slip_quiescent_window_mm,
        "max_upright_tilt_deg": max_upright_tilt_deg,
        "max_grasp_orientation_change_boundary_deg": (
            max_grasp_orientation_change_boundary_deg
        ),
        "lift_baseline_method": lift_baseline_method,
        "retained_interval": retained_interval,
        "lift_metric_source": (
            "boundary" if use_axial_placement_yaw else "quiescent_window"
        ),
        "transport_slip_mm": transport_slip_mm,
        "placement_pos_err_mm": placement_pos_err_mm,
        "placement_orient_err_deg": placement_orient_err_deg,
        "placement_yaw_err_deg": placement_yaw_err_deg,
        "final_upright_tilt_deg": final_upright_tilt_deg,
        "placement_yaw_scoring": "axial" if use_axial_placement_yaw else "full_quaternion",
        "planning_time_s": plan_time_s,
        "configured_pick_xy_m": case_pick_xy_m,
        "base_pick_xy_m": base_pick_xy_m,
        "measured_spawn_offset_mm": measured_spawn_offset_mm,
        "translation_decoupled": translation_decoupled,
        "configured_pose_unchanged": configured_pose_unchanged,
        "gates": gate_checks,
        "verdict": verdict,
    }

    out_metrics_file = case_path / "cycle_metrics.json"
    with open(out_metrics_file, "w") as jf:
        json.dump(metrics, jf, indent=2)

    return metrics


def main():
    parser = argparse.ArgumentParser(description="Analyze Stage-2A cycle metrics.")
    parser.add_argument("--case-dir", required=True, help="Path to evidence/stage2a_orientation/<case>")
    parser.add_argument("--yaw-deg", type=float, required=True, help="Configured yaw in degrees")
    parser.add_argument(
        "--target-place-xyz",
        nargs=3,
        type=float,
        default=[0.450, 0.200, 0.7725],
        help="Target placement XYZ",
    )
    parser.add_argument(
        "--target-place-yaw-deg",
        type=float,
        default=None,
        help="Target placement yaw in deg (defaults to --yaw-deg)",
    )
    args = parser.parse_args()

    target_place_yaw = args.target_place_yaw_deg if args.target_place_yaw_deg is not None else args.yaw_deg
    metrics = analyze_case(args.case_dir, args.yaw_deg, args.target_place_xyz, target_place_yaw)
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()

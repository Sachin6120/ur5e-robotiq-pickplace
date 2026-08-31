#!/usr/bin/env python3
"""Focused, ROS-free tests for force-seating-robust lift slip and grasp tilt.

Covers the boundary-referenced lift baseline and the reference-free upright
tilt that replaced the quiescent-pre-lift-window versions for Stage-2C/2D.
Synthetic streams only -- no Gazebo, no recorded evidence is read or written.
"""

import csv
import json
import math
from pathlib import Path
import sys
import tempfile

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts/perception"))
sys.path.insert(0, str(REPO / "scripts/lib"))
import stage2a_analyzer as analyzer
import slip as slipmod

IDENT = (0.0, 0.0, 0.0, 1.0)
# 10 integer digits: parse_stage_timestamps() matches \d{10}\.\d+, i.e. real
# gz_pose_observer wall-clock stamps. Shorter synthetic values parse as no
# stage markers at all and every stream metric silently comes back None.
LB = 1788136469.5193
LD = 1788136471.4275
TD = 1788136477.5457
DT = 0.0125  # 80 Hz, inside the observed 10-17 ms cadence


def q_yaw(deg):
    return tuple(analyzer.rpy_to_quaternion(0.0, 0.0, math.radians(deg)))


def q_roll(deg):
    return tuple(analyzer.rpy_to_quaternion(math.radians(deg), 0.0, 0.0))


def build_stream(samples):
    """samples: list of (t, obj_pose, flange_pose) -> analyzer-shaped streams."""
    return ([(t, o) for t, o, _ in samples], [(t, f) for _, _, f in [(s[0], s[1], s[2]) for s in samples]])


def synth(
    *,
    seating_until=None,
    seating_amp_m=0.0,
    lift_slip_m=0.0,
    tilt_deg=0.0,
    yaw_deg=0.0,
    t_start=LB - 1.6,
    t_end=TD + 1.6,
    drop_before_lb=False,
):
    """Build an (obj_stream, fla_stream) pair with known ground truth.

    The flange rises 0.12 m over the lift; the object rides with it, plus any
    requested in-gripper slip applied along world +X after the lift.
    """
    obj, fla = [], []
    n = int(round((t_end - t_start) / DT))
    for i in range(n + 1):
        t = t_start + i * DT
        if drop_before_lb and t <= LB:
            continue
        # Flange: still, then lifts linearly across [LB, LD].
        if t <= LB:
            fz = 0.90
        elif t >= LD:
            fz = 1.02
        else:
            fz = 0.90 + 0.12 * (t - LB) / (LD - LB)
        fla.append((t, (0.40, 0.0, fz) + IDENT))

        ox = 0.40
        # Force seating: object still creeping toward its seated pose right up
        # to LIFT_BEGIN, exactly the P=200 behaviour that broke the old window.
        if seating_until is not None and t < seating_until:
            remaining = (seating_until - t) / (seating_until - t_start)
            ox += seating_amp_m * remaining
        # Real in-gripper slip, applied once the lift is under way.
        if t > LB:
            ox += lift_slip_m
        oz = fz - 0.05
        # Strictly AFTER the boundary, so the pre-lift reference orientation
        # is the upright identity and any rotation counts as a change from it.
        q = IDENT
        if tilt_deg and t > LB:
            q = q_roll(tilt_deg)
        elif yaw_deg and t > LB:
            q = q_yaw(yaw_deg)
        obj.append((t, (ox, 0.0, oz) + q))
    return obj, fla


def write_case(case_dir, obj, fla, *, stamps=("LIFT_BEGIN", "LIFT_DONE", "TRANSPORT_DONE")):
    cols = []
    for e in (analyzer.OBJ, analyzer.FLA):
        cols += [f"{e}_{c}" for c in ("x", "y", "z", "qx", "qy", "qz", "qw")]
    fla_by_t = dict(fla)
    with open(case_dir / "gz_pose_stream.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["wall_ns"] + cols)
        for t, o in obj:
            fpose = fla_by_t.get(t)
            row = [int(t * 1e9)] + [f"{v:.9f}" for v in o]
            row += [f"{v:.9f}" for v in fpose] if fpose else [""] * 7
            w.writerow(row)
    lines = []
    marker_t = {"LIFT_BEGIN": LB, "LIFT_DONE": LD, "TRANSPORT_DONE": TD}
    stage_no = {"LIFT_BEGIN": 3, "LIFT_DONE": 3, "TRANSPORT_DONE": 4}
    for name in stamps:
        lines.append(f"[{marker_t[name]:.6f}] M3 STAGE {stage_no[name]} {name} cycle=0 sim=0\n")
    (case_dir / "m3_grasp.log").write_text("".join(lines))
    with open(case_dir / "m3_grasp.csv", "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=["result", "yaw_source"])
        wr.writeheader()
        wr.writerow({"result": "SUCCESS", "yaw_source": "perceived"})


def analyze(obj, fla, *, stamps=("LIFT_BEGIN", "LIFT_DONE", "TRANSPORT_DONE"), axial=True):
    with tempfile.TemporaryDirectory() as td:
        case_dir = Path(td)
        write_case(case_dir, obj, fla, stamps=stamps)
        return analyzer.analyze_case(
            case_dir,
            configured_yaw_deg=0.0,
            target_place_xyz=[0.45, 0.2, 0.7725],
            target_place_yaw_deg=0.0,
            use_axial_placement_yaw=axial,
        )


# --------------------------------------------------------------------------


def test_force_seating_until_lift_begin_yields_finite_slip_and_tilt():
    """The D2 shape: object still seating at LIFT_BEGIN. Old method -> None."""
    obj, fla = synth(seating_until=LB, seating_amp_m=0.0012, tilt_deg=0.05)
    m = analyze(obj, fla)
    # The historical quiescent window must still refuse this stream -- that is
    # the artifact being worked around, and its absence stays on the record.
    assert m["lift_slip_quiescent_window_mm"] is None
    # The boundary method must produce a finite number instead.
    assert m["lift_slip_boundary_mm"] is not None
    assert math.isfinite(m["lift_slip_boundary_mm"])
    assert m["lift_slip_mm"] == m["lift_slip_boundary_mm"]
    assert m["lift_metric_source"] == "boundary"
    # Tilt must not be suppressed by the unusable position window.
    assert m["max_upright_tilt_deg"] is not None
    assert abs(m["max_upright_tilt_deg"] - 0.05) < 1e-6
    assert m["max_grasp_tilt_deg"] == m["max_upright_tilt_deg"]
    assert m["gates"]["lift_slip"] is True
    assert m["gates"]["grasp_tilt"] is True


def test_yaw_only_rotation_is_not_counted_as_tilt():
    obj, fla = synth(yaw_deg=45.0)
    m = analyze(obj, fla)
    assert m["max_upright_tilt_deg"] < 1e-9
    assert m["max_grasp_tilt_deg"] < 1e-9
    # The full-SO(3) diagnostic still sees the yaw, so the two are not the same
    # number by accident.
    assert m["max_grasp_orientation_change_boundary_deg"] > 44.0


def test_boundary_sample_selection_is_deterministic():
    obj, fla = synth(seating_until=LB, seating_amp_m=0.0012)
    # Bracketing samples -> interpolated exactly at the boundary, repeatably.
    pose_a, method_a = analyzer.boundary_pose(obj, LB)
    pose_b, method_b = analyzer.boundary_pose(obj, LB)
    assert pose_a == pose_b and method_a == method_b
    assert method_a.startswith("interpolated:")

    prev = [(t, p) for t, p in obj if t <= LB][-1]
    nxt = [(t, p) for t, p in obj if t > LB][0]
    frac = (LB - prev[0]) / (nxt[0] - prev[0])
    expected_x = prev[1][0] + frac * (nxt[1][0] - prev[1][0])
    assert abs(pose_a[0] - expected_x) < 1e-12
    # Interpolation is bounded by its two anchors -- it cannot run into the lift.
    assert min(prev[1][0], nxt[1][0]) <= pose_a[0] <= max(prev[1][0], nxt[1][0])

    # No sample after the boundary -> fall back to the last one before it.
    truncated = [(t, p) for t, p in obj if t <= LB]
    pose_c, method_c = analyzer.boundary_pose(truncated, LB)
    assert method_c.startswith("nearest_before:")
    assert pose_c == truncated[-1][1]


def test_quiescent_case_stays_numerically_consistent():
    """Where the old method worked, the new one must agree with it."""
    obj, fla = synth(lift_slip_m=0.0004)
    m = analyze(obj, fla)
    assert m["lift_slip_quiescent_window_mm"] is not None
    assert m["lift_slip_boundary_mm"] is not None
    assert abs(m["lift_slip_boundary_mm"] - m["lift_slip_quiescent_window_mm"]) < 0.05
    # And both must recover the injected 0.4 mm slip.
    assert abs(m["lift_slip_boundary_mm"] - 0.4) < 0.05


def test_missing_ground_truth_around_lift_begin_fails_explicitly():
    obj, fla = synth(drop_before_lb=True)
    pose, method = analyzer.boundary_pose(obj, LB)
    assert pose is None
    assert method == "none:no_sample_at_or_before_boundary"

    m = analyze(obj, fla)
    assert m["lift_slip_boundary_mm"] is None
    assert m["lift_slip_mm"] is None
    assert m["gates"]["lift_slip"] is False
    assert m["verdict"] == "FAIL"
    assert m["lift_baseline_method"]["object"].startswith("none:")

    # A gap wider than BOUNDARY_MAX_GAP_S must also refuse rather than
    # interpolate a baseline across part of the lift.
    sparse = [(LB - 5.0, (0.40, 0.0, 0.85) + IDENT), (LB + 5.0, (0.90, 0.0, 0.85) + IDENT)]
    pose_g, method_g = analyzer.boundary_pose(sparse, LB)
    assert pose_g is None
    assert method_g.startswith("none:pre_boundary_gap_")


def test_post_lift_sample_is_never_used_as_baseline_to_hide_slip():
    """A large real slip during the lift must still be reported in full."""
    obj, fla = synth(seating_until=LB, seating_amp_m=0.0012, lift_slip_m=0.0030)
    m = analyze(obj, fla)
    # Baseline is anchored at/before LIFT_BEGIN, so the 3 mm shows up.
    assert m["lift_slip_boundary_mm"] > 2.5
    assert m["gates"]["lift_slip"] is False

    # Had the post-lift window been used as its own baseline, slip would
    # collapse to ~0 and the failure would vanish. Assert that explicitly.
    post_o, _, _ = analyzer.mean_window(obj, LD + 0.6, LD + 0.6 + 0.8)
    post_f, _, _ = analyzer.mean_window(fla, LD + 0.6, LD + 0.6 + 0.8)
    self_referenced_mm = slipmod.slip_m(post_f, post_o, post_f, post_o) * 1000.0
    assert self_referenced_mm < 1e-9
    assert m["lift_slip_boundary_mm"] - self_referenced_mm > 2.5

    # The baseline timestamp itself must not be after LIFT_BEGIN.
    before = [t for t, _ in obj if t <= LB]
    assert before, "fixture must provide a pre-boundary sample"
    assert m["lift_baseline_method"]["object"].startswith(("interpolated:", "nearest_before:"))


def test_lift_only_run_without_transport_still_reports_tilt():
    """Historically td was undefined here, so tilt silently came back None."""
    obj, fla = synth(tilt_deg=0.4, t_end=LD + 1.6)
    m = analyze(obj, fla, stamps=("LIFT_BEGIN", "LIFT_DONE"))
    assert m["transport_slip_mm"] is None
    assert m["max_upright_tilt_deg"] is not None
    assert abs(m["max_upright_tilt_deg"] - 0.4) < 1e-6
    assert m["retained_interval"][1] <= LD + 0.6 + 0.8 + 1e-9


def test_stage2a_path_keeps_quiescent_window_semantics():
    """use_axial_placement_yaw=False must not adopt the new authoritative values."""
    obj, fla = synth(lift_slip_m=0.0004, tilt_deg=0.3)
    m = analyze(obj, fla, axial=False)
    assert m["lift_metric_source"] == "quiescent_window"
    assert m["lift_slip_mm"] == m["lift_slip_quiescent_window_mm"]
    assert m["max_grasp_tilt_deg"] == m["max_grasp_orientation_change_deg"]
    # Boundary values are still recorded alongside, as diagnostics.
    assert m["lift_slip_boundary_mm"] is not None
    assert m["max_upright_tilt_deg"] is not None


def main():
    test_force_seating_until_lift_begin_yields_finite_slip_and_tilt()
    test_yaw_only_rotation_is_not_counted_as_tilt()
    test_boundary_sample_selection_is_deterministic()
    test_quiescent_case_stays_numerically_consistent()
    test_missing_ground_truth_around_lift_begin_fails_explicitly()
    test_post_lift_sample_is_never_used_as_baseline_to_hide_slip()
    test_lift_only_run_without_transport_still_reports_tilt()
    test_stage2a_path_keeps_quiescent_window_semantics()
    print("Lift metric boundary tests passed")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""test_gz_contact_observer.py — focused static checks for rows_for().

Not a colcon/pytest target -- this project's evidence/ tools use the same
plain-assert, run-it-directly style (see e.g.
evidence/f3_h25_hybrid_hold_infrastructure/tools/test_protocol_helpers.py).

Run: python3 scripts/perception/test_gz_contact_observer.py

No ROS, no Gazebo: exercises rows_for() against hand-built dicts shaped like
`gz topic --json-output` messages, per gz/msgs/contact.proto and
gz/msgs/joint_wrench.proto (both installed under /opt/ros/jazzy).
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gz_contact_observer import COLS, rows_for  # noqa: E402

IDX = {name: i for i, name in enumerate(COLS)}


def row_dict(row):
    return {name: row[IDX[name]] for name in COLS}


def test_empty_message_emits_one_empty_row():
    msg = {"header": {"stamp": {"sec": 1, "nsec": 0}}, "contact": []}
    rows = rows_for(msg, msg_index=1, wall_ns=1000)
    assert len(rows) == 1
    r = row_dict(rows[0])
    assert r["row_kind"] == "EMPTY"
    assert r["n_pairs"] == 0
    print("test_empty_message_emits_one_empty_row: PASS")


def _two_point_contact(body1_key="body1Wrench", body2_key="body2Wrench"):
    return {
        "collision1": {"name": "pad_fixed_link_collision"},
        "collision2": {"name": "pick_target::link::c"},
        "position": [{"x": 0.1, "y": 0.0, "z": 0.7}, {"x": 0.1, "y": 0.01, "z": 0.7}],
        "normal": [{"x": 1.0, "y": 0.0, "z": 0.0}, {"x": 1.0, "y": 0.0, "z": 0.0}],
        "depth": [0.001, 0.0005],
        "wrench": [
            {
                body1_key: {"force": {"x": 1.0, "y": 0.0, "z": 0.0},
                            "torque": {"x": 0.0, "y": 0.0, "z": 0.0}},
                body2_key: {"force": {"x": -1.0, "y": 0.0, "z": 0.0},
                            "torque": {"x": 0.0, "y": 0.0, "z": 0.0}},
            },
            {
                body1_key: {"force": {"x": 0.5, "y": 0.2, "z": 0.0},
                            "torque": {"x": 0.0, "y": 0.0, "z": 0.0}},
                body2_key: {"force": {"x": -0.5, "y": -0.2, "z": 0.0},
                            "torque": {"x": 0.0, "y": 0.0, "z": 0.0}},
            },
        ],
    }


def test_two_point_manifold_produces_two_point_rows_and_one_correct_sum():
    msg = {"header": {"stamp": {"sec": 2, "nsec": 0}}, "contact": [_two_point_contact()]}
    rows = [row_dict(r) for r in rows_for(msg, msg_index=2, wall_ns=2000)]

    points = [r for r in rows if r["row_kind"] == "POINT"]
    sums = [r for r in rows if r["row_kind"] == "PAIR_SUM"]
    assert len(points) == 2, points
    assert len(sums) == 1, sums

    assert points[0]["point_index"] == 0
    assert points[1]["point_index"] == 1
    assert points[0]["n_points"] == 2
    assert points[0]["collision1"] == "pad_fixed_link_collision"
    assert points[0]["collision2"] == "pick_target::link::c"

    s = sums[0]
    assert s["n_pairs"] == 1
    assert s["n_points"] == 2
    assert abs(s["body1_force_x"] - 1.5) < 1e-12
    assert abs(s["body1_force_y"] - 0.2) < 1e-12
    assert abs(s["body1_force_z"] - 0.0) < 1e-12
    # DART: force on body2 is exactly -force on body1 (installed
    # dart/collision/Contact.hpp). This fixture encodes that; the SUM must
    # preserve it.
    assert abs(s["body2_force_x"] + 1.5) < 1e-12
    assert abs(s["body2_force_y"] + 0.2) < 1e-12
    assert s["body2_wrench_populated"] is True
    print("test_two_point_manifold_produces_two_point_rows_and_one_correct_sum: PASS")


def test_closing_axis_projection_matches_known_geometry():
    # This project's own recorded normals confirm the world-frame closing
    # axis at yaw=+15deg is (cos15, sin15, 0) -- see the 2026-08-29
    # contact-wrench audit. A force purely along +X should project to
    # cos(15deg) of its magnitude, not its full magnitude.
    msg = {"header": {"stamp": {"sec": 3, "nsec": 0}}, "contact": [_two_point_contact()]}
    yaw = math.radians(15.0)
    axis = (math.cos(yaw), math.sin(yaw), 0.0)
    rows = [row_dict(r) for r in rows_for(msg, msg_index=3, wall_ns=3000, closing_axis=axis)]
    s = next(r for r in rows if r["row_kind"] == "PAIR_SUM")
    expected = 1.5 * math.cos(yaw) + 0.2 * math.sin(yaw)
    assert abs(s["body1_force_closing_axis"] - expected) < 1e-9, (
        s["body1_force_closing_axis"], expected
    )
    assert abs(s["closing_axis_x"] - axis[0]) < 1e-12
    # No axis given -> projection columns stay blank, not silently zero.
    rows_no_axis = [row_dict(r) for r in rows_for(msg, msg_index=4, wall_ns=4000)]
    s2 = next(r for r in rows_no_axis if r["row_kind"] == "PAIR_SUM")
    assert s2["body1_force_closing_axis"] == ""
    assert s2["closing_axis_x"] == ""
    print("test_closing_axis_projection_matches_known_geometry: PASS")


def test_body2_wrench_missing_is_recorded_as_unpopulated_not_assumed_zero():
    c = _two_point_contact()
    for w in c["wrench"]:
        del w["body2Wrench"]
    msg = {"header": {"stamp": {"sec": 5, "nsec": 0}}, "contact": [c]}
    rows = [row_dict(r) for r in rows_for(msg, msg_index=5, wall_ns=5000)]
    s = next(r for r in rows if r["row_kind"] == "PAIR_SUM")
    assert s["body2_wrench_populated"] is False
    assert s["body2_force_x"] == 0.0  # summed default, but flagged unpopulated above
    points = [r for r in rows if r["row_kind"] == "POINT"]
    assert all(p["body2_wrench_populated"] is False for p in points)
    print("test_body2_wrench_missing_is_recorded_as_unpopulated_not_assumed_zero: PASS")


def test_snake_case_wrench_keys_also_parsed():
    c = _two_point_contact(body1_key="body_1_wrench", body2_key="body_2_wrench")
    msg = {"header": {"stamp": {"sec": 6, "nsec": 0}}, "contact": [c]}
    rows = [row_dict(r) for r in rows_for(msg, msg_index=6, wall_ns=6000)]
    s = next(r for r in rows if r["row_kind"] == "PAIR_SUM")
    assert abs(s["body1_force_x"] - 1.5) < 1e-12
    assert s["body2_wrench_populated"] is True
    print("test_snake_case_wrench_keys_also_parsed: PASS")


def test_timing_columns_unchanged_across_row_kinds():
    # A downstream reader that only ever looked at wall_ns/msg_index (every
    # existing contact-timing analysis in this repo) must see identical
    # values on every row kind for one message.
    msg = {"header": {"stamp": {"sec": 7, "nsec": 500}}, "contact": [_two_point_contact()]}
    rows = [row_dict(r) for r in rows_for(msg, msg_index=42, wall_ns=999000)]
    assert all(r["wall_ns"] == 999000 for r in rows)
    assert all(r["msg_index"] == 42 for r in rows)
    assert all(r["sim_sec"] == 7 for r in rows)
    assert all(r["sim_nsec"] == 500 for r in rows)
    print("test_timing_columns_unchanged_across_row_kinds: PASS")


if __name__ == "__main__":
    test_empty_message_emits_one_empty_row()
    test_two_point_manifold_produces_two_point_rows_and_one_correct_sum()
    test_closing_axis_projection_matches_known_geometry()
    test_body2_wrench_missing_is_recorded_as_unpopulated_not_assumed_zero()
    test_snake_case_wrench_keys_also_parsed()
    test_timing_columns_unchanged_across_row_kinds()
    print("ALL PASS")

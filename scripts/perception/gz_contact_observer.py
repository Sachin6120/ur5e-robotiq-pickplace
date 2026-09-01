#!/usr/bin/env python3
"""gz_contact_observer.py — record ONE Gazebo contact-sensor stream to CSV.

WHY THIS EXISTS

    Stage-2A O1's failure was classified as "lift-onset asymmetric seating"
    purely by RECONSTRUCTING pad/object contact from the pose stream and the
    known pad geometry.  Nothing measured contact directly, because the
    parallel-jaw pad contact sensors were publishing nothing at all: their
    <contact><collision> references named the URDF-side collision names, and
    sdformat's fixed-joint lumping had renamed those collisions, so
    gz::sim::systems::Contact matched zero collisions per sensor.  See the
    2026-08-29 comments in parallel_jaw_gripper.urdf.xacro.

    With those references corrected this script is the recorder for the two
    resulting streams.

2026-08-29 FIX -- FULL CONTACT MANIFOLD, NOT JUST POINT 0

    gz.msgs.Contact carries FOUR parallel repeated arrays -- position[],
    normal[], depth[], wrench[] -- one entry per CONTACT POINT within one
    colliding collision PAIR (gz/msgs/contact.proto, gz/physics/GetContacts.hh:
    both installed under /opt/ros/jazzy). A box-face-on-box-face contact is
    routinely resolved into MULTIPLE points (a manifold), so the total force
    on a pad is the VECTOR SUM over that array, never a single point's share.

    The original version of this script kept only wrench[0]/position[0]/
    normal[0]/depth[0] and wrote `len(contacts)` (the number of PAIRS, almost
    always 1 for one sensor) into a column named n_contacts -- which reads as
    a point count but is not one. Every pad-force magnitude this project
    produced from that version is a lower bound of unknown tightness, not a
    pad total. See the 2026-08-29 contact-wrench source audit for the full
    derivation.

    Fixed here: one POINT row per contact point (n_points, point_index,
    per-point position/normal/depth/body_1_wrench/body_2_wrench), PLUS one
    PAIR_SUM row per collision pair per message carrying the vector sum of
    every point's body_1_wrench/body_2_wrench force -- the actual pad total.
    Both row kinds carry the raw world-frame force AND its projection onto
    the gripper closing axis (--closing-axis-yaw-deg), so "pad total force"
    and "pad total force along the axis that matters" are both directly
    readable without post-processing. body_2_wrench is recorded whenever the
    message carries it, with an explicit populated flag -- whether gz-sim
    actually fills it in this build is exactly the kind of thing that must be
    observed, not assumed.

WHAT IT WRITES

    One CSV row per CONTACT POINT (row_kind=POINT) and one summary row per
    collision PAIR per message (row_kind=PAIR_SUM), plus one row with
    row_kind=EMPTY, n_pairs=0 for every published message that carried no
    contacts. That EMPTY row is the whole point: it is the only way to
    distinguish "pad is not touching" from "the sensor is dead again", which
    is exactly the failure this script exists because of. A run whose CSV
    has a header and no rows at all means the sensor never published and the
    run's contact evidence is ABSENT, not negative.

    Contact TIMING semantics are unchanged from the prior version: wall_ns
    (arrival time) and msg_index are populated identically for every row
    kind, so any existing first/last-contact-instant analysis over this CSV
    (filtering on n_pairs>0 or on collision2) still works unmodified.

USAGE
    python3 scripts/perception/gz_contact_observer.py \
        --topic /world/empty/model/ur5e_robotiq/link/wrist_3_link/sensor/pad_fixed_contact/contact \
        --out evidence/<dir>/contact_pad_fixed.csv \
        --closing-axis-yaw-deg 15.0

    --closing-axis-yaw-deg is optional. Omitting it leaves the
    *_closing_axis columns blank; the raw world-frame force columns are
    always populated regardless. This project's own recorded normals
    confirm the gripper closing axis in world frame is
    (cos(yaw), sin(yaw), 0) for the configured object yaw (2026-08-29
    contact-wrench audit) -- pass that same yaw here, not an assumption
    made by this script.

    Start before the motion begins, SIGTERM to stop. Subscribes only.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

COLS = [
    "wall_ns",
    "msg_index",
    "sim_sec",
    "sim_nsec",
    "row_kind",  # POINT | PAIR_SUM | EMPTY
    "n_pairs",  # number of colliding collision PAIRS in this message
    "pair_index",  # index into that pair list
    "collision1",
    "collision2",
    "n_points",  # number of contact POINTS in this pair (manifold size)
    "point_index",  # POINT rows only; blank for PAIR_SUM/EMPTY
    "pos_x",
    "pos_y",
    "pos_z",
    "normal_x",
    "normal_y",
    "normal_z",
    "depth",
    # body_1_wrench: "contact force acting on the first body [collision1],
    # expressed in the world frame" (dart::collision::Contact / gz::physics
    # GetContactsFromLastStepFeature::ExtraContactDataT, both installed
    # source). For POINT rows this is that single point's force; for
    # PAIR_SUM it is the vector sum over every point in the pair.
    "body1_force_x",
    "body1_force_y",
    "body1_force_z",
    "body1_torque_x",
    "body1_torque_y",
    "body1_torque_z",
    # body_2_wrench: the JointWrench message's OTHER half (body_2_name/
    # body_2_wrench). DART states the force on the second body is exactly
    # -force (installed dart/collision/Contact.hpp), but whether gz-sim
    # actually populates this field in this build was NOT established by
    # source reading alone -- body2_wrench_populated records what was
    # observed, not what was assumed.
    "body2_force_x",
    "body2_force_y",
    "body2_force_z",
    "body2_torque_x",
    "body2_torque_y",
    "body2_torque_z",
    "body2_wrench_populated",
    # Projection of body_1's world-frame force onto the gripper closing
    # axis, i.e. how much of this force acts along the axis the pads
    # actually close on. For PAIR_SUM this is the projection of the summed
    # vector (equal, by linearity, to the sum of the per-point projections).
    "closing_axis_x",
    "closing_axis_y",
    "closing_axis_z",
    "body1_force_closing_axis",
    "body2_force_closing_axis",
]


def _num(d, key, default=""):
    if not isinstance(d, dict):
        return default
    v = d.get(key, default)
    return v if v is not None else default


def _vec3(d, key):
    v = d.get(key) if isinstance(d, dict) else None
    if not isinstance(v, dict):
        return None
    return (_num(v, "x", 0.0), _num(v, "y", 0.0), _num(v, "z", 0.0))


def _wrench_force_torque(wrench_msg):
    """(force_xyz, torque_xyz, populated) from one gz.msgs.Wrench dict.

    `populated` means the message carried a non-empty force object at all
    (protobuf3 JSON omits all-default submessages by default), not that the
    values are non-zero -- a genuinely-zero force is legitimate data.
    """
    if not isinstance(wrench_msg, dict):
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), False
    force = _vec3(wrench_msg, "force")
    torque = _vec3(wrench_msg, "torque")
    populated = force is not None
    force = force or (0.0, 0.0, 0.0)
    torque = torque or (0.0, 0.0, 0.0)
    return force, torque, populated


def _joint_wrench_bodies(jw):
    """(body1_force, body1_torque, body2_force, body2_torque, body2_populated)
    from one gz.msgs.JointWrench dict. Tries both the JSON camelCase
    (body1Wrench, the default protobuf-JSON mapping for body_1_wrench) and
    the literal snake_case, since gz's exact `gz topic --json-output`
    spelling is observed behaviour, not a stable contract to assume."""
    if not isinstance(jw, dict):
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), False
    w1 = jw.get("body1Wrench", jw.get("body_1_wrench"))
    w2 = jw.get("body2Wrench", jw.get("body_2_wrench"))
    f1, t1, _ = _wrench_force_torque(w1)
    f2, t2, pop2 = _wrench_force_torque(w2)
    return f1, t1, f2, t2, pop2


def json_messages(fh):
    """Yield one parsed JSON message at a time.

    gz topic --json-output may pretty-print across several lines, so this
    accumulates until the buffer parses rather than assuming one line per
    message.
    """
    buf = ""
    for line in fh:
        if not line.strip() and not buf:
            continue
        buf += line
        try:
            msg = json.loads(buf)
        except json.JSONDecodeError:
            continue
        buf = ""
        yield msg


def rows_for(msg, msg_index, wall_ns, closing_axis=None):
    """closing_axis: optional unit (x, y, z) tuple; None leaves the
    projection columns blank rather than silently assuming an axis.

    Builds each row as a dict keyed by COLS, then maps to COLS order at the
    end -- this is what actually keeps row length matched to the header
    (36 columns) as the schema grows, instead of hand-counted positional
    literals silently drifting out of sync with COLS.
    """
    header = msg.get("header", {}) or {}
    stamp = header.get("stamp", {}) or {}
    sim_sec = _num(stamp, "sec")
    sim_nsec = _num(stamp, "nsec", _num(stamp, "nsecs"))
    contacts = msg.get("contact", []) or []
    ax = closing_axis

    def base_row():
        row = {name: "" for name in COLS}
        row["wall_ns"] = wall_ns
        row["msg_index"] = msg_index
        row["sim_sec"] = sim_sec
        row["sim_nsec"] = sim_nsec
        if ax is not None:
            row["closing_axis_x"], row["closing_axis_y"], row["closing_axis_z"] = ax
        return row

    def as_list(row):
        return [row[name] for name in COLS]

    if not contacts:
        row = base_row()
        row["row_kind"] = "EMPTY"
        row["n_pairs"] = 0
        return [as_list(row)]

    def proj(force):
        if ax is None:
            return ""
        return force[0] * ax[0] + force[1] * ax[1] + force[2] * ax[2]

    out = []
    n_pairs = len(contacts)
    for pair_index, c in enumerate(contacts):
        c1 = (c.get("collision1", {}) or {}).get("name", "")
        c2 = (c.get("collision2", {}) or {}).get("name", "")
        positions = c.get("position") or []
        normals = c.get("normal") or []
        depths = c.get("depth") or []
        wrenches = c.get("wrench") or []
        n_points = max(len(positions), len(normals), len(depths), len(wrenches))

        sum_f1 = [0.0, 0.0, 0.0]
        sum_t1 = [0.0, 0.0, 0.0]
        sum_f2 = [0.0, 0.0, 0.0]
        sum_t2 = [0.0, 0.0, 0.0]
        any_pop2 = False

        for point_index in range(n_points):
            pos = positions[point_index] if point_index < len(positions) else {}
            nrm = normals[point_index] if point_index < len(normals) else {}
            depth = depths[point_index] if point_index < len(depths) else ""
            jw = wrenches[point_index] if point_index < len(wrenches) else {}
            f1, t1, f2, t2, pop2 = _joint_wrench_bodies(jw)
            for i in range(3):
                sum_f1[i] += f1[i]
                sum_t1[i] += t1[i]
                sum_f2[i] += f2[i]
                sum_t2[i] += t2[i]
            any_pop2 = any_pop2 or pop2

            row = base_row()
            row.update({
                "row_kind": "POINT",
                "n_pairs": n_pairs,
                "pair_index": pair_index,
                "collision1": c1,
                "collision2": c2,
                "n_points": n_points,
                "point_index": point_index,
                "pos_x": _num(pos, "x"), "pos_y": _num(pos, "y"), "pos_z": _num(pos, "z"),
                "normal_x": _num(nrm, "x"), "normal_y": _num(nrm, "y"), "normal_z": _num(nrm, "z"),
                "depth": depth,
                "body1_force_x": f1[0], "body1_force_y": f1[1], "body1_force_z": f1[2],
                "body1_torque_x": t1[0], "body1_torque_y": t1[1], "body1_torque_z": t1[2],
                "body2_force_x": f2[0], "body2_force_y": f2[1], "body2_force_z": f2[2],
                "body2_torque_x": t2[0], "body2_torque_y": t2[1], "body2_torque_z": t2[2],
                "body2_wrench_populated": pop2,
                "body1_force_closing_axis": proj(f1),
                "body2_force_closing_axis": proj(f2),
            })
            out.append(as_list(row))

        row = base_row()
        row.update({
            "row_kind": "PAIR_SUM",
            "n_pairs": n_pairs,
            "pair_index": pair_index,
            "collision1": c1,
            "collision2": c2,
            "n_points": n_points,
            "body1_force_x": sum_f1[0], "body1_force_y": sum_f1[1], "body1_force_z": sum_f1[2],
            "body1_torque_x": sum_t1[0], "body1_torque_y": sum_t1[1], "body1_torque_z": sum_t1[2],
            "body2_force_x": sum_f2[0], "body2_force_y": sum_f2[1], "body2_force_z": sum_f2[2],
            "body2_torque_x": sum_t2[0], "body2_torque_y": sum_t2[1], "body2_torque_z": sum_t2[2],
            "body2_wrench_populated": any_pop2,
            "body1_force_closing_axis": proj(sum_f1),
            "body2_force_closing_axis": proj(sum_f2),
        })
        out.append(as_list(row))

    return out

def main():
    ap = argparse.ArgumentParser(description="Record one Gazebo contact sensor stream.")
    ap.add_argument("--topic", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument(
        "--closing-axis-yaw-deg",
        type=float,
        default=None,
        help="Object/gripper yaw in degrees. When given, records the world-"
             "frame gripper closing axis (cos(yaw), sin(yaw), 0) and each "
             "row's force projection onto it. Omit to leave those columns "
             "blank rather than assume an axis.",
    )
    args = ap.parse_args()

    closing_axis = None
    if args.closing_axis_yaw_deg is not None:
        yaw_rad = math.radians(args.closing_axis_yaw_deg)
        closing_axis = (math.cos(yaw_rad), math.sin(yaw_rad), 0.0)

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out = open(out_path, "w", buffering=1)
    out.write(",".join(COLS) + "\n")

    msg_index = 0
    try:
        while True:
            proc = subprocess.Popen(
                ["gz", "topic", "-e", "--json-output", "-t", args.topic],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
            )
            try:
                for msg in json_messages(proc.stdout):
                    wall_ns = time.time_ns()
                    msg_index += 1
                    for row in rows_for(msg, msg_index, wall_ns, closing_axis):
                        out.write(",".join(str(v) for v in row) + "\n")
            finally:
                try:
                    proc.kill()
                except Exception:
                    pass
            # Topic not up yet (or gz exited): retry rather than dying, the
            # sensor is only advertised once the model is spawned.
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        out.flush()
        out.close()
        print(f"[gz_contact_observer] {args.topic}: {msg_index} messages", file=sys.stderr)


if __name__ == "__main__":
    main()

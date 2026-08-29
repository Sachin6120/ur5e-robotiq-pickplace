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

WHAT IT WRITES

    One CSV row per CONTACT POINT, plus one row with n_contacts=0 for every
    published message that carried no contacts.  That zero row is the whole
    point: it is the only way to distinguish "pad is not touching" from "the
    sensor is dead again", which is exactly the failure this script exists
    because of.  A run whose CSV has a header and no rows at all means the
    sensor never published and the run's contact evidence is ABSENT, not
    negative.

USAGE
    python3 scripts/perception/gz_contact_observer.py \
        --topic /world/empty/model/ur5e_robotiq/link/wrist_3_link/sensor/pad_fixed_contact/contact \
        --out evidence/<dir>/contact_pad_fixed.csv

    Start before the motion begins, SIGTERM to stop.  Subscribes only.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

COLS = [
    "wall_ns",
    "msg_index",
    "sim_sec",
    "sim_nsec",
    "n_contacts",
    "contact_index",
    "collision1",
    "collision2",
    "pos_x",
    "pos_y",
    "pos_z",
    "normal_x",
    "normal_y",
    "normal_z",
    "depth",
    "force_x",
    "force_y",
    "force_z",
]


def _num(d, key, default=""):
    if not isinstance(d, dict):
        return default
    v = d.get(key, default)
    return v if v is not None else default


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


def rows_for(msg, msg_index, wall_ns):
    header = msg.get("header", {}) or {}
    stamp = header.get("stamp", {}) or {}
    sim_sec = _num(stamp, "sec")
    sim_nsec = _num(stamp, "nsec", _num(stamp, "nsecs"))
    contacts = msg.get("contact", []) or []
    if not contacts:
        return [[wall_ns, msg_index, sim_sec, sim_nsec, 0, "", "", "",
                 "", "", "", "", "", "", "", "", "", ""]]
    out = []
    for i, c in enumerate(contacts):
        c1 = (c.get("collision1", {}) or {}).get("name", "")
        c2 = (c.get("collision2", {}) or {}).get("name", "")
        # position/normal/depth/wrench are REPEATED within one contact (one
        # entry per contact point).  Emit the first of each alongside the
        # pair; the point count is what n_contacts and contact_index carry.
        pos = (c.get("position") or [{}])[0]
        nrm = (c.get("normal") or [{}])[0]
        depths = c.get("depth") or []
        depth = depths[0] if depths else ""
        wrenches = c.get("wrench") or []
        force = {}
        if wrenches:
            force = ((wrenches[0].get("body1Wrench")
                      or wrenches[0].get("body_1_wrench") or {}).get("force") or {})
        out.append([
            wall_ns, msg_index, sim_sec, sim_nsec, len(contacts), i, c1, c2,
            _num(pos, "x"), _num(pos, "y"), _num(pos, "z"),
            _num(nrm, "x"), _num(nrm, "y"), _num(nrm, "z"),
            depth,
            _num(force, "x"), _num(force, "y"), _num(force, "z"),
        ])
    return out


def main():
    ap = argparse.ArgumentParser(description="Record one Gazebo contact sensor stream.")
    ap.add_argument("--topic", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

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
                    for row in rows_for(msg, msg_index, wall_ns):
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

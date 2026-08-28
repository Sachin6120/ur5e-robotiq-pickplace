#!/usr/bin/env python3
"""Offline, evaluation-only F3 grasp-retention analyzer.

Reads recorded Gazebo evidence; publishes and commands nothing.  The gripper
reference is explicitly ``wrist_3_link`` and the object is ``pick_target``.
Both poses must come from the same Gazebo world-pose stream.

Modern pose_recorder.py CSVs contain simulation timestamps.  Legacy M3 CSVs
contain only wall timestamps; for tooling replay only, --allow-wall-time-map
maps wall time to simulation time from the LIFT_BEGIN/LIFT_DONE log anchors.
That approximation is called out in the JSON and must not be used for an F3
verdict.  Future F3 evidence must contain native ``sim_t``.
"""

import argparse
import csv
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
import yaml

GRIPPER = "wrist_3_link"
OBJECT = "pick_target"
WINDOWS = {
    "G0": lambda b, d: (b - 0.5, b),
    "L1": lambda b, d: (d, d + 0.25),
    "L2": lambda b, d: (d + 1.5, d + 2.0),
}
EVENT_RE = re.compile(
    r"\[(?P<wall>\d+(?:\.\d+)?)\].*M3 STAGE \d+ "
    r"(?P<name>[A-Z0-9_]+).*?\bt=(?P<sim>\d+(?:\.\d+)?)")


def quat_matrix(q):
    x, y, z, w = map(float, q)
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-12:
        raise ValueError("degenerate quaternion")
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def transform_inverse_product(gripper, obj):
    """Return T_go = inverse(T_world_gripper) @ T_world_object."""
    rg = quat_matrix(gripper[4:8])
    ro = quat_matrix(obj[4:8])
    return rg.T @ (obj[1:4] - gripper[1:4]), rg.T @ ro


def parse_events(path):
    events = {}
    for line in Path(path).read_text(errors="replace").splitlines():
        m = EVENT_RE.search(line)
        if m:
            events[m.group("name")] = {
                "sim_t": float(m.group("sim")), "wall_t": float(m.group("wall"))}
    return events


def read_poses(path, events, allow_wall_map):
    rows, warning = [], None
    with open(path, newline="") as fh:
        reader = csv.DictReader(fh)
        fields = set(reader.fieldnames or [])
        native = "sim_t" in fields
        if not native and "wall_t" not in fields:
            raise ValueError("pose CSV needs sim_t, or wall_t for legacy replay")
        if not native:
            if not allow_wall_map:
                raise ValueError("pose CSV lacks native sim_t (use --allow-wall-time-map for tooling replay only)")
            a, b = events.get("LIFT_BEGIN"), events.get("LIFT_DONE")
            if not a or not b or b["wall_t"] == a["wall_t"]:
                raise ValueError("cannot map legacy wall timestamps without two event anchors")
            warning = ("legacy pose CSV lacks sim_t; affine wall->simulation-time mapping "
                       "between adjacent stage-event anchors used for tooling validation only")
            anchors = sorted((v["wall_t"], v["sim_t"]) for v in events.values())

            def mapped(wall):
                if wall <= anchors[0][0]:
                    pair = (anchors[0], anchors[1])
                elif wall >= anchors[-1][0]:
                    pair = (anchors[-2], anchors[-1])
                else:
                    pair = next((anchors[i], anchors[i+1])
                                for i in range(len(anchors)-1)
                                if anchors[i][0] <= wall <= anchors[i+1][0])
                (w0, s0), (w1, s1) = pair
                return s0+(wall-w0)*(s1-s0)/(w1-w0)
        for row in reader:
            entity = row.get("entity", "")
            if entity not in (GRIPPER, OBJECT):
                continue
            if native:
                t = float(row["sim_t"])
            else:
                t = mapped(float(row["wall_t"]))
            rows.append((t, entity, np.array([float(row[k]) for k in
                         ("x", "y", "z", "qx", "qy", "qz", "qw")])))
    return rows, native, warning


def paired_relative(rows, native_sim):
    streams = {name: sorted((t, p) for t, entity, p in rows if entity == name)
               for name in (GRIPPER, OBJECT)}
    out, j = [], 0
    tolerance = 1e-9 if native_sim else 0.02
    for tg, gripper in streams[GRIPPER]:
        objects = streams[OBJECT]
        if not objects:
            break
        while j+1 < len(objects) and abs(objects[j+1][0]-tg) < abs(objects[j][0]-tg):
            j += 1
        if abs(objects[j][0]-tg) > tolerance:
            continue
        to, obj = objects[j]
        t = (tg+to)/2
        p, r = transform_inverse_product(np.r_[tg, gripper], np.r_[to, obj])
        out.append({"t": t, "p_go": p, "R_go": r,
                    "pair_time_separation_s": abs(tg-to),
                    "object_z": float(obj[2])})
    return out


def window(samples, requested):
    lo, hi = requested
    selected = [s for s in samples if lo <= s["t"] <= hi]
    times = [s["t"] for s in selected]
    spacing = max(np.diff(times)) if len(times) > 1 else None
    duration = hi-lo
    actual_span = (times[-1]-times[0]) if len(times) > 1 else 0.0
    # Coverage-quality rule, not a physics acceptance threshold: at least two
    # paired samples, >=90% temporal span, and no gap > half the window.
    adequate = (len(times) >= 2 and actual_span >= 0.9*duration and
                spacing is not None and spacing <= 0.5*duration)
    coverage = {
        "requested_interval_sim_s": [lo, hi],
        "actual_covered_interval_sim_s": None if not times else [times[0], times[-1]],
        "sample_count": len(times),
        "max_sample_spacing_s": None if spacing is None else float(spacing),
        "adequate": bool(adequate),
        "adequacy_rule": "count>=2; actual span>=90% requested; max gap<=50% window",
    }
    if not selected:
        return coverage, None
    p = np.median(np.stack([s["p_go"] for s in selected]), axis=0)
    mid = (lo+hi)/2
    rep = min(selected, key=lambda s: abs(s["t"]-mid))
    pts = np.stack([s["p_go"] for s in selected])
    # Simple trend proxy: endpoint coordinate-wise-median blocks when enough
    # data exist, otherwise first-to-last. Reported, never thresholded.
    k = max(1, len(selected)//4)
    trend = np.median(pts[-k:], axis=0)-np.median(pts[:k], axis=0)
    rep_out = {
        "translation_m_coordinate_median": p.tolist(),
        "orientation_method": "sample nearest requested-window midpoint",
        "orientation_sample_sim_t": rep["t"],
        "rotation_matrix": rep["R_go"].tolist(),
        "object_world_z_m_median": float(np.median([s["object_z"] for s in selected])),
        "translation_peak_to_peak_m": (np.max(pts, axis=0)-np.min(pts, axis=0)).tolist(),
        "translation_peak_to_peak_norm_m": float(np.linalg.norm(np.max(pts, axis=0)-np.min(pts, axis=0))),
        "translation_trend_vector_m": trend.tolist(),
        "translation_trend_velocity_proxy_m_s": (trend/duration).tolist(),
    }
    return coverage, rep_out


def parse_contact(path, intervals):
    """Summarize protobuf-text contact messages by their simulation stamps."""
    text = Path(path).read_text(errors="replace")
    blocks = re.split(r"(?=^header \{)", text, flags=re.M)
    stamps, unexpected = [], []
    for block in blocks:
        sm = re.search(r"stamp \{\s*sec:\s*(\d+)\s*nsec:\s*(\d+)", block, re.S)
        if not sm:
            continue
        stamp = int(sm.group(1))+int(sm.group(2))*1e-9
        if OBJECT in block:
            stamps.append(stamp)
        elif "contact {" in block:
            unexpected.append(stamp)
    result = {"source": str(path), "object_contact_messages": len(stamps),
              "first_relevant_sim_t": min(stamps) if stamps else None,
              "last_relevant_sim_t": max(stamps) if stamps else None,
              "non_object_contact_messages": len(unexpected), "intervals": {}}
    for name, (lo, hi) in intervals.items():
        hit = [t for t in stamps if lo <= t <= hi]
        result["intervals"][name] = {"present": bool(hit), "message_count": len(hit),
             "first_sim_t": min(hit) if hit else None, "last_sim_t": max(hit) if hit else None,
             "non_object_contact_messages": sum(lo <= t <= hi for t in unexpected)}
    return result


def parse_joints(paths, intervals):
    """Report position movement from either joint-named or master-only CSVs."""
    result = {}
    for path in paths:
        with open(path, newline="") as fh:
            reader = csv.DictReader(fh)
            fields = set(reader.fieldnames or [])
            if "sim_t" not in fields or "position" not in fields:
                result[str(path)] = {"error": "requires sim_t and position columns"}
                continue
            grouped = {}
            for row in reader:
                name = row.get("joint") or "actuated_master"
                grouped.setdefault(name, []).append((float(row["sim_t"]), float(row["position"])))
            summary = {}
            for name, samples in grouped.items():
                summary[name] = {}
                for window_name, (lo, hi) in intervals.items():
                    vals = [(t, p) for t, p in samples if lo <= t <= hi]
                    summary[name][window_name] = {
                        "sample_count": len(vals),
                        "position_min_rad": min((p for _, p in vals), default=None),
                        "position_max_rad": max((p for _, p in vals), default=None),
                        "peak_to_peak_rad": ((max(p for _, p in vals)-min(p for _, p in vals))
                                              if vals else None),
                        "first_sim_t": vals[0][0] if vals else None,
                        "last_sim_t": vals[-1][0] if vals else None,
                    }
            result[str(path)] = summary
    return result


def yn(value):
    return "yes" if value else "no"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pose-csv", required=True)
    ap.add_argument("--event-log", required=True)
    ap.add_argument("--out")
    ap.add_argument("--allow-wall-time-map", action="store_true")
    ap.add_argument("--contact", action="append", default=[], metavar="LABEL=PATH")
    ap.add_argument("--joint-csv", action="append", default=[])
    args = ap.parse_args()
    warnings = []
    try:
        events = parse_events(args.event_log)
        if not all(k in events for k in ("LIFT_BEGIN", "LIFT_DONE")):
            raise ValueError("event log lacks LIFT_BEGIN or LIFT_DONE with simulation timestamp")
        begin, done = events["LIFT_BEGIN"]["sim_t"], events["LIFT_DONE"]["sim_t"]
        rows, native_sim, warning = read_poses(args.pose_csv, events, args.allow_wall_time_map)
        if warning: warnings.append(warning)
        samples = paired_relative(rows, native_sim)
        intervals = {k: fn(begin, done) for k, fn in WINDOWS.items()}
        checkpoints, representatives = {}, {}
        for name, interval in intervals.items():
            checkpoints[name], representatives[name] = window(samples, interval)
        required_coverage = all(checkpoints[k]["adequate"] for k in WINDOWS)

        contacts = {}
        contact_intervals = dict(intervals)
        contact_intervals["LIFT"] = (begin, done)
        for spec in args.contact:
            label, path = spec.split("=", 1)
            contacts[label] = parse_contact(path, contact_intervals)

        scene = yaml.safe_load((Path(__file__).parents[2]/"config/scene.yaml").read_text())
        threshold = float(scene["thresholds"]["post_lift_slip_max_m"])
        loss_rad = float(scene["grasp"]["grasp_loss_threshold_rad"])
        metrics = {}
        if required_coverage:
            g0, l1, l2 = (representatives[k] for k in ("G0", "L1", "L2"))
            p0, p1, p2 = (np.array(x["translation_m_coordinate_median"])
                          for x in (g0, l1, l2))
            r0, r2 = np.array(g0["rotation_matrix"]), np.array(l2["rotation_matrix"])
            angle = math.acos(float(np.clip((np.trace(r0.T@r2)-1)/2, -1, 1)))
            metrics = {
                "slip_G0_L1_m": float(np.linalg.norm(p1-p0)),
                "slip_G0_L2_m": float(np.linalg.norm(p2-p0)),
                "slip_L1_L2_m": float(np.linalg.norm(p2-p1)),
                "orientation_G0_L2_rad": angle,
                "orientation_G0_L2_deg": math.degrees(angle),
                "object_world_z_G0_L1_m": l1["object_world_z_m_median"]-g0["object_world_z_m_median"],
                "object_world_z_G0_L2_m": l2["object_world_z_m_median"]-g0["object_world_z_m_median"],
            }
        valid_f3_timebase = native_sim
        slip_pass = (yn(metrics["slip_G0_L2_m"] <= threshold)
                     if metrics and valid_f3_timebase else "indeterminate")
        if not valid_f3_timebase:
            warnings.append("non-native timebase forbids a valid F3 retention verdict")
        retained = slip_pass if slip_pass != "indeterminate" else "indeterminate"
        # A failed relative-slip result does not by itself prove a drop.
        object_drop = "indeterminate"
        out = {
            "tool": "f3_retention.py", "evaluation_only": True,
            "reference_frames": {"world": "Gazebo world pose stream", "gripper": GRIPPER,
                                 "object": OBJECT, "no_silent_tcp_substitution": True},
            "events": events, "windows": intervals, "checkpoint_coverage": checkpoints,
            "representative_poses": representatives,
            "transform_definition": "T_go=inverse(T_world_wrist_3_link)@T_world_pick_target",
            "metrics": metrics,
            "thresholds": {
                "relative_slip_max_m": {"value": threshold,
                    "source": "config/scene.yaml thresholds.post_lift_slip_max_m",
                    "semantics": "established M3 Gazebo-ground-truth relative-slip pass criterion"},
                "grasp_loss_threshold_rad": {"value": loss_rad,
                    "source": "config/scene.yaml grasp.grasp_loss_threshold_rad",
                    "semantics": "one-sided actuated-joint early-abort corroboration; not a slip threshold"}},
            "verdicts": {"retained": retained, "object_drop": object_drop,
                         "slip_pass": slip_pass},
            "contact_evidence": contacts,
            "joint_evidence": (parse_joints(args.joint_csv, contact_intervals)
                               if args.joint_csv else {"status": "not available"}),
            "native_sim_time_pose_samples": native_sim,
            "required_checkpoint_coverage_adequate": required_coverage,
            "warnings": warnings,
        }
    except Exception as exc:
        out = {"tool": "f3_retention.py", "evaluation_only": True,
               "verdicts": {"retained": "indeterminate", "object_drop": "indeterminate",
                            "slip_pass": "indeterminate"},
               "error": str(exc), "warnings": warnings}
    rendered = json.dumps(out, indent=2)
    if args.out:
        Path(args.out).write_text(rendered+"\n")
    print(rendered)
    return 0 if "error" not in out else 2


if __name__ == "__main__":
    sys.exit(main())

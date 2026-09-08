"""stage3c_contact_qual.py — Stage-3C C1.1 qualification-only helpers for
GENUINE Gazebo-physics contact evidence about dynamic_obstacle_0.

WHY THIS EXISTS

    The Stage-3C C1 closeout audit found that every "physical obstacle
    contacts = 0" figure produced so far (Stage-3B B1/B2 and Stage-3C
    C1A/C1B alike) was actually derived from MoveIt's
    /check_state_validity -- a PlanningScene collision query, not the
    Gazebo physics engine. That is a MoveIt-side model check; it is not
    evidence about what the simulated physics actually did. This module
    supplies the missing piece: it instruments the QUALIFICATION obstacle
    with a real gz::sim::systems::Contact sensor so DART's own contact
    manifolds (collision pair names, positions, normals, depths, wrenches)
    are recorded by scripts/perception/gz_contact_observer.py.

OWNERSHIP BOUNDARY (do not erode)

    Nothing here modifies production Stage-3B assets. The production
    obstacle model
        ur5e_robotiq_description/models/dynamic_obstacle/model.sdf
    and the production
        ur5e_pick_place/src/dynamic_obstacle_scene_node.cpp
    stay byte-for-byte untouched. derive_contact_instrumented_sdf() READS
    the production SDF and returns a qualification-only copy with exactly
    one added element -- a <sensor type="contact"> inside the existing
    <link> -- so the qualification obstacle's collision geometry, pose,
    inertial properties, DeterministicMotion parameters and PosePublisher
    settings are provably the production ones rather than a retyped
    lookalike. sdf_difference_summary() renders the exact line-level
    difference for the evidence directory.

CONTACT SENSOR SEMANTICS

    gz::sim::systems::Contact (loaded by this project's world,
    ur5e_robotiq_description/worlds/tabletop_rgbd.sdf) publishes
    gz.msgs.Contacts for the collisions named in the sensor's
    <contact><collision> list. Because this SDF is authored directly (not
    converted from URDF), the collision name is exactly the one written
    here; the fixed-joint lumping rename that silenced this project's pad
    sensors in 2026-08-29 cannot apply.

    OBSERVED, NOT ASSUMED -- this build publishes ONLY while contacts
    exist. It was initially expected that empty (zero-contact) messages
    would also be published, which would have made "sensor alive but
    nothing touching" self-evident. The positive-control evidence
    (evidence/stage3c_c11_contact_probe_*) falsified that: msg_index 1 is
    already the first contact, with no preceding empty messages, and a
    genuinely contact-free qualification run therefore produces a CSV with
    a header and no rows at all. That is why liveness cannot be inferred
    from message count and is instead established by run_liveness_probe(),
    which deliberately causes one real contact with the same observer
    process, on the same topic, in the same simulator session, AFTER the
    run's own evidence window has closed.
"""

from pathlib import Path
import csv
import difflib

# The collision name inside the production obstacle SDF's <link>. Asserted
# rather than assumed by derive_contact_instrumented_sdf().
OBSTACLE_COLLISION_NAME = "collision"
OBSTACLE_LINK_NAME = "obstacle_link"
CONTACT_SENSOR_NAME = "dynamic_obstacle_contact"
# Model name the spawn script uses (scripts/spawn_dynamic_obstacle.sh).
OBSTACLE_MODEL_NAME = "dynamic_obstacle"
# Scoped topic gz-sim advertises for a contact sensor on that link.
CONTACT_TOPIC = (
    f"/world/empty/model/{OBSTACLE_MODEL_NAME}/link/{OBSTACLE_LINK_NAME}"
    f"/sensor/{CONTACT_SENSOR_NAME}/contact"
)

_SENSOR_BLOCK = f"""
      <!-- STAGE-3C C1.1 QUALIFICATION-ONLY ADDITION (not production).
           Real gz::sim::systems::Contact sensor so DART's own contact
           manifolds involving this obstacle are recorded as genuine
           physics evidence, instead of inferring physical contact from
           MoveIt's /check_state_validity. This is the ONLY difference
           from the production obstacle SDF. -->
      <sensor name="{CONTACT_SENSOR_NAME}" type="contact">
        <contact>
          <collision>{OBSTACLE_COLLISION_NAME}</collision>
        </contact>
        <update_rate>200</update_rate>
        <always_on>1</always_on>
      </sensor>
"""


def add_contact_sensor(sdf_text: str) -> str:
    """Insert the contact sensor into the obstacle SDF's <link> block.

    Raises if the expected link/collision structure is not present, rather
    than silently producing an SDF whose sensor references nothing (the
    2026-08-29 silent-sensor failure mode).
    """
    if f'<collision name="{OBSTACLE_COLLISION_NAME}">' not in sdf_text:
        raise ValueError(
            f"obstacle SDF has no <collision name=\"{OBSTACLE_COLLISION_NAME}\">; "
            "refusing to add a contact sensor that would match nothing")
    if f'<link name="{OBSTACLE_LINK_NAME}">' not in sdf_text:
        raise ValueError(
            f"obstacle SDF has no <link name=\"{OBSTACLE_LINK_NAME}\">")
    if "</link>" not in sdf_text:
        raise ValueError("obstacle SDF has no </link> to insert before")
    if CONTACT_SENSOR_NAME in sdf_text:
        return sdf_text  # already instrumented
    return sdf_text.replace("</link>", _SENSOR_BLOCK + "    </link>", 1)


def derive_contact_instrumented_sdf(production_sdf_path: Path, out_path: Path) -> dict:
    """Write a qualification-only copy of the PRODUCTION obstacle SDF whose
    only difference is the added contact sensor. Returns provenance info
    (source path, output path, unified diff) for the evidence directory."""
    production_text = Path(production_sdf_path).read_text()
    instrumented = add_contact_sensor(production_text)
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    Path(out_path).write_text(instrumented)
    return {
        "source_production_sdf": str(production_sdf_path),
        "qualification_sdf": str(out_path),
        "diff": sdf_difference_summary(production_text, instrumented),
        "contact_topic": CONTACT_TOPIC,
    }


def sdf_difference_summary(production_text: str, qualification_text: str) -> str:
    return "".join(difflib.unified_diff(
        production_text.splitlines(keepends=True),
        qualification_text.splitlines(keepends=True),
        fromfile="production_model.sdf",
        tofile="qualification_model.sdf",
    ))


def summarize_staleness(ticks) -> dict:
    """Runtime sanity of the C1 monitor's obstacle-freshness telemetry.

    The C1 closeout audit found a real mixed wall/sim-time defect that made
    scene_age_ms hugely NEGATIVE while scene_stale still read 0 ("fresh").
    Both halves of that bug are now fixed (harness node uses sim time;
    is_obstacle_data_stale() treats negative/infinite ages as stale), so
    these counts must come back clean:

      negative_age_ticks       -- MUST be 0
      no_data_ticks_stale      -- ticks with age=inf that correctly read stale
      no_data_ticks_not_stale  -- MUST be 0 (missing data read as fresh)
    """
    out = {
        "ticks_examined": 0,
        "negative_age_ticks": 0,
        "no_data_ticks_stale": 0,
        "no_data_ticks_not_stale": 0,
        "stale_ticks": 0,
        "min_age_ms": None,
        "max_finite_age_ms": None,
    }
    for t in ticks or []:
        raw_age = t.get("scene_age_ms")
        raw_stale = t.get("scene_stale")
        if raw_age is None or raw_stale is None:
            continue
        out["ticks_examined"] += 1
        stale = str(raw_stale) == "1"
        if stale:
            out["stale_ticks"] += 1
        try:
            age = float(raw_age)
        except (TypeError, ValueError):
            continue
        if age != age or age in (float("inf"), float("-inf")):  # NaN/inf
            if age == float("inf"):
                if stale:
                    out["no_data_ticks_stale"] += 1
                else:
                    out["no_data_ticks_not_stale"] += 1
            continue
        if age < 0.0:
            out["negative_age_ticks"] += 1
        out["min_age_ms"] = age if out["min_age_ms"] is None else min(out["min_age_ms"], age)
        out["max_finite_age_ms"] = (
            age if out["max_finite_age_ms"] is None else max(out["max_finite_age_ms"], age))
    return out


LIVENESS_BLOCKER_NAME = "c11_liveness_blocker"


def liveness_blocker_sdf(x: float, y: float, z: float, size: float = 0.05) -> str:
    """A small static box placed inside the obstacle's own sweep, used ONLY
    after a qualification run's evidence is already captured, to prove the
    still-running contact observer instance really can record a contact in
    THIS session (see run_liveness_probe())."""
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{LIVENESS_BLOCKER_NAME}">
    <static>true</static>
    <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
    <link name="blocker_link">
      <collision name="blocker_collision">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
      </collision>
      <visual name="blocker_visual">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
      </visual>
    </link>
  </model>
</sdf>
"""


def spawn_model(name: str, sdf_text: str, timeout_ms: int = 5000):
    """Spawn one model into the running /world/empty via gz service."""
    import subprocess
    flat = sdf_text.replace("\n", " ").replace('"', '\\"')
    return subprocess.run(
        ["gz", "service", "-s", "/world/empty/create",
         "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
         "--timeout", str(timeout_ms),
         "--req", f'sdf: "{flat}", name: "{name}", allow_renaming: false'],
        capture_output=True, text=True)


def topic_info(topic: str) -> str:
    import subprocess
    return subprocess.run(
        f"gz topic -i -t {topic}", shell=True, capture_output=True, text=True).stdout


def summarize_contact_csv(csv_path: Path, filter_substrings=None,
                          wall_ns_max=None, wall_ns_min=None) -> dict:
    """Summarize a gz_contact_observer.py CSV as PHYSICS evidence.

    Returns, separately and explicitly:
      messages_total    -- every published gz.msgs.Contacts message, INCLUDING
                           the row_kind=EMPTY ones. > 0 is the observer's
                           health proof: the sensor was alive and publishing.
      contact_pair_rows -- PAIR_SUM rows, i.e. actual colliding collision
                           pairs reported by the physics engine.
      matched_pairs     -- contact pairs whose collision names matched any of
                           filter_substrings (None = match everything).
      pair_names        -- the distinct collision-name pairs observed.
      first/last_sim_s  -- sim-time bounds of matched contact events.

    wall_ns_max / wall_ns_min bound which rows are counted, by the
    observer's own arrival wall_ns column. This is what separates a run's
    QUALIFICATION window from the trailing in-session liveness probe that
    deliberately creates a contact afterwards.

    NOTE ON "no messages": this Gazebo build's gz::sim::systems::Contact
    publishes ONLY while contacts exist -- the positive-control evidence
    (evidence/stage3c_c11_contact_probe_*) shows msg_index 1 IS the first
    contact, with no preceding empty messages. So messages_total == 0 means
    "no contacts were reported", and is NOT by itself proof the sensor was
    alive. Liveness must therefore come from run_liveness_probe(), not from
    this count; observer_healthy here only reports whether any message was
    seen in the requested window.
    """
    csv_path = Path(csv_path)
    out = {
        "csv_path": str(csv_path),
        "csv_exists": csv_path.is_file(),
        "messages_total": 0,
        "empty_rows": 0,
        "contact_pair_rows": 0,
        "matched_pair_rows": 0,
        "pair_names": [],
        "matched_pair_names": [],
        "first_contact_sim_s": None,
        "last_contact_sim_s": None,
        "observer_healthy": False,
    }
    if not out["csv_exists"]:
        return out

    def in_window(row):
        raw = row.get("wall_ns")
        if raw in (None, ""):
            return True
        try:
            w = int(raw)
        except (TypeError, ValueError):
            return True
        if wall_ns_max is not None and w >= wall_ns_max:
            return False
        if wall_ns_min is not None and w < wall_ns_min:
            return False
        return True

    seen_msg_indices = set()
    pair_names = set()
    matched_names = set()
    first_s = None
    last_s = None

    with csv_path.open() as fh:
        for row in csv.DictReader(fh):
            if not in_window(row):
                continue
            idx = row.get("msg_index")
            if idx:
                seen_msg_indices.add(idx)
            kind = row.get("row_kind")
            if kind == "EMPTY":
                out["empty_rows"] += 1
                continue
            if kind != "PAIR_SUM":
                continue  # POINT rows are per-point detail of the same pair
            out["contact_pair_rows"] += 1
            c1 = row.get("collision1", "") or ""
            c2 = row.get("collision2", "") or ""
            pair_names.add(f"{c1}<->{c2}")
            if filter_substrings is None or any(
                    s in c1 or s in c2 for s in filter_substrings):
                out["matched_pair_rows"] += 1
                matched_names.add(f"{c1}<->{c2}")
                try:
                    sim_s = float(row.get("sim_sec") or 0) + \
                        float(row.get("sim_nsec") or 0) * 1e-9
                except (TypeError, ValueError):
                    sim_s = None
                if sim_s is not None:
                    first_s = sim_s if first_s is None else min(first_s, sim_s)
                    last_s = sim_s if last_s is None else max(last_s, sim_s)

    out["messages_total"] = len(seen_msg_indices)
    out["pair_names"] = sorted(pair_names)
    out["matched_pair_names"] = sorted(matched_names)
    out["first_contact_sim_s"] = first_s
    out["last_contact_sim_s"] = last_s
    out["observer_healthy"] = out["messages_total"] > 0
    return out


def run_liveness_probe(evidence_dir: Path, contact_csv: Path, blocker_xyz,
                       wait_s: float = 6.0, respawn_obstacle_sdf: Path = None,
                       blocker_size: float = 0.05) -> dict:
    """In-session proof that THIS observer instance really records contacts.

    Runs only AFTER a qualification run's own evidence is fully captured, so
    it cannot contaminate it: every contact it creates lands after
    probe_start_wall_ns and is excluded from the qualification window by
    summarize_contact_csv(wall_ns_max=probe_start_wall_ns).

    Why this is needed: this Gazebo build's contact sensor publishes nothing
    at all while nothing is touching, so a clean run legitimately produces an
    empty CSV -- indistinguishable, on its own, from a sensor that never
    worked. Deliberately causing one real contact at the end, with the same
    process on the same topic in the same simulator session, is what makes
    the run's zero-contact result falsifiable.
    """
    import subprocess
    import time

    evidence_dir = Path(evidence_dir)
    info = {
        "probe_start_wall_ns": time.time_ns(),
        "blocker_xyz": list(blocker_xyz),
        "respawned_obstacle": False,
    }
    if respawn_obstacle_sdf is not None:
        r = spawn_model(OBSTACLE_MODEL_NAME, Path(respawn_obstacle_sdf).read_text())
        info["respawned_obstacle"] = "true" in r.stdout.lower()
        info["obstacle_respawn_reply"] = r.stdout.strip()
        time.sleep(1.0)

    blocker = liveness_blocker_sdf(*blocker_xyz, size=blocker_size)
    (evidence_dir / "liveness_blocker.sdf").write_text(blocker)
    rb = spawn_model(LIVENESS_BLOCKER_NAME, blocker)
    info["blocker_spawn_reply"] = rb.stdout.strip()
    info["blocker_spawned"] = "true" in rb.stdout.lower()

    info["topic_info_during_probe"] = topic_info(CONTACT_TOPIC)
    time.sleep(wait_s)

    after = summarize_contact_csv(
        contact_csv, filter_substrings=[OBSTACLE_MODEL_NAME],
        wall_ns_min=info["probe_start_wall_ns"])
    info["contacts_after_probe"] = after
    info["liveness_proven"] = after["matched_pair_rows"] > 0
    return info

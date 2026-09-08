#!/usr/bin/env python3
"""scripts/test_stage3c_contact_probe.py — Stage-3C C1.1 POSITIVE CONTROL.

Proves, in an isolated throwaway simulator session, that this project's
Gazebo contact-telemetry pipeline can actually DETECT a known physical
contact before any zero-contact result from it is trusted.

This is NOT a manipulation qualification run. No robot, no MoveIt, no
m3_grasp, no perception. It launches only `gz sim` (server, headless) with
this project's own world (which loads gz::sim::systems::Contact), spawns
two qualification-only bodies arranged to collide deterministically under
gravity, records the contact-sensor stream with the project's existing
scripts/perception/gz_contact_observer.py, and asserts that a real contact
event with exact collision names and sim timestamps was captured.

It also enumerates the live Gazebo topic list first, so the question "does
this setup already expose a usable global physics/contact topic?" is
answered from observation rather than assumption.
"""

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR / "scripts/lib"))

import stage3c_contact_qual as cq  # noqa: E402

WORLD_FILE = REPO_DIR / "ur5e_robotiq_description/worlds/tabletop_rgbd.sdf"
PROBE_MODEL_NAME = "c11_contact_probe"
PROBE_LINK = "probe_link"
PROBE_COLLISION = "probe_collision"
PROBE_SENSOR = "probe_contact"
PROBE_TOPIC = (
    f"/world/empty/model/{PROBE_MODEL_NAME}/link/{PROBE_LINK}"
    f"/sensor/{PROBE_SENSOR}/contact"
)

# Dropped from 0.30 m onto the world's own ground plane: a deterministic,
# harmless, unambiguous contact. Nothing in the manipulation workspace.
PROBE_SDF = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{PROBE_MODEL_NAME}">
    <pose>-2.0 -2.0 0.30 0 0 0</pose>
    <link name="{PROBE_LINK}">
      <inertial>
        <mass>1.0</mass>
        <inertia><ixx>0.001</ixx><ixy>0</ixy><ixz>0</ixz>
                 <iyy>0.001</iyy><iyz>0</iyz><izz>0.001</izz></inertia>
      </inertial>
      <collision name="{PROBE_COLLISION}">
        <geometry><box><size>0.10 0.10 0.10</size></box></geometry>
      </collision>
      <visual name="probe_visual">
        <geometry><box><size>0.10 0.10 0.10</size></box></geometry>
      </visual>
      <sensor name="{PROBE_SENSOR}" type="contact">
        <contact><collision>{PROBE_COLLISION}</collision></contact>
        <update_rate>200</update_rate>
        <always_on>1</always_on>
      </sensor>
    </link>
  </model>
</sdf>
"""


def kill_probe_session():
    for cmd in [
        "pkill -9 -f 'gz sim' || true",
        "pkill -9 -f 'ruby.*gz' || true",
        "pkill -9 -f 'gz_contact_observer' || true",
        "pkill -9 -f 'gz topic -e' || true",
    ]:
        subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)


def main():
    print("=" * 71)
    print("STAGE-3C C1.1 — GAZEBO CONTACT-TELEMETRY POSITIVE CONTROL")
    print("=" * 71)

    kill_probe_session()

    ts = time.strftime("%Y%m%d_%H%M%S")
    ev = REPO_DIR / f"evidence/stage3c_c11_contact_probe_{ts}"
    ev.mkdir(parents=True, exist_ok=True)
    print(f"Evidence directory: {ev}")

    env = os.environ.copy()
    install_lib = "/home/sachin/ur5e_ws/install/ur5e_robotiq_description/lib"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{install_lib}:{env.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"

    procs = {}
    result = {"probe": "stage3c_c11_contact_probe", "world_file": str(WORLD_FILE)}

    try:
        print("\n[1/6] Launching isolated headless Gazebo server (project world)...")
        sim = subprocess.Popen(
            f"gz sim -s -r -v 2 {WORLD_FILE}", shell=True, executable="/bin/bash",
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)
        procs["sim"] = sim

        ready = False
        for _ in range(40):
            r = subprocess.run("gz topic -l", shell=True, capture_output=True, text=True)
            if "/world/empty" in r.stdout:
                ready = True
                break
            time.sleep(0.5)
        if not ready:
            raise RuntimeError("Gazebo server did not come up")
        print("Gazebo server is up.")

        print("\n[2/6] Enumerating live Gazebo topics BEFORE spawning any sensor")
        print("      (answers: does this setup already expose a global physics/contact topic?)")
        topics_before = subprocess.run(
            "gz topic -l", shell=True, capture_output=True, text=True).stdout
        (ev / "gz_topics_before_spawn.txt").write_text(topics_before)
        contactish_before = [t for t in topics_before.splitlines()
                             if "contact" in t.lower() or "physics" in t.lower()]
        print(f"  contact/physics-looking topics before spawn: {contactish_before or 'NONE'}")
        result["global_contact_topics_before_spawn"] = contactish_before

        print("\n[3/6] Spawning qualification-only probe body with a contact sensor...")
        probe_sdf_path = ev / "contact_probe.sdf"
        probe_sdf_path.write_text(PROBE_SDF)
        flat = PROBE_SDF.replace("\n", " ").replace('"', '\\"')
        spawn = subprocess.run(
            ["gz", "service", "-s", "/world/empty/create",
             "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
             "--timeout", "5000",
             "--req", f'sdf: "{flat}", name: "{PROBE_MODEL_NAME}", allow_renaming: false'],
            capture_output=True, text=True)
        print(f"  spawn reply: {spawn.stdout.strip()}")
        if "true" not in spawn.stdout.lower():
            raise RuntimeError(f"probe spawn failed: {spawn.stdout} {spawn.stderr}")

        time.sleep(1.0)
        topics_after = subprocess.run(
            "gz topic -l", shell=True, capture_output=True, text=True).stdout
        (ev / "gz_topics_after_spawn.txt").write_text(topics_after)
        if PROBE_TOPIC not in topics_after:
            raise RuntimeError(
                f"contact sensor topic {PROBE_TOPIC} was NOT advertised; "
                "the sensor did not attach (see gz_topics_after_spawn.txt)")
        print(f"  contact sensor topic advertised: {PROBE_TOPIC}")
        result["contact_topic"] = PROBE_TOPIC

        print("\n[4/6] Recording the contact stream with gz_contact_observer.py...")
        csv_path = ev / "contact_probe.csv"
        obs = subprocess.Popen(
            f"python3 {REPO_DIR}/scripts/perception/gz_contact_observer.py "
            f"--topic {PROBE_TOPIC} --out {csv_path}",
            shell=True, executable="/bin/bash", env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, start_new_session=True)
        procs["observer"] = obs

        print("      letting the probe body fall onto the ground plane (8 s)...")
        time.sleep(8.0)

        print("\n[5/6] Stopping observer and analysing captured telemetry...")
        try:
            os.killpg(obs.pid, signal.SIGTERM)
        except Exception:
            pass
        time.sleep(1.5)

        summary = cq.summarize_contact_csv(csv_path)
        result["stage_a_dynamic_body_summary"] = summary
        print(f"  messages published (health):  {summary['messages_total']}")
        print(f"  EMPTY (no-contact) rows:      {summary['empty_rows']}")
        print(f"  contact PAIR rows:            {summary['contact_pair_rows']}")
        print(f"  distinct collision pairs:     {summary['pair_names']}")
        print(f"  first/last contact sim time:  {summary['first_contact_sim_s']} / "
              f"{summary['last_contact_sim_s']}")

        stage_a_passed = (
            summary["observer_healthy"]
            and summary["contact_pair_rows"] > 0
            and len(summary["pair_names"]) > 0
            and summary["first_contact_sim_s"] is not None
        )
        result["stage_a_verdict"] = "PASS" if stage_a_passed else "FAIL"
        if not stage_a_passed:
            raise RuntimeError("Stage A (dynamic body) contact capability proof FAILED")

        # --- STAGE B ---------------------------------------------------
        # Stage A proves the pipeline works for an ORDINARY DYNAMIC body.
        # It does NOT prove it works for the body type C1A/C1B actually
        # use: dynamic_obstacle is <kinematic>true</kinematic> with gravity
        # off, driven by SetWorldPoseCmd from the DeterministicMotion
        # plugin. If that body type could not generate contacts at all, a
        # "zero contacts" result from C1A/C1B would be vacuous -- exactly
        # the class of unfalsifiable evidence this task exists to remove.
        # So: spawn the REAL production-derived, contact-instrumented
        # obstacle and put a static block inside its own sweep, and prove
        # a contact is reported for that exact configuration.
        print("\n[5b/6] STAGE B: same proof for the ACTUAL obstacle body type")
        print("       (production-derived, kinematic, gravity-off, DeterministicMotion)...")
        prod_sdf = REPO_DIR / "ur5e_robotiq_description/models/dynamic_obstacle/model.sdf"
        qual_sdf = ev / "dynamic_obstacle_contact_instrumented.sdf"
        provenance = cq.derive_contact_instrumented_sdf(prod_sdf, qual_sdf)
        (ev / "obstacle_sdf_provenance.json").write_text(json.dumps(provenance, indent=2))
        (ev / "obstacle_sdf_difference.diff").write_text(provenance["diff"])
        print(f"  derived from production SDF; difference is sensor-only "
              f"({len(provenance['diff'].splitlines())} diff lines)")

        # Static blocker placed in the middle of the production sweep
        # (X=0.70, Y in [0.25,0.45], Z=0.85), so the kinematic obstacle is
        # driven straight through it.
        blocker_sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="c11_static_blocker">
    <static>true</static>
    <pose>0.70 0.35 0.85 0 0 0</pose>
    <link name="blocker_link">
      <collision name="blocker_collision">
        <geometry><box><size>0.10 0.10 0.20</size></box></geometry>
      </collision>
      <visual name="blocker_visual">
        <geometry><box><size>0.10 0.10 0.20</size></box></geometry>
      </visual>
    </link>
  </model>
</sdf>
"""
        (ev / "static_blocker.sdf").write_text(blocker_sdf)
        for name, sdf_text in (("c11_static_blocker", blocker_sdf),
                               (cq.OBSTACLE_MODEL_NAME, qual_sdf.read_text())):
            flat_b = sdf_text.replace("\n", " ").replace('"', '\\"')
            sp = subprocess.run(
                ["gz", "service", "-s", "/world/empty/create",
                 "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
                 "--timeout", "5000",
                 "--req", f'sdf: "{flat_b}", name: "{name}", allow_renaming: false'],
                capture_output=True, text=True)
            print(f"  spawn {name}: {sp.stdout.strip()}")
            if "true" not in sp.stdout.lower():
                raise RuntimeError(f"stage B spawn failed for {name}")
            time.sleep(1.0)

        topics_b = subprocess.run(
            "gz topic -l", shell=True, capture_output=True, text=True).stdout
        (ev / "gz_topics_stage_b.txt").write_text(topics_b)
        if cq.CONTACT_TOPIC not in topics_b:
            raise RuntimeError(
                f"obstacle contact topic {cq.CONTACT_TOPIC} not advertised in stage B")
        print(f"  obstacle contact topic advertised: {cq.CONTACT_TOPIC}")

        csv_b = ev / "contact_probe_stage_b_obstacle.csv"
        obs_b = subprocess.Popen(
            f"python3 {REPO_DIR}/scripts/perception/gz_contact_observer.py "
            f"--topic {cq.CONTACT_TOPIC} --out {csv_b}",
            shell=True, executable="/bin/bash", env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, start_new_session=True)
        procs["observer_b"] = obs_b
        print("      letting the obstacle sweep through the blocker (10 s, >2 periods)...")
        time.sleep(10.0)
        try:
            os.killpg(obs_b.pid, signal.SIGTERM)
        except Exception:
            pass
        time.sleep(1.5)

        summary_b = cq.summarize_contact_csv(csv_b, filter_substrings=["dynamic_obstacle"])
        result["stage_b_kinematic_obstacle_summary"] = summary_b
        print(f"  messages published (health):  {summary_b['messages_total']}")
        print(f"  EMPTY (no-contact) rows:      {summary_b['empty_rows']}")
        print(f"  contact PAIR rows:            {summary_b['contact_pair_rows']}")
        print(f"  matched dynamic_obstacle rows:{summary_b['matched_pair_rows']}")
        print(f"  distinct collision pairs:     {summary_b['pair_names']}")
        print(f"  first/last contact sim time:  {summary_b['first_contact_sim_s']} / "
              f"{summary_b['last_contact_sim_s']}")

        stage_b_passed = (
            summary_b["observer_healthy"]
            and summary_b["matched_pair_rows"] > 0
            and summary_b["first_contact_sim_s"] is not None
        )
        result["stage_b_verdict"] = "PASS" if stage_b_passed else "FAIL"

        passed = stage_a_passed and stage_b_passed
        result["verdict"] = "PASS" if passed else "FAIL"

        print("\n[6/6] Writing evidence...")
        (ev / "contact_probe_results.json").write_text(json.dumps(result, indent=2))

        print("\n" + "=" * 71)
        print(f"  Stage A (dynamic body):            {result['stage_a_verdict']}")
        print(f"  Stage B (kinematic obstacle body): {result['stage_b_verdict']}")
        print(f"POSITIVE-CONTROL VERDICT: {result['verdict']}")
        print("=" * 71)
        return 0 if passed else 1

    finally:
        print("\nTearing down isolated probe session...")
        for p in procs.values():
            try:
                os.killpg(p.pid, signal.SIGTERM)
            except Exception:
                pass
        time.sleep(1.0)
        kill_probe_session()


if __name__ == "__main__":
    sys.exit(main())

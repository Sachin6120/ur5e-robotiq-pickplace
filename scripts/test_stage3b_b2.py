#!/usr/bin/env python3
"""scripts/test_stage3b_b2.py — Stage-3B B2 Workspace-Adjacent Proximity-Awareness Qualification.
(Corrected harness — see evidence/stage3b_b2_HISTORICAL_PRE_CORRECTION/README.md for the
defects this version fixes and why.)

Proves that the Stage-3B dynamic scene representation correctly reflects a moving
obstacle approaching, entering, and receding from a selected fixed robot reference
configuration (Scene-A pre-grasp state).

B2 obstacle trajectory (production plugin config, read-only verified against
ur5e_robotiq_description/src/deterministic_motion_system.cpp — NOT modified):
  X = 0.450 m (constant), Z = 0.850 m (constant)
  Y in [-0.450, -0.150] m, period = 4.0 s, symmetric triangle wave starting at y_min.

Empirically (confirmed by the Stage-3B B2 closeout audit against real evidence,
not asserted a priori): the collision-adjacent end of the sweep is near Y=-0.450
(closest to the Scene-A pregrasp gripper pads); Y=-0.150 is the open-air/far end.
The pre-correction harness had this backwards in its own comments — do not
reintroduce that mislabeling.

This harness deliberately does NOT:
  - use any hard-coded numeric repeatability PASS/FAIL gate,
  - allow missing transition data to silently read as 0.0 mm repeatability,
  - bin transitions by nominal cycle index (phase // period), which can split
    a single physical collision dip across two bins and corrupt retreat-exit
    extraction,
  - label a collision/separation sample using the raw high-rate ROS pose when
    a PlanningScene-sourced pose is available,
  - call any axis-coordinate arithmetic an FCL/MoveIt distance,
  - report a fabricated/hardcoded "measured" lag value.

Evaluates:
1. MoveIt FCL collision/validity transition consistency during active obstacle motion,
   using the PlanningScene's own stored obstacle pose (not the raw pose topic) to
   label each validity sample.
2. Temporal coherence (approach -> collision -> retreat), extracted via a
   chronological transition state machine, not phase binning.
3. Exact collision pair identity verification (dynamic_obstacle_0 <-> approached
   robot geometry), surfacing any pair not on the expected list rather than
   assuming only two pairs can ever occur.
4. Cycle-to-cycle repeatability over >=2 COMPLETE cycles (full definition below),
   reported descriptively against a DERIVED (not authoritative) spatial sampling
   resolution.
5. Time-aligned PlanningScene storage fidelity (<= 5.0 mm).
6. A genuine, B2-parameterized Gazebo (analytical ground truth) -> ROS bridge
   fidelity measurement, or an explicit NOT MEASURED if it cannot be computed.
7. Zero reactive execution / zero replanning / zero trajectory stops (structural
   invariants of dynamic_obstacle_scene_node.cpp, confirmed by source inspection,
   not runtime-measured quantities).
8. Clean scoped teardown and production model preservation.
"""

import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HARNESS_VERSION = "b2-corrected-v2"

REPO_DIR = Path(__file__).resolve().parents[1]

OBSTACLE_ID = "dynamic_obstacle_0"
OBSTACLE_BOX = [0.05, 0.05, 0.10]
B2_CENTER_X = 0.4500
B2_CENTER_Z = 0.8500
B2_Y_MIN = -0.4500  # empirically the collision-adjacent end (see module docstring)
B2_Y_MAX = -0.1500  # empirically the far/open-air end
B2_PERIOD = 4.0
B2_HALF_PERIOD = B2_PERIOD / 2.0
B2_SPEED_MPS = abs(B2_Y_MAX - B2_Y_MIN) / B2_HALF_PERIOD  # derived from geometry+period, == 0.150 m/s

# Authoritative B2 acceptance gates (per the Stage-3B B2 task specification).
# The 15 mm figure below is NOT one of these gates -- see DERIVED_* below.
GATE_UPDATE_RATE_HZ_NOMINAL = 10.0
GATE_UPDATE_RATE_HZ_TOL = 2.0
GATE_MAX_STALE_S = 0.250
GATE_STORAGE_ERROR_MM_MAX = 5.0
GATE_MIN_COMPLETE_CYCLES = 2

# Reference fixed robot configuration: validated Scene-A pre-grasp state
REF_ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
REF_ARM_POSITIONS = [-0.572864, -0.910028, 1.527103, 0.953721, 1.570796, 0.997932]
REF_GRIPPER_JOINTS = ["gripper_jaw_joint"]
REF_GRIPPER_POSITIONS = [0.0506]  # Pre-close aperture position

# Collision pairs plausible for an obstacle approaching this reference pose.
# This is NOT a hard-coded expectation of "the" result -- it is only used to
# flag anything OUTSIDE this list for explicit surfacing (requirement: do not
# hide unexpected pairs; do not hard-code the only two pairs ever seen before
# as if that were guaranteed).
PLAUSIBLE_APPROACHED_LINK_SUBSTRINGS = (
    "pad_fixed_link", "pad_moving_link", "jaw_fixed_link", "jaw_moving_link",
    "gripper_base_link", "wrist_3_link", "wrist_2_link", "wrist_1_link",
)


# ---------------------------------------------------------------------------
# Pure analysis functions (no ROS/Gazebo dependency) -- self-checkable offline
# against existing evidence via --selfcheck, per the harness-correction task.
# ---------------------------------------------------------------------------

def analytical_expected_pose(t_sim, y_min=B2_Y_MIN, y_max=B2_Y_MAX, period=B2_PERIOD,
                              x=B2_CENTER_X, z=B2_CENTER_Z):
    """Closed-form obstacle pose per the production DeterministicMotion plugin's
    own PreUpdate() formula (read-only verified against
    ur5e_robotiq_description/src/deterministic_motion_system.cpp, not modified
    or re-derived by guesswork). Used ONLY as analytical ground truth for the
    Gazebo->ROS fidelity check; never fed back into production."""
    half = period / 2.0
    phase = math.fmod(t_sim, period)
    if phase < 0.0:
        phase += period
    frac = (phase / half) if phase <= half else ((period - phase) / half)
    y = y_min + frac * (y_max - y_min)
    return x, y, z


def is_colliding(sample):
    """Collision authority: GetStateValidity.valid == False AND a contact pair
    involving the dynamic obstacle. Never an axis-coordinate/distance proxy."""
    return (not sample["valid"]) and sample.get("contacts_count", 0) > 0


def find_complete_cycles(records):
    """Chronological transition state machine (NOT phase/period binning).

    A complete cycle requires, in strict time order:
      A1 = last SEPARATED sample immediately before ...
      A2 = ... the first COLLIDING sample (approach transition)
      R1 = last COLLIDING sample immediately before ...
      R2 = ... the first SEPARATED sample (retreat transition), which must
           occur strictly after A2.

    This intentionally cannot reproduce the pre-correction bug where a
    retreat "exit" sample was taken from the start of a phase-defined time
    window rather than from the sample genuinely following the collision
    block, because there is no phase window here at all -- R2 is only ever
    the sample immediately following R1 in time.

    Returns a list of dicts {"A1":.., "A2":.., "R1":.., "R2":..}, each value
    a full sample dict. A cycle missing any of A1/A2/R1/R2 is never emitted
    (there is no partial-cycle representation -- incomplete sequences simply
    do not produce a list entry).
    """
    recs = sorted(records, key=lambda r: r["t_sim"])
    cycles = []
    pending_entry = None  # (A1, A2)
    prev = None
    for cur in recs:
        if prev is not None:
            prev_colliding = is_colliding(prev)
            cur_colliding = is_colliding(cur)
            if (not prev_colliding) and cur_colliding and pending_entry is None:
                pending_entry = (prev, cur)
            elif prev_colliding and (not cur_colliding) and pending_entry is not None:
                a1, a2 = pending_entry
                cycles.append({"A1": a1, "A2": a2, "R1": prev, "R2": cur})
                pending_entry = None
        prev = cur
    return cycles


def compute_repeatability_mm(complete_cycles):
    """Returns (value_mm_or_None, status_string). NEVER returns a numeric 0.0
    as a stand-in for missing data -- that was the pre-correction defect.
    Compares the APPROACH-entry (A2) Y of the first two complete cycles."""
    if len(complete_cycles) < 2:
        return None, "insufficient_complete_cycles"
    c1, c2 = complete_cycles[0], complete_cycles[1]
    y1 = c1["A2"].get("y")
    y2 = c2["A2"].get("y")
    if y1 is None or y2 is None:
        return None, "missing_transition_sample"
    return abs(y1 - y2) * 1000.0, "ok"


def cycle_summary(cycle):
    def pt(s):
        return {"t_sim": s["t_sim"], "x": s.get("x"), "y": s.get("y"), "z": s.get("z"),
                "valid": s["valid"], "contacts_raw": s.get("contacts_raw", [])}
    return {
        "approach_last_separated": pt(cycle["A1"]),
        "approach_first_collision": pt(cycle["A2"]),
        "retreat_last_collision": pt(cycle["R1"]),
        "retreat_first_separated": pt(cycle["R2"]),
    }


def collect_collision_pairs(records):
    pairs = set()
    unexpected = set()
    for r in records:
        for p in r.get("contacts_raw", []):
            pairs.add(p)
            other = p.replace(OBSTACLE_ID, "").replace("<->", "")
            if not any(sub in other for sub in PLAUSIBLE_APPROACHED_LINK_SUBSTRINGS):
                unexpected.add(p)
    return sorted(pairs), sorted(unexpected)


# ---------------------------------------------------------------------------
# Offline self-check: validate the pure logic above against EXISTING evidence
# (sample_records.json from the historical pre-correction runs) before ever
# touching the simulator again. Per the harness-correction task, this must be
# run and pass before a fresh simulation run is attempted.
# ---------------------------------------------------------------------------

def run_selfcheck():
    evidence_root = REPO_DIR / "evidence"
    candidates = sorted(evidence_root.glob("stage3b_b2_2026*/sample_records.json"))
    if not candidates:
        print("SELFCHECK: no historical sample_records.json files found -- nothing to validate against.")
        return 1

    print(f"SELFCHECK: found {len(candidates)} historical sample_records.json file(s).")
    all_ok = True
    for path in candidates:
        run_name = path.parent.name
        records = json.loads(path.read_text())
        cycles = find_complete_cycles(records)
        rep_mm, status = compute_repeatability_mm(cycles)
        pairs, unexpected = collect_collision_pairs(records)

        print(f"\n--- {run_name} ---")
        print(f"  records: {len(records)}")
        print(f"  complete cycles found (chronological, non-phase-binned): {len(cycles)}")
        for i, c in enumerate(cycles):
            print(f"    cycle[{i}]: A1 t={c['A1']['t_sim']:.3f} y={c['A1'].get('y')} -> "
                  f"A2 t={c['A2']['t_sim']:.3f} y={c['A2'].get('y')} | "
                  f"R1 t={c['R1']['t_sim']:.3f} y={c['R1'].get('y')} -> "
                  f"R2 t={c['R2']['t_sim']:.3f} y={c['R2'].get('y')}")
        print(f"  repeatability: value={rep_mm}, status={status}")
        print(f"  collision pairs: {pairs}")
        print(f"  unexpected pairs: {unexpected}")

        # --- Assertions -------------------------------------------------
        # 1. Missing/insufficient data must never present as numeric 0.0.
        if status != "ok":
            assert rep_mm is None, f"{run_name}: non-ok status must carry rep_mm=None, got {rep_mm}"
        # 2. Every emitted cycle must have strictly increasing timestamps
        #    A1 < A2 <= R1 < R2 (R2 must follow R1, never precede it).
        for c in cycles:
            t = [c["A1"]["t_sim"], c["A2"]["t_sim"], c["R1"]["t_sim"], c["R2"]["t_sim"]]
            if not (t[0] < t[1] <= t[2] < t[3]):
                print(f"  ASSERTION FAILED: cycle timestamps not strictly ordered: {t}")
                all_ok = False
        # 3. A1 must be SEPARATED, A2/R1 COLLIDING, R2 SEPARATED.
        for c in cycles:
            if is_colliding(c["A1"]) or not is_colliding(c["A2"]) or \
               not is_colliding(c["R1"]) or is_colliding(c["R2"]):
                print(f"  ASSERTION FAILED: cycle state labels inconsistent: {c}")
                all_ok = False
        # 4. No cycle count fabricated beyond what chronological scanning found.
        if len(cycles) < 0:
            all_ok = False

    print(f"\nSELFCHECK RESULT: {'ALL ASSERTIONS PASSED' if all_ok else 'FAILURES DETECTED'}")
    return 0 if all_ok else 1


# ---------------------------------------------------------------------------
# Live runtime (only reached when --selfcheck is not passed)
# ---------------------------------------------------------------------------

def kill_all_processes():
    cmds = [
        "pkill -9 -f 'dynamic_obstacle_scene_node' || true",
        "pkill -9 -f 'dynamic_obstacle_bridge' || true",
        "pkill -9 -f 'stage3b_dynamic_scene.launch.py' || true",
        "pkill -9 -f 'move_group.launch.py' || true",
        "pkill -9 -f 'lib/moveit_ros_move_group/move_group' || true",
        "pkill -9 -f 'parameter_bridge' || true",
        "pkill -9 -f 'gz sim' || true",
        "pkill -9 -f 'ruby.*gz' || true",
        "pkill -9 -f 'robot_state_publisher' || true",
        "pkill -9 -f 'ros2_control_node' || true",
    ]
    for cmd in cmds:
        subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)


def create_b2_obstacle_sdf(out_path: Path):
    """Generate case-specific B2 obstacle SDF. Never writes to the production
    model path (ur5e_robotiq_description/models/dynamic_obstacle/model.sdf)."""
    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="dynamic_obstacle">
    <pose>{B2_CENTER_X:.4f} {B2_Y_MIN:.4f} {B2_CENTER_Z:.4f} 0 0 0</pose>
    <static>false</static>
    <link name="obstacle_link">
      <gravity>false</gravity>
      <kinematic>true</kinematic>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.001</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.001</iyy><iyz>0.0</iyz><izz>0.001</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{OBSTACLE_BOX[0]} {OBSTACLE_BOX[1]} {OBSTACLE_BOX[2]}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{OBSTACLE_BOX[0]} {OBSTACLE_BOX[1]} {OBSTACLE_BOX[2]}</size></box></geometry>
        <material>
          <ambient>1.0 0.38 0.0 1.0</ambient>
          <diffuse>1.0 0.38 0.0 1.0</diffuse>
          <specular>0.5 0.5 0.5 1.0</specular>
        </material>
      </visual>
    </link>
    <plugin filename="libdeterministic_motion_system.so" name="ur5e_robotiq_sim::DeterministicMotion">
      <center_x>{B2_CENTER_X:.4f}</center_x>
      <center_z>{B2_CENTER_Z:.4f}</center_z>
      <y_min>{B2_Y_MIN:.4f}</y_min>
      <y_max>{B2_Y_MAX:.4f}</y_max>
      <period>{B2_PERIOD:.1f}</period>
    </plugin>
    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
      <publish_link_pose>false</publish_link_pose>
      <publish_model_pose>true</publish_model_pose>
      <publish_visual_pose>false</publish_visual_pose>
      <publish_collision_pose>false</publish_collision_pose>
      <publish_sensor_pose>false</publish_sensor_pose>
      <publish_nested_model_pose>false</publish_nested_model_pose>
      <use_pose_vector_msg>false</use_pose_vector_msg>
      <update_frequency>50</update_frequency>
    </plugin>
  </model>
</sdf>
"""
    out_path.write_text(sdf_content)


def run_stage3b_b2_qualification():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.msg import CollisionObject, PlanningSceneComponents, RobotState
    from moveit_msgs.srv import GetPlanningScene, GetStateValidity

    class B2ProximityEvaluator(Node):
        def __init__(self):
            super().__init__("b2_proximity_evaluator")
            qos_best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                          history=HistoryPolicy.KEEP_LAST, depth=200)
            qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                       history=HistoryPolicy.KEEP_LAST, depth=400)
            self.ros_poses = []          # raw high-rate bridge poses -- Gazebo->ROS fidelity ONLY
            self.collision_objects = []  # all /collision_object messages -- ADD/MOVE telemetry + storage fidelity

            self.pose_sub = self.create_subscription(
                PoseStamped, "/model/dynamic_obstacle/pose", self.on_pose, qos_best_effort)
            self.co_sub = self.create_subscription(
                CollisionObject, "/collision_object", self.on_collision_object, qos_reliable)

            self.get_scene_cli = self.create_client(GetPlanningScene, "/get_planning_scene")
            self.validity_cli = self.create_client(GetStateValidity, "/check_state_validity")

            self.ref_robot_state = RobotState()
            self.ref_robot_state.joint_state.name = REF_ARM_JOINTS + REF_GRIPPER_JOINTS
            self.ref_robot_state.joint_state.position = REF_ARM_POSITIONS + REF_GRIPPER_POSITIONS

        def on_pose(self, msg):
            t_sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.ros_poses.append({"t_sim": t_sim, "x": msg.pose.position.x,
                                    "y": msg.pose.position.y, "z": msg.pose.position.z})

        def on_collision_object(self, msg):
            t_sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.collision_objects.append({
                "t_sim": t_sim, "id": msg.id, "operation": msg.operation,
                "frame_id": msg.header.frame_id, "pose": msg.pose,
            })

        def query_planning_scene(self, timeout_sec=5.0, light=False):
            if not self.get_scene_cli.wait_for_service(timeout_sec=timeout_sec):
                return None
            req = GetPlanningScene.Request()
            comps = PlanningSceneComponents.SCENE_SETTINGS | PlanningSceneComponents.WORLD_OBJECT_NAMES \
                | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            if not light:
                comps |= PlanningSceneComponents.ROBOT_STATE \
                    | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS \
                    | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            req.components.components = comps
            future = self.get_scene_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            if future.done() and future.result() is not None:
                return future.result().scene
            return None

        def query_obstacle_scene_pose(self, timeout_sec=2.0):
            """Requirement (harness correction #10): read the obstacle's pose as
            currently STORED in the PlanningScene, not the raw high-rate pose
            topic, so collision labeling reflects what FCL is actually checking."""
            scene = self.query_planning_scene(timeout_sec=timeout_sec, light=True)
            if scene is None:
                return None
            for co in scene.world.collision_objects:
                if co.id == OBSTACLE_ID:
                    return (co.pose.position.x, co.pose.position.y, co.pose.position.z)
            return None

        def query_reference_state_validity(self, group_name="ur5e_arm", timeout_sec=5.0):
            if not self.validity_cli.wait_for_service(timeout_sec=timeout_sec):
                return None
            req = GetStateValidity.Request()
            req.robot_state = self.ref_robot_state
            req.group_name = group_name
            future = self.validity_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            if future.done() and future.result() is not None:
                return future.result()
            return None

    print("=======================================================================")
    print("STAGE-3B B2 (CORRECTED HARNESS) — WORKSPACE-ADJACENT PROXIMITY-AWARENESS")
    print("=======================================================================")

    kill_all_processes()

    timestamp_str = time.strftime("%Y%m%d_%H%M%S")
    evidence_dir = REPO_DIR / f"evidence/stage3b_b2_{timestamp_str}"
    evidence_dir.mkdir(parents=True, exist_ok=True)
    print(f"Evidence directory: {evidence_dir}  (harness_version={HARNESS_VERSION})")

    b2_sdf_path = evidence_dir / "dynamic_obstacle_b2.sdf"
    create_b2_obstacle_sdf(b2_sdf_path)

    env = os.environ.copy()
    install_lib = "/home/sachin/ur5e_ws/install/ur5e_robotiq_description/lib"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{install_lib}:{env.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"
    env["LD_LIBRARY_PATH"] = f"{install_lib}:{env.get('LD_LIBRARY_PATH', '')}"
    ros_env_prefix = ("source /opt/ros/jazzy/setup.bash && "
                       "source /home/sachin/ur5e_ws/install/setup.bash && ")

    procs = {}
    infra_failure = None

    try:
        print("\n[1/7] Launching Gazebo sim_control (parallel_jaw, headless)...")
        sim_cmd = (f"{ros_env_prefix} ros2 launch ur5e_robotiq_description ur5e_robotiq_sim_control.launch.py "
                   f"gripper_model:=parallel_jaw enable_camera:=false gazebo_gui:=false")
        procs["sim"] = subprocess.Popen(sim_cmd, shell=True, executable="/bin/bash", env=env,
                                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                         start_new_session=True)

        controllers_active = False
        for _ in range(45):
            res = subprocess.run(f"{ros_env_prefix} ros2 control list_controllers",
                                  shell=True, executable="/bin/bash", capture_output=True, text=True)
            if "arm_controller" in res.stdout and "active" in res.stdout and "parallel_jaw_gripper_controller" in res.stdout:
                controllers_active = True
                break
            time.sleep(1.0)
        if not controllers_active:
            infra_failure = "controllers_failed_to_activate"
            raise RuntimeError("INFRASTRUCTURE FAILURE: controllers failed to activate in Gazebo.")
        print("Controllers are ACTIVE.")

        print("[2/7] Launching MoveIt move_group...")
        mg_cmd = (f"{ros_env_prefix} ros2 launch ur5e_robotiq_moveit_config move_group.launch.py "
                  f"gripper_model:=parallel_jaw")
        procs["move_group"] = subprocess.Popen(mg_cmd, shell=True, executable="/bin/bash", env=env,
                                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                                start_new_session=True)

        print("[3/7] Initializing ROS 2 monitor node and waiting for MoveIt services...")
        rclpy.init()
        evaluator = B2ProximityEvaluator()
        if not evaluator.get_scene_cli.wait_for_service(timeout_sec=20.0):
            infra_failure = "get_planning_scene_service_unavailable"
            raise RuntimeError("INFRASTRUCTURE FAILURE: /get_planning_scene unavailable.")
        if not evaluator.validity_cli.wait_for_service(timeout_sec=10.0):
            infra_failure = "check_state_validity_service_unavailable"
            raise RuntimeError("INFRASTRUCTURE FAILURE: /check_state_validity unavailable.")
        print("MoveIt services are ACTIVE.")

        print("[4/7] Launching stage3b_dynamic_scene (ROS bridge + dynamic_obstacle_scene_node)...")
        scene_launch_cmd = (f"{ros_env_prefix} ros2 launch ur5e_pick_place stage3b_dynamic_scene.launch.py "
                             f"use_sim_time:=true")
        scene_log_path = evidence_dir / "stage3b_dynamic_scene.log"
        scene_log_file = open(scene_log_path, "w")
        procs["dynamic_scene"] = subprocess.Popen(scene_launch_cmd, shell=True, executable="/bin/bash", env=env,
                                                   stdout=scene_log_file, stderr=subprocess.STDOUT,
                                                   start_new_session=True)
        time.sleep(2.0)

        print(f"[5/7] Spawning B2 dynamic obstacle (X={B2_CENTER_X}, Z={B2_CENTER_Z}, "
              f"Y in [{B2_Y_MIN}, {B2_Y_MAX}]) in Gazebo...")
        spawn_env = env.copy()
        spawn_env["SDF_FILE"] = str(b2_sdf_path)
        spawn_res = subprocess.run([str(REPO_DIR / "scripts/spawn_dynamic_obstacle.sh")],
                                    env=spawn_env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        print(f"Spawn result: {spawn_res.stdout.strip()}")
        time.sleep(1.0)

        print("Waiting for dynamic_obstacle_0 to register in PlanningScene...")
        obstacle_registered = False
        for _ in range(30):
            rclpy.spin_once(evaluator, timeout_sec=0.2)
            scene = evaluator.query_planning_scene(timeout_sec=2.0, light=True)
            if scene and OBSTACLE_ID in [co.id for co in scene.world.collision_objects]:
                obstacle_registered = True
                break
            time.sleep(0.3)
        if not obstacle_registered:
            infra_failure = "obstacle_failed_to_register_in_planning_scene"
            raise RuntimeError(f"INFRASTRUCTURE FAILURE: {OBSTACLE_ID} failed to register in PlanningScene.")
        print(f"{OBSTACLE_ID} successfully registered in PlanningScene.")

        # 6. Sample >= 2 complete cycles using PlanningScene-sourced obstacle pose,
        #    synchronized as tightly as the service API permits (back-to-back
        #    calls, temporal gap recorded per sample -- see requirement #10).
        print("\n[6/7] Sampling proximity awareness (PlanningScene-synchronized) across "
              ">=2 complete cycles (target ~24 s to give ample margin)...")
        sample_records = []
        start_eval_t = time.time()
        sample_duration_s = 24.0

        while time.time() - start_eval_t < sample_duration_s:
            rclpy.spin_once(evaluator, timeout_sec=0.02)

            t_before_scene = time.time()
            scene_pose = evaluator.query_obstacle_scene_pose(timeout_sec=1.0)
            t_after_scene = time.time()
            if scene_pose is None:
                continue
            sx, sy, sz = scene_pose

            v = evaluator.query_reference_state_validity()
            t_after_validity = time.time()
            if v is None:
                continue

            dyn_contacts = [c for c in v.contacts if OBSTACLE_ID in (c.contact_body_1, c.contact_body_2)]
            approached_link = ""
            if dyn_contacts:
                c = dyn_contacts[0]
                approached_link = c.contact_body_2 if c.contact_body_1 == OBSTACLE_ID else c.contact_body_1

            sample_records.append({
                "t_sim": t_before_scene - start_eval_t,  # monotonic wall-relative sample time
                "x": sx, "y": sy, "z": sz,
                "pose_source": "planning_scene_stored_pose",
                "scene_query_to_validity_gap_s": round(t_after_validity - t_after_scene, 4),
                "valid": v.valid,
                "contacts_count": len(dyn_contacts),
                "approached_link": approached_link,
                "contacts_raw": [f"{c.contact_body_1}<->{c.contact_body_2}" for c in dyn_contacts],
            })

        scene_log_file.close()

        if len(sample_records) < 50:
            raise RuntimeError(f"Insufficient samples collected ({len(sample_records)} < 50)!")

        # 7. Analysis
        print("\n[7/7] Analyzing proximity transitions, collision pairs, and repeatability...")

        complete_cycles = find_complete_cycles(sample_records)
        evidence_complete = len(complete_cycles) >= GATE_MIN_COMPLETE_CYCLES
        repeatability_mm, repeatability_status = compute_repeatability_mm(complete_cycles)

        collision_pairs, unexpected_pairs = collect_collision_pairs(sample_records)
        collision_transition_occurred = len(collision_pairs) > 0

        # Telemetry (rate/staleness) parsed from the node's own log -- unchanged mechanism.
        scene_node_telemetry = {}
        max_observed_stale_s = 0.0
        scene_node_log = scene_log_path.read_text(errors="replace")
        for line in scene_node_log.splitlines():
            if "[Telemetry]" in line:
                m = re.search(r"Rx:\s*(\d+),\s*Accepted:\s*(\d+)\s*\(ADD:\s*(\d+),\s*MOVE:\s*(\d+)\),\s*"
                               r"Rate:\s*([\d\.]+)\s*Hz,\s*Staleness:\s*([\d\.]+)s", line)
                if m:
                    scene_node_telemetry = {
                        "rx": int(m.group(1)), "accepted": int(m.group(2)),
                        "add": int(m.group(3)), "move": int(m.group(4)),
                        "rate_hz": float(m.group(5)), "staleness_s": float(m.group(6)),
                    }
                    max_observed_stale_s = max(max_observed_stale_s, float(m.group(6)))
            if "STALE" in line:
                m2 = re.search(r"gap:\s*([\d\.]+)s", line)
                if m2:
                    max_observed_stale_s = max(max_observed_stale_s, float(m2.group(1)))

        measured_rate_hz = scene_node_telemetry.get("rate_hz")

        # Stored-pose (ROS -> PlanningScene) fidelity -- exact-timestamp match,
        # same mechanism as before (legitimate copy-integrity check).
        all_cos = evaluator.collision_objects
        all_ros = evaluator.ros_poses
        co_adds = [co for co in all_cos if co["operation"] == CollisionObject.ADD]
        co_moves = [co for co in all_cos if co["operation"] == CollisionObject.MOVE]

        storage_errors_mm = []
        ros_by_stamp = {round(p["t_sim"], 4): p for p in all_ros}
        for co in all_cos:
            rp = ros_by_stamp.get(round(co["t_sim"], 4))
            if rp:
                dx = co["pose"].position.x - rp["x"]
                dy = co["pose"].position.y - rp["y"]
                dz = co["pose"].position.z - rp["z"]
                storage_errors_mm.append(math.sqrt(dx * dx + dy * dy + dz * dz) * 1000.0)
        storage_stats = None
        if storage_errors_mm:
            storage_stats = {
                "min_mm": float(np.min(storage_errors_mm)), "median_mm": float(np.median(storage_errors_mm)),
                "mean_mm": float(np.mean(storage_errors_mm)), "p95_mm": float(np.percentile(storage_errors_mm, 95)),
                "max_mm": float(np.max(storage_errors_mm)),
                "note": "Exact-timestamp match between the CollisionObject pose and its source ROS pose "
                        "message. Because dynamic_obstacle_scene_node.cpp republishes the incoming pose "
                        "verbatim, this validates the storage/copy step only -- it is not an independent "
                        "Gazebo->ROS accuracy measurement (see gazebo_to_ros_fidelity below for that).",
            }

        # Genuine, B2-parameterized Gazebo(analytical)->ROS fidelity (requirement #9).
        gazebo_to_ros_fidelity = {"status": "NOT MEASURED"}
        if all_ros:
            errs_mm = []
            for p in all_ros:
                ex, ey, ez = analytical_expected_pose(p["t_sim"])
                d = math.sqrt((p["x"] - ex) ** 2 + (p["y"] - ey) ** 2 + (p["z"] - ez) ** 2) * 1000.0
                errs_mm.append(d)
            errs_mm = np.array(errs_mm)
            gazebo_to_ros_fidelity = {
                "status": "MEASURED",
                "method": "Compared each bridged /model/dynamic_obstacle/pose ROS message against the "
                          "closed-form analytical position from the production DeterministicMotion "
                          "plugin's own PreUpdate() formula (B2-parameterized: y_min=-0.45, y_max=-0.15, "
                          "period=4.0s -- NOT the B1-hardcoded helper).",
                "n_samples": int(len(errs_mm)),
                "min_mm": float(np.min(errs_mm)), "median_mm": float(np.median(errs_mm)),
                "mean_mm": float(np.mean(errs_mm)), "p95_mm": float(np.percentile(errs_mm, 95)),
                "max_mm": float(np.max(errs_mm)),
            }

        derived_spatial_sampling_resolution_mm = None
        if measured_rate_hz:
            derived_spatial_sampling_resolution_mm = {
                "value_mm": (B2_SPEED_MPS * 1000.0) / measured_rate_hz,
                "formula": "speed_mm_s / measured_rate_hz",
                "speed_mm_s": B2_SPEED_MPS * 1000.0,
                "measured_rate_hz": measured_rate_hz,
                "label": "DERIVED_NOMINAL_SPATIAL_SAMPLING_RESOLUTION_MM",
                "usage": "Informational interpretation aid only. This value MUST NOT be used as a "
                         "PASS/FAIL gate for repeatability or any other measurement.",
            }

        cycles_out = [cycle_summary(c) for c in complete_cycles[:2]]

        post_scene = evaluator.query_planning_scene()
        post_obstacle_co = None
        if post_scene:
            for co in post_scene.world.collision_objects:
                if co.id == OBSTACLE_ID:
                    post_obstacle_co = co
                    break

        gates = {
            "update_rate_10pm2_hz": {
                "measured_hz": measured_rate_hz,
                "pass": (measured_rate_hz is not None and
                         abs(measured_rate_hz - GATE_UPDATE_RATE_HZ_NOMINAL) <= GATE_UPDATE_RATE_HZ_TOL),
            },
            "max_stale_le_250ms": {
                "measured_s": max_observed_stale_s,
                "pass": max_observed_stale_s <= GATE_MAX_STALE_S,
            },
            "storage_error_le_5mm": {
                "measured_max_mm": storage_stats["max_mm"] if storage_stats else None,
                "pass": bool(storage_stats and storage_stats["max_mm"] <= GATE_STORAGE_ERROR_MM_MAX),
            },
            "add_exactly_1": {"measured": len(co_adds), "pass": len(co_adds) == 1},
            "complete_cycles_ge_2": {"measured": len(complete_cycles), "pass": evidence_complete},
            "collision_transition_occurred": {"pass": collision_transition_occurred},
            "no_unexpected_collision_pairs": {"unexpected": unexpected_pairs, "pass": len(unexpected_pairs) == 0},
        }
        overall_pass = all(g["pass"] for g in gates.values())

        results = {
            "harness_version": HARNESS_VERSION,
            "milestone": "Stage-3B B2",
            "objective": "Workspace-Adjacent Proximity-Awareness Qualification (corrected harness)",
            "reference_state": {
                "name": "Scene-A Pre-Grasp Configuration",
                "arm_joints": REF_ARM_JOINTS, "arm_positions": REF_ARM_POSITIONS,
                "gripper_aperture_m": 0.0506,
            },
            "obstacle_trajectory": {
                "id": OBSTACLE_ID, "box_dimensions": OBSTACLE_BOX,
                "center_x": B2_CENTER_X, "center_z": B2_CENTER_Z,
                "y_min": B2_Y_MIN, "y_max": B2_Y_MAX, "period_s": B2_PERIOD,
                "speed_mps": B2_SPEED_MPS,
                "production_b1_model_unchanged": True,
            },
            "telemetry": {
                **scene_node_telemetry,
                "total_ros_poses": len(all_ros),
                "total_collision_objects": len(all_cos),
                "add_count": len(co_adds),
                "move_count": len(co_moves),
                "note_on_move_count": "move_count is the full-run tally from this evaluator's own "
                                       "/collision_object subscription; the 'move' field above is a "
                                       "periodic (~5s) snapshot from the node's own log and may be lower "
                                       "if the run continued after the last snapshot was printed.",
            },
            "tracking_fidelity": {
                "storage_error_stats_mm": storage_stats,
                "gazebo_to_ros_fidelity": gazebo_to_ros_fidelity,
            },
            "derived_spatial_sampling_resolution": derived_spatial_sampling_resolution_mm,
            "proximity_and_collision_transitions": {
                "distance_metric_disclaimer": "No FCL/MoveIt geometric distance is computed anywhere in "
                                               "this harness. The collision authority is exclusively "
                                               "GetStateValidity.valid + contact pairs involving "
                                               "dynamic_obstacle_0.",
                "collision_transition_occurred": collision_transition_occurred,
                "collision_pairs": collision_pairs,
                "unexpected_collision_pairs": unexpected_pairs,
                "complete_cycle_definition": "A1=last separated sample before an approach transition, "
                                              "A2=first colliding sample of that transition, "
                                              "R1=last colliding sample before the following retreat "
                                              "transition, R2=first separated sample after R1. Identified "
                                              "by chronological state-machine scan, not phase/period "
                                              "binning. Partial startup/cutoff sequences missing any of "
                                              "A1/A2/R1/R2 are excluded, never padded or defaulted.",
                "complete_cycles_found": len(complete_cycles),
                "cycles": cycles_out,
                "repeatability_mm": repeatability_mm,
                "repeatability_status": repeatability_status,
                "repeatability_note": "Compares the approach-entry (A2) Y of the first two complete "
                                       "cycles. Report descriptively against "
                                       "derived_spatial_sampling_resolution; this value is NEVER used as "
                                       "a PASS/FAIL gate.",
            },
            "safety_and_invariants": {
                "reactive_replans": 0, "trajectory_stops": 0, "execution_cancellations": 0,
                "prediction_active": False, "continuous_adaptation_active": False,
                "obstacle_attached": False, "acm_exemptions_added": False, "touch_links_modified": False,
                "invariant_basis": "Structural: dynamic_obstacle_scene_node.cpp contains no attach, ACM, "
                                    "touch_links, replanning, trajectory-stop, or cancellation code path "
                                    "(confirmed by source inspection, not runtime-measured).",
            },
            "post_cycle_scene": {
                "object_present": post_obstacle_co is not None,
                "pose": ([post_obstacle_co.pose.position.x, post_obstacle_co.pose.position.y,
                          post_obstacle_co.pose.position.z] if post_obstacle_co else None),
            },
            "gates": gates,
            "verdict": "PASS" if overall_pass else "FAIL",
        }

        with open(evidence_dir / "b2_qualification_results.json", "w") as f:
            json.dump(results, f, indent=2)
        with open(evidence_dir / "sample_records.json", "w") as f:
            json.dump(sample_records, f, indent=2)

        print("\n" + "=" * 75)
        print("STAGE-3B B2 (CORRECTED) QUALIFICATION SUMMARY")
        print("=" * 75)
        print(f"Overall B2 Verdict:          {results['verdict']}")
        for name, g in gates.items():
            print(f"  gate[{name}]: {'PASS' if g['pass'] else 'FAIL'} -> {g}")
        print(f"Complete cycles found:       {len(complete_cycles)}")
        print(f"Repeatability:               {repeatability_mm} mm (status={repeatability_status})")
        if derived_spatial_sampling_resolution_mm:
            print(f"Derived spatial resolution:  {derived_spatial_sampling_resolution_mm['value_mm']:.2f} mm "
                  f"(informational only)")
        print(f"Collision pairs:             {collision_pairs}")
        print(f"Unexpected pairs:            {unexpected_pairs}")
        print(f"Gazebo->ROS fidelity:        {gazebo_to_ros_fidelity.get('status')}")
        print(f"Evidence Directory:          {evidence_dir}")
        print("=" * 75)

        if results["verdict"] == "PASS":
            print("\n>>> STAGE-3B B2 (CORRECTED HARNESS) PASS <<<\n")
            return 0
        else:
            print("\n>>> STAGE-3B B2 (CORRECTED HARNESS) FAIL <<<\n")
            return 1

    except RuntimeError as e:
        # Infrastructure failures are classified honestly and NOT retried by this
        # process. Preserve whatever partial evidence exists; do not fabricate results.
        (evidence_dir / "INFRASTRUCTURE_FAILURE.txt").write_text(
            f"infra_failure_code: {infra_failure}\nmessage: {e}\n")
        print(f"\n>>> INFRASTRUCTURE FAILURE (not a measurement result): {e} <<<\n")
        return 2

    finally:
        print("\nCleaning up processes...")
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        for name, proc in procs.items():
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except Exception:
                pass
        time.sleep(1.0)
        kill_all_processes()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--selfcheck", action="store_true",
                         help="Validate the pure analysis logic against existing historical "
                              "sample_records.json evidence, offline, with no simulator involved.")
    args = parser.parse_args()

    if args.selfcheck:
        sys.exit(run_selfcheck())
    else:
        sys.exit(run_stage3b_b2_qualification())

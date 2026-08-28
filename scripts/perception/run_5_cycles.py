#!/usr/bin/env python3
import csv
import json
import math
import os
import pathlib
import signal
import subprocess
import sys
import time

import yaml

project = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(project / "scripts/perception"))
import milestone_f1_harness as harness

N_A = "N/A"
EXPECTED_ALLOWED_START_TOLERANCE = 0.01
cycle_count = int(os.environ.get("REPEATABILITY_CYCLES", "5"))
if cycle_count < 1:
    raise ValueError("REPEATABILITY_CYCLES must be at least 1")

def run_cmd(cmd, timeout=30):
    proc = subprocess.Popen(
        cmd,
        shell=True,
        executable="/bin/bash",
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    try:
        stdout, stderr = proc.communicate(timeout=timeout)
        return stdout + "\n" + stderr, proc.returncode
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.communicate()
        return "TimeoutExpired", 1

def start_process(cmd, **kwargs):
    return subprocess.Popen(
        cmd,
        shell=True,
        executable="/bin/bash",
        start_new_session=True,
        **kwargs,
    )

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

def is_controller_active(out, name):
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[0] == name and parts[-1] == "active":
            return True
    return False

def csv_flag(csv_data, key):
    return csv_data.get(key) == "1"

def csv_float(csv_data, key):
    value = csv_data.get(key)
    if value in (None, "", "N/A"):
        return None
    try:
        parsed = float(value)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None

def stop_process(proc):
    # Each launcher owns a session/process group. Stop that exact group so
    # ros2 launch children cannot survive while unrelated ROS processes are
    # never touched.
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait(timeout=5)

def fail_preflight(message, *processes):
    print(f"PREFLIGHT_FAILURE: {message}. m3_grasp will not be launched.", flush=True)
    for process in processes:
        stop_process(process)
    sys.exit(1)

def wait_for_camera_topics():
    topics = (
        "/overhead_camera/image",
        "/overhead_camera/depth_image",
        "/overhead_camera/camera_info",
    )
    last_out = ""
    for _ in range(30):
        unavailable = []
        for topic in topics:
            out, rc = run_cmd(
                "source /opt/ros/jazzy/setup.bash && source install/setup.bash && "
                f"ros2 topic info {topic}",
                timeout=10,
            )
            if rc != 0 or "Publisher count: 0" in out:
                unavailable.append(topic)
                last_out = out
        if not unavailable:
            info_out, info_rc = run_cmd(
                "source /opt/ros/jazzy/setup.bash && source install/setup.bash && "
                "ros2 topic echo /overhead_camera/camera_info --once",
                timeout=5,
            )
            if info_rc == 0 and "frame_id:" in info_out:
                return True, ""
            last_out = info_out
        time.sleep(1)
    return False, last_out


def wait_for_perception_point(timeout_s=15.0):
    """Run the raw subscriber in a fresh, identically sourced interpreter."""
    probe_path = project / "scripts/perception/pointstamped_readiness_probe.py"
    output, returncode = run_cmd(
        "source /opt/ros/jazzy/setup.bash && source install/setup.bash && "
        f"python3 {probe_path} --timeout {timeout_s}",
        timeout=timeout_s + 5.0,
    )
    detail_lines = []
    sample = None
    for line in output.splitlines():
        if line.startswith("PROBE_SAMPLE_JSON="):
            sample = json.loads(line.split("=", 1)[1])
        elif line.strip():
            detail_lines.append(line.strip())
    detail = "; ".join(detail_lines)
    if returncode != 0 or sample is None:
        return None, f"returncode={returncode} {detail}"
    xyz = (sample["x"], sample["y"], sample["z"])
    stamp_s = sample["stamp_sec"] + sample["stamp_nanosec"] * 1e-9
    if (
        sample["frame_id"] != "world"
        or stamp_s <= 0.0
        or not all(math.isfinite(value) for value in xyz)
    ):
        return None, f"invalid PointStamped sample={sample} {detail}"
    # The probe uses VOLATILE durability and exits on its first sample, so the
    # positive stamp necessarily came from a live publication after startup.
    return {"stamp_s": stamp_s, "xyz": xyz}, detail

scene_path = project / "config/scene.yaml"
with open(scene_path, "r") as f:
    scene = yaml.safe_load(f)

place_cfg = scene["object"]["place_pose"]
target_xyz = [float(place_cfg["x"]), float(place_cfg["y"]), float(place_cfg["z"])]
target_rpy = [float(place_cfg["roll"]), float(place_cfg["pitch"]), float(place_cfg["yaw"])]
target_q = rpy_to_quaternion(*target_rpy)

results = []

# move_group.launch.py selects this file for gripper_model:=parallel_jaw.  A
# repeatability attempt is invalid if it silently differs from the validated
# execution configuration, so reject that state before starting Gazebo.
parallel_moveit_path = (
    project
    / "ur5e_robotiq_moveit_config/config/moveit_controllers_parallel_jaw.yaml"
)
with open(parallel_moveit_path, "r") as f:
    parallel_moveit = yaml.safe_load(f)
configured_start_tolerance = float(
    parallel_moveit["trajectory_execution"]["allowed_start_tolerance"]
)
if not math.isclose(
    configured_start_tolerance,
    EXPECTED_ALLOWED_START_TOLERANCE,
    rel_tol=0.0,
    abs_tol=1e-12,
):
    print(
        "CONFIG_MISMATCH: parallel-jaw MoveIt allowed_start_tolerance is "
        f"{configured_start_tolerance}, expected validated value "
        f"{EXPECTED_ALLOWED_START_TOLERANCE}. No cycle started.",
        flush=True,
    )
    sys.exit(2)

contamination = subprocess.run(
    [
        "pgrep",
        "-f",
        "m3_grasp|static_scene_tf|move_group|object_detector|"
        "object_position_world|[g]z sim|robot_state_publisher|ros2_control_node",
    ],
    capture_output=True,
    text=True,
    check=False,
)
if contamination.returncode == 0:
    print(
        "CONTAMINATED_ENVIRONMENT: existing ROS/Gazebo processes found; "
        "refusing to terminate unowned processes or start a cycle. PIDs: "
        f"{contamination.stdout.strip()}",
        flush=True,
    )
    sys.exit(2)

for cycle in range(1, cycle_count + 1):
    print(f"\n=======================================================", flush=True)
    print(f"       STARTING REPEATABILITY CYCLE {cycle} / {cycle_count}", flush=True)
    print(f"=======================================================", flush=True)

    # 1. Launch sim control
    print("Launching sim_control...", flush=True)
    sim_proc = start_process("source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch ur5e_robotiq_description ur5e_robotiq_sim_control.launch.py gripper_model:=parallel_jaw enable_camera:=true gazebo_gui:=false")

    # Wait for controllers
    active = False
    last_out = ""
    for _ in range(60):
        out, _ = run_cmd("source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 control list_controllers", timeout=10)
        last_out = out
        if is_controller_active(out, "arm_controller") and is_controller_active(out, "parallel_jaw_gripper_controller"):
            active = True
            break
        time.sleep(1)

    if not active:
        print(f"ERROR: Controllers failed to become active! Last list_controllers output:\n{last_out}", flush=True)
        stop_process(sim_proc)
        sys.exit(1)
    print("Controllers active.", flush=True)

    joint_state_out, joint_state_rc = run_cmd(
        "source /opt/ros/jazzy/setup.bash && source install/setup.bash && "
        "ros2 topic echo /joint_states --once",
        timeout=10,
    )
    arm_joint_names = scene["robot"]["arm_joints"]
    missing_joint_names = [name for name in arm_joint_names if name not in joint_state_out]
    if joint_state_rc != 0 or "position:" not in joint_state_out or missing_joint_names:
        fail_preflight(
            "/joint_states is unavailable or missing arm joint data "
            f"(missing={missing_joint_names}); output={joint_state_out}",
            sim_proc,
        )
    print("Joint states healthy.", flush=True)

    camera_ready, camera_detail = wait_for_camera_topics()
    if not camera_ready:
        fail_preflight(
            "/overhead_camera image, depth, and camera-info topics were not all "
            f"available with a camera-info sample; last output={camera_detail}",
            sim_proc,
        )
    print("Camera topics available.", flush=True)

    # 3. Launch move_group
    print("Launching move_group...", flush=True)
    mg_proc = start_process("source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch ur5e_robotiq_moveit_config move_group.launch.py gripper_model:=parallel_jaw")
    move_group_ready = False
    for _ in range(60):
        out, _ = run_cmd("source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 node list", timeout=10)
        if "/move_group" in out.splitlines():
            move_group_ready = True
            break
        if mg_proc.poll() is not None:
            break
        time.sleep(1)
    if not move_group_ready:
        print("ERROR: move_group failed to become ready; no cycle started.", flush=True)
        stop_process(mg_proc)
        stop_process(sim_proc)
        sys.exit(1)

    tolerance_out, tolerance_rc = run_cmd(
        "source /opt/ros/jazzy/setup.bash && source install/setup.bash && "
        "ros2 param get /move_group trajectory_execution.allowed_start_tolerance",
        timeout=10,
    )
    expected_tolerance_text = f"Double value is: {EXPECTED_ALLOWED_START_TOLERANCE}"
    if tolerance_rc != 0 or expected_tolerance_text not in tolerance_out:
        print(
            "CONFIG_MISMATCH: runtime move_group allowed_start_tolerance did not "
            f"equal {EXPECTED_ALLOWED_START_TOLERANCE}. Output:\n{tolerance_out}",
            flush=True,
        )
        stop_process(mg_proc)
        stop_process(sim_proc)
        sys.exit(2)

    # 4. Reset object
    print("Resetting Scene-A object...", flush=True)
    harness.remove_object()
    time.sleep(1.0)
    harness.spawn_object(0.45, -0.15)
    settled, msg = harness.settle_object()
    if not settled:
        print("ERROR: Object failed to settle!", flush=True)
        stop_process(mg_proc)
        stop_process(sim_proc)
        sys.exit(1)
    init_pose = harness.instantaneous_object_pose()
    print(f"Object reset and settled at: {init_pose}", flush=True)

    # 5. Launch perception nodes. m3_grasp.launch.py owns static_scene_tf;
    # launching another copy here duplicates frames and diverges from the
    # validated single-cycle launch sequence.
    print("Launching perception nodes...", flush=True)
    # These nodes log each camera frame.  Do not let that high-rate child
    # output fill the harness's stdout pipe and block the bounded readiness
    # probe or its result logging; rclcpp retains their per-node ROS logs.
    det_proc = start_process(
        "source /opt/ros/jazzy/setup.bash && source install/setup.bash && "
        "ros2 run ur5e_pick_place object_detector --ros-args -p use_sim_time:=true",
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )
    pos_proc = start_process(
        "source /opt/ros/jazzy/setup.bash && source install/setup.bash && "
        "ros2 run ur5e_pick_place object_position_world --ros-args -p use_sim_time:=true",
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )
    time.sleep(1)

    node_out, node_rc = run_cmd(
        "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 node list",
        timeout=10,
    )
    if (
        det_proc.poll() is not None
        or pos_proc.poll() is not None
        or node_rc != 0
        or "/object_detector" not in node_out.splitlines()
        or "/object_position_world" not in node_out.splitlines()
    ):
        fail_preflight(
            "object_detector or object_position_world is not alive "
            f"(detector_returncode={det_proc.poll()} "
            f"position_world_returncode={pos_proc.poll()} nodes={node_out})",
            pos_proc,
            det_proc,
            mg_proc,
            sim_proc,
        )
    print("Perception nodes alive.", flush=True)

    # Gate on a real message from the exact PointStamped topic m3_grasp
    # consumes, rather than relying on a separate ros2 CLI process.
    perception_sample, perception_detail = wait_for_perception_point()
    if perception_sample is None:
        det_status = det_proc.poll()
        pos_status = pos_proc.poll()
        print(
            "PERCEPTION_PREFLIGHT_FAILURE: no /object_detector/position_world "
            "PointStamped satisfying world/finite/current checks; m3_grasp will not be "
            "launched. "
            f"detector_returncode={det_status} position_world_returncode={pos_status} "
            f"detail={perception_detail}",
            flush=True,
        )
        results.append({
            "cycle": cycle,
            "result": "PERCEPTION_PREFLIGHT_FAILURE",
            "percept_err_mm": N_A,
            "percept_delta_mm": N_A,
            "pregrasp_attempted": False,
            "pregrasp_result": N_A,
            "descent_attempted": False,
            "cartesian_fraction": N_A,
            "stage2_tcp_err_mm": N_A,
            "grasp_attempted": False,
            "grasp_result": N_A,
            "lift_attempted": False,
            "lift_slip_mm": N_A,
            "transport_attempted": False,
            "transport_slip_mm": N_A,
            "place_release_attempted": False,
            "placement_evaluated": False,
            "dx_mm": N_A,
            "dy_mm": N_A,
            "dz_mm": N_A,
            "euc_err_mm": N_A,
            "orient_err_deg": N_A,
            "pass_fail": "FAIL",
        })
        stop_process(pos_proc)
        stop_process(det_proc)
        stop_process(mg_proc)
        stop_process(sim_proc)
        break
    print(
        "Perception world-position pipeline is ready: "
        f"stamp={perception_sample['stamp_s']:.9f} "
        f"xyz={perception_sample['xyz']} {perception_detail}",
        flush=True,
    )

    # 6. Launch m3_grasp
    csv_file = project / f"evidence/repeatability_run_{cycle}.csv"
    if csv_file.exists():
        csv_file.unlink()
    marker_prefix = project / f"evidence/repeatability_run_{cycle}_stage"

    cmd_m3 = f'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch ur5e_pick_place m3_grasp.launch.py gripper_model:=parallel_jaw use_perceived_position:=true require_perception:=true perceived_position_timeout_s:=15.0 pregrasp_joint_target:="[]" csv_path:="{csv_file}" marker_file_prefix:="{marker_prefix}"'
    print(f"Executing m3_grasp for cycle {cycle}...", flush=True)

    m3_proc = start_process(
        cmd_m3,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    m3_out = ""
    start_t = time.time()
    done = False

    while time.time() - start_t < 180.0:
        line = m3_proc.stdout.readline()
        if line:
            m3_out += line
            print(f"  [{cycle}] {line.strip()}", flush=True)
            if "RUN SUMMARY" in line:
                done = True
                time.sleep(3.0)
                break
        else:
            if m3_proc.poll() is not None:
                break
            time.sleep(0.1)

    print(f"Cycle {cycle} execution finished (done={done})", flush=True)

    # Parse CSV for evidence fields
    csv_data = {}
    if csv_file.exists():
        with open(csv_file, "r") as cf:
            rdr = csv.DictReader(cf)
            rows = list(rdr)
            if rows:
                csv_data = rows[0]

    # Extract details from log output
    sel_q = N_A
    d_descent = N_A
    d_transit = N_A
    rel_aperture = N_A
    rel_result = N_A

    for line in m3_out.splitlines():
        if "DETERMINISTIC_PREGRASP_SELECTED" in line:
            sel_q = line.split("q=")[1].split("]")[0] + "]"
            d_descent = line.split("D_descent=")[1].split(" ")[0]
            d_transit = line.split("D_transit=")[1].split(" ")[0]
        if "release:" in line and "achieved_aperture=" in line:
            rel_aperture = line.split("achieved_aperture=")[1].split(" ")[0]
            rel_result = line.split("action_result=")[1].split(" ")[0]

    m3_res_str = csv_data.get("result", "UNKNOWN")
    pregrasp_attempted = csv_flag(csv_data, "pregrasp_attempted")
    descent_attempted = csv_flag(csv_data, "descent_attempted")
    grasp_attempted = csv_flag(csv_data, "gripper_close_attempted")
    lift_attempted = csv_flag(csv_data, "lift_attempted")
    transport_attempted = csv_flag(csv_data, "transport_attempted")
    place_release_attempted = csv_flag(csv_data, "place_release_attempted")

    commanded_xyz = [csv_float(csv_data, key) for key in ("commanded_x", "commanded_y", "commanded_z")]
    if (
        csv_data.get("position_source") == "perceived"
        and all(value is not None for value in commanded_xyz)
        and init_pose
        and len(init_pose) == 7
    ):
        percept_delta = [
            (commanded_xyz[index] - init_pose[index]) * 1000.0
            for index in range(3)
        ]
        percept_err_mm = math.sqrt(sum(value * value for value in percept_delta))
    else:
        percept_delta = N_A
        percept_err_mm = N_A

    placement_evaluated = m3_res_str == "SUCCESS" and place_release_attempted
    dx_mm = dy_mm = dz_mm = euc_err_mm = orient_err_deg = N_A
    if placement_evaluated:
        final_pose = harness.instantaneous_object_pose()
        if not final_pose or len(final_pose) != 7:
            print("ERROR: could not get final ground-truth object pose!", flush=True)
            placement_evaluated = False
        else:
            actual_xyz = final_pose[:3]
            actual_q = final_pose[3:]
            dx = actual_xyz[0] - target_xyz[0]
            dy = actual_xyz[1] - target_xyz[1]
            dz = actual_xyz[2] - target_xyz[2]
            euc_err = math.sqrt(dx*dx + dy*dy + dz*dz)
            dx_mm = dx * 1000.0
            dy_mm = dy * 1000.0
            dz_mm = dz * 1000.0
            euc_err_mm = euc_err * 1000.0
            orient_err_deg = math.degrees(quaternion_angle_error(actual_q, target_q))

    pass_fail = (
        "PASS"
        if placement_evaluated and euc_err_mm < 10.0 and orient_err_deg < 5.0
        else "FAIL"
    )

    tcp_error = csv_float(csv_data, "tcp_error_m")
    achieved_q = csv_float(csv_data, "achieved_q")

    run_info = {
        "cycle": cycle,
        "result": m3_res_str,
        "percept_err_mm": percept_err_mm,
        "percept_delta_mm": percept_delta,
        "pregrasp_attempted": pregrasp_attempted,
        "selected_q": sel_q if pregrasp_attempted else N_A,
        "d_descent": d_descent if pregrasp_attempted else N_A,
        "d_transit": d_transit if pregrasp_attempted else N_A,
        "pregrasp_result": csv_data.get("pregrasp_succeeded", N_A) if pregrasp_attempted else N_A,
        "descent_attempted": descent_attempted,
        "cartesian_fraction": csv_data.get("cartesian_fraction", N_A) if descent_attempted else N_A,
        "stage2_tcp_err_mm": tcp_error * 1000.0 if descent_attempted and tcp_error is not None and tcp_error >= 0.0 else N_A,
        "grasp_attempted": grasp_attempted,
        "grasp_result": csv_data.get("gripper_result_kind", N_A) if grasp_attempted else N_A,
        "achieved_q": achieved_q if grasp_attempted and achieved_q is not None and achieved_q >= 0.0 else N_A,
        # This harness has no slip recorder. Never substitute historical or
        # nominal numbers, even when these stages execute.
        "lift_attempted": lift_attempted,
        "lift_slip_mm": N_A,
        "transport_attempted": transport_attempted,
        "transport_slip_mm": N_A,
        "place_release_attempted": place_release_attempted,
        "release_aperture_m": rel_aperture if place_release_attempted else N_A,
        "release_result": rel_result if place_release_attempted else N_A,
        "placement_evaluated": placement_evaluated,
        "dx_mm": dx_mm,
        "dy_mm": dy_mm,
        "dz_mm": dz_mm,
        "euc_err_mm": euc_err_mm,
        "orient_err_deg": orient_err_deg,
        "pass_fail": pass_fail
    }

    results.append(run_info)
    if placement_evaluated:
        print(
            f"CYCLE {cycle} RESULT: {pass_fail} | Placement Error: "
            f"{euc_err_mm:.3f} mm | Orient Error: {orient_err_deg:.3f} deg",
            flush=True,
        )
    else:
        print(
            f"CYCLE {cycle} RESULT: {pass_fail} | Placement Error: N/A | "
            "Orient Error: N/A",
            flush=True,
        )

    # Save log of run
    with open(project / f"evidence/repeatability_run_{cycle}_log.txt", "w") as lf:
        lf.write(m3_out)

    # Cleanup processes for next run AFTER reading pose
    stop_process(m3_proc)
    stop_process(pos_proc)
    stop_process(det_proc)
    stop_process(mg_proc)
    stop_process(sim_proc)

    if pass_fail == "FAIL":
        print(f"CRITICAL FAILURE on cycle {cycle}! Stopping repeatability testing immediately.", flush=True)
        break

print("\n=======================================================", flush=True)
print(f"             {cycle_count}-RUN REPEATABILITY SUMMARY", flush=True)
print("=======================================================", flush=True)
print(json.dumps(results, indent=2), flush=True)

with open(project / "evidence/repeatability_5_summary.json", "w") as sf:
    json.dump(results, sf, indent=2)

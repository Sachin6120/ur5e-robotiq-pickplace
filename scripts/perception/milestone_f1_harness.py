#!/usr/bin/env python3
#
# ===========================================================================
# EVALUATION ONLY -- NEVER A PERCEPTION INPUT
# ===========================================================================
# 1. Evaluation harness for Milestone F1. It produces no part of the sensor
#    estimate and no part of the motion target. Perception lives in
#    object_detector.cpp / object_position_world.cpp; the target is composed
#    inside m3_grasp.cpp from the perceived point and the configured geometry.
# 2. Gazebo truth is queried only by milestone_f1_truth.py, a separate
#    process, and only after this script has frozen the run to disk.
# 3. Nothing computed here may be fed back into perception or planning.
# 4. Used to validate MILESTONE F1 (perception-derived pre-grasp).
#
# PRECONDITION: the simulation stack is already up (sim + move_group, and for
# perceived runs the camera, object_detector and object_position_world).
# ===========================================================================
"""milestone_f1_harness.py -- spawn, run one m3_grasp cycle, freeze evidence."""
import argparse, importlib.util, json, os, re, shutil, subprocess, sys, time
from pathlib import Path

import yaml

REPO = str(Path(__file__).resolve().parents[2])
WORLD = "empty"
SCENE = yaml.safe_load(open(f"{REPO}/config/scene.yaml"))
OBJ = SCENE["object"]
OBJ_NAME = OBJ["name"]
OBJ_SIZE = OBJ["size"]
OBJ_Z = OBJ["pick_pose"]["z"]
GRIPPER_ACTION = "/gripper_controller/gripper_cmd"
GRIPPER_MASTER_JOINT = SCENE["gripper"]["actuated_joint"]
GRIPPER_OPEN_POSITION = 0.0  # Canonical lower joint bound; same command as sim launch.
GRIPPER_MAX_EFFORT = float(SCENE["gripper"]["max_effort"])
GRIPPER_GOAL_TOLERANCE = 0.01  # Existing gripper_controller goal_tolerance.
GZ_JOINT_STATE_TOPIC = f"/world/{WORLD}/model/ur5e_robotiq/joint_state"
GZ_POSE_TOPIC = f"/world/{WORLD}/pose/info"
CONTACT_TOPICS = {
    "left": f"/world/{WORLD}/model/ur5e_robotiq/link/robotiq_85_left_knuckle_link/"
            "sensor/left_finger_tip_contact/contact",
    "right": f"/world/{WORLD}/model/ur5e_robotiq/link/robotiq_85_right_knuckle_link/"
             "sensor/right_finger_tip_contact/contact",
}
_pose_spec = importlib.util.spec_from_file_location(
    "sample_pose", f"{REPO}/scripts/lib/sample_pose.py")
_sample_pose = importlib.util.module_from_spec(_pose_spec)
_pose_spec.loader.exec_module(_sample_pose)


def gz(args, timeout=30):
    return subprocess.run(["gz"] + args, capture_output=True, text=True, timeout=timeout)


def spawn_object(x, y):
    sx, sy, sz = OBJ_SIZE
    sdf = (
        f"<?xml version='1.0'?><sdf version='1.9'><model name='{OBJ_NAME}'>"
        f"<pose>{x} {y} {OBJ_Z} 0.0 0.0 0.0</pose>"
        f"<link name='link'><inertial><mass>{OBJ['mass']}</mass>"
        f"<inertia><ixx>1e-4</ixx><iyy>1e-4</iyy><izz>1e-4</izz>"
        f"<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
        f"<collision name='c'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
        f"<surface><friction><ode><mu>{OBJ['surface']['mu']}</mu>"
        f"<mu2>{OBJ['surface']['mu2']}</mu2></ode></friction></surface></collision>"
        f"<visual name='v'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
        f"<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>"
        f"<specular>1 1 1 1</specular></material></visual>"
        f"</link></model></sdf>"
    )
    req = 'sdf: "%s", name: "%s"' % (sdf.replace('"', '\\"'), OBJ_NAME)
    r = gz(["service", "-s", f"/world/{WORLD}/create", "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", req])
    return (r.stdout + r.stderr).strip()


def remove_object():
    r = gz(["service", "-s", f"/world/{WORLD}/remove", "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean", "--timeout", "5000",
            "--req", f'name: "{OBJ_NAME}", type: MODEL'])
    return (r.stdout + r.stderr).strip()


def gazebo_master_joint_position():
    """Read the physics-side master position, never ROS controller feedback."""
    r = gz(["topic", "-e", "-t", GZ_JOINT_STATE_TOPIC, "-n", "1"])
    if r.returncode != 0:
        return None
    blocks = re.split(r"\njoint \{", "\n" + r.stdout)
    for block in blocks:
        if re.search(rf'name:\s*"{re.escape(GRIPPER_MASTER_JOINT)}"', block):
            # gz.msgs.Model reports revolute state as axis1.position (not a
            # scalar position{} field like gz.msgs.Joint). Restrict the match
            # to axis1 so link-pose position blocks cannot be mistaken for the
            # joint coordinate.
            m = re.search(r"axis1\s*\{.*?\bposition:\s*([-+0-9.eE]+)", block, re.S)
            return float(m.group(1)) if m else None
    return None


def initialize_trial_gripper():
    """Canonical open command + action/controller/Gazebo verification.

    This is an evaluation boundary, not production grasp behavior. The 30 s
    process bound only prevents a dead diagnostic from hanging forever; the
    action itself retains its existing controller result semantics.
    """
    evidence = {"open_target_rad": GRIPPER_OPEN_POSITION,
                "max_effort": GRIPPER_MAX_EFFORT,
                "goal_tolerance_rad": GRIPPER_GOAL_TOLERANCE,
                "action_server": GRIPPER_ACTION}
    print("TRIAL_INIT_BEGIN")
    start = gazebo_master_joint_position()
    evidence["starting_joint_position_rad"] = start
    print(f"TRIAL_INIT_START_JOINT_POSITION={start}")
    if start is None:
        evidence.update(result="FAIL", failure="GAZEBO_START_POSITION_UNAVAILABLE")
        print("TRIAL_INIT_FAIL=GAZEBO_START_POSITION_UNAVAILABLE")
        return False, evidence

    try:
        controllers = subprocess.run(
            ["ros2", "control", "list_controllers"], capture_output=True, text=True,
            timeout=10)
        actions = subprocess.run(
            ["ros2", "action", "list"], capture_output=True, text=True, timeout=10)
    except subprocess.TimeoutExpired:
        evidence.update(result="FAIL", failure="CONTROLLER_DISCOVERY_TIMEOUT")
        print("TRIAL_INIT_FAIL=CONTROLLER_DISCOVERY_TIMEOUT")
        return False, evidence
    controller_ok = (controllers.returncode == 0 and
                     re.search(r"^gripper_controller\s+.*\bactive\b", controllers.stdout, re.M)
                     is not None)
    action_ok = actions.returncode == 0 and GRIPPER_ACTION in actions.stdout.splitlines()
    evidence["controller_active"] = controller_ok
    evidence["action_available"] = action_ok
    if not controller_ok or not action_ok:
        evidence.update(result="FAIL", failure="CONTROLLER_OR_ACTION_UNAVAILABLE")
        print(f"TRIAL_INIT_CONTROLLER_OK={controller_ok and action_ok}")
        print("TRIAL_INIT_FAIL=CONTROLLER_OR_ACTION_UNAVAILABLE")
        return False, evidence

    goal = ("{command: {position: %.1f, max_effort: %.1f}}" %
            (GRIPPER_OPEN_POSITION, GRIPPER_MAX_EFFORT))
    t0 = time.monotonic()
    try:
        action = subprocess.run(
            ["ros2", "action", "send_goal", GRIPPER_ACTION,
             "control_msgs/action/GripperCommand", goal],
            capture_output=True, text=True, timeout=30)
    except subprocess.TimeoutExpired as exc:
        evidence.update(result="FAIL", failure="OPEN_ACTION_TIMEOUT",
                        action_elapsed_s=time.monotonic() - t0,
                        action_output=(exc.stdout or "") + (exc.stderr or ""))
        print("TRIAL_INIT_OPEN_RESULT=TIMEOUT")
        print("TRIAL_INIT_FAIL=OPEN_ACTION_TIMEOUT")
        return False, evidence
    action_output = action.stdout + action.stderr
    elapsed = time.monotonic() - t0
    reached = re.search(r"reached_goal:\s*true", action_output) is not None
    stalled = re.search(r"stalled:\s*true", action_output) is not None
    evidence.update(action_returncode=action.returncode, action_elapsed_s=elapsed,
                    action_reached_goal=reached, action_stalled=stalled,
                    action_output=action_output.strip())
    print(f"TRIAL_INIT_OPEN_RESULT={'REACHED_GOAL' if reached and not stalled else 'FAIL'}")
    if action.returncode != 0 or not reached or stalled:
        evidence.update(result="FAIL", failure="OPEN_ACTION_NOT_REACHED")
        print("TRIAL_INIT_FAIL=OPEN_ACTION_NOT_REACHED")
        return False, evidence

    final = gazebo_master_joint_position()
    evidence["initialized_joint_position_rad"] = final
    position_ok = final is not None and abs(final - GRIPPER_OPEN_POSITION) <= GRIPPER_GOAL_TOLERANCE
    evidence["position_verified"] = position_ok
    evidence["controller_responsive"] = controller_ok and action_ok and reached and not stalled
    print(f"TRIAL_INIT_JOINT_POSITION={final}")
    print(f"TRIAL_INIT_CONTROLLER_OK={evidence['controller_responsive']}")
    if not position_ok:
        evidence.update(result="FAIL", failure="OPEN_POSITION_VERIFY_FAILURE")
        print("TRIAL_INIT_FAIL=OPEN_POSITION_VERIFY_FAILURE")
        return False, evidence
    evidence["result"] = "PASS"
    print("TRIAL_INIT_PASS")
    return True, evidence


def settle_object(timeout=20.0):
    r = subprocess.run(
        f"python3 {REPO}/scripts/lib/gz_settle.py pose --topic /world/{WORLD}/pose/info "
        f"--eps 0.0005 --timeout {timeout} --poll 0.15 {OBJ_NAME}",
        shell=True, capture_output=True, text=True, timeout=timeout + 15)
    return r.returncode == 0, (r.stdout + r.stderr).strip()


def object_pose(timeout=25.0):
    """Settled object pose. Used for the displacement check (before/after) --
    a physical measurement of whether the robot disturbed the object, not an
    input to any target."""
    r = subprocess.run(
        f"python3 {REPO}/scripts/lib/sample_pose.py --topic /world/{WORLD}/pose/info "
        f"--entities {OBJ_NAME} --window-s 1.0 --tol-m 0.0005 --timeout-s {timeout}",
        shell=True, capture_output=True, text=True, timeout=timeout + 15)
    if r.returncode != 0:
        return None
    for line in r.stdout.splitlines():
        p = line.split()
        if p and p[0].endswith(OBJ_NAME):
            return [float(v) for v in p[1:8]]
    return None


def instantaneous_object_pose():
    """One Gazebo pose message for phase ordering; never used as a target."""
    r = gz(["topic", "-e", "-t", GZ_POSE_TOPIC, "-n", "1"])
    if r.returncode != 0:
        return None
    poses = _sample_pose.parse_pose_v(r.stdout.splitlines())
    return list(poses[OBJ_NAME]) if OBJ_NAME in poses else None


def run_m3(flags, csv_path, marker_prefix, log_path, timeout=240.0):
    cmd = ["ros2", "launch", "ur5e_pick_place", "m3_grasp.launch.py",
           f"csv_path:={csv_path}", f"marker_file_prefix:={marker_prefix}"] + flags
    env = dict(os.environ, PYTHONUNBUFFERED="1")
    phase_poses = {}
    with open(log_path, "w") as log:
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, env=env)
        ready = marker_prefix + ".run_summary_ready"
        end = time.time() + timeout
        while time.time() < end:
            log.flush()
            text = open(log_path, errors="replace").read()
            if "after_pregrasp" not in phase_poses and "F2 TARGETS FROZEN" in text:
                after_targets = text.split("F2 TARGETS FROZEN", 1)[1]
                if "Execute request success!" in after_targets:
                    phase_poses["after_pregrasp"] = instantaneous_object_pose()
            if "after_descent_before_close" not in phase_poses and \
                    "execution reported SUCCESS" in text:
                phase_poses["after_descent_before_close"] = instantaneous_object_pose()
            if os.path.exists(ready):
                time.sleep(2.0)
                break
            if proc.poll() is not None:
                break
            time.sleep(0.5)
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                proc.kill()
    subprocess.run(["pkill", "-9", "-f", "m3_grasp.launch.py"], capture_output=True)
    time.sleep(2.0)
    return open(log_path, errors="replace").read(), phase_poses


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", required=True)
    ap.add_argument("--x", type=float)
    ap.add_argument("--y", type=float)
    ap.add_argument("--absent", action="store_true")
    ap.add_argument("--init-only", action="store_true")
    ap.add_argument("--flags", nargs="*", default=[])
    ap.add_argument("--out", default="/tmp/ur5e_pickplace/f1_results")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    base = f"{a.out}/{a.scene}"
    os.makedirs(base + "_markers", exist_ok=True)

    log = {"scene": a.scene, "absent": a.absent, "requested_xy": [a.x, a.y],
           "flags": a.flags}
    print(f"\n{'='*66}\nF1 SCENE {a.scene}  absent={a.absent}  xy=({a.x},{a.y})\n"
          f"flags: {' '.join(a.flags)}\n{'='*66}")

    print("[setup] removing any existing object"); remove_object(); time.sleep(1.0)
    init_ok, init_evidence = initialize_trial_gripper()
    log["trial_initialization"] = init_evidence
    if not init_ok:
        log["frozen_at_walltime"] = time.time()
        json.dump(log, open(base + "_sensor.json", "w"), indent=2)
        print("[STOP] trial initialization failed; M1 and all arm motion are forbidden")
        return 2
    if a.init_only:
        log["init_only"] = True
        log["frozen_at_walltime"] = time.time()
        json.dump(log, open(base + "_sensor.json", "w"), indent=2)
        print("[STOP] initialization-only validation complete; m3_grasp was not launched")
        return 0
    if not a.absent:
        print(f"[setup] spawning {OBJ_NAME} at ({a.x}, {a.y}, {OBJ_Z})")
        print("        " + spawn_object(a.x, a.y))
        ok, msg = settle_object()
        print(f"[setup] settle ok={ok} {msg}")
        log["settled"] = ok
        if not ok:
            print("[STOP] object did not settle")
            json.dump(log, open(base + "_sensor.json", "w"), indent=2)
            return 2
        before = object_pose()
        log["object_pose_before"] = before
        print(f"[setup] object pose BEFORE = {before[:3] if before else None}")

    csv_path = base + "_m3.csv"
    log_path = base + "_m3.log"
    print(f"[run] launching m3_grasp ...")
    contact_procs = {}
    contact_files = {}
    for side, topic in CONTACT_TOPICS.items():
        path = f"{base}_contact_{side}.log"
        fh = open(path, "w")
        contact_files[side] = (path, fh)
        contact_procs[side] = subprocess.Popen(
            ["gz", "topic", "-e", "-t", topic], stdout=fh,
            stderr=subprocess.STDOUT, text=True)
    out, phase_poses = run_m3(
        a.flags, csv_path, base + "_markers/stage", log_path)
    for proc in contact_procs.values():
        proc.terminate()
    for proc in contact_procs.values():
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
    for path, fh in contact_files.values():
        fh.close()
    log["object_pose_after_pregrasp"] = phase_poses.get("after_pregrasp")
    log["object_pose_after_descent_before_close"] = phase_poses.get(
        "after_descent_before_close")
    log["fingertip_contact"] = {}
    for side, (path, _) in contact_files.items():
        contact_text = open(path, errors="replace").read()
        log["fingertip_contact"][side] = {
            "topic": CONTACT_TOPICS[side],
            "pick_target_contact": "pick_target::" in contact_text,
            "messages_with_pick_target": contact_text.count('name: "pick_target::'),
        }

    # --- pull the structured evidence out of the run ------------------------
    for key, pat in [
        ("run_summary", r"RUN SUMMARY.*"),
        ("perception_used", r"PERCEPTION_POSITION_USED:.*"),
        ("perception_timeout", r"PERCEPTION_TIMEOUT:.*"),
        ("perception_fallback", r"PERCEPTION_FALLBACK:.*"),
        ("m1_stationary", r"F1: M1_STATIONARY.*"),
        ("pregrasp_target", r"pre-grasp tool0 target \(world\):.*"),
        ("pregrasp_verify", r"F1 pre-grasp verification:.*"),
        ("f1_stop", r"F1 STOP:.*"),
    ]:
        m = re.findall(pat, out)
        log[key] = m[-1].strip() if m else None

    # forbidden-action probes: these strings must be ABSENT for a valid F1 run
    log["evidence_gripper_command"] = len(re.findall(r"pre-close:|gripper_close_and_hold|GRIPPER MODE", out))
    log["evidence_descent"] = len(re.findall(r"executing Cartesian descent", out))
    log["evidence_transport"] = len(re.findall(r"attempted_transport|TRANSPORT", out))
    log["stage_markers"] = sorted(os.listdir(base + "_markers"))

    if os.path.exists(csv_path):
        rows = open(csv_path).read().strip().splitlines()
        log["csv_header"] = rows[0] if rows else None
        log["csv_row"] = rows[1] if len(rows) > 1 else None
        if len(rows) > 1:
            log["csv"] = dict(zip(rows[0].split(","), rows[1].split(",")))
    else:
        log["csv_header"] = log["csv_row"] = None

    if not a.absent:
        after = object_pose()
        log["object_pose_after"] = after
        if log.get("object_pose_before") and after:
            b = log["object_pose_before"]
            d = [after[i] - b[i] for i in range(3)]
            log["object_displacement_xyz_m"] = d
            log["object_displacement_m"] = sum(v * v for v in d) ** 0.5
            print(f"[check] object displacement = "
                  f"{log['object_displacement_m']*1000:.4f} mm")

    log["frozen_at_walltime"] = time.time()
    json.dump(log, open(base + "_sensor.json", "w"), indent=2)
    print(f"\n>>> F1 SENSOR EVIDENCE FROZEN to {base}_sensor.json "
          f"at wall {log['frozen_at_walltime']:.3f}")
    print(">>> ONLY NOW does any ground-truth query occur.\n")
    for k in ("run_summary", "m1_stationary", "perception_used", "perception_timeout",
              "perception_fallback", "pregrasp_target", "pregrasp_verify", "f1_stop"):
        if log.get(k):
            print(f"  {k}: {log[k]}")
    print(f"  csv: {log.get('csv_row')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

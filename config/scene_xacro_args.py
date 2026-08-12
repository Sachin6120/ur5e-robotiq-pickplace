# scene_xacro_args.py — single source for deriving xacro arguments from
# scene.yaml that place the robot in the world.
#
# WHY THIS EXISTS
#   Before this file, three launch files each independently turned
#   robot.base_pose into "x y z" / "r p y" strings for the xacro's
#   base_xyz/base_rpy args: ur5e_robotiq_sim_control.launch.py (Gazebo
#   bringup), move_group.launch.py (MoveIt's own robot model), and
#   m2_cartesian_approach.launch.py (the M2 node's local robot model).
#   Harmless while all three agreed — but nothing enforced that they would
#   keep agreeing. Found during M2: elevating the sim's base to table height
#   changed nothing about reachability until move_group.launch.py was ALSO
#   updated, because MoveIt was silently still planning against a
#   ground-mounted robot model. That is exactly the sim/planner divergence
#   config/scene.yaml exists to prevent, and it had a gap: scene.yaml
#   governed poses, not the xacro args that place the robot itself.
#
# USAGE
#   Loaded via importlib.util.spec_from_file_location (not a normal package
#   import — this file lives next to scene.yaml in config/, not inside any
#   one ROS package, since all three consuming launch files are in
#   different packages). See any of the three launch files above for the
#   load snippet.

def xacro_base_args(scene: dict) -> dict:
    """Return {'base_xyz': '...', 'base_rpy': '...'} xacro mapping strings
    derived from a loaded scene.yaml dict's robot.base_pose. Every launch
    file that constructs this project's robot_description must derive these
    two args from here, not recompute them independently."""
    base = scene["robot"]["base_pose"]
    return {
        "base_xyz": f"{base['x']} {base['y']} {base['z']}",
        "base_rpy": f"{base['roll']} {base['pitch']} {base['yaw']}",
    }


def _load_gripper_geometry_module():
    # scripts/lib/gripper_geometry.py, loaded the same way this file itself
    # is loaded by its callers (spec_from_file_location, not a package
    # import) — it lives in a different package's scripts/lib/, not next to
    # scene.yaml, and has no __init__.py chain to this file either.
    import importlib.util
    import os

    module_path = os.path.expanduser("~/ur5e_pickplace/scripts/lib/gripper_geometry.py")
    spec = importlib.util.spec_from_file_location("gripper_geometry", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def xacro_gripper_args(scene: dict) -> dict:
    """Return {'fingertip_grasp_theta': '...'} — the actuated joint angle
    (radians, as gripper_geometry.theta_for_width computes it) at which the
    pads just touch the current scene object.

    WHY THIS EXISTS
        2026-08-12: both fingertip joints (left_finger_tip_joint,
        right_finger_tip_joint) changed from continuously mimic-tracking the
        knuckle to FIXED, at an angle chosen so the pad is exactly parallel
        (net tilt zero) AT THIS ANGLE — see robotiq_2f_85_macro.urdf.xacro's
        OVERRIDE comment for the full reasoning (JointCmdProbe traced the
        fingertip runaway to dartsim's contact-force resolution overriding a
        correctly-computed mimic command under load; a fixed joint has no DOF
        for that to act on). That angle is therefore OBJECT-SIZE-SPECIFIC: it
        must be re-derived whenever object.size changes, never hand-typed
        into the URDF. This function is the single source for that
        derivation, exactly as xacro_base_args() is for base_xyz/base_rpy.
    """
    obj = scene["object"]
    width_m = obj["size"][obj["grasp_width_axis"]]
    gripper_geometry = _load_gripper_geometry_module()
    theta_grasp = gripper_geometry.theta_for_width(width_m)
    return {"fingertip_grasp_theta": repr(theta_grasp)}

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

import importlib.util
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


# install/.../launch is a symlink to this source file in the active colcon
# workspace, so resolving this file yields the project root independently of
# HOME.  Gazebo still uses HOME for its writable runtime state; project config
# must not.
PROJECT_ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = PROJECT_ROOT / "config"


def _load_scene_xacro_args_module():
    # config/scene_xacro_args.py is the single source for turning
    # scene.yaml's robot.base_pose into xacro mapping strings — shared by
    # this file, ur5e_robotiq_sim_control.launch.py, and
    # m2_cartesian_approach.launch.py. It's loaded by path (not a normal
    # package import) because it lives next to scene.yaml, not inside any
    # one ROS package that all three could import from.
    path = CONFIG_DIR / "scene_xacro_args.py"
    spec = importlib.util.spec_from_file_location("scene_xacro_args", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _setup(context, *args, **kwargs):
    gripper_model = LaunchConfiguration("gripper_model").perform(context)

    # CORRECTED (M2): the .setup_assistant-recorded xacro_args are
    # "name:=ur5e_robotiq sim_ignition:=true" — no base_xyz/base_rpy. Without
    # passing them here too, MoveItConfigsBuilder regenerates robot_description
    # from the xacro's own defaults ("0 0 0"), independent of whatever base
    # pose the sim was actually spawned with. That mismatch is invisible until
    # something tries to plan: move_group's own kinematic model silently
    # disagrees with Gazebo's about where base_link is, so IK/reachability
    # results are computed against a robot that isn't the one running. Found
    # via compute_ik: elevating the sim's base to table height (scene.yaml
    # robot.base_pose) changed nothing until this file also passed the same
    # base_xyz/base_rpy into its own robot_description mapping.
    scene_file = CONFIG_DIR / "scene.yaml"
    with open(scene_file, "r") as fh:
        scene = yaml.safe_load(fh)
    scene_xacro_args = _load_scene_xacro_args_module()
    base_args = scene_xacro_args.xacro_base_args(scene)
    # fingertip_grasp_theta (see robotiq_2f_85_macro.urdf.xacro's TENTH
    # OVERRIDE): needed here for the same reason base_xyz/base_rpy are --
    # without it, move_group's own robot model regenerates from the xacro's
    # default fingertip angle instead of the one the sim actually spawned
    # with, and a planning scene collision check against a differently-tilted
    # pad is checking against a gripper that isn't the one running.
    gripper_args = scene_xacro_args.xacro_gripper_args(scene)

    controllers_file = (
        "config/moveit_controllers_parallel_jaw.yaml"
        if gripper_model == "parallel_jaw"
        else "config/moveit_controllers.yaml"
    )

    moveit_config = (
        MoveItConfigsBuilder("ur5e_robotiq", package_name="ur5e_robotiq_moveit_config")
        .robot_description(
            mappings={**base_args, **gripper_args, "gripper_model": gripper_model}
        )
        .robot_description_semantic(
            file_path="config/ur5e_robotiq.srdf.xacro",
            mappings={"gripper_model": gripper_model},
        )
        .trajectory_execution(file_path=controllers_file)
        .to_moveit_configs()
    )
    move_group_ld = generate_move_group_launch(moveit_config)

    # CORRECTED (2026-08-04): generate_move_group_launch() does not set
    # use_sim_time and exposes no launch arg for it, so move_group defaulted
    # to wall-clock time while /clock (and everything downstream of the
    # controller_manager) runs on sim time. That mismatch doesn't show up as a
    # missing-joint problem — it breaks trajectory_execution_manager's
    # timestamp-freshness check on /joint_states: every message looks
    # impossibly stale when compared against wall-clock "now" versus a sim
    # clock only ~600s past epoch. Symptom was 20/20 plans succeeding, then
    # every execute() aborting with "couldn't receive full current joint
    # state within 1s" — confirmed live via `ros2 param get /move_group
    # use_sim_time` returning False while /clock was already running.
    #
    # SetParameter must be placed BEFORE the move_group node action in
    # execution order, not appended after — launch actions apply to nodes
    # launched after them, and generate_move_group_launch() already has the
    # node action built into its returned LaunchDescription. Prepending via
    # .entities, rather than adding after, is what makes it actually take
    # effect on this specific node.
    return [
        SetParameter(name="use_sim_time", value=True),
        *move_group_ld.entities,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "gripper_model",
            default_value="robotiq_linkage",
            description="robotiq_linkage (default) or parallel_jaw (docs/GRIPPER_REDESIGN_DESIGN.md).",
        ),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=_setup)])

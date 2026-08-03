"""m1_joint_goal.launch.py — run the M1 planning-reliability trial.

PRECONDITION
    The merged sim and move_group must already be running in another terminal.
    This launch starts ONLY the M1 node; it does not bring up Gazebo, so a
    failure here can never be a half-started stack masquerading as a planning
    problem.

USAGE
    ros2 launch ur5e_pick_place m1_joint_goal.launch.py

WHY THE YAML IS READ HERE AND NOT IN C++
    config/scene.yaml is the single source of truth. Reading it in Python and
    passing values as ROS parameters keeps that true without dragging a yaml-cpp
    dependency into the node — and it means the node cannot silently fall back
    to a compiled-in default, because it has none.
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _load_scene(path):
    """Read scene.yaml. Fail loudly and immediately if it is missing."""
    if not os.path.isfile(path):
        raise RuntimeError(
            f"CONFIG_ERROR: scene.yaml not found at {path}. "
            "Pass scene_file:=/abs/path/to/scene.yaml"
        )
    with open(path, "r") as fh:
        return yaml.safe_load(fh)


def _setup(context, *args, **kwargs):
    scene_file = LaunchConfiguration("scene_file").perform(context)
    csv_path = LaunchConfiguration("csv_path").perform(context)
    trials = int(LaunchConfiguration("trials").perform(context))

    scene = _load_scene(scene_file)

    # ------------------------------------------------------------------
    # Pull values out of scene.yaml. Each lookup is explicit so a missing
    # key raises here, at launch, naming the key — rather than the node
    # planning to a default that was never written down.
    # ------------------------------------------------------------------
    try:
        arm_joints = scene["robot"]["arm_joints"]
        goal_positions = scene["milestones"]["m1"]["goal_positions"]
        thresholds = scene["thresholds"]
        grasp_mode = scene["grasp_mode"]["mode"]
    except KeyError as exc:
        raise RuntimeError(
            f"CONFIG_ERROR: missing key {exc} in {scene_file}. "
            "See docs/M-1_reference_report.md for the expected structure."
        ) from exc

    if len(arm_joints) != len(goal_positions):
        raise RuntimeError(
            f"CONFIG_ERROR: robot.arm_joints has {len(arm_joints)} entries but "
            f"milestones.m1.goal_positions has {len(goal_positions)}. "
            "These must correspond one-to-one."
        )

    moveit_config = (
        MoveItConfigsBuilder(
            "ur5e_robotiq", package_name="ur5e_robotiq_moveit_config"
        )
        .to_moveit_configs()
    )

    m1_node = Node(
        package="ur5e_pick_place",
        executable="m1_joint_goal",
        name="m1_joint_goal",
        output="screen",
        emulate_tty=True,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {
                "use_sim_time": True,
                "arm_joints": arm_joints,
                "goal_positions": [float(v) for v in goal_positions],
                "trials": trials,
                "planning_time_s": float(thresholds["planning_time_s"]),
                "plan_attempts": int(thresholds["plan_attempts"]),
                "velocity_scaling": 0.1,
                "acceleration_scaling": 0.1,
                "csv_path": csv_path,
                "grasp_mode": grasp_mode,
            },
        ],
    )
    return [m1_node]


def generate_launch_description():
    # scene.yaml is the project repo's own config file, not part of any ROS
    # package's installed share — get_package_share_directory would point at
    # ur5e_robotiq_description's install tree, which only carries
    # controllers.yaml. The real file lives at <repo>/config/scene.yaml, and
    # every other script in this project (00_recon.sh, m0_verify.sh, ...)
    # already treats it as repo-relative. expanduser keeps that convention
    # without hardcoding a username.
    default_scene = os.path.expanduser("~/ur5e_pickplace/config/scene.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scene_file",
                default_value=default_scene,
                description="Absolute path to scene.yaml, the single source of truth.",
            ),
            DeclareLaunchArgument(
                "csv_path",
                default_value="m1_planning.csv",
                description="Where to write the per-trial CSV evidence.",
            ),
            DeclareLaunchArgument(
                "trials",
                default_value="20",
                description="Number of planning attempts. M1 requires 20.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )

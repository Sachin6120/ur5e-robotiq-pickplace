"""d3_demo.launch.py — Single-command visual demonstration launch for Stage-2D D3.

ros2 launch ur5e_pick_place d3_demo.launch.py

Orchestration-only entry point:
- RViz2 with MoveIt MotionPlanning plugin using existing moveit.rviz configuration
- Authoritative Stage-2D D3 runner (run_stage2d_pose_case.py) managing:
  * Gazebo GUI (ur5e_robotiq_sim_control.launch.py)
  * Controllers (arm_controller, parallel_jaw_gripper_controller, joint_state_broadcaster)
  * MoveIt move_group (move_group.launch.py)
  * D3 object spawning at (+30 mm, -30 mm, yaw +45 deg)
  * Production RGB-D perception (object_detector, object_position_world)
  * M3 pick-and-place cycle with PlanningSceneManager collision lifecycle
  * Clean teardown and post-hoc gate verification
"""

import importlib.util
from pathlib import Path
import sys
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _resolve_project_root():
    resolved = Path(__file__).resolve()
    for parent in [resolved.parent, *resolved.parents]:
        if (parent / "scripts/perception/run_stage2d_pose_case.py").is_file():
            return parent
    default_root = Path.home() / "ur5e_pickplace"
    if (default_root / "scripts/perception/run_stage2d_pose_case.py").is_file():
        return default_root
    raise RuntimeError(
        f"CONFIG_ERROR: Could not locate scripts/perception/run_stage2d_pose_case.py relative to {resolved}"
    )


PROJECT_ROOT = _resolve_project_root()
CONFIG_DIR = PROJECT_ROOT / "config"
RUNNER_SCRIPT = PROJECT_ROOT / "scripts/perception/run_stage2d_pose_case.py"


def _load_scene_xacro_args_module():
    path = CONFIG_DIR / "scene_xacro_args.py"
    spec = importlib.util.spec_from_file_location("scene_xacro_args", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_setup(context, *args, **kwargs):
    record_evidence = (
        LaunchConfiguration("record_evidence").perform(context).lower() == "true"
    )
    gazebo_gui = LaunchConfiguration("gazebo_gui").perform(context).lower() == "true"
    case_name = LaunchConfiguration("case_name").perform(context)

    # 1. Load scene.yaml and build MoveIt configuration for RViz
    scene_file = CONFIG_DIR / "scene.yaml"
    with open(scene_file, "r") as fh:
        scene = yaml.safe_load(fh)
    scene_xacro_args = _load_scene_xacro_args_module()
    base_args = scene_xacro_args.xacro_base_args(scene)
    gripper_args = scene_xacro_args.xacro_gripper_args(scene)
    gripper_model = "parallel_jaw"
    controllers_file = "config/moveit_controllers_parallel_jaw.yaml"

    moveit_config = (
        MoveItConfigsBuilder(
            "ur5e_robotiq", package_name="ur5e_robotiq_moveit_config"
        )
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

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur5e_robotiq_moveit_config"), "config", "moveit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # 2. Stage-2D D3 runner command (D3 frozen parameters: spawn offset (+30, -30) mm, spawn yaw +45 deg)
    runner_cmd = [
        sys.executable,
        str(RUNNER_SCRIPT),
        "--case",
        case_name,
        "--spawn-offset-x-m",
        "0.030",
        "--spawn-offset-y-m",
        "-0.030",
        "--spawned-yaw-deg",
        "45.0",
        "--configured-pick-yaw-deg",
        "0.0",
        "--target-place-yaw-deg",
        "0.0",
        "--target-source",
        "perceived",
        "--use-perceived-yaw",
    ]
    if gazebo_gui:
        runner_cmd.append("--gui")
    if record_evidence:
        runner_cmd.append("--record-diagnostics")

    runner_action = ExecuteProcess(
        cmd=runner_cmd,
        output="screen",
        emulate_tty=True,
    )

    # Clean shutdown of all launch nodes when runner completes
    shutdown_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=runner_action,
            on_exit=[Shutdown(reason="d3_demo runner completed")],
        )
    )

    return [
        SetParameter(name="use_sim_time", value=True),
        rviz_node,
        runner_action,
        shutdown_handler,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz2 with MoveIt planning scene visualization.",
        ),
        DeclareLaunchArgument(
            "record_evidence",
            default_value="true",
            description="Record diagnostic evidence streams (contact forces, gripper joint trace, perceived points).",
        ),
        DeclareLaunchArgument(
            "case_name",
            default_value="D3_demo",
            description="Evidence case identifier for the Stage-2D runner.",
        ),
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Launch Gazebo in visible GUI mode (default: true).",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

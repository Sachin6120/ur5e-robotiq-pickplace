"""m2_cartesian_approach.launch.py — M2: static scene TF + Cartesian approach.

PRECONDITION
    The merged sim and move_group must already be running in another terminal,
    same as M1. This launch starts the static TF publisher and the M2
    approach node.

GROUND-TRUTH EVIDENCE, NOT A ROS_GZ_BRIDGE
    The M2 node's TCP-versus-commanded evidence must come from Gazebo ground
    truth, not TF (TF is MoveIt's own belief, the thing under test — see
    m2_cartesian_approach.cpp's file header). A ros_gz_bridge
    gz.msgs.Pose_V -> tf2_msgs/msg/TFMessage bridge was tried first, but
    verified live to publish every transform with an EMPTY child_frame_id
    (the per-entity name is lost in that conversion for this topic) — so no
    bridge node is launched here. The node instead shells out to
    `gz topic -e`, same method as scripts/05_measure_gripper_geometry.sh;
    "gz_world" is passed down so it can build the right topic path itself.

WHY THE YAML IS READ HERE AND NOT IN C++
    Same reasoning as m1_joint_goal.launch.py: config/scene.yaml is the single
    source of truth, read in Python and passed down as ROS parameters so the
    node has no compiled-in default to silently fall back to.
"""

import importlib.util
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


def _load_scene_xacro_args_module():
    # Single source for scene.yaml's robot.base_pose -> xacro mapping
    # strings, shared with ur5e_robotiq_sim_control.launch.py and
    # move_group.launch.py — see config/scene_xacro_args.py for why this
    # exists (MoveIt silently planning against a different base pose than
    # Gazebo simulates, found during M2).
    path = os.path.expanduser("~/ur5e_pickplace/config/scene_xacro_args.py")
    spec = importlib.util.spec_from_file_location("scene_xacro_args", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _setup(context, *args, **kwargs):
    scene_file = LaunchConfiguration("scene_file").perform(context)
    csv_path = LaunchConfiguration("csv_path").perform(context)
    world = LaunchConfiguration("world").perform(context)

    scene = _load_scene(scene_file)

    try:
        frames = scene["frames"]
        object_pose = scene["object"]["pick_pose"]
        grasp = scene["grasp"]
        thresholds = scene["thresholds"]
        grasp_mode = scene["grasp_mode"]["mode"]
        base_pose = scene["robot"]["base_pose"]
    except KeyError as exc:
        raise RuntimeError(
            f"CONFIG_ERROR: missing key {exc} in {scene_file}. "
            "See docs/M-1_reference_report.md for the expected structure."
        ) from exc

    if grasp.get("tcp_offset") is None:
        raise RuntimeError(
            "CONFIG_ERROR: grasp.tcp_offset is null in scene.yaml. M2 cannot "
            "run without a measured tcp_offset."
        )

    pick_pose = [
        float(object_pose["x"]), float(object_pose["y"]), float(object_pose["z"]),
        float(object_pose["roll"]), float(object_pose["pitch"]), float(object_pose["yaw"]),
    ]
    approach_axis = [float(v) for v in grasp["approach_axis"]]
    base_args = _load_scene_xacro_args_module().xacro_base_args(scene)

    world_frame = frames["world"]
    flange_frame = frames["flange"]
    tcp_frame = frames["tcp"]
    object_frame_name = "object_frame"
    grasp_frame_name = "grasp_frame"

    static_tf_node = Node(
        package="ur5e_pick_place",
        executable="static_scene_tf",
        name="static_scene_tf",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": True,
                "world_frame": world_frame,
                "object_frame_name": object_frame_name,
                "grasp_frame_name": grasp_frame_name,
                "flange_frame": flange_frame,
                "tcp_frame_name": tcp_frame,
                "pick_pose": pick_pose,
                "approach_axis": approach_axis,
                "gripper_roll": float(grasp["gripper_roll"]),
                "tcp_offset": float(grasp["tcp_offset"]),
            }
        ],
    )

    # CORRECTED: MoveItConfigsBuilder's default xacro mappings (from
    # .setup_assistant) don't include base_xyz/base_rpy, so without this the
    # M2 node's own locally-loaded robot model (separate from move_group's,
    # confirmed by "Loading robot model..." appearing in this node's own log
    # output) would still think base_link is at the ground-mounted default —
    # same class of mismatch found and fixed in move_group.launch.py.
    moveit_config = (
        MoveItConfigsBuilder(
            "ur5e_robotiq", package_name="ur5e_robotiq_moveit_config"
        )
        .robot_description(mappings=base_args)
        .to_moveit_configs()
    )

    m2_node = Node(
        package="ur5e_pick_place",
        executable="m2_cartesian_approach",
        name="m2_cartesian_approach",
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
                "world_frame": world_frame,
                "grasp_frame_name": grasp_frame_name,
                "tool0_frame": flange_frame,
                "standoff": float(grasp["standoff"]),
                "tcp_offset": float(grasp["tcp_offset"]),
                "tf_lookup_timeout_s": float(thresholds["tf_lookup_timeout_s"]),
                "cartesian_fraction_min": float(thresholds["cartesian_fraction_min"]),
                "planning_time_s": float(thresholds["planning_time_s"]),
                "plan_attempts": int(thresholds["plan_attempts"]),
                "velocity_scaling": 0.1,
                "acceleration_scaling": 0.1,
                "eef_step": 0.01,
                "csv_path": csv_path,
                "grasp_mode": grasp_mode,
                "gt_wrist3_link_name": "wrist_3_link",
                "gz_world": world,
                # Base-pose guard: compared against Gazebo ground truth at
                # startup so a launch file that forgets to derive base_xyz
                # from scene_xacro_args.py becomes a named CONFIG_ERROR, not
                # a confusing downstream planning failure (see M2's own
                # move_group base-pose bug for why this exists).
                "expected_base_xyz": [
                    float(base_pose["x"]), float(base_pose["y"]), float(base_pose["z"]),
                ],
                "expected_base_rpy": [
                    float(base_pose["roll"]), float(base_pose["pitch"]), float(base_pose["yaw"]),
                ],
            },
        ],
    )

    return [static_tf_node, m2_node]


def generate_launch_description():
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
                default_value="m2_approach.csv",
                description="Where to write the per-run CSV evidence.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="empty",
                description="Gazebo world name, used to build the pose/info topic path.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )

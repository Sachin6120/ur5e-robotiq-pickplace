"""m3_grasp.launch.py — M3 first pass: pad-centre-corrected grasp +
grasp-success verification.

PRECONDITION
    The merged sim and move_group must already be running in another
    terminal, same as M1/M2. This launch starts the static TF publisher and
    the M3 grasp node.

WHY THE YAML IS READ HERE AND NOT IN C++
    Same reasoning as m1/m2's launch files: config/scene.yaml is the single
    source of truth, read in Python and passed down as ROS parameters so the
    node has no compiled-in default to silently fall back to.
    config/grasp_table.yaml is the second source of truth this node needs
    (see its own header for what it is and why it's separate from
    scene.yaml) — flattened into two parallel parameter arrays here rather
    than parsed in C++, same treatment.
"""

import importlib.util
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _load_yaml(path, what):
    if not os.path.isfile(path):
        raise RuntimeError(f"CONFIG_ERROR: {what} not found at {path}.")
    with open(path, "r") as fh:
        return yaml.safe_load(fh)


def _load_scene_xacro_args_module():
    # Single source for scene.yaml's robot.base_pose -> xacro mapping
    # strings, shared with every launch file that spawns a robot model —
    # see config/scene_xacro_args.py for why this exists.
    path = os.path.expanduser("~/ur5e_pickplace/config/scene_xacro_args.py")
    spec = importlib.util.spec_from_file_location("scene_xacro_args", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _setup(context, *args, **kwargs):
    scene_file = LaunchConfiguration("scene_file").perform(context)
    grasp_table_file = LaunchConfiguration("grasp_table_file").perform(context)
    csv_path = LaunchConfiguration("csv_path").perform(context)
    world = LaunchConfiguration("world").perform(context)

    scene = _load_yaml(scene_file, "scene.yaml")
    grasp_table = _load_yaml(grasp_table_file, "grasp_table.yaml")

    try:
        frames = scene["frames"]
        object_pose = scene["object"]["pick_pose"]
        object_size = scene["object"]["size"]
        grasp_width_axis = int(scene["object"]["grasp_width_axis"])
        grasp = scene["grasp"]
        gripper = scene["gripper"]
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
            "CONFIG_ERROR: grasp.tcp_offset is null in scene.yaml. M3 cannot "
            "run without a measured tcp_offset."
        )
    if grasp.get("pad_centre_offset") is None:
        raise RuntimeError(
            "CONFIG_ERROR: grasp.pad_centre_offset is null in scene.yaml. "
            "M3's whole point is the pad-centre correction; it cannot run "
            "without this value. See docs/HANDOFF_M3.md."
        )

    rows = grasp_table.get("rows") or []
    if not rows:
        raise RuntimeError(
            f"CONFIG_ERROR: {grasp_table_file} has no rows under 'rows:'."
        )
    # Sorted ascending by width — interpolate_grip_angle() in m3_grasp.cpp
    # assumes this.
    rows = sorted(rows, key=lambda r: float(r["width_m"]))
    grasp_table_widths_m = [float(r["width_m"]) for r in rows]
    grasp_table_grip_angles_rad = [float(r["grip_angle_rad"]) for r in rows]

    object_width_m = float(object_size[grasp_width_axis])

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

    # NOTE: static_scene_tf publishes tool0->grasp_tcp using tcp_offset
    # ALONE (correct for what that frame documents — see
    # static_scene_tf.cpp's header). m3_grasp does not use the grasp_tcp
    # frame; it computes its own pad-centre-corrected tool0 target directly
    # from grasp_frame, passed tcp_offset and pad_centre_offset separately
    # as its own parameters.
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

    # Same base-pose-mismatch fix as m2's launch file — see that file's
    # comment for why this exists.
    moveit_config = (
        MoveItConfigsBuilder(
            "ur5e_robotiq", package_name="ur5e_robotiq_moveit_config"
        )
        .robot_description(mappings=base_args)
        .to_moveit_configs()
    )

    m3_node = Node(
        package="ur5e_pick_place",
        executable="m3_grasp",
        name="m3_grasp",
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
                "pad_centre_offset": float(grasp["pad_centre_offset"]),
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
                "expected_base_xyz": [
                    float(base_pose["x"]), float(base_pose["y"]), float(base_pose["z"]),
                ],
                "expected_base_rpy": [
                    float(base_pose["roll"]), float(base_pose["pitch"]), float(base_pose["yaw"]),
                ],
                "gripper_ctrl": "gripper_controller",
                "actuated_joint": gripper["actuated_joint"],
                "gripper_command_timeout_s": float(gripper["command_timeout_s"]),
                "gripper_max_effort": float(gripper["max_effort"]),
                "object_width_m": object_width_m,
                "grasp_table_widths_m": grasp_table_widths_m,
                "grasp_table_grip_angles_rad": grasp_table_grip_angles_rad,
                "grasp_tolerance_rad": float(grasp["grasp_tolerance_rad"]),
                "preclose_margin_rad": float(grasp["preclose_margin_rad"]),
            },
        ],
    )

    return [static_tf_node, m3_node]


def generate_launch_description():
    default_scene = os.path.expanduser("~/ur5e_pickplace/config/scene.yaml")
    default_grasp_table = os.path.expanduser("~/ur5e_pickplace/config/grasp_table.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scene_file",
                default_value=default_scene,
                description="Absolute path to scene.yaml, the single source of truth.",
            ),
            DeclareLaunchArgument(
                "grasp_table_file",
                default_value=default_grasp_table,
                description="Absolute path to grasp_table.yaml (measured grip_angle-vs-width rows).",
            ),
            DeclareLaunchArgument(
                "csv_path",
                default_value="m3_grasp.csv",
                description="Where to write the per-run CSV evidence.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="empty",
                description="Gazebo world name, used to build the pose/info and joint_state topic paths.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )

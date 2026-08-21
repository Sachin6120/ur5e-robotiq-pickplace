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


def _load_gripper_geometry_module():
    # scripts/lib/gripper_geometry.py's closed-form pad kinematics, loaded the
    # same way scene_xacro_args.py is above. release_position_rad is computed
    # HERE rather than typed into scene.yaml by hand: the pads travel 13.5mm
    # along the approach axis across the aperture range, so an arbitrary
    # "open" angle is a vertical motion as well as a lateral one, with the
    # object sitting on the table underneath the place descent.
    path = os.path.expanduser("~/ur5e_pickplace/scripts/lib/gripper_geometry.py")
    spec = importlib.util.spec_from_file_location("gripper_geometry", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _setup(context, *args, **kwargs):
    scene_file = LaunchConfiguration("scene_file").perform(context)
    grasp_table_file = LaunchConfiguration("grasp_table_file").perform(context)
    csv_path = LaunchConfiguration("csv_path").perform(context)
    world = LaunchConfiguration("world").perform(context)
    marker_file_prefix = LaunchConfiguration("marker_file_prefix").perform(context)

    scene = _load_yaml(scene_file, "scene.yaml")
    grasp_table = _load_yaml(grasp_table_file, "grasp_table.yaml")

    try:
        frames = scene["frames"]
        object_pose = scene["object"]["pick_pose"]
        place_pose_cfg = scene["object"]["place_pose"]
        object_size = scene["object"]["size"]
        # DEPRECATED cross-check only — the real width axis is derived in
        # scene_xacro_args.resolve_closing_axis(), which validates against
        # this value and raises CONFIG_ERROR on disagreement. Still read here
        # so a missing key stays a named CONFIG_ERROR rather than a KeyError
        # deeper in.
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

    scene_xacro_args = _load_scene_xacro_args_module()

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

    # object_width_m is DERIVED from the gripper's actual closing direction,
    # not indexed with the hand-set object.grasp_width_axis (2026-08-21, M6:
    # that index named object.size[0] while the gripper closes along
    # object-frame +Y, i.e. size[1] — silent for five milestones because the
    # object was a cube). resolve_grasp_width_m() is the single source; it
    # also raises CONFIG_ERROR if scene.yaml's deprecated grasp_width_axis
    # cross-check field still disagrees with the derivation.
    object_width_m = scene_xacro_args.resolve_grasp_width_m(scene)

    pick_pose = [
        float(object_pose["x"]), float(object_pose["y"]), float(object_pose["z"]),
        float(object_pose["roll"]), float(object_pose["pitch"]), float(object_pose["yaw"]),
    ]
    place_pose = [
        float(place_pose_cfg["x"]), float(place_pose_cfg["y"]), float(place_pose_cfg["z"]),
        float(place_pose_cfg["roll"]), float(place_pose_cfg["pitch"]), float(place_pose_cfg["yaw"]),
    ]
    approach_axis = [float(v) for v in grasp["approach_axis"]]
    base_args = scene_xacro_args.xacro_base_args(scene)
    # fingertip_grasp_theta (robotiq_2f_85_macro.urdf.xacro's TENTH
    # OVERRIDE): this node's own robot model must match the sim's spawned
    # gripper geometry, same reasoning as base_xyz/base_rpy immediately
    # above.
    gripper_args = scene_xacro_args.xacro_gripper_args(scene)

    # RELEASE_CLEARANCE_M: added to the object's own width before solving for
    # the release aperture, so the pads open clear of the object rather than
    # stopping exactly at its width (which would still graze it going past on
    # the retreat). Not in scene.yaml -- a fixed 10mm margin, same order of
    # magnitude as the ~13.6mm the pads themselves move vertically across the
    # aperture range, chosen here rather than measured because "how much
    # clearance is enough" isn't a geometric fact to derive, it's a margin to
    # pick.
    RELEASE_CLEARANCE_M = 0.010
    gripper_geometry = _load_gripper_geometry_module()
    release_position_rad = gripper_geometry.theta_for_width(
        object_width_m + RELEASE_CLEARANCE_M
    )

    world_frame = frames["world"]
    flange_frame = frames["flange"]
    tcp_frame = frames["tcp"]
    object_frame_name = "object_frame"
    grasp_frame_name = "grasp_frame"
    place_frame_name = "place_frame"

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
                "place_frame_name": place_frame_name,
                "flange_frame": flange_frame,
                "tcp_frame_name": tcp_frame,
                "pick_pose": pick_pose,
                "place_pose": place_pose,
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
        .robot_description(mappings={**base_args, **gripper_args})
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
                "place_frame_name": place_frame_name,
                "tool0_frame": flange_frame,
                "standoff": float(grasp["standoff"]),
                "retreat": float(grasp["retreat"]),
                "slip_sample_dwell_s": float(grasp["slip_sample_dwell_s"]),
                "marker_file_prefix": marker_file_prefix,
                "release_position_rad": release_position_rad,
                "tcp_offset": float(grasp["tcp_offset"]),
                "pad_centre_offset": float(grasp["pad_centre_offset"]),
                "tf_lookup_timeout_s": float(thresholds["tf_lookup_timeout_s"]),
                "cartesian_fraction_min": float(thresholds["cartesian_fraction_min"]),
                "grasp_pose_error_max_m": float(thresholds["grasp_pose_error_max_m"]),
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
                "grasp_loss_threshold_rad": float(grasp["grasp_loss_threshold_rad"]),
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
            DeclareLaunchArgument(
                "marker_file_prefix",
                default_value="",
                description="Empty (default) disables it. When set, LIFT_DONE/"
                "TRANSPORT_DONE additionally touch <prefix>.liftdone_ready / "
                "<prefix>.transportdone_ready -- see transport.hpp's "
                "TransportParams::marker_file_prefix for why: a filesystem-write "
                "signal for scripts/11_m3_cycles.sh's sweep watcher, robust to "
                "the live-stdout-parsing failure confirmed under the sweep "
                "harness's nested bash -c / backgrounded-tail process tree.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )

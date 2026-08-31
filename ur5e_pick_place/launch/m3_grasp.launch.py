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
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


# install/.../launch is a symlink to this source file in the active colcon
# workspace, so resolving this file yields the project root independently of
# HOME. Gazebo uses HOME only for writable runtime state.
PROJECT_ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = PROJECT_ROOT / "config"
SCRIPT_LIB_DIR = PROJECT_ROOT / "scripts/lib"


def _load_yaml(path, what):
    if not os.path.isfile(path):
        raise RuntimeError(f"CONFIG_ERROR: {what} not found at {path}.")
    with open(path, "r") as fh:
        return yaml.safe_load(fh)


def _load_scene_xacro_args_module():
    # Single source for scene.yaml's robot.base_pose -> xacro mapping
    # strings, shared with every launch file that spawns a robot model —
    # see config/scene_xacro_args.py for why this exists.
    path = CONFIG_DIR / "scene_xacro_args.py"
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
    path = SCRIPT_LIB_DIR / "gripper_geometry.py"
    spec = importlib.util.spec_from_file_location("gripper_geometry", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_parallel_jaw_geometry_module():
    # scripts/lib/parallel_jaw_geometry.py -- the SINGLE SOURCE for
    # gripper_model:=parallel_jaw's geometry formulas (aperture(q)=0.085-q,
    # grasp_centre_offset(q)=q/2, preclose_aperture, etc.). m3_grasp.cpp does
    # NOT reimplement these in C++; every parallel-jaw geometric value is
    # computed HERE, launch-side, and passed down as an already-resolved ROS
    # parameter, same treatment as fingertip_grasp_theta above for the vendor
    # gripper.
    path = SCRIPT_LIB_DIR / "parallel_jaw_geometry.py"
    spec = importlib.util.spec_from_file_location("parallel_jaw_geometry", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _setup(context, *args, **kwargs):
    scene_file = LaunchConfiguration("scene_file").perform(context)
    grasp_table_file = LaunchConfiguration("grasp_table_file").perform(context)
    csv_path = LaunchConfiguration("csv_path").perform(context)
    world = LaunchConfiguration("world").perform(context)
    marker_file_prefix = LaunchConfiguration("marker_file_prefix").perform(context)
    close_and_hold_only = (
        LaunchConfiguration("close_and_hold_only").perform(context).lower() == "true"
    )
    use_perceived_position = (
        LaunchConfiguration("use_perceived_position").perform(context).lower() == "true"
    )
    use_perceived_yaw = (
        LaunchConfiguration("use_perceived_yaw").perform(context).lower() == "true"
    )
    require_perception = (
        LaunchConfiguration("require_perception").perform(context).lower() == "true"
    )
    pregrasp_only = (
        LaunchConfiguration("pregrasp_only").perform(context).lower() == "true"
    )
    descent_only = (
        LaunchConfiguration("descent_only").perform(context).lower() == "true"
    )
    grasp_only = LaunchConfiguration("grasp_only").perform(context).lower() == "true"
    lift_only = LaunchConfiguration("lift_only").perform(context).lower() == "true"
    transport_only = (
        LaunchConfiguration("transport_only").perform(context).lower() == "true"
    )
    pre_lift_barrier_file = LaunchConfiguration("pre_lift_barrier_file").perform(context)
    pre_lift_barrier_timeout_s = float(
        LaunchConfiguration("pre_lift_barrier_timeout_s").perform(context)
    )
    pregrasp_joint_target_raw = LaunchConfiguration("pregrasp_joint_target").perform(context)
    try:
        pregrasp_joint_target = yaml.safe_load(pregrasp_joint_target_raw)
    except yaml.YAMLError as exc:
        raise RuntimeError(
            "CONFIG_ERROR: pregrasp_joint_target must be a YAML/ROS list of six numbers."
        ) from exc
    if pregrasp_joint_target is None:
        pregrasp_joint_target = []
    if not isinstance(pregrasp_joint_target, list):
        raise RuntimeError("CONFIG_ERROR: pregrasp_joint_target must be a list.")
    pregrasp_joint_target = [float(v) for v in pregrasp_joint_target]
    experiment_cartesian_fjt_path = LaunchConfiguration(
        "experiment_cartesian_fjt_path"
    ).perform(context)

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

    # --- gripper_model dispatch (2026-08-25) ---------------------------------
    # "robotiq_linkage" (default): every line below this block that reads
    # gripper[...] / grasp[...] for the vendor gripper is UNCHANGED from
    # before this arg existed. "parallel_jaw": opt-in, computed here from
    # parallel_jaw_geometry.py using the object_width_m already resolved
    # above, and dispatched into the m3_node parameters dict further down --
    # see that dict's own gripper_model-conditioned entries.
    gripper_model = LaunchConfiguration("gripper_model").perform(context)
    # DIAGNOSTIC-ONLY OVERRIDE, 2026-08-29. Default reproduces
    # parallel_jaw_geometry.GRASP_TCP_FIXED_SIDE_CLEARANCE_M (0.0020 m as
    # of 2026-08-31, raised from 0.0015 m) exactly -- passing nothing
    # changes no run. See its own
    # DeclareLaunchArgument below for the full rationale; this is the one
    # override this diagnostic control is authorized to touch.
    pj_fixed_side_clearance_m_raw = LaunchConfiguration(
        "parallel_jaw_fixed_side_clearance_m"
    ).perform(context)
    if gripper_model not in ("robotiq_linkage", "parallel_jaw"):
        raise RuntimeError(
            f"CONFIG_ERROR: gripper_model='{gripper_model}' is not recognised. "
            "Must be 'robotiq_linkage' (default) or 'parallel_jaw'."
        )
    is_parallel_jaw = gripper_model == "parallel_jaw"

    pj_params = {}
    if is_parallel_jaw:
        pjg = _load_parallel_jaw_geometry_module()
        pj_preclose_aperture_m = pjg.preclose_aperture_m(object_width_m)
        pj_q_preclose = pjg.q_for_aperture(pj_preclose_aperture_m)
        pj_q_final_expected = pjg.q_for_width(object_width_m)
        # First-contact effort ceiling and stall semantics are FROZEN by the
        # pre-contact calibration checkpoint (2026-08-25): 5.0 N, derived
        # from the 0.7212 N static-retention requirement (m=0.15kg,
        # mu_eff=1.0202) with headroom below the URDF's 20 N joint limit --
        # not retuned here.
        PARALLEL_JAW_FIRST_CONTACT_MAX_EFFORT_N = 5.0
        # Explicitly-derived linear tolerance for the informational
        # grasp-success check -- see m3_grasp.cpp's pj_grasp_tolerance_m
        # comment for why this is its own parameter, never grasp_tolerance_rad
        # reinterpreted. Same order of magnitude as the controller's own
        # goal_tolerance (0.5mm, controllers.yaml), rounded to a clean number,
        # analogous to RELEASE_CLEARANCE_M above (a margin to pick, not a
        # geometric fact to derive).
        PARALLEL_JAW_GRASP_TOLERANCE_M = 0.001
        if pj_fixed_side_clearance_m_raw == "":
            preclose_offset_x_m = pjg.preclose_pose_offset_m(object_width_m)
        else:
            # Diagnostic path only, exercised when the caller explicitly
            # passes parallel_jaw_fixed_side_clearance_m. Everything else
            # about pre-close (aperture, q_preclose) and the final-close
            # target (Q_MAX_M, commanded past the object) is untouched --
            # only the ARM/TCP positioning offset that sets how close the
            # fixed pad starts to the object's fixed-side face changes.
            preclose_offset_x_m = pjg.preclose_pose_offset_m(
                object_width_m, c_fixed_m=float(pj_fixed_side_clearance_m_raw)
            )
        pj_params = {
            "gripper_model": gripper_model,
            "parallel_jaw_q_preclose": pj_q_preclose,
            "parallel_jaw_q_final_expected": pj_q_final_expected,
            "parallel_jaw_q_close_commanded": pjg.Q_MAX_M,
            "parallel_jaw_preclose_offset_x_m": preclose_offset_x_m,
            "parallel_jaw_tcp_offset_z_m": pjg.TCP_OFFSET_Z_M,
            "parallel_jaw_grasp_tolerance_m": PARALLEL_JAW_GRASP_TOLERANCE_M,
        }
        gripper_ctrl_value = "parallel_jaw_gripper_controller"
        actuated_joint_value = "gripper_jaw_joint"
        gripper_max_effort_value = PARALLEL_JAW_FIRST_CONTACT_MAX_EFFORT_N
    else:
        pj_params = {"gripper_model": gripper_model}
        gripper_ctrl_value = "gripper_controller"
        actuated_joint_value = gripper["actuated_joint"]
        gripper_max_effort_value = float(gripper["max_effort"])

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
    if is_parallel_jaw:
        # Full-open q = 0.0 m (resulting aperture = 0.085 m), guaranteed to clear any object dimension.
        # Do not reuse object-width-based release logic for parallel_jaw.
        release_position_rad = 0.0
    else:
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

    controllers_file = (
        "config/moveit_controllers_parallel_jaw.yaml"
        if is_parallel_jaw
        else "config/moveit_controllers.yaml"
    )
    controller_path = PROJECT_ROOT / "ur5e_robotiq_moveit_config" / controllers_file
    controller_config = _load_yaml(str(controller_path), "MoveIt controller config")
    try:
        startup_m1_tolerance_rad = float(
            controller_config["trajectory_execution"]["allowed_start_tolerance"]
        )
    except KeyError as exc:
        raise RuntimeError(
            "CONFIG_ERROR: MoveIt controller config must define "
            "trajectory_execution.allowed_start_tolerance for startup M1 verification."
        ) from exc
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

    node_params = {
        "use_sim_time": True,
        "world_frame": world_frame,
        "object_frame_name": object_frame_name,
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
        "gripper_ctrl": gripper_ctrl_value,
        "actuated_joint": actuated_joint_value,
        "gripper_command_timeout_s": float(gripper["command_timeout_s"]),
        "gripper_max_effort": gripper_max_effort_value,
        "object_width_m": object_width_m,
        **pj_params,
        "grasp_table_widths_m": grasp_table_widths_m,
        "grasp_table_grip_angles_rad": grasp_table_grip_angles_rad,
        "grasp_tolerance_rad": float(grasp["grasp_tolerance_rad"]),
        "preclose_margin_rad": float(grasp["preclose_margin_rad"]),
        "grasp_loss_threshold_rad": float(grasp["grasp_loss_threshold_rad"]),
        "close_and_hold_only": close_and_hold_only,
        "use_perceived_position": use_perceived_position,
        "perceived_position_topic": "object_detector/position_world",
        "use_perceived_yaw": use_perceived_yaw,
        "perceived_pose_topic": "object_detector/pose_world",
        "perceived_position_timeout_s": float(
            LaunchConfiguration("perceived_position_timeout_s").perform(context)
        ),
        "object_height_m": float(object_size[2]),
        "require_perception": require_perception,
        "pregrasp_only": pregrasp_only,
        "descent_only": descent_only,
        "grasp_only": grasp_only,
        "lift_only": lift_only,
        "transport_only": transport_only,
        "pre_lift_barrier_file": pre_lift_barrier_file,
        "pre_lift_barrier_timeout_s": pre_lift_barrier_timeout_s,
        "m1_joint_names": [str(j) for j in scene["robot"]["arm_joints"]],
        "m1_goal_positions": [
            float(v) for v in scene["milestones"]["m1"]["goal_positions"]
        ],
        "experiment_cartesian_fjt_path": experiment_cartesian_fjt_path,
        "stationary_velocity_eps": float(
            LaunchConfiguration("stationary_velocity_eps").perform(context)
        ),
        "stationary_consecutive_samples": int(
            LaunchConfiguration("stationary_consecutive_samples").perform(context)
        ),
        "stationary_timeout_s": float(
            LaunchConfiguration("stationary_timeout_s").perform(context)
        ),
        "startup_m1_tolerance_rad": startup_m1_tolerance_rad,
        "pregrasp_pose_error_max_m": float(
            LaunchConfiguration("pregrasp_pose_error_max_m").perform(context)
        ),
    }
    if pregrasp_joint_target:
        node_params["pregrasp_joint_target"] = pregrasp_joint_target

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
            node_params,
        ],
    )

    return [static_tf_node, m3_node]


def generate_launch_description():
    default_scene = str(CONFIG_DIR / "scene.yaml")
    default_grasp_table = str(CONFIG_DIR / "grasp_table.yaml")
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
            DeclareLaunchArgument(
                "close_and_hold_only",
                default_value="false",
                description="Default false preserves normal M3/M6 behavior (close/"
                "stall continues into lift/transport/place/release/retreat). When "
                "true: run pre-grasp, descent, and gripper_close_and_hold() exactly "
                "as normal, record the close result, then skip the entire "
                "attempted_transport block and go straight to the final summary/CSV "
                "and a clean exit. See m3_grasp.cpp's close_and_hold_only comment.",
            ),
            DeclareLaunchArgument(
                "use_perceived_position",
                default_value="false",
                description="Use one world-frame RGB-D position for the pick. "
                "false preserves the configured classical pipeline.",
            ),
            DeclareLaunchArgument(
                "use_perceived_yaw",
                default_value="false",
                description="Use one fresh world-frame object pose yaw for the pick. "
                "Requires use_perceived_position:=true and never falls back to "
                "configured yaw.",
            ),
            DeclareLaunchArgument(
                "require_perception",
                default_value="false",
                description="Strict perception mode. true => a perception "
                "timeout is a typed PERCEPTION_TIMEOUT failure that plans and "
                "executes nothing; there is NO fall back to scene.yaml. "
                "Required for Milestone F1 evidence.",
            ),
            DeclareLaunchArgument(
                "pregrasp_only",
                default_value="false",
                description="Milestone F1 stop mode. Stop immediately after pre-grasp "
                "pose is executed and verified. No descent, pre-close, gripper "
                "command, lift, transport, place or release. Not the same as "
                "close_and_hold_only, which stops after contact.",
            ),
            DeclareLaunchArgument(
                "descent_only",
                default_value="false",
                description="Stage-2 descent validation stop mode. Stop after Stage-2 "
                "Cartesian descent and ground-truth pose verification. No gripper "
                "closure command, no lift, no transport, no place or release.",
            ),
            DeclareLaunchArgument(
                "pregrasp_joint_target",
                default_value="[]",
                description="Experiment-only explicit Stage-1 arm target in scene arm_joints order. "
                "Empty preserves the normal pregrasp pose-goal IK behavior.",
            ),
            DeclareLaunchArgument(
                "experiment_cartesian_fjt_path",
                default_value="",
                description="Optional experiment-only path for the exact post-scaling Stage-2 trajectory "
                "passed to MoveIt execution.",
            ),
            DeclareLaunchArgument(
                "grasp_only",
                default_value="false",
                description="Milestone F2 stop mode. Run the existing pre-grasp, "
                "pre-close, Cartesian descent, grasp-pose verification, and direct-"
                "effort gripper close/hold path, then stop before lift, transport, "
                "place, or release. Mutually exclusive with pregrasp_only.",
            ),
            DeclareLaunchArgument(
                "lift_only",
                default_value="false",
                description="Milestone F3 execution boundary. Complete grasp and the "
                "existing Stage-3 lift, post-lift grasp-loss check, and full dwell, "
                "then stop before TRANSPORT_BEGIN. Mutually exclusive with all other "
                "boundary modes. F3 remains unvalidated until measured evidence passes.",
            ),
            DeclareLaunchArgument(
                "transport_only",
                default_value="false",
                description="Transport execution boundary. Complete Stage 4 transport and dwell, "
                "then stop before PLACE_DESCEND_BEGIN. Mutually exclusive with all other boundary modes.",
            ),
            DeclareLaunchArgument(
                "pre_lift_barrier_file",
                default_value="",
                description="Evaluation-only pre-lift barrier. Empty (default) "
                "disables it and preserves existing behaviour exactly. When set "
                "AND lift_only is true, m3_grasp establishes the grasp, touches "
                "<marker_file_prefix>.pre_lift_ready, and blocks immediately "
                "before the lift until this file appears. Synchronisation only: "
                "no controller, gain, physics, perception, grasp, geometry, "
                "transport, place or release behaviour changes. See "
                "docs/F3_P12_5_LIFT_PLAN.md.",
            ),
            DeclareLaunchArgument(
                "pre_lift_barrier_timeout_s",
                default_value="300.0",
                description="Wall-clock seconds to wait for the pre-lift barrier "
                "release before failing with PRE_LIFT_BARRIER_TIMEOUT. On timeout "
                "the lift is NOT attempted and no transport, place or release "
                "occurs. Ignored when pre_lift_barrier_file is empty.",
            ),
            DeclareLaunchArgument(
                "stationary_velocity_eps",
                default_value="0.001",
                description="Max |joint velocity| (rad/s) counting as stationary "
                "at M1 before perception is accepted.",
            ),
            DeclareLaunchArgument(
                "stationary_consecutive_samples",
                default_value="6",
                description="Consecutive below-threshold /joint_states samples "
                "required. One sample is not evidence of rest.",
            ),
            DeclareLaunchArgument(
                "stationary_timeout_s",
                default_value="25.0",
                description="Give up waiting for M1 stationarity after this long.",
            ),
            DeclareLaunchArgument(
                "pregrasp_pose_error_max_m",
                default_value="0.010",
                description="Max ground-truth TCP error against the commanded "
                "pre-grasp pose in pregrasp_only mode.",
            ),
            DeclareLaunchArgument(
                "perceived_position_timeout_s",
                default_value="2.0",
                description="Bounded wait for object_detector/position_world; "
                "timeout falls back to scene.yaml's configured pick position.",
            ),
            DeclareLaunchArgument(
                "gripper_model",
                default_value="robotiq_linkage",
                description="robotiq_linkage (default): unchanged vendor 2F-85 "
                "linkage path (grasp_table.yaml, radians, gripper_controller). "
                "parallel_jaw: opt-in docs/GRIPPER_REDESIGN_DESIGN.md path "
                "(metres, newtons, parallel_jaw_gripper_controller). The "
                "matching gripper_model:=parallel_jaw must ALSO be passed to "
                "ur5e_robotiq_sim_control.launch.py and move_group.launch.py.",
            ),
            DeclareLaunchArgument(
                "parallel_jaw_fixed_side_clearance_m",
                default_value="",
                description="DIAGNOSTIC-ONLY, 2026-08-29. Empty (default) "
                "preserves parallel_jaw_geometry.py's own "
                "GRASP_TCP_FIXED_SIDE_CLEARANCE_M (0.0020 m as of "
                "2026-08-31, raised from 0.0015 m) exactly -- every "
                "existing and future run that does not pass this argument is "
                "byte-for-byte unaffected. When set, overrides ONLY the "
                "fixed-side pre-close clearance passed into "
                "preclose_pose_offset_m()'s c_fixed_m, i.e. how far the arm "
                "positions the fixed pad's inner face from the object's "
                "fixed-side face before the final close. Nothing else about "
                "pre-close aperture, the final-close target, gripper command, "
                "effort, friction, controllers, or trajectories changes. "
                "Ignored when gripper_model is not parallel_jaw.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )

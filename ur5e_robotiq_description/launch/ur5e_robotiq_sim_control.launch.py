# ur5e_robotiq_sim_control.launch.py — bring up the M-1 merged platform in
# Gazebo Harmonic: robot_state_publisher, gz-sim, the /clock bridge, and the
# three controllers from config/controllers.yaml.
#
# Modelled directly on ur_simulation_gz's ur_sim_control.launch.py — that file
# already spawns the arm successfully (confirmed in the arm-only sanity run),
# so its structure is the reference, not something reconstructed from memory.
#
# THIS IS THE STRUCTURAL SPAWN CHECK, NOT M0. No MoveIt, no planning, no
# grasp. Point of this launch is exactly:
#   ros2 control list_hardware_interfaces   (both components active?)
#   ros2 control list_controllers           (3 active?)
#   gz topic -l | grep joint_state          (M0-C's ground-truth topic exists?)
#
# DELIBERATE CHOICES, so they don't need re-deriving later:
#
# 1. NO ros2_control_node HERE. gz_ros2_control's system plugin (wired inside
#    the xacro's <gazebo> block) IS the controller_manager for this model.
#    Launching a second one would start a competing controller_manager fighting
#    the first for the same hardware interfaces — it would half-work, which is
#    a worse failure mode than an outright crash because nothing complains
#    loudly enough to point at the cause.
#
# 2. use_sim_time True everywhere, and /clock bridged from Gazebo. If
#    robot_state_publisher stamps TF against wall-clock while physics runs on
#    sim-time, tf2 lookups either fail outright or silently return stale
#    transforms — which surfaces later as the spec's tf2 lookup timeout enum,
#    at which point it looks like a grasp-pipeline bug rather than a clock
#    source mismatch.
#
# 3. Gazebo starts unpaused (-r) always, GUI is OFF by default. A paused world
#    means controllers never activate, which reads as a controller failure,
#    not "the world is paused". Headless is the right default for a structural
#    check on WSL — there is no reason to fight rendering here; pass
#    gazebo_gui:=true to actually look at it.
#
# 4. Controller spawners are chained SEQUENTIALLY:
#    joint_state_broadcaster -> arm_controller -> gripper_controller, each
#    started only after the previous one exits. Racing controller_manager
#    at startup with parallel spawners is exactly what the donor repo worked
#    around with staged spawners — don't reintroduce that race here.
#
# 5. GZ_SIM_RESOURCE_PATH is set to every AMENT_PREFIX_PATH entry's share/ dir.
#    robotiq_description's meshes are referenced as bare package://... URIs
#    (not resolved at xacro-expansion time via $(find), unlike the UR meshes),
#    so gz-sim itself has to resolve them at load time. Without this, the
#    gripper spawns as placeholder/missing geometry — it still "works"
#    structurally, which makes the missing meshes easy to miss.
#
# 6. The gripper is commanded open once, right after gripper_controller_spawner
#    exits (same OnProcessExit chaining as note 4). The xacro sets
#    ros2_control initial_value for the six arm joints but not for
#    robotiq_85_left_knuckle_joint — the gripper_controller can take 10-15s to
#    activate, and until it does nothing holds that joint or its mimic
#    followers (those are overridden in software BY THE CONTROLLER, which
#    isn't running yet either), so gravity is free to close the linkage
#    before anything takes hold. CONFIRMED empirically, not assumed: 5/5
#    fresh launches tested spawned the gripper near-closed (~0.767rad, its
#    gravity-rest position against the URDF's self-collision limits) rather
#    than open (~0rad) — every prior script's "gripper OPEN" precondition
#    comment was never true, on any fresh launch, all session. This command
#    makes the starting state deterministic; it does not replace the
#    precondition assertion each probe script now runs before proceeding
#    (scripts/lib/gz_settle.py's assert-joint mode) — that assertion is what
#    catches it if this command is ever issued too early or fails silently.
#    See docs/HANDOFF_M3.md's spawn-state investigation for the full case.

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_scene():
    # Shared by base-pose defaulting and table-geometry spawning below —
    # both derive from the same scene.yaml, loaded once here rather than
    # each re-implementing its own existence check.
    scene_file = os.path.expanduser("~/ur5e_pickplace/config/scene.yaml")
    if not os.path.isfile(scene_file):
        return None
    import yaml
    with open(scene_file, "r") as fh:
        return yaml.safe_load(fh)


def _load_table_sdf_module():
    import importlib.util
    module_path = os.path.expanduser("~/ur5e_pickplace/config/scene_table_sdf.py")
    spec = importlib.util.spec_from_file_location("scene_table_sdf", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _table_spawn_args(scene):
    # config/scene_table_sdf.py is the single source deriving the world's
    # table collision geometry from scene.yaml's table: block — same
    # treatment scene_xacro_args.py gives robot.base_pose. Returns None if
    # scene.yaml (or its table: block) isn't present, matching
    # _default_base_args()'s own graceful fallback for M0-style structural
    # checks that predate scene.yaml. Returns (sdf_string, pose_args) —
    # BOTH are required, not just the SDF: see table_pose_args()'s
    # docstring for why `ros_gz_sim create -string` needs the pose passed
    # as separate -x/-y/-z/-R/-P/-Y flags too.
    if not scene or "table" not in scene:
        return None
    module = _load_table_sdf_module()
    return module.table_sdf(scene), module.table_pose_args(scene)


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    tf_prefix = LaunchConfiguration("tf_prefix")
    sim_ignition = LaunchConfiguration("sim_ignition")
    gripper_rotation = LaunchConfiguration("gripper_rotation")
    base_xyz = LaunchConfiguration("base_xyz")
    base_rpy = LaunchConfiguration("base_rpy")
    controllers_file = LaunchConfiguration("controllers_file")
    description_file = LaunchConfiguration("description_file")
    model_name = LaunchConfiguration("model_name")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")

    # See design note 5 above. Every ROS prefix's share/ dir goes on
    # GZ_SIM_RESOURCE_PATH so package://robotiq_description/... resolves.
    ament_prefixes = os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
    share_dirs = [os.path.join(p, "share") for p in ament_prefixes if p]
    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    gz_resource_path = os.pathsep.join(
        [d for d in share_dirs if d] + ([existing_gz_path] if existing_gz_path else [])
    )

    # name:= is passed explicitly and MUST be — $(arg name) is used on this
    # xacro's own root <robot> tag, resolved before its in-document
    # <xacro:arg name="name"> declaration is registered. Confirmed this is not
    # specific to our file: ur_simulation_gz's own ur_gz.urdf.xacro has the
    # identical pattern and fails the identical way with no args at all.
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            description_file,
            " ",
            "name:=",
            model_name,
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "sim_ignition:=",
            sim_ignition,
            " ",
            "gripper_rotation:=",
            gripper_rotation,
            " ",
            # base_xyz/base_rpy default to "0 0 0" — Command runs through a
            # shell, so an unquoted value with embedded spaces gets split into
            # separate positional arguments (xacro then sees multiple input
            # files and refuses). Quote the value to keep it one shell token.
            "base_xyz:='",
            base_xyz,
            "' ",
            "base_rpy:='",
            base_rpy,
            "' ",
            "controllers_file:=",
            controllers_file,
        ]
    )
    # ParameterValue(..., value_type=str) is required, not cosmetic: launch_ros
    # otherwise runs the expanded URDF string through YAML type-inference to
    # decide the parameter's type, and this particular content is ambiguous
    # enough to trip it ("Unable to parse the value of parameter
    # robot_description as yaml"). Forcing str sidesteps that inference.
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    # --- controller spawners, chained sequentially (design note 4) ----------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    delay_arm_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )
    delay_gripper_controller_after_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    # Open the gripper once, deterministically, right after its controller
    # activates — design note 6. `ros2 action send_goal` itself waits for the
    # action server to become available, so this doesn't race the controller
    # being slightly slower to expose the action interface than the spawner
    # process exiting.
    open_gripper_on_activation = ExecuteProcess(
        cmd=[
            "ros2", "action", "send_goal",
            "/gripper_controller/gripper_cmd",
            "control_msgs/action/GripperCommand",
            "{command: {position: 0.0, max_effort: 50.0}}",
        ],
        output="screen",
    )
    delay_open_gripper_after_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[open_gripper_on_activation],
        )
    )

    # --- Gazebo ---------------------------------------------------------------
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            model_name,
            "-allow_renaming",
            "false",
        ],
    )

    # The table: spawns a static collision surface at the pose/size
    # scene.yaml's table: block describes (see _table_spawn_args /
    # config/scene_table_sdf.py). Same design as gz_spawn_entity above —
    # no ordering dependency on gz_launch_description below; `ros_gz_sim
    # create` already tolerates that for the robot spawn, so it does here
    # too. None if scene.yaml/table: isn't present — no table gets
    # spawned, matching _default_base_args()'s own fallback.
    _table_spawn = _table_spawn_args(_load_scene())
    gz_spawn_table = (
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=(
                ["-string", _table_spawn[0]]
                + _table_spawn[1]
                + ["-name", "table", "-allow_renaming", "false"]
            ),
        )
        if _table_spawn
        else None
    )

    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            # Always unpaused (-r) — design note 3. GUI only if asked for.
            "gz_args": IfElseSubstitution(
                gazebo_gui,
                if_value=[" -r -v 4 ", world_file],
                else_value=[" -s -r -v 4 ", world_file],
            )
        }.items(),
    )

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=gz_resource_path
    )

    actions = [
        set_gz_resource_path,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller_after_joint_state_broadcaster,
        delay_gripper_controller_after_arm_controller,
        delay_open_gripper_after_gripper_controller,
        gz_spawn_entity,
        gz_launch_description,
        gz_sim_bridge,
    ]
    if gz_spawn_table is not None:
        actions.append(gz_spawn_table)
    return actions


def _default_base_args(scene):
    # CORRECTED (M2): base_xyz/base_rpy used to default to "0 0 0"
    # unconditionally, independent of scene.yaml. That was invisible right up
    # until it mattered: a plain `ros2 launch ...` with no extra args span the
    # robot ground-mounted even after scene.yaml's robot.base_pose was raised
    # to table height, because nothing here ever read that field — the
    # elevated base only took effect when base_xyz was passed explicitly by
    # hand on the command line. Defaulting from scene.yaml via the same
    # shared module move_group.launch.py and m2_cartesian_approach.launch.py
    # use closes that gap: a bare invocation now matches what MoveIt and M2
    # assume, rather than requiring every caller to remember to pass it.
    #
    # Falls back to "0 0 0" if scene.yaml or robot.base_pose isn't there —
    # this file is also used standalone for M0-style structural checks that
    # predate scene.yaml's base_pose field and don't need it.
    if not scene or "robot" not in scene or "base_pose" not in scene["robot"]:
        return {"base_xyz": "0 0 0", "base_rpy": "0 0 0"}
    import importlib.util

    args_module_path = os.path.expanduser("~/ur5e_pickplace/config/scene_xacro_args.py")
    spec = importlib.util.spec_from_file_location("scene_xacro_args", args_module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.xacro_base_args(scene)


def generate_launch_description():
    default_base_args = _default_base_args(_load_scene())
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("sim_ignition", default_value="true"),
        DeclareLaunchArgument("gripper_rotation", default_value="0.0"),
        DeclareLaunchArgument("base_xyz", default_value=default_base_args["base_xyz"]),
        DeclareLaunchArgument("base_rpy", default_value=default_base_args["base_rpy"]),
        DeclareLaunchArgument(
            "model_name",
            default_value="ur5e_robotiq",
            description="Must match scripts/m0_verify.sh's MODEL default, and "
            "the xacro's own `name` arg default — M0-C's ground-truth topic is "
            "path-keyed on this value: /world/<world>/model/<model_name>/joint_state.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur5e_robotiq_description"), "config", "controllers.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur5e_robotiq_description"),
                    "urdf",
                    "ur5e_robotiq.urdf.xacro",
                ]
            ),
        ),
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="false",
            description="Headless by default for the structural check (design "
            "note 3) — pass true to actually look at the spawn.",
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value="empty.sdf",
            description="World name inside this becomes 'empty', matching "
            "scripts/m0_verify.sh's WORLD default.",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

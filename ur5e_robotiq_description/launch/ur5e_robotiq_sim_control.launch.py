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

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
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

    return [
        set_gz_resource_path,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller_after_joint_state_broadcaster,
        delay_gripper_controller_after_arm_controller,
        gz_spawn_entity,
        gz_launch_description,
        gz_sim_bridge,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("sim_ignition", default_value="true"),
        DeclareLaunchArgument("gripper_rotation", default_value="0.0"),
        DeclareLaunchArgument("base_xyz", default_value="0 0 0"),
        DeclareLaunchArgument("base_rpy", default_value="0 0 0"),
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

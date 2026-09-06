# stage3b_dynamic_scene.launch.py — Stage-3B Dynamic Scene Awareness launch file.
#
# Brings up the ROS_GZ bridge for the dynamic obstacle pose and the
# dynamic_obstacle_scene_node which manages the persistent CollisionObject
# (dynamic_obstacle_0) in MoveIt PlanningScene at 10 Hz.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    target_update_rate_hz = LaunchConfiguration("target_update_rate_hz")
    stale_threshold_s = LaunchConfiguration("stale_threshold_s")
    obstacle_id = LaunchConfiguration("obstacle_id")
    frame_id = LaunchConfiguration("frame_id")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock from Gazebo /clock",
        ),
        DeclareLaunchArgument(
            "target_update_rate_hz",
            default_value="10.0",
            description="Target MoveIt PlanningScene update rate in Hz",
        ),
        DeclareLaunchArgument(
            "stale_threshold_s",
            default_value="0.250",
            description="Stale pose threshold in seconds",
        ),
        DeclareLaunchArgument(
            "obstacle_id",
            default_value="dynamic_obstacle_0",
            description="CollisionObject ID in MoveIt PlanningScene",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="world",
            description="Planning frame for CollisionObject",
        ),
    ]

    # ROS-GZ parameter bridge for dynamic obstacle pose
    dynamic_obstacle_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="dynamic_obstacle_bridge",
        arguments=[
            "/model/dynamic_obstacle/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Dynamic obstacle scene node
    dynamic_obstacle_scene_node = Node(
        package="ur5e_pick_place",
        executable="dynamic_obstacle_scene_node",
        name="dynamic_obstacle_scene_node",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "target_update_rate_hz": target_update_rate_hz,
                "stale_threshold_s": stale_threshold_s,
                "obstacle_id": obstacle_id,
                "frame_id": frame_id,
                "box_size_x": 0.05,
                "box_size_y": 0.05,
                "box_size_z": 0.10,
                "collision_object_topic": "/collision_object",
                "input_pose_topic": "/model/dynamic_obstacle/pose",
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        declared_arguments + [dynamic_obstacle_bridge, dynamic_obstacle_scene_node]
    )

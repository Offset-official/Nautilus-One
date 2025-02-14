"""
Launch an AprilTag detector.

Includes MAVROS
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_ros.actions import Node

def generate_launch_description():
    pkg_auv_bringup = get_package_share_directory("auv_bringup")
    apriltag_params_file = PathJoinSubstitution(
        [pkg_auv_bringup, "params", "apriltag_params.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                output="screen",
                parameters=[apriltag_params_file],
                remappings=[
                    ('/image_rect','/image_raw')
                ]
            ),
            ExecuteProcess(
                cmd=['ros2', 'topic', 'echo', '/topic_name'],
                output='screen', 
            ),
        ]
    )

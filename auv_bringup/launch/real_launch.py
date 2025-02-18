"""
Launch a connection to the real robot.

Includes MAVROS
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_auv_bringup = get_package_share_directory("auv_bringup")
    mavros_params_file = PathJoinSubstitution(
        [pkg_auv_bringup, "params", "real_mavros_params.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="mavros",
                executable="mavros_node",
                output="screen",
                parameters=[mavros_params_file],
            ),
            Node(
                package="base_controller_py",
                executable="depth_publisher"
            ),
        ]
    )

"""
<<<<<<< HEAD
Launch mavros for real robot.
=======
Launch a connection to the real robot.
>>>>>>> 40bdb0b769a7915cd255f5aadc91a18d7e500afa

Includes MAVROS
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
<<<<<<< HEAD
from launch.actions import (
    DeclareLaunchArgument,
)
=======
>>>>>>> 40bdb0b769a7915cd255f5aadc91a18d7e500afa
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
        ]
    )

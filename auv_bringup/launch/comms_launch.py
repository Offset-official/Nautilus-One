"""
Launch nodes for communication reception.

"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description():
    pkg_auv_bringup = get_package_share_directory("auv_bringup")
    color_detector_params_file = PathJoinSubstitution(
        [pkg_auv_bringup, "params", "color_detector_params.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="auv_comms",
                executable="color_detector",
                output="screen",
                parameters=[color_detector_params_file],
                remappings=[ ('/input_image/compressed','/image_raw/compressed') ]
            ),
        ]
    )

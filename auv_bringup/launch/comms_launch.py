"""
Launch nodes for communication reception.

"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_auv_bringup = get_package_share_directory("auv_bringup")
    color_detector_params_file = PathJoinSubstitution(
        [pkg_auv_bringup, "params", "color_detector_params.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "debug",
                default_value="False",
                description="Only launch the color detector?",
            ),
            Node(
                package="auv_comms",
                executable="color_detector_server",
                output="screen",
                parameters=[color_detector_params_file],
                condition=UnlessCondition(LaunchConfiguration("debug")),
            ),
            Node(
                package="auv_comms",
                executable="read_sequence_server",
                output="screen",
                remappings=[("/input_image/compressed", "/image_raw/compressed")],
                condition=UnlessCondition(LaunchConfiguration("debug")),
            ),
            Node(
                package="auv_comms",
                executable="color_detector_pub",
                output="screen",
                parameters=[color_detector_params_file],
                remappings=[("/input_image/compressed", "/image_raw/compressed")],
                condition=IfCondition(LaunchConfiguration("debug")),
            ),
        ]
    )

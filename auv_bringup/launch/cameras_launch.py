"""
Launch two cameras from the jetson

"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description():
    usb_cam = get_package_share_directory("auv_bringup")
    cam1_params_file = PathJoinSubstitution([usb_cam, "params", "params_1.yaml"])
    cam2_params_file = PathJoinSubstitution([usb_cam, "params", "params_2.yaml"])
    return LaunchDescription(
        [
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                output="screen",
                parameters=[cam1_params_file],
                remappings=[("__ns", "/usb_cam_1")],
            ),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                output="screen",
                parameters=[cam2_params_file],
                remappings=[("__ns", "/usb_cam_2")],
            ),
        ]
    )

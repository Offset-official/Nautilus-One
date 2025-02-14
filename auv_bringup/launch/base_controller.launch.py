from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    auv_controller_dir = get_package_share_directory("auv_controller")
    auv_bringup_dir = get_package_share_directory('auv_bringup')
    real_launch_file = os.path.join(auv_bringup_dir,"launch","real_launch.py")
    params_file = os.path.join(
        auv_controller_dir, "config", "base_controller_params.yaml"
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(real_launch_file)),
            Node(
                package="auv_controller",
                executable="base_controller",
                name="base_controller",
                parameters=[params_file],
                output="screen",
            ),
        ]
    )

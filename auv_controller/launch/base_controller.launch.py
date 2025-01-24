from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share_dir = get_package_share_directory("auv_controller")

    params_file = os.path.join(
        package_share_dir, "config", "base_controller_params.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="auv_controller",
                executable="base_controller",
                name="base_controller",
                parameters=[params_file],
                output="screen",
            ),
            Node(
                package="auv_controller",
                executable="velocity_plotter.py",
                name="velocity_plotter",
                output="screen",
            ),
        ]
    )

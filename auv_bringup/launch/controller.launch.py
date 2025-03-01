from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    auv_bringup_dir = get_package_share_directory("auv_bringup")
    real_auv_controller_params_file = os.path.join(
        auv_bringup_dir, "params", "real_controller_params.yaml"
    )

    return LaunchDescription(
        [
            # IncludeLaunchDescription(PythonLaunchDescriptionSource(real_launch_file)),
            Node(
                package="auv_controller",
                executable="dumb_controller",
                name="dumb_controller",
                parameters=[real_auv_controller_params_file],
                output="screen",
            )
        ]
    )

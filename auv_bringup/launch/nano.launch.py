import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    camera_launch_file = os.path.join(get_package_share_directory('auv_bringup'),'launch','cameras_launch.py')
    camera_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(camera_launch_file))

    diagnostics_launch_file = os.path.join(get_package_share_directory('auv_diagnostics'),'launch','diagnostics_launch.py')
    diagnostics_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(diagnostics_launch_file))

    return LaunchDescription([
        camera_launch,diagnostics_launch
        ])

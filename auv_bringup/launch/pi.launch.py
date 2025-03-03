import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    real_launch_file = os.path.join(get_package_share_directory('auv_bringup'),'launch','real_launch.py')
    real_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(real_launch_file))
    controller_launch_file = os.path.join(get_package_share_directory('auv_bringup'),'launch','controller.launch.py')
    controller_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(controller_launch_file))

    return LaunchDescription([
        real_launch,controller_launch
        ])

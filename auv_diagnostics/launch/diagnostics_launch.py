from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auv_diagnostics',
            executable='diagnostics',
            name='diagnostics',
            output='screen'
        ),
        Node(
            package='auv_diagnostics',
            executable='serial_communication',
            name='serial_communication',
            output='screen'
        ),
    ])
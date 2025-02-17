from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auv_diagnostics',
            executable='diagnostics',
            name='auv_diagnostics_node'
        ),
        Node(
            package='auv_diagnostics',
            executable='serial_communication',
            name='serial_communication_node'
        ),
    ])
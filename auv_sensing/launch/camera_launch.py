from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the bridge arguments directly for Gz Sim
    arguments = [
        "/auv_camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image"
    ]

    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge_camera',
            output='screen',
            arguments=arguments,
        ),
        Node(
            package="auv_sensing",
            executable="show_camera"
        )
    ])

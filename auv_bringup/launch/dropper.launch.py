from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="auv_manipulator",
                executable="dropper",
                name="dropper",
                parameters=[
                    {"pulse_pwm": 1540},
                    {"pulse_duration_ms": 200},  # Set the fixed pulse duration here
                ],
                output="screen",
            )
        ]
    )

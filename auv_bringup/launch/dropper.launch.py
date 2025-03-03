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
                    {"pulse_pwm": 1550},
                    {"pulse_duration_ms": 25},  # Set the fixed pulse duration here
                ],
                output="screen",
            )
        ]
    )

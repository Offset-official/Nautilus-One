#!/usr/bin/env python3

import launch
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

    # Node for infer_camera, using the "compressed" launch argument.
    infer_node = launch_ros.actions.Node(
        package='auv_sensing',
        executable='infer_camera_gate',
        name='infer_camera_gate',
        output='screen',
        parameters=[{
            'camera_source_topic': '/auv_camera_front/image_raw',
            'camera_pub_topic': '/auv_camera_front/image_inferred',
            'detections_pub_topic': '/auv_camera_front/detections',
            'inference_service': 'yolo_inference_gate',
            'fastapi_server_ip': '192.168.2.4'
            # 'compressed': LaunchConfiguration('compressed')
        }]
    )

    return LaunchDescription([
        # compressed_arg,
        # uncomp_node,
        infer_node
    ])

if __name__ == '__main__':
    generate_launch_description()

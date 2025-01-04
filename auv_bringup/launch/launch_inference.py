from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    bridge_arguments = [
        "/auv_camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image"
    ]

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_camera',
        output='screen',
        arguments=bridge_arguments,
    )

    yolo_node = Node(
        package="auv_ml",
        executable="yolo_inference_server",
        name="yolo_inference_server",
        output='screen'
    )

    infer_camera_node = Node(
        package="auv_sensing",
        executable="infer_camera",
        name="infer_camera",
        output='screen'
    )

    show_camera_node = Node(
        package="auv_sensing",
        executable="show_camera",
        name="show_camera",
        arguments=["/auv_camera/image_inferred"],
        output='screen'
    )

    # Start infer_camera_node 1 second after yolo_node starts
    start_infer_camera = RegisterEventHandler(
        OnProcessStart(
            target_action=yolo_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[infer_camera_node]
                )
            ]
        )
    )

    # Start show_camera_node 1 second after infer_camera_node starts
    start_show_camera = RegisterEventHandler(
        OnProcessStart(
            target_action=infer_camera_node,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[show_camera_node]
                )
            ]
        )
    )

    return LaunchDescription([
        bridge_node,
        yolo_node,
        start_infer_camera,
        start_show_camera
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments.
    front_inference_arg = DeclareLaunchArgument(
        "front_inference",
        default_value="gate",
        description="Inference type for the front camera (gate, bucket, or None).",
    )
    down_inference_arg = DeclareLaunchArgument(
        "down_inference",
        default_value="bucket",
        description="Inference type for the down camera (gate, bucket, or None).",
    )
    jetson_arg = DeclareLaunchArgument(
        "jetson",
        default_value="false",
        description="Use auv_ml_jetson package if true, otherwise use auv_ml package.",
    )

    # Bridge node to bring images from Gazebo to ROS.
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_camera",
        output="screen",
        arguments=[
            "/auv_camera_down/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            "/auv_camera_front/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
        ],
    )

    # Determine package based on jetson argument.
    # If jetson is "true", then use "auv_ml_jetson", else "auv_ml".
    package_substitution = PythonExpression(
        ["'auv_ml_jetson' if '", LaunchConfiguration("jetson"), "' == 'true' else 'auv_ml'"]
    )

    # Inference service nodes:
    # Launch the gate server if either camera is set to "gate".
    yolo_gate_node = Node(
        package=package_substitution,
        executable="yolo_inference_server_gate",
        name="yolo_inference_gate",
        output="screen",
        condition=IfCondition(
            PythonExpression(
                [
                    "('",
                    LaunchConfiguration("front_inference"),
                    "' == 'gate') or ('",
                    LaunchConfiguration("down_inference"),
                    "' == 'gate')",
                ]
            )
        ),
    )
    # Launch the bucket server if either camera is set to "bucket".
    yolo_bucket_node = Node(
        package=package_substitution,
        executable="yolo_inference_server_bucket",
        name="yolo_inference_bucket",
        output="screen",
        condition=IfCondition(
            PythonExpression(
                [
                    "('",
                    LaunchConfiguration("front_inference"),
                    "' == 'bucket') or ('",
                    LaunchConfiguration("down_inference"),
                    "' == 'bucket')",
                ]
            )
        ),
    )

    # Infer camera nodes (only launched if inference is enabled).
    infer_camera_front_node = Node(
        package="auv_sensing",
        executable="infer_camera",
        name="infer_camera_front",
        output="screen",
        parameters=[
            {
                "camera_source_topic": "/auv_camera_front/image_raw",
                "camera_pub_topic": "/auv_camera_front/image_inferred",
                "detections_pub_topic": "/auv_camera_front/detections",
                "inference_service": [
                    "yolo_inference_",
                    LaunchConfiguration("front_inference"),
                ],
            }
        ],
        condition=UnlessCondition(
            PythonExpression(
                ["'", LaunchConfiguration("front_inference"), "' == 'None'"]
            )
        ),
    )
    infer_camera_down_node = Node(
        package="auv_sensing",
        executable="infer_camera",
        name="infer_camera_down",
        output="screen",
        parameters=[
            {
                "camera_source_topic": "/auv_camera_down/image_raw",
                "camera_pub_topic": "/auv_camera_down/image_inferred",
                "detections_pub_topic": "/auv_camera_down/detections",
                "inference_service": [
                    "yolo_inference_",
                    LaunchConfiguration("down_inference"),
                ],
            }
        ],
        condition=UnlessCondition(
            PythonExpression(
                ["'", LaunchConfiguration("down_inference"), "' == 'None'"]
            )
        ),
    )

    # Show camera nodes.
    # When inference is enabled, show the inferred image (delayed launch).
    show_camera_front_inferred = Node(
        package="auv_sensing",
        executable="show_camera",
        name="show_camera_front_inferred",
        arguments=["/auv_camera_front/image_inferred"],
        output="screen",
        condition=UnlessCondition(
            PythonExpression(
                ["'", LaunchConfiguration("front_inference"), "' == 'None'"]
            )
        ),
    )
    show_camera_down_inferred = Node(
        package="auv_sensing",
        executable="show_camera",
        name="show_camera_down_inferred",
        arguments=["/auv_camera_down/image_inferred"],
        output="screen",
        condition=UnlessCondition(
            PythonExpression(
                ["'", LaunchConfiguration("down_inference"), "' == 'None'"]
            )
        ),
    )
    # When inference is disabled, show the original (raw) camera feed.
    show_camera_front_raw = Node(
        package="auv_sensing",
        executable="show_camera",
        name="show_camera_front_raw",
        arguments=["/auv_camera_front/image_raw"],
        output="screen",
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("front_inference"), "' == 'None'"]
            )
        ),
    )
    show_camera_down_raw = Node(
        package="auv_sensing",
        executable="show_camera",
        name="show_camera_down_raw",
        arguments=["/auv_camera_down/image_raw"],
        output="screen",
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("down_inference"), "' == 'None'"]
            )
        ),
    )

    # Timer actions to delay startup.
    # For cameras using inference, delay the start of the inference node and then the display.
    infer_camera_front_timer = TimerAction(
        period=6.0,
        actions=[infer_camera_front_node],
        # Only start if inference is enabled.
        condition=UnlessCondition(
            PythonExpression(
                ["'", LaunchConfiguration("front_inference"), "' == 'None'"]
            )
        ),
    )
    infer_camera_down_timer = TimerAction(
        period=6.0,
        actions=[infer_camera_down_node],
        condition=UnlessCondition(
            PythonExpression(
                ["'", LaunchConfiguration("down_inference"), "' == 'None'"]
            )
        ),
    )
    show_camera_front_inferred_timer = TimerAction(
        period=5.0,
        actions=[show_camera_front_inferred],
        condition=UnlessCondition(
            PythonExpression(
                ["'", LaunchConfiguration("front_inference"), "' == 'None'"]
            )
        ),
    )
    show_camera_down_inferred_timer = TimerAction(
        period=5.0,
        actions=[show_camera_down_inferred],
        condition=UnlessCondition(
            PythonExpression(
                ["'", LaunchConfiguration("down_inference"), "' == 'None'"]
            )
        ),
    )
    # For cameras with no inference, show the raw feed shortly after launch.
    show_camera_front_raw_timer = TimerAction(
        period=1.0,
        actions=[show_camera_front_raw],
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("front_inference"), "' == 'None'"]
            )
        ),
    )
    show_camera_down_raw_timer = TimerAction(
        period=1.0,
        actions=[show_camera_down_raw],
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("down_inference"), "' == 'None'"]
            )
        ),
    )

    return LaunchDescription(
        [
            front_inference_arg,
            down_inference_arg,
            jetson_arg,
            #        bridge_node,
            yolo_gate_node,
            yolo_bucket_node,
            # Timer actions for inference nodes.
            infer_camera_front_timer,
            infer_camera_down_timer,
            # Timer actions for show_camera nodes.
            show_camera_front_inferred_timer,
            show_camera_down_inferred_timer,
            show_camera_front_raw_timer,
            show_camera_down_raw_timer,
        ]
    )

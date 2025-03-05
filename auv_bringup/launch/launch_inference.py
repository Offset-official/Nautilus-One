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
        description="If true, use compressed camera topics (and show_camera) without launching the gz bridge."
    )

    # Convenience substitution for the jetson flag.
    jetson = LaunchConfiguration("jetson")

    # -------------------------------------------------------------------------
    # Bridge Node: launched only when jetson is false.
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_camera",
        output="screen",
        arguments=[
            "/auv_camera_down/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            "/auv_camera_front/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
        ],
        condition=UnlessCondition(PythonExpression(["'", jetson, "' == 'true'"]))
    )

    # -------------------------------------------------------------------------
    # Topic names: when jetson is true, use compressed topics.
    down_topic = PythonExpression([
        "'/auv_camera_down/image_raw/compressed' if '", jetson, "' == 'true' else '/auv_camera_down/image_raw'"
    ])
    front_topic = PythonExpression([
        "'/auv_camera_front/image_raw/compressed' if '", jetson, "' == 'true' else '/auv_camera_front/image_raw'"
    ])

    # -------------------------------------------------------------------------
    # Display executables: use "show_camera" for compressed (jetson true)
    # and "show_camera_uncomp" for uncompressed.
    down_display_exec = PythonExpression([
        "'show_camera' if '", jetson, "' == 'true' else 'show_camera_uncomp'"
    ])
    front_display_exec = PythonExpression([
        "'show_camera' if '", jetson, "' == 'true' else 'show_camera_uncomp'"
    ])

    # -------------------------------------------------------------------------
    # Determine package for inference service nodes:
    # If jetson is true, use 'auv_ml_jetson', else 'auv_ml'
    package_substitution = PythonExpression(
        ["'auv_ml_jetson' if '", jetson, "' == 'true' else 'auv_ml'"]
    )

    # -------------------------------------------------------------------------
    # Service nodes:
    # Launch the gate server if either camera inference is set to "gate".
    yolo_gate_node = Node(
        package=package_substitution,
        executable="yolo_inference_server_gate",
        name="yolo_inference_gate",
        output="screen",
        condition=IfCondition(
            PythonExpression([
                "('", LaunchConfiguration("front_inference"),
                "' == 'gate') or ('", LaunchConfiguration("down_inference"),
                "' == 'gate')"
            ])
        ),
    )
    # Launch the bucket server if either camera inference is set to "bucket".
    yolo_bucket_node = Node(
        package=package_substitution,
        executable="yolo_inference_server_bucket",
        name="yolo_inference_bucket",
        output="screen",
        condition=IfCondition(
            PythonExpression([
                "('", LaunchConfiguration("front_inference"),
                "' == 'bucket') or ('", LaunchConfiguration("down_inference"),
                "' == 'bucket')"
            ])
        ),
    )

    # -------------------------------------------------------------------------
    # Inference nodes: add a "compressed" parameter based on the jetson flag.
    # The parameter is passed as a string "true" or "false".
    compressed_param = PythonExpression([
        "'true' if '", jetson, "' == 'true' else 'false'"
    ])

    infer_camera_front_node = Node(
        package="auv_sensing",
        executable="infer_camera",
        name="infer_camera_front",
        output="screen",
        parameters=[{
            "camera_source_topic": front_topic,
            "camera_pub_topic": "/auv_camera_front/image_inferred",
            "detections_pub_topic": "/auv_camera_front/detections",
            "inference_service": ["yolo_inference_", LaunchConfiguration("front_inference")],
            "compressed": compressed_param
        }],
        condition=UnlessCondition(
            PythonExpression(["'", LaunchConfiguration("front_inference"), "' == 'None'"])
        ),
    )
    infer_camera_down_node = Node(
        package="auv_sensing",
        executable="infer_camera",
        name="infer_camera_down",
        output="screen",
        parameters=[{
            "camera_source_topic": down_topic,
            "camera_pub_topic": "/auv_camera_down/image_inferred",
            "detections_pub_topic": "/auv_camera_down/detections",
            "inference_service": ["yolo_inference_", LaunchConfiguration("down_inference")],
            "compressed": compressed_param
        }],
        condition=UnlessCondition(
            PythonExpression(["'", LaunchConfiguration("down_inference"), "' == 'None'"])
        ),
    )

    # -------------------------------------------------------------------------
    # Display nodes for inferred images.
    show_camera_front_inferred = Node(
        package="auv_sensing",
        executable=front_display_exec,
        name="show_camera_front_inferred",
        output="screen",
        arguments=["/auv_camera_front/image_inferred"],
        condition=UnlessCondition(
            PythonExpression(["'", LaunchConfiguration("front_inference"), "' == 'None'"])
        ),
    )
    show_camera_down_inferred = Node(
        package="auv_sensing",
        executable=down_display_exec,
        name="show_camera_down_inferred",
        output="screen",
        arguments=["/auv_camera_down/image_inferred"],
        condition=UnlessCondition(
            PythonExpression(["'", LaunchConfiguration("down_inference"), "' == 'None'"])
        ),
    )

    # Display nodes for raw images when inference is disabled.
    show_camera_front_raw = Node(
        package="auv_sensing",
        executable=front_display_exec,
        name="show_camera_front_raw",
        output="screen",
        arguments=[front_topic],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("front_inference"), "' == 'None'"])
        ),
    )
    show_camera_down_raw = Node(
        package="auv_sensing",
        executable=down_display_exec,
        name="show_camera_down_raw",
        output="screen",
        arguments=[down_topic],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("down_inference"), "' == 'None'"])
        ),
    )

    # -------------------------------------------------------------------------
    # Timer actions to delay startup for inference and display nodes.
    infer_camera_front_timer = TimerAction(
        period=6.0,
        actions=[infer_camera_front_node],
        condition=UnlessCondition(
            PythonExpression(["'", LaunchConfiguration("front_inference"), "' == 'None'"])
        ),
    )
    infer_camera_down_timer = TimerAction(
        period=6.0,
        actions=[infer_camera_down_node],
        condition=UnlessCondition(
            PythonExpression(["'", LaunchConfiguration("down_inference"), "' == 'None'"])
        ),
    )
    show_camera_front_inferred_timer = TimerAction(
        period=5.0,
        actions=[show_camera_front_inferred],
        condition=UnlessCondition(
            PythonExpression(["'", LaunchConfiguration("front_inference"), "' == 'None'"])
        ),
    )
    show_camera_down_inferred_timer = TimerAction(
        period=5.0,
        actions=[show_camera_down_inferred],
        condition=UnlessCondition(
            PythonExpression(["'", LaunchConfiguration("down_inference"), "' == 'None'"])
        ),
    )
    show_camera_front_raw_timer = TimerAction(
        period=1.0,
        actions=[show_camera_front_raw],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("front_inference"), "' == 'None'"])
        ),
    )
    show_camera_down_raw_timer = TimerAction(
        period=1.0,
        actions=[show_camera_down_raw],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("down_inference"), "' == 'None'"])
        ),
    )

    # -------------------------------------------------------------------------
    return LaunchDescription([
        front_inference_arg,
        down_inference_arg,
        jetson_arg,
        bridge_node,  # Only launches when jetson is false.
        yolo_gate_node,
        yolo_bucket_node,
        infer_camera_front_timer,
        infer_camera_down_timer,
        show_camera_front_inferred_timer,
        show_camera_down_inferred_timer,
        show_camera_front_raw_timer,
        show_camera_down_raw_timer,
    ])


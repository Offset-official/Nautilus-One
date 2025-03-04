from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import UnlessCondition

def generate_launch_description():
    jetson_arg = DeclareLaunchArgument(
        'jetson',
        default_value='false',
        description='If true, use compressed camera topics and do not launch gz bridge.'
    )

    # Launch the gz bridge only if jetson is false.
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_camera',
        output='screen',
        arguments=[
            "/auv_camera_down/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            "/auv_camera_front/image_raw@sensor_msgs/msg/Image@gz.msgs.Image"
        ],
        condition=UnlessCondition(
            PythonExpression(["'", LaunchConfiguration("jetson"), "' == 'true'"])
        )
    )

    # Determine topic names based on jetson parameter.
    down_topic = PythonExpression([
        "'/auv_camera_down/image_raw/compressed' if '",
        LaunchConfiguration("jetson"),
        "' == 'true' else '/auv_camera_down/image_raw'"
    ])
    front_topic = PythonExpression([
        "'/auv_camera_front/image_raw/compressed' if '",
        LaunchConfiguration("jetson"),
        "' == 'true' else '/auv_camera_front/image_raw'"
    ])

    # Determine which executable to use for showing the camera feed.
    # If jetson is true, use "show_camera" for compressed images.
    # If jetson is false, use "show_camera_uncomp" for uncompressed images.
    down_executable = PythonExpression([
        "'show_camera' if '",
        LaunchConfiguration("jetson"),
        "' == 'true' else 'show_camera_uncomp'"
    ])
    front_executable = PythonExpression([
        "'show_camera' if '",
        LaunchConfiguration("jetson"),
        "' == 'true' else 'show_camera_uncomp'"
    ])

    show_camera_down = Node(
        package="auv_sensing",
        executable=down_executable,
        name="show_camera_down",
        output="screen",
        arguments=[down_topic]
    )
    show_camera_front = Node(
        package="auv_sensing",
        executable=front_executable,
        name="show_camera_front",
        output="screen",
        arguments=[front_topic]
    )

    return LaunchDescription([
        jetson_arg,
        bridge_node,
        show_camera_down,
        show_camera_front
    ])

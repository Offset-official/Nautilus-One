from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Camera device path'
    )
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='auv_camera_front',
        description='Camera name'
    )
    
    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value='15',
        description='Camera frame rate'
    )
    
    camera_device_arg_2 = DeclareLaunchArgument(
        'camera_device_2',
        default_value='/dev/video2',
        description='Camera device path'
    )
    
    camera_name_arg_2 = DeclareLaunchArgument(
        'camera_name_2',
        default_value='auv_camera_bottom',
        description='Camera name'
    )

    # Create and return launch description
    return LaunchDescription([
        camera_device_arg,
        camera_name_arg,
        camera_device_arg_2,
        camera_name_arg_2,
        frame_rate_arg,
        Node(
            package='auv_camera',
            executable='publish_camera',
            name='publish_camera',
            output='screen',
            parameters=[{
                'camera_device': LaunchConfiguration('camera_device'),
                'camera_name': LaunchConfiguration('camera_name'),
                'frame_rate': LaunchConfiguration('frame_rate')
            }]
        ),
        Node(
            package='auv_camera',
            executable='publish_camera',
            name='publish_camera',
            output='screen',
            parameters=[{
                'camera_device': LaunchConfiguration('camera_device_2'),
                'camera_name': LaunchConfiguration('camera_name_2'),
                'frame_rate': LaunchConfiguration('frame_rate')
            }]
        )
    ])
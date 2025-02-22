import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to your Next.js project
    nextjs_project_dir = os.path.expanduser("~/auv_ws/src/auv_ros2/auv_hud/dashboard")  # Change this to your actual path
    
    return LaunchDescription([
        # Start rosbridge websocket server
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen'
        ),
        
        # Start Next.js development server
        ExecuteProcess(
            cmd=["pnpm", "run", "dev"],
            cwd=nextjs_project_dir,
            shell=True,
            output="screen"
        )
    ])

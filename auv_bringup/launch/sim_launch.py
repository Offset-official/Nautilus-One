"""
Launch a simulation.

Includes Gazebo, ArduSub, MAVROS
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
    ExecuteProcess,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    TextSubstitution,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_auv_description = get_package_share_directory("auv_description")
    pkg_auv_bringup = get_package_share_directory("auv_bringup")
    gz_launch_path = PathJoinSubstitution(
        [pkg_ros_gz_sim, "launch", "gz_sim.launch.py"]
    )
    gz_model_path = PathJoinSubstitution([pkg_auv_description, "models"])
    gz_config_path = PathJoinSubstitution([pkg_auv_bringup, "cfg", "gz_blue.config"])
    ardusub_params_file = PathJoinSubstitution([pkg_auv_bringup, "cfg", "sub.param"])
    mavros_params_file = PathJoinSubstitution(
        [pkg_auv_bringup, "params", "sim_mavros_params.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="pool",
                choices=["pool", "competition", "underwater", "pool_gate"],
                description="World to load into Gazebo",
            ),
            DeclareLaunchArgument(
                "mavros",
                default_value="True",
                description="Launch mavros?",
            ),
            SetLaunchConfiguration(
                name="world_file",
                value=[LaunchConfiguration("world"), TextSubstitution(text=".world")],
            ),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_model_path),
            ExecuteProcess(
                cmd=[
                    "ardusub",
                    "-S",
                    "-w",
                    "-M",
                    "JSON",
                    "--defaults",
                    ardusub_params_file,
                    "-I0",
                    "--home",
                    "33.810313,-118.39386700000001,0.0,0",
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch_path),
                launch_arguments={
                    "gz_args": [
                        "-r -v4 ",
                        "--gui-config ",
                        gz_config_path,
                        " ",
                        PathJoinSubstitution(
                            [
                                pkg_auv_description,
                                "worlds",
                                LaunchConfiguration("world_file"),
                            ]
                        ),
                    ],
                    "on_exit_shutdown": "True",
                }.items(),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[],
                remappings=[],
                output="screen",
            ),
            Node(
                package="mavros",
                executable="mavros_node",
                output="screen",
                parameters=[mavros_params_file],
                condition=IfCondition(LaunchConfiguration("mavros")),
            ),
        ]
    )

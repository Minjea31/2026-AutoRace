from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory('autorace_mux')) / 'config' / 'mux.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('output_topic', default_value='/cmd_vel'),
        Node(
            package='autorace_mux', executable='mux_node', name='mux', output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[('/cmd_vel', LaunchConfiguration('output_topic'))],
        ),
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output_topic', default_value='/cmd_vel'),
        Node(
            package='autorace_mux', executable='mux_node', name='mux', output='screen',
            remappings=[('/cmd_vel', LaunchConfiguration('output_topic'))],
        ),
    ])

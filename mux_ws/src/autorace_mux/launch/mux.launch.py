from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('parking_guard_enabled',default_value='false'),
        DeclareLaunchArgument('output_topic', default_value='/cmd_vel'),
        Node(
            package='autorace_mux', executable='mux_node', name='mux', output='screen',
            parameters=[{'parking_guard_enabled':ParameterValue(LaunchConfiguration('parking_guard_enabled'),value_type=bool)}],
            remappings=[('/cmd_vel', LaunchConfiguration('output_topic'))],
        ),
    ])

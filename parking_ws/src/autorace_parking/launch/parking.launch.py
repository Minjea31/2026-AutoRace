from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('autorace_parking'))
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=str(share / 'config/parking.yaml')),
        Node(package='autorace_parking', executable='parking_node', output='screen',
             parameters=[LaunchConfiguration('params_file'), {'use_sim_time': sim_time}]),
        Node(package='autorace_parking', executable='parking_robot_visual',
             condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[{'use_sim_time': sim_time}]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(share / 'config/parking.rviz')], parameters=[{'use_sim_time': sim_time}]),
    ])

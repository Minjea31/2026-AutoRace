"""Visualization only: no sensors, mission controllers or motion publishers."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('tensor_bringup')) / 'rviz/autorace_missions.rviz'
    helper = Node(package='tensor_bringup', executable='mission_visualizer.py', output='screen',
                  parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])
    viewer = Node(package='rviz2', executable='rviz2', arguments=['-d', str(config)], output='screen',
                  parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        RegisterEventHandler(OnProcessExit(target_action=helper, on_exit=[EmitEvent(event=Shutdown(reason='Viewer helper exited'))])),
        RegisterEventHandler(OnProcessExit(target_action=viewer, on_exit=[EmitEvent(event=Shutdown(reason='RViz closed'))])),
        helper,
        viewer,
    ])

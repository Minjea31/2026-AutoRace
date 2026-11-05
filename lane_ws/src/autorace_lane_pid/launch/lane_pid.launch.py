"""Simulator lane keeping: lane_lite perception + error-based PID controller.

Two separate windows (both optional):
  lane_gui:=true  lane_lite_tuner  - perception parameters only
  gui:=true       lane_pid_tuner   - PID / speed parameters only (default on)
The controller always publishes /lane_pid/command; /mission/lane only with
enable_vehicle_output:=true (then run simul/adapter.sh or the mux).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('autorace_lane_pid')
    arg = LaunchConfiguration
    sim_time = ParameterValue(arg('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('pid_config',
                              default_value=PathJoinSubstitution([share, 'config', 'sim', 'pid.yaml'])),
        DeclareLaunchArgument('lane', default_value='true',
                              description='also start lane_lite_node (profile sim)'),
        DeclareLaunchArgument('lane_config', default_value=PathJoinSubstitution(
            [FindPackageShare('autorace_lane_lite'), 'config', 'sim', 'lane.yaml'])),
        DeclareLaunchArgument('lane_gui', default_value='false', description='open lane_lite_tuner'),
        DeclareLaunchArgument('gui', default_value='true', description='open lane_pid_tuner'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enable_vehicle_output', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('autorace_lane_lite'), 'launch', 'lane_lite.launch.py'])),
            condition=IfCondition(arg('lane')),
            launch_arguments={'profile': 'sim', 'config_file': arg('lane_config'),
                              'use_sim_time': arg('use_sim_time'), 'gui': arg('lane_gui')}.items()),
        Node(package='autorace_lane_pid', executable='lane_pid_controller', output='screen',
             parameters=[arg('pid_config'), {
                 'use_sim_time': sim_time,
                 'enable_vehicle_output': ParameterValue(arg('enable_vehicle_output'), value_type=bool)}]),
        Node(package='autorace_lane_pid', executable='lane_pid_tuner', output='screen',
             condition=IfCondition(arg('gui')), arguments=['--config', arg('pid_config')]),
    ])

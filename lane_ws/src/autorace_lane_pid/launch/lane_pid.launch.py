"""Lane keeping: lane_lite perception + error-based PID controller.

profile:=sim | real switches BOTH configs at once:
  autorace_lane_lite/config/<profile>/lane.yaml  (perception)
  autorace_lane_pid/config/<profile>/pid.yaml    (control)
and the use_sim_time default (true for sim, false for real).

Run directly with ros2 launch (no wrapper script needed):
  ros2 launch autorace_lane_pid lane_pid.launch.py profile:=real          # perception + PID
  ros2 launch autorace_lane_pid lane_pid.launch.py profile:=real lane:=false   # PID only
  (sim: source simul/ros_env.sh first so the simulator DDS domain is used)

Two separate windows (both optional):
  lane_gui:=true  lane_lite_tuner  - perception parameters only
  gui:=true       lane_pid_tuner   - PID / speed parameters only
                  (default: on for sim, off for real - the car has no display
                   over ssh; run the tuner on the laptop instead)
/mission/lane is published by default (enable_vehicle_output:=true); the mux
(real) or simul/adapter.sh (sim) forwards it to the vehicle. Pass
enable_vehicle_output:=false to only publish the /lane_pid/command preview.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('autorace_lane_pid')
    lane_share = FindPackageShare('autorace_lane_lite')
    arg = LaunchConfiguration
    profile = arg('profile')
    sim_time = ParameterValue(arg('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='sim', choices=['sim', 'real'],
                              description='sim | real: picks config/<profile>/ of both packages'),
        DeclareLaunchArgument('pid_config',
                              default_value=PathJoinSubstitution([share, 'config', profile, 'pid.yaml'])),
        DeclareLaunchArgument('lane', default_value='true', description='also start lane_lite_node'),
        DeclareLaunchArgument('lane_config', default_value=PathJoinSubstitution(
            [lane_share, 'config', profile, 'lane.yaml'])),
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw/compressed'),
        DeclareLaunchArgument('lane_gui', default_value='false', description='open lane_lite_tuner'),
        DeclareLaunchArgument('debug', default_value='false',
                              description='lane_lite_node publishes /lane_lite/debug (BEV picture)'),
        DeclareLaunchArgument('gui', default_value=PythonExpression(
            ["'true' if '", profile, "' == 'sim' else 'false'"]), description='open lane_pid_tuner'),
        DeclareLaunchArgument('use_sim_time', default_value=PythonExpression(
            ["'true' if '", profile, "' == 'sim' else 'false'"])),
        DeclareLaunchArgument('enable_vehicle_output', default_value='true',
                              description='publish /mission/lane (false = /lane_pid/command preview only)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([lane_share, 'launch', 'lane_lite.launch.py'])),
            condition=IfCondition(arg('lane')),
            launch_arguments={'profile': profile, 'config_file': arg('lane_config'),
                              'image_topic': arg('image_topic'),
                              'use_sim_time': arg('use_sim_time'), 'gui': arg('lane_gui'),
                              'debug': arg('debug')}.items()),
        Node(package='autorace_lane_pid', executable='lane_pid_controller', output='screen',
             parameters=[arg('pid_config'), {
                 'use_sim_time': sim_time,
                 'enable_vehicle_output': ParameterValue(arg('enable_vehicle_output'), value_type=bool)}]),
        Node(package='autorace_lane_pid', executable='lane_pid_tuner', output='screen',
             condition=IfCondition(arg('gui')), arguments=['--config', arg('pid_config')]),
    ])

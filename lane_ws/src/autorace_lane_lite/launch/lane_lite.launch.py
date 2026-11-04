"""lane_lite perception (+ optional tuner GUI).

profile:=sim | real picks config/<profile>/lane.yaml (sim and real are tuned separately).
Publishes /lane_lite/target only; no drive command.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('autorace_lane_lite')
    arg = LaunchConfiguration
    sim_time = ParameterValue(arg('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='real', description='sim | real (config/<profile>/)'),
        DeclareLaunchArgument('config_file',
                              default_value=PathJoinSubstitution([share, 'config', arg('profile'), 'lane.yaml'])),
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw/compressed'),
        DeclareLaunchArgument('transport', default_value='compressed', description='compressed | raw'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='true opens lane_lite_tuner (needs autorace_lane_single at build time)'),
        Node(package='autorace_lane_lite', executable='lane_lite_node', output='screen',
             parameters=[{'config_file': arg('config_file'), 'image_topic': arg('image_topic'),
                          'transport': arg('transport'), 'use_sim_time': sim_time}]),
        Node(package='autorace_lane_lite', executable='lane_lite_tuner', output='screen',
             condition=IfCondition(arg('gui')),
             arguments=['--config', arg('config_file'), '--topic', arg('image_topic'),
                        '--transport', arg('transport')],
             parameters=[{'use_sim_time': sim_time}]),
    ])

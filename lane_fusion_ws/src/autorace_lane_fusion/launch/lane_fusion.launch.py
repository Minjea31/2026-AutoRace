"""lane_lite perception (+ optional perception tuner GUI). No control here.

profile:=sim | real picks config/<profile>/lane.yaml (sim and real are tuned
separately) and the use_sim_time default (true for sim). Steering/speed come from the separate autorace_lane_fusion_pid package.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('autorace_lane_fusion')
    arg = LaunchConfiguration
    sim_time = ParameterValue(arg('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='sim', choices=['sim', 'real'],
                              description='sim | real (config/<profile>/)'),
        DeclareLaunchArgument('config_file',
                              default_value=PathJoinSubstitution([share, 'config', arg('profile'), 'lane.yaml'])),
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw/compressed',
                              description='sensor_msgs/CompressedImage (JPEG)'),
        DeclareLaunchArgument('use_sim_time', default_value=PythonExpression(
            ["'true' if '", arg('profile'), "' == 'sim' else 'false'"])),
        DeclareLaunchArgument('gui', default_value='false', description='true opens lane_fusion_tuner'),
        DeclareLaunchArgument('debug', default_value='false',
                              description='true publishes /lane_fusion/debug (BEV picture, built only while subscribed)'),
        DeclareLaunchArgument('hsv', default_value='true',
                              description='false: HSV detector off (deep mask only, e.g. tensor_bringup lane_mode:=deep)'),
        DeclareLaunchArgument('max_rate', default_value='0.0',
                              description='processing cap (Hz) on the newest camera frame; 0 = every frame'),
        Node(package='autorace_lane_fusion', executable='lane_fusion_node', output='screen', condition=IfCondition(arg('hsv')),
             parameters=[{'config_file': arg('config_file'), 'image_topic': arg('image_topic'),
                          'use_sim_time': sim_time,
                          'debug': ParameterValue(arg('debug'), value_type=bool),
                          'max_rate': ParameterValue(arg('max_rate'), value_type=float)}]),
        Node(package='autorace_lane_fusion', executable='lane_fusion_node', output='screen', condition=UnlessCondition(arg('hsv')),
             parameters=[{'config_file': arg('config_file'), 'image_topic': arg('image_topic'),
                          'use_sim_time': sim_time,
                          'debug': ParameterValue(arg('debug'), value_type=bool),
                          'max_rate': ParameterValue(arg('max_rate'), value_type=float),
                          # deep only: the same pipeline with the HSV detector off
                          'yellow_enabled': 0.0, 'white_enabled': 0.0}]),
        Node(package='autorace_lane_fusion', executable='lane_fusion_tuner', output='screen',
             condition=IfCondition(arg('gui')),
             arguments=['--config', arg('config_file'), '--topic', arg('image_topic')],
             parameters=[{'use_sim_time': sim_time}]),
    ])

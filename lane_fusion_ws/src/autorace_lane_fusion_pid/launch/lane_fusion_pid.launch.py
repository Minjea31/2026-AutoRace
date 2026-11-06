"""Lane keeping (fusion): lanedeep U-Net mask + lane_fusion perception + PID controller.

profile:=sim | real switches BOTH configs at once:
  autorace_lane_fusion/config/<profile>/lane.yaml  (perception)
  autorace_lane_fusion_pid/config/<profile>/pid.yaml    (control)
and the use_sim_time default (true for sim, false for real).

Run directly with ros2 launch (no wrapper script needed):
  ros2 launch autorace_lane_fusion_pid lane_fusion_pid.launch.py profile:=real          # perception + PID
  ros2 launch autorace_lane_fusion_pid lane_fusion_pid.launch.py profile:=real lane:=false   # PID only
  (sim: source simul/ros_env.sh first so the simulator DDS domain is used)

Deep mask node (lane_fusion_deep_mask -> /lane_fusion/deep_mask):
  deep:=true (default real) / false; model_path:=<.engine|.onnx|.pt> (default: bundled
  lane_640.onnx; on the Jetson pass the TensorRT engine built with tools/build_tensorrt.py).
  Without masks lane_fusion_node runs HSV-only, so deep:=false is still drivable.

Two separate windows (both optional):
  lane_gui:=true  lane_fusion_tuner  - perception parameters only
  gui:=true       lane_fusion_pid_tuner   - PID / speed parameters only
                  (default: on for sim, off for real - the car has no display
                   over ssh; run the tuner on the laptop instead)
/mission/lane is published by default (enable_vehicle_output:=true); the mux
(real) or simul/adapter.sh (sim) forwards it to the vehicle. Pass
enable_vehicle_output:=false to only publish the /lane_fusion_pid/command preview.
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
    share = FindPackageShare('autorace_lane_fusion_pid')
    lane_share = FindPackageShare('autorace_lane_fusion')
    deep_share = FindPackageShare('autorace_lane_fusion_deep')
    arg = LaunchConfiguration
    profile = arg('profile')
    sim_time = ParameterValue(arg('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='sim', choices=['sim', 'real'],
                              description='sim | real: picks config/<profile>/ of both packages'),
        DeclareLaunchArgument('pid_config',
                              default_value=PathJoinSubstitution([share, 'config', profile, 'pid.yaml'])),
        DeclareLaunchArgument('lane', default_value='true', description='also start lane_fusion_node'),
        DeclareLaunchArgument('lane_config', default_value=PathJoinSubstitution(
            [lane_share, 'config', profile, 'lane.yaml'])),
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw/compressed'),
        DeclareLaunchArgument('lane_gui', default_value='false', description='open lane_fusion_tuner'),
        DeclareLaunchArgument('debug', default_value='false',
                              description='lane_fusion_node publishes /lane_fusion/debug (BEV picture)'),
        DeclareLaunchArgument('gui', default_value=PythonExpression(
            ["'true' if '", profile, "' == 'sim' else 'false'"]), description='open lane_fusion_pid_tuner'),
        DeclareLaunchArgument('use_sim_time', default_value=PythonExpression(
            ["'true' if '", profile, "' == 'sim' else 'false'"])),
        DeclareLaunchArgument('enable_vehicle_output', default_value='true',
                              description='publish /mission/lane (false = /lane_fusion_pid/command preview only)'),
        DeclareLaunchArgument('deep', default_value=PythonExpression(
            ["'true' if '", profile, "' == 'real' else 'false'"]),
            description='start lane_fusion_deep_mask (U-Net) for gated fusion'),
        DeclareLaunchArgument('model_path', default_value='',
                              description='U-Net model (.engine/.onnx/.pt); empty = bundled ONNX'),
        DeclareLaunchArgument('deep_device', default_value='cuda',
                              description='cuda | cpu (ONNX Runtime without CUDA provider needs cpu)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([deep_share, 'launch', 'deep_mask.launch.py'])),
            condition=IfCondition(arg('deep')),
            launch_arguments={'model_path': arg('model_path'), 'device': arg('deep_device'),
                              'camera_topic': arg('image_topic'),
                              'use_sim_time': arg('use_sim_time')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([lane_share, 'launch', 'lane_fusion.launch.py'])),
            condition=IfCondition(arg('lane')),
            launch_arguments={'profile': profile, 'config_file': arg('lane_config'),
                              'image_topic': arg('image_topic'),
                              'use_sim_time': arg('use_sim_time'), 'gui': arg('lane_gui'),
                              'debug': arg('debug')}.items()),
        Node(package='autorace_lane_fusion_pid', executable='lane_fusion_pid_controller', output='screen',
             parameters=[arg('pid_config'), {
                 'use_sim_time': sim_time,
                 'enable_vehicle_output': ParameterValue(arg('enable_vehicle_output'), value_type=bool)}]),
        Node(package='autorace_lane_fusion_pid', executable='lane_fusion_pid_tuner', output='screen',
             condition=IfCondition(arg('gui')), arguments=['--config', arg('pid_config')]),
    ])

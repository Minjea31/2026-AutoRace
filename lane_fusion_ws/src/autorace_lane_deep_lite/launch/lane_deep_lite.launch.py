"""Deep-only lane keeping: U-Net mask -> lane_deep_lite (no HSV) -> fusion PID.

  ros2 launch autorace_lane_deep_lite lane_deep_lite.launch.py profile:=real
  ros2 launch autorace_lane_deep_lite lane_deep_lite.launch.py profile:=real pid:=false   # perception only
  ros2 launch autorace_lane_deep_lite lane_deep_lite.launch.py profile:=real debug:=true  # /lane_deep/debug

Nodes:
  lane_fusion_deep_mask  camera -> /lane_fusion/deep_mask    (deep:=false if already running)
  lane_deep_lite_node    /lane_fusion/deep_mask -> /lane_deep/target
  lane_fusion_pid_controller  /lane_deep/target -> /mission/lane   (pid:=true)

Do not run lane_fusion_pid.launch.py at the same time: both would publish /mission/lane.
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
    share = FindPackageShare('autorace_lane_deep_lite')
    pid_share = FindPackageShare('autorace_lane_fusion_pid')
    deep_share = FindPackageShare('autorace_lane_fusion_deep')
    arg = LaunchConfiguration
    profile = arg('profile')
    sim_time = ParameterValue(arg('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='real', choices=['sim', 'real']),
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution(
            [share, 'config', profile, 'lane.yaml'])),
        DeclareLaunchArgument('pid_config', default_value=PathJoinSubstitution(
            [pid_share, 'config', profile, 'pid.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value=PythonExpression(
            ["'true' if '", profile, "' == 'sim' else 'false'"])),
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw/compressed'),
        DeclareLaunchArgument('deep', default_value='true', description='start lane_fusion_deep_mask'),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('deep_device', default_value='auto'),
        DeclareLaunchArgument('pid', default_value='true', description='start the fusion PID on /lane_deep/target'),
        DeclareLaunchArgument('gui', default_value='false', description='lane_fusion_pid_tuner window'),
        DeclareLaunchArgument('enable_vehicle_output', default_value='true'),
        DeclareLaunchArgument('debug', default_value='false', description='publish /lane_deep/debug'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([deep_share, 'launch', 'deep_mask.launch.py'])),
            condition=IfCondition(arg('deep')),
            launch_arguments={'model_path': arg('model_path'), 'device': arg('deep_device'),
                              'camera_topic': arg('image_topic'),
                              'use_sim_time': arg('use_sim_time')}.items()),
        Node(package='autorace_lane_deep_lite', executable='lane_deep_lite_node', output='screen',
             parameters=[{'config_file': arg('config_file'), 'use_sim_time': sim_time,
                          'debug': ParameterValue(arg('debug'), value_type=bool)}]),
        Node(package='autorace_lane_fusion_pid', executable='lane_fusion_pid_controller', output='screen',
             condition=IfCondition(arg('pid')),
             parameters=[arg('pid_config'), {
                 'use_sim_time': sim_time, 'target_topic': '/lane_deep/target',
                 'enable_vehicle_output': ParameterValue(arg('enable_vehicle_output'), value_type=bool)}]),
        Node(package='autorace_lane_fusion_pid', executable='lane_fusion_pid_tuner', output='screen',
             condition=IfCondition(PythonExpression(["'", arg('pid'), "' == 'true' and '", arg('gui'), "' == 'true'"])),
             arguments=['--config', arg('pid_config')]),
    ])

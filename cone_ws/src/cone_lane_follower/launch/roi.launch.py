from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from cone_lane_follower.node import ROI_DEFAULTS


def generate_launch_description():
    parameters = {name: ParameterValue(LaunchConfiguration(name), value_type=float)
                  for name in ROI_DEFAULTS}
    parameters['use_sim_time'] = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=(
            get_package_share_directory('cone_lane_follower') + '/config/clustering.yaml')),
        *[DeclareLaunchArgument(name, default_value=str(default))
          for name, default in ROI_DEFAULTS.items()],
        Node(package='cone_lane_follower', executable='roi_node', output='screen',
             parameters=[LaunchConfiguration('params_file'), parameters]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', get_package_share_directory('cone_lane_follower') + '/rviz/roi.rviz'],
             parameters=[{'use_sim_time': parameters['use_sim_time']}]),
    ])

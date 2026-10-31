from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    args = ['--config',value('config'),'--map',value('map'),'--view',value('view'),'--mission',value('mission')]
    if value('headless').lower() == 'true':
        args.append('--headless')
    nodes = [Node(package='limo_parking_sim',executable='limo_parking_sim',
                  name='limo_parking_sim',output='screen',arguments=args)]
    if value('parking_adapter').lower() == 'true':
        nodes.append(Node(package='limo_parking_sim',executable='parking_sim_adapter',
                          output='screen',arguments=['--config',value('config')]))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('limo_parking_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=str(share/'config/limo_pro.yaml')),
        DeclareLaunchArgument('map',default_value=str(share/'config/parking_map.yaml')),
        DeclareLaunchArgument('headless',default_value='false'),
        DeclareLaunchArgument('parking_adapter',default_value='false'),
        DeclareLaunchArgument('view',default_value='parking'),
        DeclareLaunchArgument('mission',default_value='parking'),
        OpaqueFunction(function=setup),
    ])

"""Read-only mission web; bind all interfaces for devices on the same Wi-Fi."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    runner = Path.home() / '2026-AutoRace' / 'web_viewer' / 'run.sh'
    return LaunchDescription([
        DeclareLaunchArgument('web_host', default_value='0.0.0.0',
                              description='HTTP bind address; 0.0.0.0 accepts LAN clients'),
        DeclareLaunchArgument('web_port', default_value='8080', description='HTTP TCP port'),
        LogInfo(msg=['Mission web listen: http://', LaunchConfiguration('web_host'), ':',
                     LaunchConfiguration('web_port'), ' (LAN clients use the platform IP)']),
        ExecuteProcess(cmd=[str(runner), '--host', LaunchConfiguration('web_host'),
                            '--port', LaunchConfiguration('web_port')], output='screen'),
    ])

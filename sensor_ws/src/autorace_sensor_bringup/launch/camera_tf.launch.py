"""Camera mount transforms; camera internal/optical TF belongs to Orbbec."""
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    profile = value('tf_profile')
    if profile not in ('existing', 'wego_limo_pro'):
        raise ValueError('tf_profile must be existing or wego_limo_pro')
    mount = ('0.2', '0.1', '0.06') if profile == 'existing' else ('0.103', '0', '0.06')
    pitch = float(value('degree'))
    if not math.isfinite(pitch):
        raise ValueError('degree must be a finite pitch in radians')

    def transform(name, parent, child, xyz, pitch='0'):
        return Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=name, output='both', arguments=[
                '--x', xyz[0], '--y', xyz[1], '--z', xyz[2],
                '--roll', '0', '--pitch', pitch, '--yaw', '0',
                '--frame-id', parent, '--child-frame-id', child,
            ],
        )

    return [
        transform('base_link_to_camera_mount', 'base_link', 'camera_mount', mount),
        transform('camera_mount_to_rotate', 'camera_mount', 'camera_rotate',
                  ('0', '0', '0'), str(pitch)),
        transform('rotate_to_camera_link', 'camera_rotate', value('camera_name') + '_link',
                  ('0.03', '-0.05', '0')),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('degree', default_value='0.0'),
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('tf_profile', default_value='wego_limo_pro'),
        OpaqueFunction(function=_setup),
    ])

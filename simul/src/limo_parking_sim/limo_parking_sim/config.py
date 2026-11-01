"""Configuration loading shared by the simulator and hardware-free checks."""
import copy
import math
from pathlib import Path

import yaml


def share_directory():
    local = Path(__file__).resolve().parents[1]
    if (local / 'config').is_dir():
        return local
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('limo_parking_sim'))


def load_config(vehicle=None, world=None):
    share = share_directory()
    with open(vehicle or share / 'config/limo_pro.yaml', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    with open(world or share / 'config/parking_map.yaml', encoding='utf-8') as f:
        cfg['world'] = yaml.safe_load(f)
    cfg = copy.deepcopy(cfg)
    from .world import expand_vehicle_layout
    expand_vehicle_layout(cfg['world'])
    validate(cfg)
    return cfg


def validate(cfg):
    def positive(section, names):
        for name in names:
            x = cfg[section][name]
            if not isinstance(x, (float, int)) or not math.isfinite(x) or x <= 0:
                raise ValueError(f'{section}.{name} must be positive and finite')
    positive('simulation', ['physics_hz', 'telemetry_hz', 'status_hz',
                            'real_time_factor', 'command_timeout'])
    positive('vehicle', ['length', 'width', 'height', 'wheelbase', 'driver_track',
                        'wheel_track', 'max_speed', 'min_turn_radius',
                        'max_inner_steering', 'max_acceleration', 'max_steering_rate'])
    positive('lidar', ['frequency', 'sample_rate', 'range_max', 'physical_range_max'])
    if not -180 <= cfg['lidar']['angle_min_deg'] < cfg['lidar']['angle_max_deg'] <= 180:
        raise ValueError('Lidar angles must satisfy -180 <= min < max <= 180')
    if not 0 <= cfg['lidar']['dropout_probability'] <= 1 or cfg['lidar']['noise_stddev'] < 0:
        raise ValueError('Invalid lidar noise/dropout')
    for name in ('color', 'depth'):
        c = cfg['camera'][name]
        if not isinstance(c['width'], int) or not isinstance(c['height'], int):
            raise ValueError('Image width/height must be integers')
        if min(c['width'], c['height'], c['fps']) <= 0:
            raise ValueError('Image dimensions/fps must be positive')
        if not 0 < c['hfov_deg'] < 180 or not 0 < c['vfov_deg'] < 180:
            raise ValueError('Camera FOV must be between 0 and 180 degrees')
        if c['intrinsics'] is not None:
            fx, fy, cx, cy = c['intrinsics']
            if not all(math.isfinite(v) for v in (fx, fy, cx, cy)) or min(fx, fy) <= 0:
                raise ValueError('Intrinsics require finite fx,fy,cx,cy and positive fx,fy')
        if c['fps'] > cfg['simulation']['physics_hz']:
            raise ValueError('Camera fps cannot exceed physics_hz')
    if not 0 < cfg['camera']['near'] < cfg['camera']['far']:
        raise ValueError('Camera near/far planes are invalid')
    wall = cfg['world'].get('boundary_wall')
    if wall:
        for name in ('height', 'thickness'):
            value = wall[name]
            if not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'world.boundary_wall.{name} must be positive and finite')
        if len(wall['color']) != 4 or any(not math.isfinite(c) or not 0 <= c <= 1 for c in wall['color']):
            raise ValueError('Boundary wall color must contain four RGBA values in [0, 1]')
    x0, x1, y0, y1 = cfg['world']['parking']['goal']
    if not 0 <= x0 < x1 <= cfg['world']['size'][0] or not 0 <= y0 < y1 <= cfg['world']['size'][1]:
        raise ValueError('Parking goal must be a positive rectangle inside the map')
    # Reject accidental duplicate publishers caused by topic configuration errors.
    values = list(cfg['topics'].values())
    if len(values) != len(set(values)) or any(not t.startswith('/') for t in values):
        raise ValueError('Topic names must be unique absolute ROS names')

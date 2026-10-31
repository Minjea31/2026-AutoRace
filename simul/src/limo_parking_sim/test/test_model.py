import json
import math
from pathlib import Path

import numpy as np
import pytest

from limo_parking_sim.config import load_config, share_directory, validate
from limo_parking_sim.model import Box, Simulation, camera_mount, intrinsics, overlaps
from limo_parking_sim.parking_adapter import drive_to_twist


@pytest.fixture
def cfg():
    return load_config()


def advance(model, seconds, v, w):
    for _ in range(round(seconds*120)):
        model.command(v,w)
        model.step(1/120)


def test_reverse_changes_yaw_sign_without_pivot(cfg):
    cfg['simulation']['spawn'] = [3,3,0]
    a = Simulation(cfg)
    advance(a,1,0.,1.)
    assert (a.x,a.y,a.yaw) == (3,3,0)
    advance(a,1,.2,.2)
    assert a.x > 3 and a.yaw > 0
    b = Simulation(cfg)
    advance(b,1,-.2,-.2)
    assert b.x < 3 and b.yaw < 0
    assert a.steering == pytest.approx(b.steering)
    assert a.wheel_travel>0 and b.wheel_travel<0


def test_exact_constant_curvature_and_driver_limit(cfg):
    cfg['simulation']['spawn'] = [3,3,0]
    cfg['vehicle']['max_acceleration'] = 1000.
    cfg['vehicle']['max_steering_rate'] = 1000.
    m = Simulation(cfg)
    advance(m,1,.2,.2)
    assert m.x == pytest.approx(3+math.sin(.2),abs=1e-10)
    assert m.y == pytest.approx(3+1-math.cos(.2),abs=1e-10)
    assert m.yaw == pytest.approx(.2,abs=1e-10)
    advance(m,.5,1.,100.)
    assert m.steering == pytest.approx(m.max_steering)
    assert abs(m.yaw_rate) <= abs(m.speed)/.4+1e-8
    expected = .2/math.tan(.48869)+.172/2
    assert m.min_radius == pytest.approx(expected)


def test_timeout_and_invalid_command_stop(cfg):
    m = Simulation(cfg)
    advance(m,.3,.3,0)
    assert m.speed > 0
    for _ in range(65):
        m.step(1/120)
    assert m.speed == 0 and m.status()['command_stale']
    assert not m.command(float('nan'),0)
    m.step(1/120)
    assert m.speed == 0


def test_swept_collision_and_escape(cfg):
    cfg['simulation']['spawn'] = [2,2,0]
    cfg['vehicle']['max_acceleration'] = 10000
    m = Simulation(cfg)
    m.obstacles = [Box('thin_pole',2.4,2,0,.01,1,0,.3)]
    m.command(1,0)
    m.step(.4)
    assert m.collision and m.collisions == 1
    assert m.x+.322/2 < 2.4-.005
    assert m.x > 2.1
    advance(m,.15,-.2,0)
    assert not m.collision and m.speed < 0


def test_sat_corners_height_and_boundary(cfg):
    a = Box('a',0,0,math.pi/4,.322,.22,0,.25)
    assert overlaps(a,Box('b',0,0,0,.322,.22,0,.25))
    assert not overlaps(a,Box('b',0,0,0,.322,.22,.3,.5))
    assert not overlaps(a,Box('b',1,0,0,.322,.22,0,.25))
    m = Simulation(cfg)
    assert m.collisions_at(.05,3,0) == ['map_boundary']
    assert not m.collisions_at(3,3,0)


def test_lidar_metric_range_frame_and_height(cfg):
    cfg['simulation']['spawn'] = [2,2,0]
    m = Simulation(cfg)
    assert len(m.angles) == 245
    assert np.degrees(m.angles[[0,-1]]) == pytest.approx([-110,110])
    m.obstacles = [Box('front',3.103,2,0,.2,1,0,.3)]
    mid = len(m.angles)//2
    assert m.scan()[mid] == pytest.approx(.9,abs=1e-6)
    m.obstacles[0].zmin = .2
    assert m.scan()[mid] == 0  # above 116 mm laser plane
    m.obstacles = [Box('left',2.103,3,math.pi/4,.2,.2,0,.3)]
    distances = m.scan()
    hit_angles = m.angles[distances>0]
    assert hit_angles.size and np.all(hit_angles > 0)
    m.obstacles = [Box('too_far',16,2,0,.2,1,0,.3)]
    assert m.scan()[mid] == 0  # 64 m ROS metadata does not invent a 64 m T-mini Pro
    m.obstacles = []
    assert np.all(m.scan() == 0)  # paint/map border is not a lidar wall


def test_noise_seed_and_inf_profile(cfg):
    cfg['lidar']['noise_stddev'] = .01
    a,b = Simulation(cfg),Simulation(cfg)
    assert np.array_equal(a.scan(),b.scan())
    cfg['lidar']['invalid_range_is_inf'] = True
    c = Simulation(cfg)
    c.obstacles = []
    assert np.all(np.isinf(c.scan()))


def test_camera_intrinsics_and_tilt(cfg):
    p,r = camera_mount(cfg,'color')
    assert p == pytest.approx([.133,-.05,.06])
    assert r == pytest.approx(np.eye(3))
    cfg['camera']['pitch'] = .3
    p,r = camera_mount(cfg,'color')
    assert r[2,0] < 0  # positive pitch points camera down
    cfg['camera']['color']['intrinsics'] = [400.,390.,315.,231.]
    assert intrinsics(cfg['camera']['color']) == (400.,390.,315.,231.)


def test_cad_dimensions_not_pixel_tracing(cfg):
    data = json.loads((share_directory()/'assets/track_geometry.json').read_text())
    assert data['size_m'] == [10.,6.]
    assert len(data['polylines_m']) == 276
    x0,x1,y0,y1 = cfg['world']['parking']['goal']
    assert x1-x0 == pytest.approx(.5,abs=.0004)
    assert y1-y0 == pytest.approx(.6,abs=.0004)
    assert np.any(np.all(np.isclose(np.concatenate(data['polylines_m']),[x0,y0]),axis=1))


def test_topic_contract_from_docs(cfg):
    root = Path(__file__).resolve().parents[4]
    architecture = json.loads((root/'docs/architecture-topics.json').read_text())
    types = {t['name']:t['type'] for edge in architecture for t in edge['topics']}
    assert types[cfg['topics']['compressed']] == 'sensor_msgs/msg/CompressedImage'
    assert types[cfg['topics']['scan']] == 'sensor_msgs/msg/LaserScan'
    assert types[cfg['topics']['cmd_vel']] == 'geometry_msgs/msg/Twist'
    assert types[cfg['topics']['parking_command']] == 'ackermann_msgs/msg/AckermannDriveStamped'


def test_adapter_units_reverse_and_finite(cfg):
    args = (.2,1.,.42)
    assert drive_to_twist(-.2,.3,*args) == pytest.approx((-.2,-.2*math.tan(.3)/.2))
    assert drive_to_twist(0,.3,*args) == (0.,0.)
    assert drive_to_twist(float('inf'),.3,*args) == (0.,0.)


def test_reset_clock_monotonic_and_no_scoring(cfg):
    m = Simulation(cfg)
    advance(m,.3,.2,0)
    before = m.time
    m.reset()
    assert m.time == before
    assert (m.x,m.y,m.yaw) == tuple(cfg['simulation']['spawn'])
    assert not {'score','parked','corners_inside'} & m.status().keys()


def test_config_validation(cfg):
    cfg['lidar']['angle_min_deg'] = 120
    with pytest.raises(ValueError):
        validate(cfg)

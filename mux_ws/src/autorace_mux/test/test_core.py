from dataclasses import replace
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from autorace_mux.core import Arbitrator, Candidate, Config, INPUTS, TwistConverter


EPOCH = 1_000_000_000_000


def receive(mux, source, speed=0.4, angle=0.1, at=0.0, stamp=None):
    now_ns = EPOCH + round(at * 1e9)
    return mux.receive(source, speed, angle, now_ns if stamp is None else stamp, at, now_ns)


def select(mux, at=0.0):
    return mux.select(at, EPOCH + round(at * 1e9))


def test_lane_is_default_and_no_input_stops():
    mux = Arbitrator(Config())
    assert select(mux) is None
    assert receive(mux, 'lane')[0]
    assert select(mux).source == 'lane'


def test_all_priorities_override_lane_independent_of_arrival_order():
    mux = Arbitrator(Config())
    for source, _ in INPUTS:
        assert receive(mux, source)[0]
    for source, _ in INPUTS:
        assert select(mux).source == source
        mux.discard(source)
    assert select(mux) is None


def test_stop_mission_is_active_and_stops_immediately():
    cfg = Config(max_acceleration_mps2=100.0)
    mux, converter = Arbitrator(cfg), TwistConverter(cfg)
    receive(mux, 'lane')
    converter.convert(select(mux), 0.0)
    assert converter.convert(select(mux), 0.1)[0] > 0
    receive(mux, 'crosswalk', speed=0.0)
    assert select(mux).source == 'crosswalk'
    assert converter.convert(select(mux), 0.11) == (0.0, 0.0)


def test_expired_mission_falls_back_to_fresh_lane_then_stops():
    mux = Arbitrator(Config())
    receive(mux, 'crosswalk', speed=0.0)
    receive(mux, 'lane', at=0.2)
    assert select(mux, 0.24).source == 'crosswalk'
    assert select(mux, 0.26).source == 'lane'
    assert select(mux, 0.46) is None


def test_header_age_expires_even_when_replayed_with_new_receipt_time():
    mux = Arbitrator(Config())
    receive(mux, 'lane', at=0.0)
    assert receive(mux, 'lane', at=0.2, stamp=EPOCH)[0]
    assert select(mux, 0.24) is not None
    assert select(mux, 0.26) is None


@pytest.mark.parametrize('stamp', [0, EPOCH - 1_000_000_000, EPOCH + 1_000_000_000])
def test_missing_old_and_future_stamps_are_rejected(stamp):
    mux = Arbitrator(Config())
    assert not receive(mux, 'crosswalk', stamp=stamp)[0]
    assert select(mux) is None


def test_future_stamp_within_tolerance_and_unstamped_opt_in():
    mux = Arbitrator(Config())
    assert receive(mux, 'lane', stamp=EPOCH + 20_000_000)[0]
    assert select(mux) is not None
    mux = Arbitrator(Config(require_stamp=False))
    assert receive(mux, 'lane', stamp=0)[0]
    assert select(mux, 0.26) is None


def test_out_of_order_message_keeps_newer_candidate():
    mux = Arbitrator(Config())
    receive(mux, 'lane', speed=0.4, at=0.1)
    assert not receive(mux, 'lane', speed=0.1, at=0.11, stamp=EPOCH)[0]
    assert select(mux, 0.12).speed == 0.4


def test_stale_out_of_order_message_preserves_stop_without_refreshing_timeout():
    mux = Arbitrator(Config())
    receive(mux, 'lane', at=0.3)
    receive(mux, 'crosswalk', speed=0.0, at=0.3)
    stop = select(mux, 0.31)
    assert stop.source == 'crosswalk'

    accepted, reason = receive(mux, 'crosswalk', at=0.32, stamp=EPOCH)
    assert not accepted
    assert reason == 'out-of-order stamp'
    assert select(mux, 0.33) is stop

    receive(mux, 'lane', at=0.5)
    assert select(mux, 0.54) is stop
    assert select(mux, 0.56).source == 'lane'


@pytest.mark.parametrize('stamp', [0, -1, EPOCH + 1_000_000_000])
def test_invalid_stamp_still_removes_previous_candidate(stamp):
    mux = Arbitrator(Config())
    receive(mux, 'lane', at=0.3)
    assert not receive(mux, 'lane', at=0.32, stamp=stamp)[0]
    assert select(mux, 0.33) is None


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize('field', ['speed', 'angle'])
def test_invalid_command_removes_previous_candidate(value, field):
    mux = Arbitrator(Config())
    receive(mux, 'lane')
    assert not receive(mux, 'lane', **{field: value})[0]
    assert select(mux) is None


def test_reverse_is_allowed_only_for_parking():
    mux = Arbitrator(Config())
    for source, _ in INPUTS:
        accepted, _ = receive(mux, source, speed=-0.2)
        assert accepted == (source == 'parking')
    assert select(mux).source == 'parking'


def test_ros_clock_reset_drops_cached_candidates():
    mux = Arbitrator(Config())
    receive(mux, 'lane', at=1.0)
    assert mux.select(1.1, EPOCH - 1) is None


def test_conversion_clamps_and_uses_vehicle_geometry():
    cfg = Config(max_acceleration_mps2=100.0, max_deceleration_mps2=100.0,
                 max_yaw_rate_rps=20.0, steering_limit_enabled=True, yaw_rate_limit_enabled=True)
    converter = TwistConverter(cfg)
    candidate = Candidate('lane', 10.0, 1.0, EPOCH, 0.0)
    converter.convert(candidate, 0.0)
    speed, yaw = converter.convert(candidate, 0.1)
    assert speed == cfg.max_forward_speed_mps
    assert yaw == pytest.approx(speed * math.tan(cfg.max_steering_angle_rad) / cfg.wheelbase_m)
    limited = TwistConverter(replace(cfg, max_yaw_rate_rps=0.2))
    limited.convert(candidate, 0.0)
    assert limited.convert(candidate, 0.1)[1] == 0.2
    assert limited.convert(None, 0.11) == (0.0, 0.0)


def test_reverse_conversion_and_no_turn_in_place():
    cfg = Config(max_acceleration_mps2=100.0)
    converter = TwistConverter(cfg)
    candidate = Candidate('parking', -2.0, 0.2, EPOCH, 0.0)
    converter.convert(candidate, 0.0)
    speed, yaw = converter.convert(candidate, 0.1)
    assert speed == -cfg.max_reverse_speed_mps
    assert yaw == pytest.approx(speed * math.tan(0.2) / cfg.wheelbase_m)
    assert converter.convert(replace(candidate, speed=0.0), 0.11) == (0.0, 0.0)


def test_acceleration_and_direction_change_pass_through_zero():
    converter = TwistConverter(Config())
    forward = Candidate('lane', 0.5, 0.0, EPOCH, 0.0)
    reverse = replace(forward, source='parking', speed=-0.3)
    assert converter.convert(forward, 0.0)[0] == 0.0
    assert converter.convert(forward, 0.1)[0] == pytest.approx(0.1)
    assert converter.convert(reverse, 0.2)[0] == 0.0
    assert converter.convert(reverse, 0.3)[0] == pytest.approx(-0.1)


@pytest.mark.parametrize('name', [
    'publish_rate_hz', 'input_timeout_sec', 'wheelbase_m', 'max_forward_speed_mps',
    'max_reverse_speed_mps', 'max_steering_angle_rad', 'max_yaw_rate_rps',
    'max_acceleration_mps2', 'max_deceleration_mps2',
])
@pytest.mark.parametrize('value', [0.0, -1.0, math.nan, math.inf])
def test_invalid_configuration_fails_at_startup(name, value):
    with pytest.raises(ValueError):
        Config(**{name: value})


def test_singular_steering_limit_is_rejected():
    with pytest.raises(ValueError):
        Config(max_steering_angle_rad=math.pi / 2)


def test_inputs_and_output_match_saved_drawio_and_topic_manifest():
    repo = Path(__file__).resolve().parents[4]
    if not (repo / 'docs/2026_AutoRace.drawio').exists():
        pytest.skip('Repository diagram is not installed with the package')
    graph = ET.parse(repo / 'docs/2026_AutoRace.drawio')
    edges = graph.findall('./diagram/mxGraphModel/root/mxCell[@edge="1"]')
    expected = {topic for _, topic in INPUTS}
    actual = {topic for edge in edges if edge.get('target') == 'mux_ws'
              for topic in expected if topic in edge.get('value', '')}
    assert actual == expected
    manifest = json.loads((repo / 'docs/architecture-topics.json').read_text())
    inputs = [e for e in manifest if e['target'] == 'mux_ws']
    assert {e['topics'][0]['name'] for e in inputs} == expected
    assert all(e['topics'][0]['type'] == 'ackermann_msgs/msg/AckermannDriveStamped'
               for e in inputs)
    outputs = [e for e in manifest if e['source'] == 'mux_ws']
    assert len(outputs) == 1
    assert outputs[0]['topics'] == [{'name': '/cmd_vel', 'type': 'geometry_msgs/msg/Twist'}]


def test_disabled_steering_and_yaw_caps_preserve_requested_turn():
    config = Config(max_steering_angle_rad=.4, max_yaw_rate_rps=1.5,
                    steering_limit_enabled=False, yaw_rate_limit_enabled=False)
    converter = TwistConverter(config)
    candidate = Candidate('parking', .5, .65, 1, 0.)
    converter.convert(candidate, 0.)
    for i in range(1, 21):
        speed, yaw = converter.convert(candidate, i*.05)
    assert speed == pytest.approx(.5)
    assert yaw == pytest.approx(.5*math.tan(.65)/config.wheelbase_m)
    assert yaw > config.max_yaw_rate_rps

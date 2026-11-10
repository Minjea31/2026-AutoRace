"""The default run profile and optional launch YAML share the same policy."""
from dataclasses import fields
import math
from pathlib import Path

import pytest

from autorace_mux.core import Arbitrator, Candidate, Config, TwistConverter


@pytest.mark.parametrize('requested,expected', [(2.0, .8), (-2.0, -.3)])
def test_default_speed_limits_are_enforced_with_turn_caps_disabled(requested, expected):
    config = Config(max_acceleration_mps2=100., max_deceleration_mps2=100.)
    assert not config.steering_limit_enabled and not config.yaw_rate_limit_enabled
    converter = TwistConverter(config)
    candidate = Candidate('parking', requested, 1.0, 1, 0.)
    converter.convert(candidate, 0.)
    speed, yaw = converter.convert(candidate, .1)
    assert speed == pytest.approx(expected)
    assert yaw == pytest.approx(expected * math.tan(1.) / config.wheelbase_m)


@pytest.mark.parametrize('angle', [-math.pi, -math.pi/2, math.pi/2, math.pi])
def test_singular_or_reversed_steering_input_removes_the_old_candidate(angle):
    mux = Arbitrator(Config())
    assert mux.receive('parking', .2, .1, 100, 0., 100)[0]
    accepted, reason = mux.receive('parking', .2, angle, 101, .01, 101)
    assert not accepted and 'steering' in reason
    assert mux.select(.01, 101) is None


@pytest.mark.parametrize('field', ['steering_limit_enabled', 'yaw_rate_limit_enabled'])
@pytest.mark.parametrize('value', [0, 1, 'false'])
def test_optional_limit_switches_require_boolean_values(field, value):
    with pytest.raises(ValueError):
        Config(**{field: value})


def test_run_defaults_match_launch_yaml_when_the_package_has_one():
    path = Path(__file__).resolve().parents[1] / 'config/mux.yaml'
    if not path.exists():
        pytest.skip('Platform launch uses code defaults without a YAML file')
    yaml = pytest.importorskip('yaml')
    values = yaml.safe_load(path.read_text())['mux']['ros__parameters']
    defaults = Config()
    assert {f.name: values[f.name] for f in fields(Config)} == vars(defaults)

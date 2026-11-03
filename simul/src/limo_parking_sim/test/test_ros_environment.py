"""Check simulator DDS routing without starting any DDS participants."""
import json
import os
from pathlib import Path
import subprocess

import pytest


@pytest.mark.parametrize('inherited,override,expected', [
    (None, None, '42'),
    ('0', None, '42'),
    ('42', None, '42'),
    ('144', None, '144'),
    ('0', '176', '176'),
    ('42', '0', '0'),
])
def test_simulator_domain_and_localhost(inherited, override, expected):
    environment = os.environ.copy()
    for key in ('ROS_DOMAIN_ID', 'AUTORACE_SIM_DOMAIN_ID'):
        environment.pop(key, None)
    environment['ROS_LOCALHOST_ONLY'] = '0'
    if inherited is not None:
        environment['ROS_DOMAIN_ID'] = inherited
    if override is not None:
        environment['AUTORACE_SIM_DOMAIN_ID'] = override
    script = Path(__file__).resolve().parents[3] / 'ros_env.sh'
    result = subprocess.run([
        '/bin/bash', '--noprofile', '--norc', '-c',
        'source "$1" && /usr/bin/python3 -c '
        "'import json, os; print(json.dumps([os.environ[\"ROS_DOMAIN_ID\"], "
        "os.environ[\"ROS_LOCALHOST_ONLY\"]]))'", 'test', str(script),
    ], env=environment, text=True, capture_output=True, check=True)
    assert json.loads(result.stdout) == [expected, '1']

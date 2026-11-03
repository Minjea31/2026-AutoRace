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


@pytest.mark.parametrize('arguments,expected', [
    (['topic', 'list'], ['topic', 'list', '--no-daemon', '--spin-time', '2']),
    (['topic', 'list', '--spin-time', '3', '-t'],
     ['topic', 'list', '--no-daemon', '--spin-time', '2', '--spin-time', '3', '-t']),
    (['topic', 'echo', '/scan', '--once', '--qos-reliability', 'best_effort'],
     ['topic', 'echo', '--no-daemon', '--spin-time', '2', '/scan', '--once', '--qos-reliability', 'best_effort']),
    (['topic', 'info', '/scan'], ['topic', 'info', '--no-daemon', '--spin-time', '2', '/scan']),
    (['topic', 'type', '/scan'], ['topic', 'type', '--no-daemon', '--spin-time', '2', '/scan']),
    (['topic', 'find', 'sensor_msgs/msg/LaserScan'],
     ['topic', 'find', '--no-daemon', '--spin-time', '2', 'sensor_msgs/msg/LaserScan']),
    (['run', 'autorace_parking', 'parking_node'], ['run', 'autorace_parking', 'parking_node']),
    (['launch', 'autorace_parking', 'parking.launch.py', 'rviz:=true'],
     ['launch', 'autorace_parking', 'parking.launch.py', 'rviz:=true']),
    (['topic', 'pub', '/test', 'std_msgs/msg/Bool', '{data: true}'],
     ['topic', 'pub', '/test', 'std_msgs/msg/Bool', '{data: true}']),
    (['daemon', 'stop'], ['daemon', 'stop']),
])
def test_plain_ros2_queries_bypass_daemon_and_preserve_other_commands(arguments, expected):
    environment = os.environ.copy()
    environment['ROS_DISTRO'] = 'humble'
    script = Path(__file__).resolve().parents[3] / 'ros_env.sh'
    # Replace only the final CLI call; use the actual environment and dispatcher.
    shell = '''
source "$1" || exit 1
shift
function /opt/ros/humble/bin/ros2() {
    /usr/bin/python3 -c 'import json, sys; print(json.dumps(sys.argv[1:]))' "$@"
    return 7
}
cd /tmp || exit 1
ros2 "$@"
'''
    result = subprocess.run(['/bin/bash', '--noprofile', '--norc', '-c', shell,
                             'test', str(script), *arguments],
                            env=environment, text=True, capture_output=True)
    assert result.returncode == 7, result.stderr
    assert json.loads(result.stdout) == expected

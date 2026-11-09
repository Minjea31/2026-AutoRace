import importlib.util
import os
from pathlib import Path


def test_sensor_launch_refuses_a_port_held_by_another_driver(tmp_path):
    path=Path(__file__).resolve().parents[1]/'launch/sensors.launch.py'
    spec=importlib.util.spec_from_file_location('sensor_launch_port_test',path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    port=tmp_path/'serial'
    port.write_bytes(b'')
    alias=tmp_path/'ydlidar'
    alias.symlink_to(port)
    with port.open('rb'):
        assert os.getpid() in module.serial_port_owners([alias])[str(alias)]
    assert not module.serial_port_owners([alias])

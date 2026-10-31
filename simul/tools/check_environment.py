#!/usr/bin/env python3
"""Read-only diagnosis; never installs or changes Conda/system packages."""
import importlib
import os
import sys

print('Python:',sys.executable,sys.version.split()[0])
print('Conda:',os.environ.get('CONDA_PREFIX','not active'))
print('ROS domain:',os.environ.get('ROS_DOMAIN_ID','0'))
good = sys.version_info[:2] == (3,10)
for name in ('numpy','yaml','PIL','panda3d.core','rclpy','sensor_msgs.msg','tf2_ros'):
    try:
        module = importlib.import_module(name)
        print(name,'OK',getattr(module,'__version__',''))
    except (ImportError,OSError) as exc:
        good = False
        print(name,'FAILED:',exc)
raise SystemExit(0 if good else 1)

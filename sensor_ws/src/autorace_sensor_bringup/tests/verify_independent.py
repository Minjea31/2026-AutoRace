"""Audit package and library resolution without starting sensor hardware."""
from pathlib import Path
import ctypes
import os
import subprocess
from ament_index_python.packages import get_package_prefix

root=Path(__file__).resolve().parents[3]
packages=['autorace_sensor_bringup','limo_base','limo_msgs','limo_description','ydlidar_ros2_driver','orbbec_camera','orbbec_camera_msgs','robot_localization']
for name in packages:
    prefix=Path(get_package_prefix(name))
    assert prefix==root/'install'/name,(name,prefix)
    print(name, prefix)
for key in ('AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH','LD_LIBRARY_PATH','PYTHONPATH'):
    assert '/home/wego/wego_ws' not in os.environ.get(key,''),key
for path in [root/'install/limo_base/lib/limo_base/limo_base',root/'install/ydlidar_ros2_driver/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node',root/'install/orbbec_camera/lib/liborbbec_camera.so',root/'install/robot_localization/lib/robot_localization/ekf_node']:
    result=subprocess.run(['ldd',str(path)],check=True,text=True,capture_output=True).stdout
    assert 'not found' not in result,(path,result)
    assert '/home/wego/wego_ws' not in result,(path,result)
ctypes.CDLL(str(root/'install/orbbec_camera/lib/liborbbec_camera.so'))
assert '/home/wego/wego_ws' not in (root/'install/setup.bash').read_text()
print('PASS: all 8 package prefixes local; executable/shared-library dependencies resolve without wego_ws; Orbbec component library loads without starting a camera.')

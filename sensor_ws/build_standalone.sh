#!/usr/bin/env bash
# Build from the system ROS underlay, excluding inherited wego_ws/Conda paths.
set -euo pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec env -i HOME="$HOME" USER="${USER:-wego}" LANG=C.UTF-8 \
  PATH=/usr/bin:/bin:/usr/sbin:/sbin CMAKE_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 \
  /bin/bash --noprofile --norc -c '
    set -e
    source /opt/ros/humble/setup.bash
    cd -- "$1"
    colcon build --packages-up-to autorace_sensor_bringup --symlink-install \
      --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3 \
      -DPYTHON_EXECUTABLE=/usr/bin/python3
    # Match the existing wego_ws LiDAR build (no Release optimization).
    exec colcon build --packages-select ydlidar_ros2_driver --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE:STRING= -DBUILD_TESTING=OFF
  ' build-sensors "$workspace"

#!/usr/bin/env bash
# 단독 빌드(conda 비활성 깨끗한 환경, 시스템 python3). ~/.local numpy 2.x 가림 방지.
cd "$(dirname -- "${BASH_SOURCE[0]}")/.."
exec env -i HOME="$HOME" PATH=/usr/bin:/bin PYTHONNOUSERSITE=1 bash -c \
  'source /opt/ros/humble/setup.bash && colcon build --symlink-install'

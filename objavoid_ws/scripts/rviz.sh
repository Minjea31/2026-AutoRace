#!/usr/bin/env bash
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
exec rviz2 -d "$(ros2 pkg prefix autorace_objavoid_v3)/share/autorace_objavoid_v3/rviz/objavoid_v3.rviz" \
  --ros-args -p use_sim_time:=true

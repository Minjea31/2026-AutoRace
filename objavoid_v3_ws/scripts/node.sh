#!/usr/bin/env bash
# 회피 노드. 인자는 launch 인자: v_max:=0.3 rviz:=true auto_activate:=true command_topic:=...
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
exec ros2 launch autorace_objavoid_v3 objavoid_v3.launch.py "$@"

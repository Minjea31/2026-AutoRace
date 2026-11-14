#!/usr/bin/env bash
# 수동 활성화: ./scripts/activate.sh  / 중지: ./scripts/activate.sh stop
# 다시 시험: 시뮬 창 F5(시작 위치 복귀) 후 ./scripts/activate.sh
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh" >/dev/null
v=true; [[ "${1:-}" == stop ]] && v=false
exec ros2 topic pub --once /objavoid_v3/activate std_msgs/msg/Bool "{data: $v}"

#!/usr/bin/env bash
# 시뮬 어댑터를 V3 전용 토픽에 직접 연결(MUX·다른 어댑터·기존 objavoid_node와 동시 실행 금지).
# 실행 전 같은 도메인에 기존 장애물 노드가 없는지 확인한다.
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
export PYTHONNOUSERSITE=1
source "$ROOT/simul/env.sh" >/dev/null || exit 1
nodes="$(/opt/ros/humble/bin/ros2 node list --no-daemon --spin-time 2 2>/dev/null)"  # daemon 캐시(종료된 노드) 무시
if grep -qE "objavoid_node|objavoid_test_node" <<<"$nodes"; then
  echo "기존 장애물 노드가 실행 중입니다(도메인 $ROS_DOMAIN_ID):"; grep -E "objavoid" <<<"$nodes"; exit 1
fi
exec "$ROOT/simul/adapter.sh" --no-gui --topic /objavoid_v3/cmd "$@"

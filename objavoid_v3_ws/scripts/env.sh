#!/usr/bin/env bash
# 회피 노드·RViz용 환경(시스템 ROS + 이 워크스페이스). source scripts/env.sh
# 도메인: 미설정/0이면 시뮬 규칙대로 42, ROS_LOCALHOST_ONLY=1
_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
[[ -f "$_ws/install/setup.bash" ]] && source "$_ws/install/setup.bash"
if [[ -z "${ROS_DOMAIN_ID:-}" || "$ROS_DOMAIN_ID" == 0 ]]; then export ROS_DOMAIN_ID=42; fi
export ROS_LOCALHOST_ONLY=1
export PYTHONNOUSERSITE=1  # ~/.local numpy 2.x가 시스템 numpy를 가리는 문제 방지
echo "[objavoid_v3] ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
unset _ws

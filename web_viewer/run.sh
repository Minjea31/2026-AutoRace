#!/usr/bin/env bash
set -eo pipefail
task_viewer_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONNOUSERSITE=1
# Explicit demo needs only the system Python standard library, no ROS discovery.
task_demo=false
for task_arg in "$@"; do
  if [[ "$task_arg" == --demo ]]; then task_demo=true; fi
done
if [[ "$task_demo" != true ]]; then
  source /opt/ros/humble/setup.bash
fi
export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=0  # 대회 본선: 도메인 99, localhost 전용 해제
export MISSION_DOMAIN_SOURCE=environment
echo "[mission-web] ROS_DOMAIN_ID=$ROS_DOMAIN_ID source=$MISSION_DOMAIN_SOURCE ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
exec nice -n 10 /usr/bin/python3 "$task_viewer_root/mission_server.py" --host "${HOST:-0.0.0.0}" --port "${PORT:-8080}" "$@"

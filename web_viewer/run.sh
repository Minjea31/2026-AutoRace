#!/usr/bin/env bash
set -eo pipefail
task_viewer_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-96}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONNOUSERSITE=1
exec nice -n 10 /usr/bin/python3 "$task_viewer_root/server.py" --port "${PORT:-8080}" --max-fps "${FPS:-10}" "$@"

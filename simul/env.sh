#!/usr/bin/env bash
# Source this file AFTER `conda activate autorace-simul` in every ROS terminal.
if [[ -z "${CONDA_PREFIX:-}" ]]; then
  echo '먼저 conda activate autorace-simul 을 실행하세요.' >&2
  return 1 2>/dev/null || exit 1
fi
if ! "$CONDA_PREFIX/bin/python" -c 'import sys; assert sys.version_info[:2] == (3, 10)' 2>/dev/null; then
  echo 'ROS 2 Humble 연동에는 Conda Python 3.10 환경이 필요합니다.' >&2
  return 1 2>/dev/null || exit 1
fi
_autorace_sim_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_autorace_sim_root/ros_env.sh" || { return 1 2>/dev/null || exit 1; }
export PYTHONPATH="$_autorace_sim_root/src/limo_parking_sim${PYTHONPATH:+:$PYTHONPATH}"
unset _autorace_sim_root

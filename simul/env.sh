#!/usr/bin/env bash
# Usage: source simul/env.sh (persists activation in the current terminal).
# Direct execution prepares the environment, but cannot change its parent shell.
_autorace_sim_packages_ready() {
  "$CONDA_PREFIX/bin/python" - "$1" <<'PY'
import sys
from importlib.metadata import version

assert sys.version_info[:2] == (3, 10)
import numpy, yaml, PIL, tkinter, panda3d.core, pytest, pip

assert numpy.__version__.split('.')[:2] == ['1', '26']
assert version('panda3d') == '1.10.16'
assert int(version('setuptools').split('.')[0]) < 81
sys.path.insert(0, sys.argv[1] + '/src/limo_parking_sim')
from limo_parking_sim.qt_support import load_qt
load_qt()
PY
}

_autorace_sim_setup() {
  local sim_root conda_exe conda_base conda_hook candidate env_prefix
  sim_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)" || return 1

  # Conda and system ROS are prerequisites; Python dependencies live in Conda.
  if [[ ! -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]]; then
    echo "ROS 2 환경 파일이 없습니다: /opt/ros/${ROS_DISTRO:-humble}/setup.bash" >&2
    echo "시스템 ROS 설치 방법: $sim_root/docs/SETUP.md" >&2
    return 1
  fi

  conda_exe="${CONDA_EXE:-$(type -P conda)}"
  if [[ ! -x "$conda_exe" ]]; then
    for candidate in "$HOME"/{anaconda3,miniconda3,miniforge3,mambaforge}/bin/conda; do
      if [[ -x "$candidate" ]]; then
        conda_exe="$candidate"
        break
      fi
    done
  fi
  if [[ ! -x "$conda_exe" ]]; then
    echo 'Conda를 찾지 못했습니다. Conda를 설치하거나 CONDA_EXE에 실행 파일 경로를 지정하세요.' >&2
    return 1
  fi

  conda_base="$("$conda_exe" info --base)" || return 1
  if [[ -f "$conda_base/etc/profile.d/conda.sh" ]]; then
    source "$conda_base/etc/profile.d/conda.sh" || return 1
  else
    conda_hook="$("$conda_exe" shell.bash hook)" || return 1
    eval "$conda_hook" || return 1
  fi

  # Query first so an activation error never triggers creation over an existing env.
  env_prefix="$(conda env list --json | "$conda_base/bin/python" -c '
import json, os, sys
for prefix in json.load(sys.stdin)["envs"]:
    if os.path.basename(prefix) == "autorace-simul":
        print(prefix)
        break
')" || return 1
  if [[ -z "$env_prefix" ]]; then
    echo '[autorace-simul] 환경 생성 및 필수 패키지 설치 (최초 실행에는 시간이 걸립니다).'
    conda env create -y -n autorace-simul -f "$sim_root/environment.yml" || return 1
  fi
  conda activate autorace-simul || return 1

  if ! _autorace_sim_packages_ready "$sim_root" >/dev/null 2>&1; then
    echo '[autorace-simul] 누락되거나 호환되지 않는 필수 패키지를 보완합니다.'
    conda env update -n autorace-simul -f "$sim_root/environment.yml" || return 1
    conda activate autorace-simul || return 1
    if ! _autorace_sim_packages_ready "$sim_root"; then
      echo '[autorace-simul] 패키지 검사 실패. 위 오류를 확인하고 다시 실행하세요.' >&2
      return 1
    fi
  fi

  source "$sim_root/ros_env.sh" || return 1
  # Avoid accumulating the source path when sourced repeatedly.
  case ":${PYTHONPATH:-}:" in
    *":$sim_root/src/limo_parking_sim:"*) ;;
    *) export PYTHONPATH="$sim_root/src/limo_parking_sim${PYTHONPATH:+:$PYTHONPATH}" ;;
  esac
  if ! "$CONDA_PREFIX/bin/python" -c \
      'import rclpy, sensor_msgs.msg, geometry_msgs.msg, nav_msgs.msg, rosgraph_msgs.msg, std_msgs.msg, std_srvs.srv, tf2_ros'; then
    echo "ROS 모듈 연결 실패. 시스템 ROS 패키지를 확인하세요: $sim_root/docs/SETUP.md" >&2
    return 1
  fi
  echo "[autorace-simul] 준비 완료: Python 3.10 / ROS_DOMAIN_ID=$ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
}

if _autorace_sim_setup; then
  unset -f _autorace_sim_setup _autorace_sim_packages_ready
else
  unset -f _autorace_sim_setup _autorace_sim_packages_ready
  return 1 2>/dev/null || exit 1
fi

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo '직접 실행은 환경 준비만 수행합니다. 현재 터미널에 적용하려면 다음을 실행하세요:'
  printf '  source %q\n' "${BASH_SOURCE[0]}"
fi

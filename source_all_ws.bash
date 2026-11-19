#!/usr/bin/env bash
# Load only workspaces under /home/wego/2026-AutoRace into the current shell.
# Usage: source ~/2026-AutoRace/source_all_ws.bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo '실행 방법: source ~/2026-AutoRace/source_all_ws.bash' >&2
  exit 1
fi

_autorace_source_selected_ws() {
  local workspace setup failed=0

  # Load ROS once, then only each selected workspace (skip recorded underlays).
  if ! source /opt/ros/humble/setup.bash; then
    return 1
  fi
  local -a workspaces=(
    '2026-AutoRace/cone_ws'
    '2026-AutoRace/crosswalk_ws'
    '2026-AutoRace/lane_fusion_ws'
    '2026-AutoRace/lane_ws'
    '2026-AutoRace/mux_ws'
    '2026-AutoRace/objavoid_ws'
    '2026-AutoRace/parking_ws'
    '2026-AutoRace/sensor_ws'
    '2026-AutoRace/signal_ws'
    '2026-AutoRace/slow_ws'
    '2026-AutoRace/tensor_bringup'
    '2026-AutoRace/yolo_ws'
  )

  for workspace in "${workspaces[@]}"; do
    setup="$HOME/$workspace/install/local_setup.bash"
    if [[ ! -f "$setup" ]]; then
      printf '파일 없음: %s\n' "$setup" >&2
      failed=1
      continue
    fi
    printf '소싱: %s\n' "$setup"
    if ! source "$setup"; then
      printf '소싱 실패: %s\n' "$setup" >&2
      failed=1
    fi
  done
  return "$failed"
}

if _autorace_source_selected_ws; then
  unset -f _autorace_source_selected_ws
else
  unset -f _autorace_source_selected_ws
  return 1
fi


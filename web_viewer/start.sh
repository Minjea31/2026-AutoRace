#!/usr/bin/env bash
set -eo pipefail
task_viewer_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$task_viewer_root/.run"
task_pidfile="$task_viewer_root/.run/viewer.pid"
if [[ -f "$task_pidfile" ]]; then
  task_viewer_pid=$(cat "$task_pidfile")
  if [[ "$task_viewer_pid" =~ ^[0-9]+$ ]] && kill -0 "$task_viewer_pid" 2>/dev/null && [[ "$(tr '\0' ' ' < "/proc/$task_viewer_pid/cmdline")" == *"$task_viewer_root/mission_server.py"* ]]; then
    echo "Web viewer already running (PID $task_viewer_pid)."
    exit 0
  fi
fi
nohup "$task_viewer_root/run.sh" "$@" > "$task_viewer_root/.run/viewer.log" 2>&1 < /dev/null &
task_viewer_pid=$!
echo "$task_viewer_pid" > "$task_pidfile"
sleep 1
if ! kill -0 "$task_viewer_pid" 2>/dev/null; then
  cat "$task_viewer_root/.run/viewer.log"
  exit 1
fi
echo "Web viewer started (PID $task_viewer_pid, port ${PORT:-8080})."
echo "Log: $task_viewer_root/.run/viewer.log"

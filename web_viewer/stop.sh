#!/usr/bin/env bash
set -eo pipefail
task_viewer_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_pidfile="$task_viewer_root/.run/viewer.pid"
if [[ ! -f "$task_pidfile" ]]; then echo 'Web viewer is not running.'; exit 0; fi
task_viewer_pid=$(cat "$task_pidfile")
if [[ "$task_viewer_pid" =~ ^[0-9]+$ ]] && kill -0 "$task_viewer_pid" 2>/dev/null; then
  if [[ "$(tr '\0' ' ' < "/proc/$task_viewer_pid/cmdline")" != *"$task_viewer_root/mission_server.py"* ]]; then
    echo 'PID belongs to another process; it was not stopped.' >&2
    exit 1
  fi
  kill -TERM "$task_viewer_pid"
  for task_wait in {1..40}; do
    if ! kill -0 "$task_viewer_pid" 2>/dev/null; then break; fi
    sleep .1
  done
  echo "Web viewer stopped (PID $task_viewer_pid)."
else
  echo 'Web viewer was already stopped.'
fi
rm -f "$task_pidfile"

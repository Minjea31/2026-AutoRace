#!/usr/bin/env bash
# Launch the terminal GPU dashboard from any working directory.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$script_dir/gpu_monitor.py" "$@"

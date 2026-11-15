#!/usr/bin/env bash
# Per-node CPU/memory load on the car (options: tools/node_load.py --help).
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$script_dir/tools/node_load.py" "$@"

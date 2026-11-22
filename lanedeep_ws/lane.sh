#!/usr/bin/env bash
# Entry point for the current production deep lane stack.
set -eo pipefail
autorace_lane_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$autorace_lane_root/lane_fusion_ws/deep.sh" "$@"

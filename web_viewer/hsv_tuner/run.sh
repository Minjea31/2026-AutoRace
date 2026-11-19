#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
exec /usr/bin/python3 "$(dirname "$(realpath "$0")")/server.py" "$@"

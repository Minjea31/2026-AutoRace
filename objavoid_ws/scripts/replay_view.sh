#!/usr/bin/env bash
# Looping bag + lane model + NEW C++ planner + read-only viewer.
set -eo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
bag="${1:-$HOME/AutoRace_bag/rosbag2_2026_11_16-08_49_37}"
[[ -f "$bag/metadata.yaml" ]] || { echo "Bag metadata not found: $bag" >&2; exit 1; }
source /opt/ros/humble/setup.bash
[[ -f "$repo/objavoid_ws/install/local_setup.bash" ]] || { echo 'Build objavoid_ws first: bash scripts/build.sh' >&2; exit 1; }
source "$repo/objavoid_ws/install/local_setup.bash"
export ROS_DOMAIN_ID="${REPLAY_DOMAIN:-109}" ROS_LOCALHOST_ONLY=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export PYTHONPATH="$repo/lane_fusion_ws/src/autorace_lane_fusion_deep${PYTHONPATH:+:$PYTHONPATH}"
py="$repo/objavoid_ws/.replay-venv/bin/python3"
[[ -x "$py" ]] || { echo 'Replay Python environment is missing' >&2; exit 1; }
"$py" -c 'import onnxruntime, rclpy' || exit 1
port="${PORT:-8086}"
# Fail before creating children if another viewer already owns the port.
/usr/bin/python3 - "$port" <<'CHECK'
import socket,sys
with socket.socket() as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('127.0.0.1',int(sys.argv[1])))
CHECK
logdir="${REPLAY_LOG_DIR:-$repo/objavoid_ws/.replay}"
mkdir -p "$logdir" "$repo/objavoid_ws/.build-cache"
export TMPDIR="$repo/objavoid_ws/.build-cache"
pids=()
terminal_state=""
if [[ -t 0 ]]; then terminal_state=$(stty -g); fi
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$terminal_state" ]]; then stty "$terminal_state" </dev/tty 2>/dev/null || true; fi
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  for attempt in {1..30}; do
    alive=0
    for pid in "${pids[@]}"; do kill -0 "$pid" 2>/dev/null && alive=1; done
    [[ "$alive" == 0 ]] && break
    sleep 0.1
  done
  for pid in "${pids[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
setsid "$py" -u -c 'from autorace_lane_fusion_deep.mask_node import main; main()' \
  --ros-args -p use_sim_time:=true -p device:=cpu -p max_rate:=10.0 \
  -p mask_topic:=/objavoid_eval/deep_mask -p model_path:="$repo/lane_fusion_ws/src/autorace_lane_fusion_deep/model/lane_640_mask.onnx" >"$logdir/mask.log" 2>&1 &
pids+=("$!")
setsid ros2 launch autorace_objavoid_v3 objavoid_v3.launch.py profile:=real car:=04 \
  use_sim_time:=true auto_activate:="${REPLAY_ACTIVATE:-false}" start_legacy:="${REPLAY_LEGACY_START:-true}" command_topic:=/objavoid_eval/command output_prefix:=/objavoid_eval lane_mask_topic:=/objavoid_eval/deep_mask \
  >"$logdir/planner.log" 2>&1 &
pids+=("$!")
viewer_remaps=()
for output in markers state path done active has_control entrance_armed; do
  viewer_remaps+=(-r "/objavoid_v3/$output:=/objavoid_eval/$output")
done
viewer_remaps+=(-r "/mux/active_source:=/objavoid_eval/mux_source")
setsid /usr/bin/python3 "$repo/objavoid_ws/web_viewer.py" --host 127.0.0.1 --port "$port" --read-only --ros-args "${viewer_remaps[@]}" \
  >"$logdir/viewer.log" 2>&1 &
pids+=("$!")
echo "NEW planner viewer: http://localhost:$port/"
echo "Logs: $logdir/{mask,planner,viewer}.log"
echo "Bag: $bag (ROS_DOMAIN_ID=$ROS_DOMAIN_ID, loop=${REPLAY_LOOP:-true})"
echo 'Only regenerated /objavoid_eval outputs are displayed.'
if [[ -t 0 ]]; then echo 'SPACE: pause/resume bag | Ctrl+C: stop this session'; fi
sleep 3
bag_args=("$bag" --clock --rate "${RATE:-0.5}" --topics
  /camera/color/image_raw/compressed /scan /odom /imu
  /yolo/LR_sign /yolo/LR_detail /mission/lane)
if [[ "${REPLAY_LOOP:-true}" == true ]]; then bag_args+=(--loop); fi
if [[ "${REPLAY_START_PAUSED:-false}" == true ]]; then bag_args+=(--start-paused); fi
if [[ -t 0 ]]; then
  # Explicitly retain terminal input: ordinary background jobs otherwise receive /dev/null.
  setsid ros2 bag play "${bag_args[@]}" </dev/tty >"$logdir/bag.log" 2>&1 &
else
  setsid ros2 bag play "${bag_args[@]}" --disable-keyboard-controls >"$logdir/bag.log" 2>&1 &
fi
pids+=("$!")
wait -n "${pids[@]}"

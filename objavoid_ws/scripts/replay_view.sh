#!/usr/bin/env bash
# Looping bag + lane model + NEW C++ planner + read-only viewer.
set -eo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
bag="${1:-$HOME/AutoRace_bag/rosbag2_2026_11_16-08_49_37}"
[[ -f "$bag/metadata.yaml" ]] || { echo "Bag metadata not found: $bag" >&2; exit 1; }
source /opt/ros/humble/setup.bash
[[ -f "$repo/objavoid_ws/install/local_setup.bash" ]] || { echo 'Build objavoid_ws first: bash scripts/build.sh' >&2; exit 1; }
source "$repo/source_all_ws.bash"
export ROS_DOMAIN_ID="${REPLAY_DOMAIN:-109}" ROS_LOCALHOST_ONLY=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export PYTHONPATH="$repo/lane_fusion_ws/src/autorace_lane_fusion_deep${PYTHONPATH:+:$PYTHONPATH}"
model="${REPLAY_MODEL:-$repo/lane_fusion_ws/src/autorace_lane_fusion_deep/model/lane_640_mask.engine}"
[[ -f "$model" ]] || model="$repo/lane_fusion_ws/src/autorace_lane_fusion_deep/model/lane_640_mask.onnx"
[[ -f "$model" ]] || { echo "Lane model not found: $model" >&2; exit 1; }
# Use the installed executable's interpreter (the real-car TensorRT environment),
# rather than assuming a separate .replay-venv exists on the vehicle.
ros2 pkg prefix autorace_lane_fusion_deep >/dev/null
mask_pythonpath="$PYTHONPATH"
mask_librarypath="/usr/local/cuda-12.6/lib64:${LD_LIBRARY_PATH:-}"
if [[ "$model" == *.engine || "$model" == *.plan ]]; then
  trt_site="${REPLAY_TRT_PYTHONPATH:-$HOME/miniconda3/envs/dl_env/lib/python3.10/site-packages}"
  if [[ -d "$trt_site" ]]; then mask_pythonpath="$trt_site:$mask_pythonpath"; fi
  env PYTHONPATH="$mask_pythonpath" LD_LIBRARY_PATH="$mask_librarypath" /usr/bin/python3 -c 'import tensorrt,rclpy'
fi
port="${PORT:-8086}"
viewer_host="${REPLAY_HOST:-0.0.0.0}"
# Fail before creating children if another viewer already owns the port.
/usr/bin/python3 - "$viewer_host" "$port" <<'CHECK'
import socket,sys
with socket.socket() as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((sys.argv[1],int(sys.argv[2])))
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
setsid env PYTHONPATH="$mask_pythonpath" LD_LIBRARY_PATH="$mask_librarypath" ros2 run autorace_lane_fusion_deep lane_fusion_deep_mask \
  --ros-args -p use_sim_time:=true -p device:=auto -p max_rate:=10.0 \
  -p mask_topic:=/objavoid_eval/deep_mask -p model_path:="$model" >"$logdir/mask.log" 2>&1 &
pids+=("$!")
planner_args=()
if [[ -n "${REPLAY_LEGACY_START:-}" ]]; then
  planner_args+=("start_legacy:=$REPLAY_LEGACY_START")
fi
setsid ros2 launch autorace_objavoid_v3 objavoid_v3.launch.py profile:=real car:=04 \
  use_sim_time:=true auto_activate:="${REPLAY_ACTIVATE:-false}" "${planner_args[@]}" command_topic:=/objavoid_eval/command output_prefix:=/objavoid_eval lane_mask_topic:=/objavoid_eval/deep_mask \
  >"$logdir/planner.log" 2>&1 &
pids+=("$!")
viewer_remaps=()
for output in markers state path done active has_control entrance_armed; do
  viewer_remaps+=(-r "/objavoid_v3/$output:=/objavoid_eval/$output")
done
viewer_remaps+=(-r "/mux/active_source:=/objavoid_eval/mux_source")
setsid /usr/bin/python3 "$repo/objavoid_ws/web_viewer.py" --host "$viewer_host" --port "$port" --read-only --ros-args "${viewer_remaps[@]}" \
  >"$logdir/viewer.log" 2>&1 &
pids+=("$!")
echo "NEW planner viewer: http://$(hostname -I | awk '{print $1}'):$port/ (bind=$viewer_host)"
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

#!/usr/bin/env bash
# 퓨전 차선의 딥 마스크 노드(/lane_fusion/deep_mask)를 lane_fusion_ws 소스 그대로 실행(빌드·수정 없음).
# V3는 이 마스크의 노란 클래스 + 같은 프레임 원본 HSV로 노란 실선을 쓴다. 실차는 tensor_bringup/퓨전이 이미 띄움.
# onnxruntime이 ~/.local에 있으므로 PYTHONNOUSERSITE를 설정하지 않는다.
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
DEEP="$ROOT/lane_fusion_ws/src/autorace_lane_fusion_deep"
source /opt/ros/humble/setup.bash
if [[ -z "${ROS_DOMAIN_ID:-}" || "$ROS_DOMAIN_ID" == 0 ]]; then export ROS_DOMAIN_ID=42; fi
export ROS_LOCALHOST_ONLY=1 PYTHONPATH="$DEEP${PYTHONPATH:+:$PYTHONPATH}"
unset PYTHONNOUSERSITE
exec python3 -u -c 'from autorace_lane_fusion_deep.mask_node import main; main()' \
  --ros-args -p use_sim_time:=true -p model_path:="${MODEL:-$DEEP/model/lane_640_mask.onnx}" "$@"

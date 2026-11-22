#!/usr/bin/env bash
set -euo pipefail
# Compressed images plus the actual parking inputs/outputs; do not record raw RGB/deep masks.
ros2 bag record -o "${1:-parking_review_$(date +%Y%m%d_%H%M%S)}" \
 /camera/color/image_raw/compressed /parking/floor_observation /scan /odom /imu \
 /odometry/filtered /localization/health /limo_status /yolo/parking_sign \
 /parking/control_state /parking/status /parking/path /parking/localization/odometry \
 /parking/slot_map /parking/obstacle_memory /parking/active /parking/lane_handoff \
 /parking/approach_start /parking/approach_ready /lane_deep/target \
 /mission/parking/command_03 /mission/lane /cmd_vel /rosout

#!/usr/bin/env bash
# Topic lists checked against src + real launch/config on 2026-11-18.
# Add topics here when interfaces change. all is the union, never ros2 record -a.
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat <<'EOF'
사용법: ./tools/record_bag.sh MODE [MODE ...] [옵션] [-- ros2 bag record 옵션]

모드 (여러 개 지정 가능):
  all                 아래 모든 모드의 토픽 합집합 (코드에 쓰이는 토픽만)
  lane, 차선          HSV / deep / fusion 차선 입력, target, PID, debug
  obstacle, 장애물    objavoid_v3 입력, 상태, 경로, 제어, 갈림길 인지
  cone, 라바콘        LiDAR, 클러스터, 경계, 제어 경로
  parking, 주차       주차 입력, 상태, 위치추정, 경로, 인계
  crosswalk, 횡단보도 YOLO 표지판, 흰색 ROI, 정지 명령
  signal, 신호등      신호 인지, 흰색 ROI, 정지 명령
  slow, 서행          방지턱/서행 표지판, 흰색 ROI, 미션 상태
  yolo                영상 및 모든 YOLO 인지 결과
  sensor, 센서        영상, LiDAR, odom, IMU, 위치추정, 차량 상태

공통: /mission/lane, /cmd_vel, /mux/active_source, TF, /clock, /rosout
옵션:
  -o, --output DIR  저장 경로 (기본: 저장소/bags/MODE_날짜_시간)
  --list           선택한 토픽만 출력하고 종료 (ROS 환경 불필요)
  --dry-run        녹화 명령만 출력하고 종료 (폴더 생성 안 함)
  -h, --help       도움말

예:
  ./tools/record_bag.sh all
  ./tools/record_bag.sh lane
  ./tools/record_bag.sh 장애물
  ./tools/record_bag.sh lane obstacle -o ~/bags/lane_obstacle_test
  ./tools/record_bag.sh obstacle --list
  ./tools/record_bag.sh all -- --max-bag-duration 60

센서/미션은 별도로 실행하세요. 녹화는 Ctrl+C로 종료합니다.
발행이 늦게 시작되는 선택 토픽도 녹화 중 계속 탐색합니다.
EOF
}

die() { printf '오류: %s\n' "$*" >&2; exit 2; }
declare -a modes=() topics=() extra=()
declare -A selected=() seen=()
output='' list=false dry_run=false

while (($#)); do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    -o|--output)
      (($# >= 2)) && [[ -n "$2" && "$2" != -* ]] || die '--output 뒤에 저장 경로를 지정하세요.'
      output="$2"; shift 2; continue ;;
    --list) list=true ;;
    --dry-run) dry_run=true ;;
    --) shift; extra=("$@"); break ;;
    -*) die "알 수 없는 옵션: $1 (rosbag 옵션은 -- 뒤에 지정)" ;;
    *)
      case "$1" in
        all|전체) mode=all ;;
        lane|차선) mode=lane ;;
        obstacle|objavoid|obj|장애물) mode=obstacle ;;
        cone|라바콘) mode=cone ;;
        parking|주차) mode=parking ;;
        crosswalk|횡단보도) mode=crosswalk ;;
        signal|신호등) mode=signal ;;
        slow|서행|방지턱) mode=slow ;;
        yolo) mode=yolo ;;
        sensor|sensors|센서) mode=sensor ;;
        *) die "알 수 없는 모드: $1 (--help 참고)" ;;
      esac
      if [[ -z "${selected[$mode]:-}" ]]; then modes+=("$mode"); selected[$mode]=1; fi ;;
  esac
  shift
done
if ((${#modes[@]} == 0)); then usage; exit 2; fi

add() {
  local topic
  for topic in "$@"; do
    if [[ -z "${seen[$topic]:-}" ]]; then topics+=("$topic"); seen[$topic]=1; fi
  done
}
camera() { add /camera/color/image_raw/compressed; }
motion() { add /odom /imu /odometry/filtered; }
localization() { motion; add /localization/wheel /localization/imu /localization/health /limo_status; }
parking_handoff() {
  add /yolo/parking_sign /parking_start /parking/ready /parking/active \
    /parking/approach_start /parking/approach_ready /parking/control_state /parking/lane_handoff
}

profile() {
  local name
  case "$1" in
    all)
      for name in lane obstacle cone parking crosswalk signal slow yolo sensor; do profile "$name"; done ;;
    lane)
      # lane_ws + all three lane_fusion_ws modes and their PID controllers.
      camera; motion; parking_handoff
      add /cone_lane/exit_lane_active \
        /lane_lite/target /lane_lite/debug /lane_pid/command /lane_pid/state \
        /lane_deep/target /lane_deep/debug /lane_fusion/deep_mask \
        /lane_fusion/target /lane_fusion/debug /lane_fusion/yellow_boundary \
        /lane_fusion_pid/command /lane_fusion_pid/state ;;
    obstacle)
      # tensor_bringup uses command_05; standalone V3 uses /objavoid_v3/cmd.
      camera; motion
      add /scan /lane_fusion/deep_mask /yolo/LR_sign /yolo/LR_detail \
        /slow/mission_active /mission/slow/command_04 \
        /objavoid_v3/activate /objavoid_v3/remap /objavoid_v3/active \
        /objavoid_v3/entrance_armed /objavoid_v3/has_control /objavoid_v3/state \
        /objavoid_v3/done /objavoid_v3/markers /objavoid_v3/path /objavoid_v3/cmd \
        /mission/objavoid/command_05 ;;
    cone)
      # cone_lane_follower/cpp/roi_node.cpp (TF supplies motion compensation).
      motion
      add /scan /cone_lane/roi_marker /cone_lane/roi_points /cone_lane/clusters \
        /cone_lane/cone_centers /cone_lane/boundaries /cone_lane/control_points \
        /cone_lane/center_path /cone_lane/center_spot /cone_lane/backup_path \
        /cone_lane/control_status /cone_lane/exit_lane_active /mission/cone/command_06 ;;
    parking)
      # node.py, start_sequence.py, floor_gate.py, robot_visual.py + real config.
      camera; localization; parking_handoff
      add /scan /lane_fusion/target /lane_fusion/yellow_boundary \
        /parking/status /parking/path /parking/localization/odometry \
        /parking/approach_odometry /parking/slot_map /parking/obstacle_memory \
        /parking/obstacles /parking/target /parking/candidate_roi /parking/ldf \
        /parking/lookahead /parking/robot_description /mission/parking/command_03 ;;
    crosswalk)
      camera
      add /yolo/crosswalk_sign /start_crosswalk /mission/crosswalk/command_01 \
        /crosswalk/white_roi/debug/compressed /crosswalk/white_roi/ratio /crosswalk/white_roi/detected ;;
    signal)
      camera
      add /yolo/signal /start_signal /mission/signal/command_02 \
        /signal/white_roi/debug/compressed /signal/white_roi/ratio /signal/white_roi/detected ;;
    slow)
      camera
      add /yolo/bump_sign /yolo/slow_sign /start_crosswalk /start_signal \
        /slow/mission_active /mission/slow/command_04 \
        /slow/white_roi/debug/compressed /slow/white_roi/ratio /slow/white_roi/detected ;;
    yolo)
      camera
      add /yolo/LR_sign /yolo/LR_detail /yolo/bump_sign /yolo/slow_sign \
        /yolo/crosswalk_sign /yolo/parking_sign /yolo/signal /yolo/debug/compressed ;;
    sensor)
      camera; localization
      # EKF also subscribes to the PoseWithCovarianceStamped set_pose topic.
      add /scan /joint_states /robot_description /diagnostics /set_pose ;;
  esac
}

add /mission/lane /cmd_vel /mux/active_source /tf /tf_static /clock /rosout
if [[ -n "${selected[all]:-}" ]]; then modes=(all); fi
for mode in "${modes[@]}"; do profile "$mode"; done

if $list; then printf '%s\n' "${topics[@]}"; exit 0; fi
label="$(IFS=_; printf '%s' "${modes[*]}")"
output="${output:-$ROOT/bags/${label}_$(date +%Y%m%d_%H%M%S_%N)}"
cmd=(ros2 bag record -o "$output" "${extra[@]}" "${topics[@]}")
printf '모드: %s | 선택 토픽: %d개\n저장: %s\n' "${modes[*]}" "${#topics[@]}" "$output"
printf '명령: '; printf '%q ' "${cmd[@]}"; printf '\n'
if $dry_run; then exit 0; fi

[[ ! -e "$output" && ! -L "$output" ]] || die "저장 경로가 이미 존재합니다: $output"
# ROS setup scripts access unset variables. Only local setups: avoid stale underlays.
set +u
source /opt/ros/humble/setup.bash
for setup in "$ROOT"/*/install/local_setup.bash; do
  if [[ -f "$setup" ]]; then source "$setup"; fi
done
set -u
mkdir -p -- "$(dirname -- "$output")"
printf '녹화를 시작합니다. Ctrl+C로 종료하고 metadata.yaml 저장이 끝날 때까지 기다리세요.\n'
exec "${cmd[@]}"

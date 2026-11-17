# 미션별 bag 녹화

```bash
cd ~/2026-AutoRace
./tools/record_bag.sh all         # 코드에 쓰이는 전체 토픽
./tools/record_bag.sh lane        # 차선 (opencv / deep / fusion 모두 대응)
./tools/record_bag.sh obstacle    # 장애물 (objavoid, obj, 장애물도 가능)
./tools/record_bag.sh cone        # 라바콘
./tools/record_bag.sh parking     # 주차
./tools/record_bag.sh crosswalk   # 횡단보도
./tools/record_bag.sh signal      # 신호등
./tools/record_bag.sh slow        # 방지턱·서행
./tools/record_bag.sh yolo        # 영상·YOLO 인지
./tools/record_bag.sh sensor      # 센서·위치추정
```

센서와 필요한 미션 노드를 별도로 켠 뒤 실행합니다. 스크립트는 ROS와 저장소의 설치된 워크스페이스 환경을 자동으로 읽습니다. 현재 터미널의 `ROS_DOMAIN_ID`, RMW 설정을 사용하므로 주행 노드와 같은 환경에서 실행하세요. `Ctrl+C`로 종료하고 bag 저장 완료를 기다립니다.

기본 저장 위치는 `~/2026-AutoRace/bags/<모드>_<날짜>_<시간>/`입니다. 안에 `.db3`와 `metadata.yaml`이 저장됩니다. 다른 위치를 지정할 수 있고, 이미 존재하는 경로는 덮어쓰지 않습니다.

```bash
./tools/record_bag.sh lane obstacle -o ~/bags/lane_obstacle_test
./tools/record_bag.sh 장애물 --list      # 녹화 없이 토픽 목록 확인
./tools/record_bag.sh all --dry-run      # 폴더 생성 없이 명령 확인
./tools/record_bag.sh all -- --max-bag-duration 60  # 60초마다 파일 분할 (종료 시간이 아님)
```

`all`은 스크립트에 명시된 토픽의 합집합입니다. 각 모드는 관련 센서 입력, 판단 결과, 상태, 경로, 제어 출력을 포함합니다. 공통으로 `/mission/lane`, `/cmd_vel`, `/mux/active_source`, `/tf`, `/tf_static`, `/clock`, `/rosout`을 선택합니다. 미션 간 의존성도 포함하므로, 예를 들어 차선에는 주차 인계 신호와 라바콘 출구 상태가 들어가고 장애물에는 서행 제어 상태가 들어갑니다. 중복 토픽은 한 번만 기록합니다.

모든 선택 토픽이 항상 발행되는 것은 아닙니다. 실행한 차선 모드나 미션 상태에 따라 발행되는 토픽만 bag에 저장되며, 녹화 중 나중에 생기는 토픽도 탐색합니다. 목록은 2026-11-18의 소스와 실차 launch/config 기준입니다. 기본 설정에서 쓰는 압축 카메라와 deep mask, 디버그 영상도 포함하므로 영상이 켜져 있으면 파일이 커질 수 있습니다. 입력을 별도 토픽으로 remap한 경우에는 `record_bag.sh`의 해당 프로필도 함께 수정하세요.

현재 구현에서 서행은 `command_04`, 장애물은 `command_05`, 라바콘은 `command_06`입니다. 예전 설계 문서나 개별 녹화 예시의 번호와 다를 수 있습니다. 주차의 예전 `/parking/floor_observation`은 현재 코드에서 사용하지 않아 제외했습니다.

토픽 목록의 근거는 `lane_ws/src`, `lane_fusion_ws/src`의 노드, `objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp`, `cone_ws/src/cone_lane_follower/cpp/roi_node.cpp`, 각 미션 `node.py`, 센서 실차 설정과 `tensor_bringup/launch/tensor_bringup.launch.py`입니다. 토픽 인터페이스를 바꾸면 스크립트의 `profile()`도 갱신하세요.

녹화 내용은 `ros2 bag info <저장폴더>`로 확인할 수 있습니다.

2026-11-18 재검사에서 EKF의 위치 초기화 토픽 `/set_pose`를 `sensor`와 `all`에 추가했습니다. 같은 이름의 `/set_pose` 서비스와는 별개이며, 서비스 호출 자체는 이 bag에 기록되지 않습니다. 전체 토픽과 소스 위치의 대응표는 [BAG_TOPIC_AUDIT.md](BAG_TOPIC_AUDIT.md)에 있습니다.

현재 소스에서 갈림길 `/yolo/LR_sign`, `/yolo/LR_detail`은 장애물 V3 노드가 구독하므로 `obstacle`과 `yolo`에 포함합니다. `lane`은 실제 차선 노드가 사용하는 입력과 출력만 선택하며, 갈림길 인지도 함께 확인하려면 `lane obstacle`로 실행하세요.

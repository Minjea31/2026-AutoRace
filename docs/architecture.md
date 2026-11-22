# 2026 AutoRace — 현재 실차 알고리즘과 토픽

기준: 현재 main 소스와 `tensor_bringup`의 `lane_mode:=deep lane_speed:=0.8`, 기본 car06 설정입니다. 센서는 별도 `sensors.launch.py`로 실행합니다. 실제 실행 인자·설정 파일을 바꾸면 연결과 동작도 달라집니다.

- [편집 가능한 Draw.io](2026_AutoRace.drawio)
- [PNG](2026_AutoRace.png) / [SVG](2026_AutoRace.svg)
- [도식 연결 명세 JSON](architecture-topics.json)
- [미사용 코드 정리·삭제 전후 검증](logic-cleanup.md)

도식은 기존 워크스페이스 배치를 유지합니다. 왼쪽 센서 컨테이너에 camera·LiDAR·IMU·LIMO base·sensor_fusion·EKF를 나누고, 표시용 토픽은 하단 `visualization`에 모았습니다. 연결선은 직각이며 진한 실선은 주행·제어, 보라색 점선은 관측 분기입니다. 하단 카드와 설명 박스는 토픽을 묶어 보여주는 영역이며 별도 ROS 노드를 뜻하지 않습니다.

## 워크스페이스별 알고리즘

| 워크스페이스 | 현재 역할·알고리즘 | 상세 문서 |
| --- | --- | --- |
| camera_compat_ws | ROS Image/OpenCV 변환·압축 전송 호환성 라이브러리 | [README](../camera_compat_ws/README.md) |
| sensor_ws | 휠·IMU 검증/정지 바이어스 보정 + robot_localization EKF | [README](../sensor_ws/README.md) |
| yolo_ws | YOLO26 + confidence·박스 면적·새 영상 연속 확인 | [README](../yolo_ws/README.md) |
| lane_fusion_ws | MobileNetV3/U-Net 분할 → BEV 경계 추적·중앙 경로, 지그재그 형상 판정 | [README](../lane_fusion_ws/README.md) |
| lane_ws | 횡오차·방향·목표점 결합 PID + 주차 접근 | [README](../lane_ws/README.md) |
| crosswalk_ws | 표지 확인·HSV 흰색 ROI·지연 정지·유지·재진입 상태기계 | [README](../crosswalk_ws/README.md) |
| signal_ws | 빨강/초록 연속 확인·새 흰색 ROI·정지 상태기계 | [README](../signal_ws/README.md) |
| slow_ws | 표지판 타이머 + 지그재그 상태기계·고정 속도 서행 | [README](../slow_ws/README.md) |
| objavoid_ws | LiDAR/차선 지도·Gap·5차 다항식 Lattice·DP·경로 오차 피드백 | [README](../objavoid_ws/README.md) |
| cone_ws | LiDAR 인접 군집·좌우 경계 추적·중앙/오프셋 경로·오차 피드백 | [README](../cone_ws/README.md) |
| parking_ws | 주차칸 기하·융합 위치/2D 점-직선 정합·Hybrid A*·보정 Pure Pursuit | [README](../parking_ws/README.md) |
| mux_ws | 고정 우선순위·입력 만료·주차 제어 인계·Ackermann/Twist 변환 | [README](../mux_ws/README.md) |

각 README는 알고리즘 설명·입출력 타입·구현 파일을 포함합니다. 모델·표준 알고리즘의 논문과 자체 미션 규칙을 구분하며, 관련 연구와 유사하다는 이유만으로 논문 전체를 재현했다고 표기하지 않습니다.

## 현재 실행되는 차선 흐름

```text
camera CompressedImage
  → lane_fusion_deep_mask
  → /lane_fusion/deep_mask (Image, mono8)
  → lane_deep_lite_node
  → /lane_deep/target (Float32MultiArray)
  → lane_pid_controller (lane_ws)
  → /mission/lane (AckermannDriveStamped)
  → mux
```

`/lane_deep/zigzag`(`std_msgs/msg/UInt8`)는 0=없음, 1=한쪽, 2=양쪽이며 slow가 구독합니다. `/lane_fusion/deep_mask`는 objavoid도 사용하므로 시각화 전용 토픽이 아닙니다. `lane_speed:=0.8`은 기본 속도 설정이고 감속·정지·미션 제어에서 실제 출력 속도가 달라집니다.

**Fusion 알고리즘·노드·설정은 보존합니다.** `lane_mode:=fusion`에서는 HSV/딥 후보를 융합하는 `lane_fusion_node`와 별도 fusion PID를 실행합니다. `lane_mode:=opencv`의 HSV 경로도 별도 기능으로 유지합니다. 이 둘은 현재 deep 실행의 활성 노드와 구분합니다.

## 센서·위치 입력

| 발행자 | 토픽 | 타입 | 주요 구독자 |
| --- | --- | --- | --- |
| 카메라 | `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | YOLO·deep mask·crosswalk·signal·slow·parking floor gate |
| 카메라 | `/camera/color/image_raw` | `sensor_msgs/msg/Image` | 선택적 RViz·영상 도구 |
| LiDAR | `/scan` | `sensor_msgs/msg/LaserScan` | parking·cone·objavoid·선택적 관측 |
| LIMO base | `/odom` | `nav_msgs/msg/Odometry` | sensor_fusion·deep_lite·lane PID guard·parking·objavoid |
| LIMO base | `/imu` | `sensor_msgs/msg/Imu` | sensor_fusion·objavoid(car06 IMU yaw) |
| LIMO base | `/limo_status` | `limo_msgs/msg/LimoStatus` | objavoid·parking(manual guard 활성화 시) |
| sensor_fusion | `/localization/wheel` | `nav_msgs/msg/Odometry` | EKF |
| sensor_fusion | `/localization/imu` | `sensor_msgs/msg/Imu` | EKF·parking |
| sensor_fusion | `/localization/health` | `std_msgs/msg/String` | parking |
| EKF | `/odometry/filtered` | `nav_msgs/msg/Odometry` | parking |
| TF 발행 노드 | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | cone·RViz 등 |

EKF는 2D/30 Hz, `parking_odom` 기준이며 EKF 자체의 TF 발행은 꺼져 있습니다. `/localization/calibrate_gyro`(`std_srvs/srv/Trigger`)와 `/set_pose`(`robot_localization/srv/SetPose`)는 보정·초기 자세 서비스입니다.

## YOLO 미션 신호

| 토픽 | 타입 | 현재 의미 / 소비자 |
| --- | --- | --- |
| `/yolo/slow_sign` | `std_msgs/msg/Bool` | 서행 표지 존재 → slow |
| `/yolo/crosswalk_sign` | `std_msgs/msg/Bool` | 횡단보도 표지 존재 → crosswalk·slow |
| `/yolo/parking_sign` | `std_msgs/msg/Bool` | 주차 표지 존재 → parking·lane PID |
| `/yolo/LR_sign` | `std_msgs/msg/Bool` | false=왼쪽/A, true=오른쪽/B → objavoid |
| `/yolo/LR_detail` | `std_msgs/msg/Float32MultiArray` | 좌우 표지 박스·촬영 시각 → objavoid |
| `/yolo/signal` | `std_msgs/msg/Bool` | false=빨강, true=초록 → signal |

현재 YOLO는 slow/crosswalk/parking의 미검출을 false로 발행합니다. 좌우 방향·신호등은 미검출·모호함·영상 만료 시 발행하지 않습니다. `/yolo/bump_sign` 발행은 비활성이고 slow 쪽 구독만 남아 있습니다. 도식에 활성 YOLO 발행선으로 그리지 않습니다.

## 미션 명령과 차량 출력

모든 미션 후보의 타입은 `ackermann_msgs/msg/AckermannDriveStamped`입니다.

| 우선순위 | 발행 워크스페이스 | 토픽 |
| --- | --- | --- |
| 01 | crosswalk_ws | `/mission/crosswalk/command_01` |
| 02 | signal_ws | `/mission/signal/command_02` |
| 03 | parking_ws | `/mission/parking/command_03` |
| 04 | slow_ws | `/mission/slow/command_04` |
| 05 | objavoid_ws | `/mission/objavoid/command_05` |
| 06 | cone_ws | `/mission/cone/command_06` |
| 기본 | lane_ws | `/mission/lane` |

Crosswalk·signal·slow는 유효한 lane 조향을 유지하며 속도를 조정합니다. 유실/정지 예외는 각 README를 따릅니다. 비활성 미션은 후보를 발행하지 않고, 정지 중인 미션은 속도 0을 계속 발행해 제어권을 유지합니다.

Mux는 입력 유효성·주차 인계 정책을 적용한 뒤 `/cmd_vel`(`geometry_msgs/msg/Twist`)을 LIMO base에 보냅니다. 변환은 `linear.x=v`, `angular.z=v·tan(steering)/wheelbase`입니다. `/mux/active_source`(`std_msgs/msg/String`)로 실제 선택 주체를 parking·cone·objavoid·관측 도구에 알립니다.

## 미션 사이의 인계

| 발행자 | 토픽 | 타입 | 구독자 / 용도 |
| --- | --- | --- | --- |
| crosswalk | `/start_crosswalk` | `std_msgs/msg/Bool` | slow의 서행 종료 |
| signal | `/start_signal` | `std_msgs/msg/Bool` | slow의 서행 종료 |
| slow | `/slow/mission_active` | `std_msgs/msg/Bool` | objavoid 시작 차단·관측 |
| lane PID | `/parking/approach_start` | `std_msgs/msg/Header` | parking 접근 기준점·mux 접근 상태 |
| parking | `/parking/approach_ready` | `std_msgs/msg/Header` | lane PID 접근 준비 |
| parking | `/parking/active` | `std_msgs/msg/Bool` | lane PID·관측 |
| parking | `/parking/control_state` | `std_msgs/msg/String` | lane PID·mux 제어 상태 |
| parking | `/parking/lane_handoff` | `std_msgs/msg/Header` | mux의 lane 인계 |

현재 deep PID는 `/parking_start`, `/parking/ready`를 구독하지 않습니다. 이 출력은 별도 fusion PID가 사용합니다. 현재 objavoid car06은 `fork_shape=false`여서 `/mission/lane` 구독을 생성하지 않으며 `/start_crosswalk`도 직접 구독하지 않습니다.

## 설정에 따른 차이

- Parking 실차 기본값은 `manual_mode_guard=true`, `parking_calibration_stop=true`, `exit_on_lane_reacquired=false`, `exit_curve_handoff=false`, `exit_curve_only=true`입니다. `/limo_status`는 구독하고 `/mission/lane`·`/lane_fusion/target`의 출차 인계 구독은 만들지 않습니다.
- Parking은 `/lane_fusion/yellow_boundary`를 구독하지만 현재 deep 실행에는 발행자가 없습니다. 실차 설정의 `yellow_boundary_enabled=false`에서는 이 경계의 충돌 검사를 사용하지 않습니다.
- Parking의 `assume_unobserved_free=true`는 미관측 구간을 빈 공간으로 취급하는 설정입니다. 미관측 구간을 항상 거부한다고 설명하지 않습니다.
- `/lane_fusion/target`·`/lane_fusion/yellow_boundary`·`/lane_fusion/debug`는 fusion 인식 모드의 출력입니다. 현재 deep 모드의 출력으로 임의 연결하지 않습니다.
- 현재 signal에는 10초 자동 해제가 없습니다. 초록 확인으로 정지를 해제합니다.

## visualization

| 출력 그룹 | 토픽 | 타입 |
| --- | --- | --- |
| 영상 | `/yolo/debug/compressed` | `sensor_msgs/msg/CompressedImage` |
| 차선 BEV | `/lane_deep/debug` | `sensor_msgs/msg/Image` |
| 바닥 ROI | `/{slow,crosswalk,signal}/white_roi/debug/compressed` | `sensor_msgs/msg/CompressedImage` |
| 바닥 ROI | 같은 prefix의 `ratio`, `detected` | `std_msgs/msg/Float32`, `std_msgs/msg/Bool` |
| 주차 | `/parking/obstacles`, `target`, `slot_map`, `candidate_roi`, `lookahead` | `visualization_msgs/msg/MarkerArray` |
| 주차 | `/parking/path`, `/parking/obstacle_memory` | `nav_msgs/msg/Path`, `sensor_msgs/msg/PointCloud2` |
| 주차 상태 | `/parking/status`, `/parking/ldf` | `std_msgs/msg/String`, `std_msgs/msg/Float32` |
| 주차 위치 | `/parking/localization/odometry`, `/parking/approach_odometry` | `nav_msgs/msg/Odometry` |
| 콘 | `/cone_lane/roi_marker`, `/cone_lane/roi_points` | `visualization_msgs/msg/Marker` |
| 콘 | `/cone_lane/{clusters,cone_centers,boundaries,control_points,center_path,center_spot,backup_path}` | `visualization_msgs/msg/MarkerArray` |
| OBJ | `/objavoid_v3/markers`, `/objavoid_v3/path` | `visualization_msgs/msg/MarkerArray`, `nav_msgs/msg/Path` |
| PID | `/lane_pid/command`, `/lane_pid/state` | `ackermann_msgs/msg/AckermannDriveStamped`, `std_msgs/msg/Float32MultiArray` |
| 미션 표시 노드 | `/missions_viz/parking/*`, `/missions_viz/command_path` | 주차 원본과 같은 타입 / `visualization_msgs/msg/MarkerArray` |

웹/RViz/mission_visualizer는 선택 실행입니다. 명령·센서·상태를 관측하는 분기는 원래 발행자를 복제하지 않습니다. `/cone_lane/control_status`는 Marker 타입이지만 bringup 시작 검사에도 사용하므로 표시 전용 목록으로만 분류하지 않습니다.

Legacy 뷰어의 `/lane_fusion/debug`·`/lane_fusion/fork_debug`와 현재 deep BEV를 구분합니다. 실제 deep BEV는 `/lane_deep/debug`이며 legacy 웹은 이를 직접 구독하지 않습니다. 장애물 전용 웹에는 실행·속도 등 제어 기능도 있으므로 모든 웹을 읽기 전용이라고 부르지 않습니다.

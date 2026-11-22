# crosswalk_ws — 흰색 바닥 감지 후 정지 및 재감지

ROS 2 Humble의 `autorace_crosswalk` 패키지입니다. `slow_ws`와 같은 흰색 바닥 표시 감지를 사용합니다. 흰색 픽셀 비율을 판정하며 정지선 모양이나 실제 거리는 구별하지 않습니다.

## 확정된 구현 계획 (Proposed Plan)

흰색 바닥 표시 감지 조건을 추가한 최신 합의와 구현 기준입니다.

### 동작

- `/yolo/crosswalk_sign`을 `std_msgs/msg/Bool`, `/mission/lane`을 `ackermann_msgs/msg/AckermannDriveStamped`로 구독합니다.
- true를 연속 2회 받으면 흰색 바닥 표시를 기다립니다. 이후 촬영된 유효한 영상에서 ROI의 흰색 비율이 8% 이상이면 1.5초 뒤 정지를 시작합니다. true 2회 확인 전 false를 받으면 확인 횟수를 초기화하며, 메시지 간 시간 제한은 두지 않습니다.
- 정지 중에는 최신 lane 메시지를 복사하여 `drive.speed`만 `0.0`으로 바꾸고 `/mission/crosswalk/command_01`로 30 Hz 발행합니다. 메시지 타입은 `ackermann_msgs/msg/AckermannDriveStamped`입니다. 조향 등 나머지 drive 필드와 frame은 보존하고 헤더 시각은 현재 시각으로 갱신합니다.
- 정지 명령을 시작하는 순간 `/start_crosswalk` (`std_msgs/msg/Bool`, True)를 한 번 발행합니다. 표지판 확정·바닥 대기·1.5초 지연·lane 입력 대기 중에는 발행하지 않으며, 정지 중 반복 발행하거나 종료 시 False를 보내지 않습니다. QoS는 Reliable / Volatile / KeepLast(1)입니다. `slow_node`는 이를 받아 서행 미션을 종료합니다.
- 실제 경과 시간 기준 3초 동안 정지 명령을 발행합니다. 정지 중 false를 받아도 3초를 완료합니다.
- 정지 완료 후 발행을 중단하고 실제 경과 시간으로 10초 동안 표지판·흰색 감지를 무시합니다. 10초가 지나면 확인 횟수를 비운 상태로 다시 대기합니다. 이후 새 true 연속 2회와 확인 이후 촬영된 새 흰색 영상이 있어야 다시 정지합니다. 대기·재감지 금지 중에는 명령을 발행하지 않습니다.

### 구현

- `crosswalk_ws`에 ROS 2 Python 패키지 `autorace_crosswalk`와 실행 노드 `crosswalk_node`, `crosswalk.launch.py`를 추가합니다.
- 상태는 표지판 감지 대기 → 흰색 감지 대기 → 3초 정지 → 10초 재감지 금지 → 표지판 감지 대기로 구성합니다. 흰색 대기 중 false가 들어와도 대기를 유지합니다.
- 흰색 ROI 감지를 추가하며, 선 모양 판별·표지판 수신 시간 초과는 추가하지 않습니다. 정지 완료 후 10초가 지나면 자동으로 재감지를 허용합니다.
- launch에서 `use_sim_time`을 지원하며 기본값은 기존 미션과 동일하게 true로 둡니다. 3초 측정에는 steady clock을 사용합니다.
- README에 빌드·실행·토픽·수동 시험 방법을 기록합니다.

### 검증 기준

- true 1회에는 발행하지 않고, 연속 2회 이후 새 흰색 감지가 있어야 정지하는지 확인합니다.
- true → false → true에서는 정지를 시작하지 않는지 확인합니다.
- 조향 `0.2`, 속도 `1.0` 입력이 조향 `0.2`, 속도 `0.0`으로 출력되며, 정지 중 새 lane 조향도 반영되는지 확인합니다.
- 정지 중 false에도 3초를 완료하고, 완료 후 10초 동안 감지를 무시하며, 이후 새 표지판·흰색 조건이 모두 충족될 때만 재정지하는지 확인합니다.
- 표지판 확인 전 촬영·중복·오래된 영상·과도한 미래 시각·디코딩 실패 영상이 정지를 시작시키지 않는지 확인합니다.
- ROS 영상·명령 토픽 연동과 패키지 빌드를 확인합니다.

### 전제

- lane 명령은 항상 정상적으로 수신된다는 사용자 전제를 따릅니다. 최초 lane 수신 전에는 발행하지 않습니다.
- false일 때 발행하지 않는 기존 요청에는 **이미 시작한 3초 정지는 완료한다**는 최신 결정을 적용합니다.
- 발행 중단 후 실제 lane 복귀 시점은 기존 mux의 후보 만료 시간에 따릅니다.

## 동작

- `/yolo/crosswalk_sign`의 true를 연속 2회 받으면 흰색 바닥 표시를 기다립니다. 이후 촬영된 유효한 영상에서 흰색 비율이 기준 이상이면 1.5초 뒤 정지합니다. true 2회 확인 전 false는 확인 횟수를 초기화합니다. 메시지 간 시간 제한은 없습니다.
- 최신 `/mission/lane`을 복사해 `drive.speed`만 0으로 바꾸고 `/mission/crosswalk/command_01`로 30 Hz 발행합니다. 조향각·조향각 변화율·가속도·jerk·frame은 유지하며 헤더 시각은 현재 ROS 시각으로 갱신합니다.
- 정지 시작 후 **실제 경과 시간으로 3초** 동안 발행합니다. 정지 중 false나 추가 true를 받아도 정지 시간을 바꾸지 않습니다.
- 3초 후 발행을 중단하고 **정지 완료 시점부터 10초 동안 재감지를 금지**합니다. 금지 중의 true·흰색은 저장하거나 다음 정지에 사용하지 않습니다. 10초 후 새 true 연속 2회 → 새 흰색 감지가 있으면 다시 3초 정지합니다. false 수신이나 노드 재시작은 재감지의 필수 조건이 아닙니다.
- lane은 항상 정상 수신된다는 전제입니다. 최초 lane 수신 전에는 발행하지 않고, 표지판·흰색 조건을 모두 충족한 상태를 유지하다 lane 수신과 1.5초 지연이 모두 충족되면 3초 정지를 시작합니다. lane 만료 검사나 기본 정지 명령은 추가하지 않았습니다.
- 정지 명령 발행 중단 후 기존 mux 후보가 만료되면 lane으로 복귀합니다. 기본 mux timeout은 0.25초이며 다른 미션이 있으면 기존 우선순위를 따릅니다.

| 방향 | 토픽 | 메시지 타입 |
| --- | --- | --- |
| 구독 | `/yolo/crosswalk_sign` | `std_msgs/msg/Bool` |
| 구독 | `/mission/lane` | `ackermann_msgs/msg/AckermannDriveStamped` |
| 구독 | `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` |
| 발행 | `/mission/crosswalk/command_01` | `ackermann_msgs/msg/AckermannDriveStamped` |

lane·표지판·명령의 QoS는 Reliable / Volatile / KeepLast(1)이며, 영상은 sensor data QoS를 사용합니다. 2회 확인은 수신한 Bool 메시지 수이며 서로 다른 원본 영상인지는 구별하지 않습니다. `/cmd_vel`은 발행하지 않습니다.

정지 지연·유지·재진입 대기는 `stop_delay_sec`(1.5), `stop_hold_sec`(3.0), `cooldown_sec`(10.0) 파라미터로 조정합니다. 반복 흰선 감지는 예약된 정지 시각을 늦추지 않습니다.

## 흰색 바닥 표시 감지 설정

`src/autorace_crosswalk/config/crosswalk.yaml`에서 조정합니다. 변경 후 노드를 재시작하세요.

| 파라미터 | 기본값 | 의미 |
| --- | --- | --- |
| `white_roi_left` / `white_roi_right` | 0.20 / 0.80 | 원본 영상 중앙 폭 60% |
| `white_roi_top` / `white_roi_bottom` | 0.86 / 0.92 | 영상 하단의 얕은 띠 |
| `white_roi_minimum_ratio` | 0.08 | ROI 픽셀 중 흰색 비율 8% 이상 |
| `white_image_timeout_sec` | 0.30 | 영상 촬영 시각의 허용 나이 |
| `white_image_hz` | 10.0 | 영상 처리 상한 |
| `white_image_topic` | `/camera/color/image_raw/compressed` | 입력 영상 |
| `lane_white_config` | 빈 문자열 | lane HSV 설정 YAML 경로, 비어 있으면 자동 검색 |
| `lane_white_node` | `/lane_fusion_node` | HSV 파라미터를 조회할 lane 노드 |

`slow_ws`의 `WhiteROI`와 같은 코드를 패키지 내부에 두어 slow 노드 실행 없이 사용합니다. HSV의 S 상한과 V 하한은 설치된 `autorace_lane_fusion`의 `config/real/lane.yaml`에서 읽습니다. 설치된 패키지가 없으면 같은 저장소의 `lane_fusion_ws` 소스를 검색합니다. 둘 다 없으면 `lane_white_config`를 직접 지정해야 합니다. `lane_lite.white_s_max`, `lane_lite.white_v_min` 키를 사용하며 현재 저장소 값은 70, 120입니다. 실행 중 lane 노드의 파라미터 변경도 따라갑니다. lane 노드가 없으면 파일 값을 사용합니다.

표지판 확인 전의 흰색 결과는 저장하지 않습니다. 확인 전 촬영 영상·중복·역순·빈 stamp·촬영 후 0.3초 초과 영상·0.05초보다 먼 미래 영상은 시작 조건에서 제외합니다. 영상과 노드의 시간 기준은 같아야 합니다. 흰색 대기 중 false가 들어와도 계속 기다립니다.

감지 결과는 `/crosswalk/white_roi/detected` (`Bool`), 비율은 `/crosswalk/white_roi/ratio` (`Float32`)로 발행합니다. 미리보기는 구독자가 있을 때 `/crosswalk/white_roi/debug/compressed` (`CompressedImage`)로 발행합니다. 감지 결과 true만으로 정지하는 것은 아니며 표지판 확인 이후 촬영 조건도 만족해야 합니다.

## 빌드와 실행

시스템 ROS Python과 `sensor_msgs`, OpenCV, NumPy, PyYAML이 필요합니다.

```bash
cd ~/2026-AutoRace
source crosswalk_ws/env.sh
cd crosswalk_ws
/usr/bin/python3 -m colcon build --symlink-install --packages-select autorace_crosswalk
source env.sh
ros2 launch autorace_crosswalk crosswalk.launch.py
```

launch의 `use_sim_time` 기본값은 `true`입니다. 실차나 시스템 시계를 사용하는 시험에서는 다음과 같이 실행합니다.

```bash
ros2 launch autorace_crosswalk crosswalk.launch.py use_sim_time:=false
```

노드만 직접 실행하려면 `ros2 run autorace_crosswalk crosswalk_node`를 사용합니다. 직접 실행 시 ROS 기본값인 시스템 시계를 사용합니다. 3초 정지와 10초 재감지 금지 타이머는 시뮬레이션 일시정지에도 진행합니다. lane·YOLO·mux는 별도로 실행하고 ROS 도메인을 맞춰야 합니다.

## 수동 시험

각 터미널에서 `source ~/2026-AutoRace/crosswalk_ws/env.sh`를 실행합니다. 실제 lane·YOLO가 없는 시스템 시계 시험에서 아래 명령을 사용합니다. 카메라는 별도로 실행하고, 표지판 확인 후 ROI에 흰색 바닥 표시를 보여 주세요.

```bash
# 터미널 1: lane 입력을 계속 발행
ros2 topic pub -r 30 /mission/lane ackermann_msgs/msg/AckermannDriveStamped \
  '{header: auto, drive: {speed: 1.0, steering_angle: 0.2}}'

# 터미널 2: 감지 전부터 출력을 관찰
ros2 topic echo /mission/crosswalk/command_01

# 터미널 3: true 2회로 흰색 감지 대기. 이후 새 흰색 영상 감지 시 3초 정지
ros2 topic pub -r 10 --times 2 /yolo/crosswalk_sign std_msgs/msg/Bool '{data: true}'
```

정지 완료 후 10초 동안 true와 흰색을 반복해도 정지 명령 출력이 없어야 합니다. 10초가 지난 뒤 위 true 명령으로 다시 2회를 확인하고 새 흰색 영상을 보여 주면 다시 3초 정지해야 합니다. 실제 lane·YOLO 발행자가 있으면 수동 발행을 중복 실행하지 마세요.

## 검증

```bash
cd ~/2026-AutoRace/crosswalk_ws
source env.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  -p no:cacheprovider -q src/autorace_crosswalk/test
```

상태 전환·ROI 색상 계산·정확한 3초 정지·10초 재감지 금지 경계를 검증하고, 별도 도메인·고유 토픽에서 실제 DDS 영상·명령 입출력으로 영상 유효성·흰색 감지 조건·조향 갱신·다른 필드 보존·false 중 정지 유지·금지 중 감지 무시·금지 해제 후 새 조건으로 재정지를 확인합니다.

정지 유지 시간은 3초이다 (`STOP_HOLD_SEC=3.0`). 흰선 감지 후 정지 시작까지의 지연은 1.5초이며, 정지 종료 후 재감지 대기 간격은 10초이다.

현재 흰색 HSV 기본값은 `autorace_lane_deep_lite/config/real/mission_vision.yaml`에서 읽습니다.
`lane_white_node` 기본값은 빈 문자열이며, 별도 파라미터 서버를 명시할 때만 실시간 HSV 조회를 사용합니다.

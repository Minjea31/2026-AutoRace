# slow_ws — 방지턱·서행 미션

ROS 2 Humble의 `autorace_slow` 패키지입니다. YOLO Bool 두 개와 lane 명령, 카메라의 흰색 바닥 표시를 받아 `/mission/slow/command_04`을 발행합니다. 이번 사용자 지정 동작은 `docs`의 속도 절반 제한 대신 **고정 0.2 m/s**를 사용합니다.

## 동작

| 입력 | 진입 | 활성 중 속도·조향 |
| --- | --- | --- |
| `/yolo/slow_sign` | true 2회 연속 후 새 흰색 ROI, 0.5초 지연 | 속도 0.2 m/s, 최신 lane 조향 유지, 15초 |
| `/yolo/bump_sign` | true 2회 연속 후 새 흰색 ROI, 0.5초 지연 | 최신 lane 조향 유지, 속도 0.2 m/s, 10초 |

- 입력별로 감지 횟수와 미션 타이머를 독립적으로 관리합니다. 다른 입력의 true가 횟수에 섞이지 않습니다.
- 같은 입력에서 true 2회가 확인되면 바닥 표시를 기다립니다. 새 카메라 영상에서 흰색 ROI 조건을 충족한 뒤 0.5초 후 감속을 시작합니다. 반복 감지로 예약 시작 시각을 늦추지 않습니다.
- 서행은 새 표지판 2회와 새 ROI 감지로 갱신하며, 방지턱은 활성 중 true 한 번으로 그 시점부터 10초를 다시 셉니다.
- false와 0.5초 초과 수신 공백은 미확정 감지 횟수만 초기화합니다. false로 활성 미션을 취소하거나 대기 시간을 연장하지 않습니다. 미검출 false를 보내지 않는 인지 노드와도 연결됩니다.
- 두 미션이 동시에 활성화되면 **방지턱 우선**입니다. 방지턱 종료 후 서행 미션이 남아 있으면 조향각 0으로 돌아갑니다.
- 미션이 끝나면 해당 입력은 다시 true 2회와 새 흰색 ROI가 필요합니다. 둘 다 비활성이면 command 발행을 멈추고, mux가 마지막 후보를 만료시킨 뒤 lane으로 복귀합니다. 기본 mux 만료 시간은 0.25초입니다.

서행 표지판 미션은 lane 조향각과 관계없이 `drive.steering_angle=0.0`으로 직진 명령을 보냅니다. 과거 조향 이력이나 시작 전 대기는 사용하지 않습니다. 해당 처리부의 `TODO: IMU 토픽으로 변경 예정` 주석은 유지하며, 실제 차량 yaw 유지 제어는 아직 구현하지 않습니다.

활성 미션의 명령은 20 Hz로 발행합니다. 최신 유효 lane 메시지의 헤더 frame과 조향각 변화율·가속도·jerk는 보존하고, 속도 및 서행 미션의 조향각만 덮어씁니다. 헤더 stamp는 현재 ROS 시각으로 갱신합니다. **lane의 속도와 관계없이 설정된 고정 속도를 사용합니다.**

lane이 없거나 수신 후 0.25초 이상 경과했거나 원본 stamp가 만료되면 기본 정지 명령(속도·조향 0)을 발행합니다. 빈 stamp, 음수 속도, NaN/Inf 등 비정상 lane도 기본 정지로 처리합니다. 역순 lane은 최신 메시지를 덮거나 수신 유효 시간을 연장하지 않습니다. ROS 시계가 뒤로 이동하면 저장한 lane을 비웁니다.

## 빌드와 실행

저장소에 포함된 `simul/ros_env.sh`와 시스템 ROS 2 Humble을 사용합니다. `rclpy`, `ackermann_msgs`, `std_msgs`, launch 및 colcon이 필요합니다. `ackermann_msgs`가 없으면 `sudo apt install ros-humble-ackermann-msgs`로 설치합니다.

```bash
cd ~/2026-AutoRace/slow_ws
source env.sh
/usr/bin/python3 -m colcon build --symlink-install --packages-select autorace_slow
source env.sh
ros2 launch autorace_slow slow.launch.py
```

환경 파일은 **각 터미널에서 source**로 적용합니다. 기본 ROS 도메인은 42이고 localhost 통신을 사용합니다. 사용자 지정 도메인은 기존 공통 환경 규칙을 따르므로 시뮬레이터·lane·YOLO·mux와 같은 도메인을 쓰세요. 빌드 산출물은 `slow_ws/.gitignore`에서 제외합니다.

launch의 `use_sim_time` 기본값은 **true**입니다. 실차나 `/clock` 없는 시험에는 다음을 사용합니다.

```bash
ros2 launch autorace_slow slow.launch.py use_sim_time:=false
```

노드만 실행하거나 값을 조정하려면 launch 대신 실행합니다.

```bash
ros2 run autorace_slow slow_node --ros-args   -p use_sim_time:=true -p hold_sec:=10.0 -p speed_mps:=0.2
```

시뮬레이션 시간은 메시지 stamp 검사와 출력 헤더에만 사용합니다. **시뮬레이션이 멈춰도 지연·유지 타이머와 발행 타이머는 계속 진행**됩니다. lane 수신이 끊기면 그동안은 기본 정지 명령을 발행합니다.

## 토픽과 설정

| 방향 | 토픽 | 타입 |
| --- | --- | --- |
| 구독 | `/yolo/slow_sign` | `std_msgs/msg/Bool` |
| 구독 | `/yolo/bump_sign` | `std_msgs/msg/Bool` |
| 구독 | `/mission/lane` | `ackermann_msgs/msg/AckermannDriveStamped` |
| 구독 | `/start_crosswalk` | `std_msgs/msg/Bool` |
| 구독 | `/start_signal` | `std_msgs/msg/Bool` |
| 발행 | `/mission/slow/command_04` | `ackermann_msgs/msg/AckermannDriveStamped` |

QoS는 Reliable / Volatile / KeepLast(1)이며 토픽은 ROS remapping으로 변경할 수 있습니다. `/mission/lane`이나 `/cmd_vel`에는 발행하지 않습니다.

| 파라미터 | 기본값 | 의미 |
| --- | --- | --- |
| `confirm_frames` | 2 | 입력별 연속 true 확인 횟수 |
| `hold_sec` | 10.0 | 방지턱 유지 시간(초), 활성 중 true로 갱신 |
| `slow_hold_sec` | 15.0 | 서행 유지 시간(초) |
| `start_delay_sec` | 0.5 | 새 흰선 확인 후 감속 시작 지연(초) |
| `signal_gap_timeout_sec` | 0.5 | 진입 감지 횟수를 초기화하는 수신 공백 |
| `speed_mps` | 0.2 | 고정 주행 속도(m/s) |
| `publish_rate_hz` | 20.0 | 활성 명령 발행 주기 |
| `lane_timeout_sec` | 0.25 | lane 수신 시각과 stamp 유효 시간 |
| `future_tolerance_sec` | 0.05 | 미래 lane stamp 허용 범위 |

파라미터는 실행 시 지정하며 실행 중 변경은 허용하지 않습니다.

## 시뮬레이터와 수동 시험

시뮬레이터·lane·인지 노드는 별도 실행합니다. 공용 어댑터를 사용하면 창에서 **서행(slow)과 차선(lane)을 모두 ON**으로 켭니다.

```bash
cd ~/2026-AutoRace/simul
source env.sh
./adapter
```

mux로 `/cmd_vel`을 전달하는 구성에서는 어댑터를 중복 실행하지 않습니다. 현재 저장소의 `MuxNode`는 내부 구독 목록을 덮어써 `use_sim_time=true`에서 `/clock` 처리가 누락되는 문제가 확인됐습니다. 시뮬레이션 연결은 공용 어댑터를 사용하세요. 이 패키지는 `slow_ws` 밖의 mux 코드를 수정하지 않습니다. 어댑터만 켜서는 lane 명령이나 감지 Bool이 만들어지지 않습니다.

시험할 미션의 true를 별도 터미널에서 보냅니다.

```bash
cd ~/2026-AutoRace/slow_ws
source env.sh

# 서행: 2회 확인 + 새 흰선, 0.5초 뒤 15초간 lane 조향 추종
ros2 topic pub -r 10 --times 3 /yolo/slow_sign std_msgs/msg/Bool '{data: true}'

# 방지턱: 2회 확인 + 새 흰선, 0.5초 뒤 10초간 lane 조향 추종
ros2 topic pub -r 10 --times 3 /yolo/bump_sign std_msgs/msg/Bool '{data: true}'

# 서행 갱신: 새 표지판 2회와 이후 촬영된 새 흰선 필요
ros2 topic pub --once /yolo/slow_sign std_msgs/msg/Bool '{data: true}'

ros2 topic list
```

true를 계속 발행하면 서행도 계속 유지됩니다. 실제 lane 발행자가 있을 때 테스트 lane을 중복 발행하지 마세요.

## 검증

```bash
cd ~/2026-AutoRace/slow_ws
source env.sh
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1   /usr/bin/python3 -m pytest -p no:cacheprovider -q src/autorace_slow/test
```

연속 확인·수신 공백·0.5초 지연·방지턱 10초 갱신·독립 타이머·우선순위, 서행과 방지턱의 최신 lane 조향 유지, lane 유효성·필드 보존, 실제 DDS와 mux 복귀를 검사합니다. ROS 검사는 별도 도메인과 고유 토픽을 쓰며 실제 `/cmd_vel`에는 발행하지 않습니다. mux 테스트는 저장소의 `mux_ws` 소스를 참조하며 수정하지 않습니다.


## 흰색 바닥 표시로 미션 시작

bump와 slow만 YOLO True 3회 확인 후 바닥 ROI가 1회 양성이어야 시작합니다.
YOLO 확정 전의 흰색 결과는 저장하지 않으며, 확정 시각보다 과거에 촬영한 영상·중복 stamp·0.3초보다 오래된 영상은 트리거로 쓰지 않습니다.
YOLO 확정 뒤 표지판이 사라져도 바닥 표시를 기다립니다. 미션 실행 또는 노드 재시작 전까지 대기를 유지합니다.
방지턱은 활성 중 YOLO True 1회로 10초를 연장합니다. 서행은 새 표지판 2회와 새 흰선으로 갱신합니다. 미션 종료 후에는 다시 두 조건이 필요합니다.

`src/autorace_slow/config/slow.yaml`에서 조정합니다.

| 파라미터 | 기본값 | 의미 |
| --- | --- | --- |
| white_roi_enabled | true | 바닥 표시 조건 사용 |
| white_roi_left / right | 0.20 / 0.80 | 원본 영상 중앙 폭 60% |
| white_roi_top / bottom | 0.86 / 0.92 | 하단의 얕은 띠 |
| white_roi_minimum_ratio | 0.08 | ROI 전체 픽셀 중 흰색 비율 8% 이상 |
| white_image_timeout_sec | 0.30 | 촬영 시각의 허용 나이 |
| white_image_hz | 10.0 | 영상 처리 상한 |

HSV S 상한과 V 하한은 real lane.yaml의 `lane_lite.white_s_max`, `white_v_min`에서 읽으며, lane_fusion_node가 실행 중이면 해당 ROS 파라미터 변경도 따라갑니다.
차선의 두께·길이 제거 후처리는 바닥 표시를 지울 수 있어 적용하지 않습니다.
ROI 확인 영상: `/slow/white_roi/debug/compressed`, 비율: `/slow/white_roi/ratio`, 검출 결과: `/slow/white_roi/detected`.
웹 뷰어의 '서행 · 방지턱 ROI' 화면에서 위치와 실제 흰색 비율을 확인할 수 있습니다.
수동 Bool 발행만으로는 시작되지 않으며 카메라 바닥 조건도 충족해야 합니다.

## 서행 갱신 및 횡단보도 연동 (현재 동작)

서행과 방지턱은 표지판 True 연속 2회와 이후 새 바닥 ROI 양성을 확인하고
`start_delay_sec` 기본값인 0.5초 뒤 감속을 시작합니다. 서행은
`slow_hold_sec` 기본값인 15초 동안 0.2 m/s로 lane 조향을 따릅니다.
실행 중에도 표지판 2회 확인 후 새 ROI 양성 확인 시점에 0.5초 지연과 15초 유지 시간을 더해 기한을 갱신하며, 기존 서행은 끊지 않습니다.
기존 시간이 먼저 만료되면 갱신 대기도 초기화합니다.
`/start_crosswalk` (std_msgs/msg/Bool, Reliable/Volatile)의 True는 서행과 대기를
초기화하며 False는 무시합니다. 이후 새 진입 조건으로 재시작할 수 있습니다.
방지턱은 기존 hold_sec=10초 및 표지판 재감지 연장 정책을 유지합니다.
지연·유지 시간은 node.py 상단 상수 또는 같은 이름의 ROS 파라미터로 조정합니다.

## 횡단보도·신호등에서 서행 종료

`/start_crosswalk=True` 또는 `/start_signal=True`를 받으면 서행 미션의
타이머·표지판 확인 횟수·흰색 ROI 대기 상태를 초기화합니다. False는 무시합니다.
`/start_signal`은 `signal_ws`가 빨강을 확정하고 첫 정지 명령을 발행할 때 보냅니다.
방지턱 미션은 유지하며, 다음 서행은 새 표지판 확인과 새 흰색 영상이 필요합니다.

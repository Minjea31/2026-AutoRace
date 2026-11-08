# slow_ws — 방지턱·서행 미션

ROS 2 Humble의 `autorace_slow` 패키지입니다. 카메라·IMU를 직접 구독하지 않고 YOLO Bool 두 개와 lane 명령을 받아 `/mission/slow/command_08`을 발행합니다. 이번 사용자 지정 동작은 `docs`의 속도 절반 제한 대신 **고정 0.2 m/s**를 사용합니다.

## 동작

| 입력 | 진입 | 활성 중 속도·조향 |
| --- | --- | --- |
| `/yolo/slow_sign` | true 3회 연속 후 흰색 ROI 1회 | 속도 0.2 m/s, 조향각은 항상 0으로 고정 |
| `/yolo/bump_sign` | true 3회 연속 후 흰색 ROI 1회 | 최신 lane 메시지를 복사하고 속도만 0.2 m/s로 변경 |

- 입력별로 감지 횟수와 미션 타이머를 독립적으로 관리합니다. 다른 입력의 true가 횟수에 섞이지 않습니다.
- 같은 입력에서 true 3회가 확인되면 바닥 표시 대기 상태가 됩니다. 이후 촬영된 새 카메라 영상에서 흰색 ROI 비율을 한 번 충족하면 **실제 경과 시간으로 8초**를 셉니다. 활성 중 true를 한 번이라도 다시 받으면 그때부터 다시 8초를 셉니다. 수신 간격이 0.5초를 넘었어도 활성 미션은 true 한 번으로 연장됩니다.
- false와 0.5초 초과 수신 공백은 미확정 감지 횟수만 초기화합니다. false로 활성 미션을 취소하거나 대기 시간을 연장하지 않습니다. 미검출 false를 보내지 않는 인지 노드와도 연결됩니다.
- 두 미션이 동시에 활성화되면 **방지턱 우선**입니다. 방지턱 종료 후 서행 미션이 남아 있으면 조향각 0으로 돌아갑니다.
- 미션이 끝나면 해당 입력은 다시 true 3회가 필요합니다. 둘 다 비활성이면 command 발행을 멈추고, mux가 마지막 후보를 만료시킨 뒤 lane으로 복귀합니다. 기본 mux 만료 시간은 0.25초입니다.

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
ros2 run autorace_slow slow_node --ros-args   -p use_sim_time:=true -p hold_sec:=8.0 -p speed_mps:=0.2
```

시뮬레이션 시간은 메시지 stamp 검사와 출력 헤더에만 사용합니다. **시뮬레이션이 멈춰도 8초 타이머와 발행 타이머는 계속 진행**됩니다. lane 수신이 끊기면 그동안은 기본 정지 명령을 발행합니다.

## 토픽과 설정

| 방향 | 토픽 | 타입 |
| --- | --- | --- |
| 구독 | `/yolo/slow_sign` | `std_msgs/msg/Bool` |
| 구독 | `/yolo/bump_sign` | `std_msgs/msg/Bool` |
| 구독 | `/mission/lane` | `ackermann_msgs/msg/AckermannDriveStamped` |
| 발행 | `/mission/slow/command_08` | `ackermann_msgs/msg/AckermannDriveStamped` |

QoS는 Reliable / Volatile / KeepLast(1)이며 토픽은 ROS remapping으로 변경할 수 있습니다. `/mission/lane`이나 `/cmd_vel`에는 발행하지 않습니다.

| 파라미터 | 기본값 | 의미 |
| --- | --- | --- |
| `confirm_frames` | 3 | 입력별 연속 true 확인 횟수 |
| `hold_sec` | 8.0 | 마지막 true부터 유지할 실제 시간(초) |
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

# 서행: 3회 확인 후 8초, 조향각 0 고정
ros2 topic pub -r 10 --times 3 /yolo/slow_sign std_msgs/msg/Bool '{data: true}'

# 방지턱: 3회 확인 후 8초, 최신 lane 조향 유지
ros2 topic pub -r 10 --times 3 /yolo/bump_sign std_msgs/msg/Bool '{data: true}'

# 서행 활성 중 한 번 더 보내면 지금부터 8초 연장
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

연속 확인·수신 공백·8초 연장·독립 타이머·우선순위, 서행 조향각 0 고정과 방지턱 최신 조향 유지, lane 유효성·필드 보존, 실제 DDS와 mux 복귀를 검사합니다. ROS 검사는 별도 도메인과 고유 토픽을 쓰며 실제 `/cmd_vel`에는 발행하지 않습니다. mux 테스트는 저장소의 `mux_ws` 소스를 참조하며 수정하지 않습니다.


## 흰색 바닥 표시로 미션 시작

bump와 slow만 YOLO True 3회 확인 후 바닥 ROI가 1회 양성이어야 시작합니다.
YOLO 확정 전의 흰색 결과는 저장하지 않으며, 확정 시각보다 과거에 촬영한 영상·중복 stamp·0.3초보다 오래된 영상은 트리거로 쓰지 않습니다.
YOLO 확정 뒤 표지판이 사라져도 바닥 표시를 기다립니다. 미션 실행 또는 노드 재시작 전까지 대기를 유지합니다.
실행 중 YOLO True 1회로 8초를 연장하는 기존 정책은 유지합니다. 미션 종료 후에는 다시 두 조건이 필요합니다.

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

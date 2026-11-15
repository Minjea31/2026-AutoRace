# signal_ws — 신호등 정지 제어

ROS 2 Humble의 `autorace_signal` 패키지입니다. [신호·미션 토픽 규격](../docs/architecture.md)을 사용하며, 빨간불 확인 후 카메라의 새 흰색 바닥 표시를 감지해 지연 정지합니다.

## 동작

- `/yolo/signal`의 **false는 빨강, true는 초록**입니다. 최초 미수신·미확정 상태에서는 명령을 발행하지 않습니다.
- **같은 값이 2회 연속** 들어와야 확정합니다. 반대 값이 들어오거나 수신 간격이 0.5초를 넘으면 연속 횟수를 초기화합니다. 확인 중에는 기존 정지 상태를 유지합니다.
- 빨강 2회 확인 후 촬영된 유효 영상에서 흰색 바닥 ROI 비율이 8% 이상이면 0.5초 뒤 정지합니다. 대기 중에는 명령을 발행하지 않습니다.
- 정지 시 최신 lane 메시지를 복사해 **`drive.speed`만 0**으로 바꿔 30 Hz 발행합니다. 헤더 시각은 현재 ROS 시각으로 갱신하며 조향각·조향각 변화율·가속도·jerk·frame은 유지합니다.
- lane이 없거나 만료·비정상이면 **속도와 조향이 0인 기본 정지 명령**을 발행합니다. 정상 lane이 복구되면 그 조향을 반영합니다.
- **빨강 2회 확인 시점부터 고정 10초**를 셉니다. 흰선 대기와 정지 지연 시간도 포함하며, 빨강을 다시 받아도 기한을 연장하지 않습니다.
- **초록 2회 연속 확인 또는 고정 10초 경과** 시 흰선 대기·예약 정지·정지 발행을 해제합니다. 초록 1회만으로는 해제하지 않습니다.
- 시간 초과 후 같은 빨강이 연속 수신되어도 즉시 재정지하지 않습니다. 초록 확정 또는 신호 수신 공백 이후 새 빨강 확인과 새 흰선 감지가 필요합니다.
- 정지 명령 발행 중단 후 mux의 기존 후보가 만료되어야 lane으로 복귀합니다. 기본 mux timeout은 **0.25초**이며, 다른 유효 미션이 있으면 mux 우선순위를 따릅니다.

`10초`는 **실제 경과 시간(steady clock)**입니다. 시뮬레이션을 일시정지해도 대기는 계속 진행됩니다. `use_sim_time`은 명령 헤더와 lane stamp 검사에 사용하는 ROS 시계를 선택합니다.

## 설정

[node.py](src/autorace_signal/autorace_signal/node.py)의 파일 상단에서 조정합니다. 변경 후 노드를 재시작하세요. symlink 빌드에서는 Python 상수 변경만으로 다시 빌드할 필요는 없습니다.

```python
CONFIRM_FRAMES = 2
STOP_DELAY_SEC = 0.5
MAX_RED_WAIT_SEC = 10.0
SIGNAL_GAP_TIMEOUT_SEC = 0.5
PUBLISH_RATE_HZ = 30.0
LANE_TIMEOUT_SEC = 0.25
FUTURE_TOLERANCE_SEC = 0.05
```

동일한 소문자 이름을 ROS 파라미터로 제공합니다. 예를 들어 `CONFIRM_FRAMES`는 `confirm_frames`입니다. 실행 시 명시한 파라미터가 상수 기본값보다 우선하며 실행 중 변경은 허용하지 않습니다. launch/YAML에는 이 기본값을 중복 고정하지 않습니다.

lane은 수신 후 경과 시간과 원본 `header.stamp`가 모두 0.25초 미만이어야 합니다. 미래 stamp는 0.05초까지 허용합니다. 빈 stamp, 음수 속도, NaN/Inf는 거부합니다. 역순 패킷은 최신 lane을 덮거나 수신 유효 시간을 연장하지 않습니다. ROS 시계가 뒤로 이동하면 저장한 lane을 비우고 기본 정지를 사용합니다.

## 빌드

시스템 ROS Python을 사용합니다. `rclpy`, `std_msgs`, `ackermann_msgs`, `sensor_msgs`, OpenCV, NumPy, PyYAML, lane HSV 설정과 colcon이 필요합니다. 필요하면 `sudo apt install ros-humble-ackermann-msgs python3-colcon-common-extensions python3-pytest`로 설치합니다.

```bash
cd ~/2026-AutoRace
source signal_ws/env.sh
cd signal_ws
/usr/bin/python3 -m colcon build --symlink-install --packages-select autorace_signal
cd ..
source signal_ws/env.sh
```

환경 파일은 **각 터미널에서 source**로 적용합니다. `./env.sh`는 현재 터미널의 환경을 바꾸지 못합니다. ROS 도메인은 변경하지 않으므로 lane·YOLO·mux와 같은 도메인으로 설정하세요.

## ROS 실행

```bash
cd ~/2026-AutoRace
source signal_ws/env.sh
ros2 launch autorace_signal signal.launch.py
```

launch의 `use_sim_time` 기본값은 **true**이므로 시뮬레이션에서는 별도 인자가 필요 없습니다. 실차나 시스템 시계로 실행할 때는 `ros2 launch autorace_signal signal.launch.py use_sim_time:=false`를 사용합니다.

노드만 직접 실행하려면 위 launch 대신 사용합니다.

```bash
ros2 run autorace_signal signal_node

# 설정 변경 예시
ros2 run autorace_signal signal_node --ros-args \
  -p confirm_frames:=2 -p max_red_wait_sec:=10.0 -p stop_delay_sec:=0.5
```

`signal_node`는 `/cmd_vel`을 발행하지 않습니다. 실차에서는 lane·YOLO·mux를 별도로 실행합니다. 비활성·출발·종료 시에는 signal command 발행을 멈추며, mux가 마지막 후보를 만료시킵니다. 상태 전환은 터미널 로그로 확인합니다.

## 시뮬레이터 연결

시뮬레이터와 같은 도메인에서 실행합니다. 일반적인 시뮬레이터 기본 도메인은 42입니다.

```bash
cd ~/2026-AutoRace
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
source signal_ws/env.sh
ros2 launch autorace_signal signal.launch.py
```

시뮬레이터와 lane은 별도로 실행합니다. mux를 사용하면 signal과 lane을 자동으로 선택합니다. mux 없이 `simul/adapter.sh`를 사용한다면 **signal과 lane 입력을 둘 다 ON**으로 켭니다. 초록·시간 초과 후에는 signal 입력이 만료되어 lane을 선택합니다. 같은 `/cmd_vel`에 mux와 어댑터를 동시에 연결하지 않습니다.

현재 YOLO 모델은 `/yolo/signal`을 발행하지 않으므로 아래 수동 신호로 시험할 수 있습니다. `Bool`에는 원본 영상 시각이 없으므로 실제 YOLO 연결 시 오래된 영상 결과나 미검출을 false로 발행하지 않아야 합니다. false는 빨강이라는 뜻입니다.

## 수동 시험 — 별도 터미널

```bash
cd ~/2026-AutoRace
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
source signal_ws/env.sh

# 빨강 연속 2회: 새 흰선 대기 시작 (카메라 영상 필요)
ros2 topic pub -r 10 --times 2 /yolo/signal std_msgs/msg/Bool '{data: false}'

# 초록 연속 2회: 정지 명령 발행 중단
ros2 topic pub -r 10 --times 2 /yolo/signal std_msgs/msg/Bool '{data: true}'

# 정지 중 속도와 유지된 조향 확인 (초록/시간 초과 후에는 새 메시지가 없음)
ros2 topic echo /mission/signal/command_02 --once
```

실제 lane이 없다면 다른 터미널에서 테스트 명령을 발행할 수 있습니다. 기존 lane 발행자가 있으면 중복 실행하지 않습니다.

```bash
ros2 topic pub -r 20 --use-sim-time /mission/lane \
  ackermann_msgs/msg/AckermannDriveStamped \
  '{header: auto, drive: {speed: 0.3, steering_angle: 0.0}}'
```

위 예제는 실행 중인 시뮬레이터의 `/clock`을 사용합니다. 실시간 시계로 시험할 때는 signal launch의 `use_sim_time`을 false로 두고 lane pub의 `--use-sim-time`을 생략합니다. lane이 연결된 상태에서는 초록 2회 연속 확인 또는 빨강 확정 후 고정 10초 경과로 정지가 해제되고, mux의 후보 만료 후 차량이 움직입니다.

## 토픽

| 방향 | 토픽 | 타입 |
| --- | --- | --- |
| 구독 | `/yolo/signal` | `std_msgs/msg/Bool` |
| 구독 | `/mission/lane` | `ackermann_msgs/msg/AckermannDriveStamped` |
| 구독 | `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` |
| 발행 | `/mission/signal/command_02` | `ackermann_msgs/msg/AckermannDriveStamped` |

QoS는 Reliable / Volatile / KeepLast(1)입니다. 2회 확인은 **수신한 메시지 수**를 세며, 서로 다른 원본 영상인지는 Bool만으로 구분할 수 없습니다. 토픽은 ROS remapping으로 변경할 수 있습니다.

흰색 감지는 기존 `node.py` 안에 포함되어 별도 helper 파일이 필요 없습니다.
기본 ROI는 가로 5~95%, 세로 96.1~100%이며, lane의 HSV 설정을 사용합니다.
영상은 0.3초 이내의 새 프레임만 사용하고 표지판 확인 이전 영상은 시작 조건에서 제외합니다.
`/signal/white_roi/detected`, `/signal/white_roi/ratio`, `/signal/white_roi/debug/compressed`로 감지 결과를 확인할 수 있습니다.

## 검증

```bash
cd ~/2026-AutoRace
source signal_ws/env.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  -p no:cacheprovider -q signal_ws/src/autorace_signal/test
```

연속 2회·노이즈·수신 공백·새 흰선·0.5초 지연·반복 빨강에도 고정된 10초 기한·초록 해제·새 빨강 재정지, lane 필드 보존과 유효성, 실제 DDS pub/sub와 기존 mux의 정지 유지 및 lane 복귀를 검사합니다. ROS 통합 테스트는 별도 도메인과 고유 토픽을 사용하며 실제 `/cmd_vel`에는 발행하지 않습니다. mux 통합 테스트는 같은 저장소의 `mux_ws` 소스를 사용합니다.

## 서행 종료 신호

빨강 확정·새 흰선·0.5초 지연 후 첫 `/mission/signal/command_02` 정지 명령을 발행할 때
`/start_signal` (`std_msgs/msg/Bool`)에 `True`를 한 번 발행합니다.
같은 정지 에피소드에서는 반복 발행하지 않습니다.
초록 또는 시간 초과로 정지가 해제된 뒤 새 빨강으로 다시 정지하면 새 신호를 발행합니다.
`slow_ws`는 이 신호로 서행 타이머·감지 횟수·흰색 ROI 대기를 초기화합니다.
방지턱 미션은 유지하며, 다음 서행은 새 표지판 확인과 새 흰색 영상으로 시작합니다.

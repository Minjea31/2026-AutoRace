# mux_ws

ROS 2 Humble용 `autorace_mux` 패키지입니다. 저장된 [draw.io](../docs/2026_AutoRace.drawio)의 미션 명령 6개와 기본 차선 명령을 받고, 선택한 명령을 `/cmd_vel` (`geometry_msgs/msg/Twist`)로 차량에 직접 발행합니다. **기본값은 `/mission/lane`입니다.**

## 입력과 선택 규칙

모든 입력 타입은 `ackermann_msgs/msg/AckermannDriveStamped`입니다. 우선순위는 토픽에 붙은 숫자에 따라 고정하며, 메시지 도착 순서로 바꾸지 않습니다.

| 우선순위 | 입력 토픽 |
| --- | --- |
| 01 | `/mission/crosswalk/command_01` |
| 02 | `/mission/signal/command_02` |
| 03 | `/mission/parking/command_03` |
| 04 | `/mission/cone/command_04` |
| 05 | `/mission/objavoid/command_05` |
| 08 | `/mission/slow/command_08` |
| DEFAULT | `/mission/lane` |

1. 최신 유효 미션 중 번호가 가장 작은 명령을 선택합니다. **속도 0인 미션도 활성 정지 명령**으로 선택합니다.
2. 미션이 없거나 모두 만료되면 최신 유효 lane 명령을 선택합니다.
3. lane도 없거나 만료되면 선속도·각속도 0을 계속 발행합니다.

미션 노드는 비활성·완료 시 발행을 멈추고, 수행 중에는 주기 발행해야 합니다. 발행 중단 후 기본 0.25초가 지나면 다음 후보나 lane으로 복귀합니다. `lane_ws`만 `/mission/lane`을 발행하며, mux는 이 토픽이나 미션 상태 토픽을 발행하지 않습니다. 방지턱·지그재그는 같은 slow 입력을 사용하고, LR는 기존 lane 구조를 유지합니다.

## 유효성 검사와 출력

- 수신 후 경과 시간과 `header.stamp`의 나이가 모두 timeout 미만이어야 합니다. 기본적으로 빈 stamp를 거부하므로 발행자는 매 메시지의 `header.stamp`를 현재 ROS 시각으로 채워야 합니다. 미래 stamp는 기본 0.05초까지만 허용합니다.
- NaN/Inf, 오래된 stamp, 허용 범위를 벗어난 미래 stamp, 주차 이외의 후진 명령은 거부합니다. 이런 입력은 해당 소스의 이전 후보도 제거합니다. 같은 소스의 역순 stamp는 최신 후보를 유지하며 무시합니다.
- 후진은 parking 입력만 허용합니다. 범위를 넘는 속도·조향각은 설정한 한계로 제한합니다.
- `linear.x = speed`, `angular.z = speed * tan(steering_angle) / wheelbase`로 변환합니다. 입력 조향각 단위는 rad, 출력은 m/s와 rad/s입니다. 속도 0에서는 각속도도 0입니다.
- 일반 속도 변경에는 설정한 가속·감속 한계를 적용하고, 방향 전환은 속도 0을 거칩니다. 선택된 정지 명령과 timeout 정지는 감속 램프를 건너뛰고 즉시 0을 발행합니다.
- `drive.speed`와 `drive.steering_angle`을 출력 계산에 사용합니다. `steering_angle_velocity`, `acceleration`, `jerk` 필드는 유한값인지 검사하지만 개별 메시지의 제한값으로 사용하지 않습니다. 가감속 한계는 mux 설정값을 사용합니다.
- 수신 timeout과 출력 타이머는 steady clock을 사용해 `/clock`이 멈춰도 정지합니다. ROS 시각이 뒤로 이동하면 이전 후보를 비웁니다. SIGINT/SIGTERM 종료 시 마지막 정지 Twist를 발행합니다.

입출력 QoS는 Reliable / Volatile / KeepLast(1)입니다. 구현은 [core.py](src/autorace_mux/autorace_mux/core.py)와 [node.py](src/autorace_mux/autorace_mux/node.py)에 있습니다.

## 빌드와 실행

Humble 환경에서 필요한 의존성을 설치한 뒤 이 워크스페이스만 빌드합니다.

```bash
sudo apt install ros-humble-ackermann-msgs python3-colcon-common-extensions python3-pytest
source /opt/ros/humble/setup.bash
cd /home/a/2026-AutoRace/mux_ws
/usr/bin/colcon build --symlink-install --packages-select autorace_mux
source install/setup.bash
ros2 launch autorace_mux mux.launch.py
```

실행 노드 이름은 `/mux`입니다. launch는 [mux.yaml](src/autorace_mux/config/mux.yaml)을 불러옵니다. `ros2 run autorace_mux mux_node`로도 기본 설정으로 실행할 수 있습니다.

## 설정

| 파라미터 | 기본값 | 의미 |
| --- | --- | --- |
| `publish_rate_hz` | 30.0 | Twist 발행 주기 Hz |
| `input_timeout_sec` | 0.25 | 후보 수신 시간·원본 stamp 유효 시간 s |
| `future_tolerance_sec` | 0.05 | 미래 stamp 허용 오차 s |
| `require_stamp` | true | stamp 필수. false면 stamp=0 입력만 수신 시각으로 판단 |
| `wheelbase_m` | 0.2 | 축간거리 m, 저장소 LIMO 드라이버 값 기준 |
| `max_forward_speed_mps` | 0.5 | 최대 전진 속도 m/s |
| `max_reverse_speed_mps` | 0.3 | 주차 최대 후진 속도의 크기 m/s |
| `max_steering_angle_rad` | 0.4886921906 | 최대 조향각 크기 rad, 기본 28도 |
| `max_yaw_rate_rps` | 1.5 | 최대 각속도 크기 rad/s |
| `max_acceleration_mps2` | 1.0 | 속도 크기 증가 한계 m/s² |
| `max_deceleration_mps2` | 2.0 | 속도 크기 감소 한계 m/s² |
| `topics.<이름>` | 위 표의 입력 토픽 | 입력 이름 crosswalk/signal/parking/cone/objavoid/slow/lane |
| `output_topic` | `/cmd_vel` | Twist 출력 토픽 |

한계값은 초기 설정이며 실차에 맞게 조정합니다. 파라미터는 시작 시 읽고 실행 중 변경은 거부합니다. 사용자 YAML은 `params_file:=/절대/경로/mux.yaml`로 지정합니다. 출력 토픽만 바꿀 때는 launch 인자 `output_topic:=/다른/토픽`을 사용할 수 있습니다.

## 로컬 통신 확인

각 터미널에서 같은 ROS 환경을 불러오고 아래 domain 설정을 사용합니다. 출력은 테스트용 토픽으로 보냅니다.

```bash
source /opt/ros/humble/setup.bash
source /home/a/2026-AutoRace/mux_ws/install/setup.bash
export ROS_DOMAIN_ID=199
export ROS_LOCALHOST_ONLY=1
```

첫 터미널에서 실행합니다.

```bash
ros2 launch autorace_mux mux.launch.py output_topic:=/mux_test/cmd_vel
```

다른 터미널에서 lane 명령을 계속 보냅니다. `header: auto`가 매 발행 시각을 채웁니다.

```bash
ros2 topic pub --rate 20 /mission/lane ackermann_msgs/msg/AckermannDriveStamped \
  '{header: auto, drive: {speed: 0.4, steering_angle: 0.1}}'
```

또 다른 터미널에서 정지 미션을 보내면 lane보다 우선합니다.

```bash
ros2 topic pub --rate 20 /mission/crosswalk/command_01 ackermann_msgs/msg/AckermannDriveStamped \
  '{header: auto, drive: {speed: 0.0, steering_angle: 0.1}}'
```

`ros2 topic echo /mux_test/cmd_vel`로 확인합니다. crosswalk 발행을 멈추면 약 0.25초 후 lane으로 복귀해 가속하고, lane 발행도 멈추면 정지합니다.

## 자동 검증

```bash
cd /home/a/2026-AutoRace/mux_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/colcon test --packages-select autorace_mux --event-handlers console_direct+
/usr/bin/colcon test-result --verbose
```

테스트는 우선순위, lane 기본값, 정지 미션, timeout, stamp, 잘못된 수치, 후진 제한, 변환·가감속, 저장된 도면과의 토픽 일치를 확인합니다. 실제 ROS pub/sub 테스트는 별도 domain과 임시 입력·출력 토픽을 사용하고, `/clock` 없는 시뮬레이션에서도 watchdog이 동작하는지 확인합니다. 전체 계약은 [architecture.md](../docs/architecture.md)에 있습니다.

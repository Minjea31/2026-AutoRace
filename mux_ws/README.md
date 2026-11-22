# mux_ws

lane과 미션 노드의 주행 명령을 받아 우선순위가 가장 높은 유효 명령 하나를 선택하고, 차량용 `/cmd_vel`로 변환합니다.
입력은 `ackermann_msgs/msg/AckermannDriveStamped`, 출력은 `geometry_msgs/msg/Twist`입니다.

## 알고리즘

각 명령의 시각·수치 유효성을 검사하고 가장 높은 우선순위의 미션을 선택합니다. 선택한 속도·조향을 Twist로 변환해 차량에 전달하며, 주차 상태와 제어 인계를 반영합니다. 후보가 없으면 lane, lane도 유효하지 않으면 정지합니다.

![mux_ws 알고리즘 개념 그림](../docs/logic/mux_ws.svg)

## 빌드와 실행

```bash
cd ~/2026-AutoRace/mux_ws
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=0
ros2 run autorace_mux mux_node
```

launch로 실행해도 기본값은 같습니다.

```bash
ros2 launch autorace_mux mux.launch.py
```

별도 YAML 없이 코드 기본값을 사용합니다. 모든 노드는 같은 ROS 도메인을 사용해야 합니다.

## 입력 우선순위

| 우선순위 | 입력 토픽 |
|---|---|
| 01 | `/mission/crosswalk/command_01` |
| 02 | `/mission/signal/command_02` |
| 03 | `/mission/parking/command_03` |
| 04 | `/mission/slow/command_04` |
| 05 | `/mission/objavoid/command_05` |
| 06 | `/mission/cone/command_06` |
| 기본 | `/mission/lane` |

번호가 작은 유효 미션을 선택합니다. 미션 입력이 없거나 만료되면 lane으로 돌아가고, lane도 유효하지 않으면 정지합니다.
속도 0을 계속 발행하는 미션도 유효한 제어 명령이므로 lane에 제어권을 넘기려면 해당 미션의 발행을 멈춰야 합니다.
기본 입력 만료 시간은 **0.25초**입니다.

## 현재 제어 주체 확인

`/mux/active_source` (`std_msgs/msg/String`)에 MUX가 실제로 선택한 입력 이름을
제어 출력과 같은 주기로 발행합니다. 기본 주기는 **30Hz**이며,
값은 `crosswalk`, `signal`, `parking`, `cone`, `objavoid`, `slow`, `lane`, `stop`입니다.
유효한 입력이 없거나 parking guard가 출력을 막으면 `stop`입니다.
미션이 속도 0을 명령 중이면 해당 미션 이름을 유지합니다.

```bash
ros2 topic echo /mux/active_source
```

늦게 연결한 구독자도 최신 값을 받을 수 있도록 transient-local QoS를 사용합니다.
필요하면 `active_source_topic` 파라미터로 토픽 이름을 변경할 수 있습니다.
로그는 제어 주체가 바뀔 때 한 번만 남기며, 같은 주체가 유지되면 반복하지 않습니다.
선택된 입력 토픽, 출력 토픽, 실제 출력 속도와 각속도를 함께 기록합니다.
정상 종료 시에는 최종 `stop`을 발행합니다.

```text
Selected: lane | input=/mission/lane | output=/cmd_vel | speed=0.400 m/s, yaw_rate=0.000 rad/s
Selected: crosswalk | input=/mission/crosswalk/command_01 | output=/cmd_vel | speed=0.000 m/s, yaw_rate=0.000 rad/s
Selected: STOP (no fresh input) | input=none | output=/cmd_vel | speed=0.000 m/s, yaw_rate=0.000 rad/s
```

## 기본 제어값

- 최대 전진 속도: **0.8m/s**, 최대 후진 속도: **0.3m/s**.
- 조향각 제한과 각속도 제한: 기본 OFF. 비정상 수치와 ±90° 이상의 입력 조향각은 거부합니다.
- 선택된 중앙 조향각을 `각속도 = 속도 × tan(조향각) / 축간거리`로 바꿉니다. LIMO 드라이버가 이를 차체 명령으로 변환합니다.
- 가감속 제한과 전후진 전환 시 정지 처리는 유지합니다.

설정 기본값은 `src/autorace_mux/autorace_mux/core.py`, 토픽 연결은 `node.py`에서 확인할 수 있습니다.


## 주차 이후 lane 전용 인계
주차 시작 상태(HOLD/APPROACH/DRIVE)를 받은 뒤에는 parking과 주차 접근용 lane만
제어 후보로 사용한다. 곡선 후진 완료 후 `/parking/lane_handoff` 또는 정상 RELEASE를
받으면 즉시 lane만 선택하며, 횡단보도·신호·감속·OBJ·cone 명령은 0속도 명령까지
모두 제외한다. lane 명령도 만료되면 정지하며 다른 미션으로 넘어가지 않는다.
새 parking 트리거가 정상 시작 상태 또는 새로운 주차 명령을 만들면 parking으로
재진입하고, 다음 완료 때 다시 lane 전용으로 돌아간다. 오래된 주차 명령·상태·인계는
이미 끝난 주차를 다시 켜지 못한다. MUX 재시작 또는 ROS 시간 되감기 전까지 이 정책을 유지한다.
노드 프로세스를 종료하는 방식이 아니라 차량 출력의 제어권을 제한한다.

주차 종료 후 lane 우선권을 단조 시계로 10초 유지한다. 이 시간에는 parking 시작/상태/명령 재수신도 무시하며 재시작 요청을 예약하지 않는다. 10초 후 새 parking 표지를 받고 흰 바닥선 조건을 만족하면 다시 parking으로 전환한다. lane 명령이 끊기면 정지한다. 다른 미션은 주차 이후 lane/parking 반복에서 선택하지 않는다.

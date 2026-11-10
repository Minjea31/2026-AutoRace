# mux_ws

lane과 미션 노드의 주행 명령을 받아 우선순위가 가장 높은 유효 명령 하나를 선택하고, 차량용 `/cmd_vel`로 변환합니다.
입력은 `ackermann_msgs/msg/AckermannDriveStamped`, 출력은 `geometry_msgs/msg/Twist`입니다.

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
| --- | --- |
| 01 | `/mission/crosswalk/command_01` |
| 02 | `/mission/signal/command_02` |
| 03 | `/mission/parking/command_03` |
| 04 | `/mission/cone/command_04` |
| 05 | `/mission/objavoid/command_05` |
| 08 | `/mission/slow/command_08` |
| 기본 | `/mission/lane` |

번호가 작은 유효 미션을 선택합니다. 미션 입력이 없거나 만료되면 lane으로 돌아가고, lane도 유효하지 않으면 정지합니다.
속도 0을 계속 발행하는 미션도 유효한 제어 명령이므로 lane에 제어권을 넘기려면 해당 미션의 발행을 멈춰야 합니다.
기본 입력 만료 시간은 **0.25초**입니다.

## 기본 제어값

- 최대 전진 속도: **0.8m/s**, 최대 후진 속도: **0.3m/s**.
- 조향각 제한과 각속도 제한: 기본 OFF. 비정상 수치와 ±90° 이상의 입력 조향각은 거부합니다.
- 선택된 중앙 조향각을 `각속도 = 속도 × tan(조향각) / 축간거리`로 바꿉니다. LIMO 드라이버가 이를 차체 명령으로 변환합니다.
- 가감속 제한과 전후진 전환 시 정지 처리는 유지합니다.

설정 기본값은 `src/autorace_mux/autorace_mux/core.py`, 토픽 연결은 `node.py`에서 확인할 수 있습니다.

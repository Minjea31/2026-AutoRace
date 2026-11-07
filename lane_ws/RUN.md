# lane_ws 실행 방법 (차선 인식 + PID 제어)

경로는 저장소를 `~/2026-AutoRace`에 clone했다고 가정합니다. 위치가 다르면 경로만 바꿉니다.

| 노드 | 역할 | 출력 |
| --- | --- | --- |
| `lane_lite_node` | 차선 인식 | `/lane_lite/target` |
| `lane_pid_controller` | PID 제어 | `/mission/lane` (+ `/lane_pid/command`, `/lane_pid/state`) |
| mux (실차) / `simul/adapter.sh` (시뮬) | 명령 선택·변환 | `/cmd_vel` |

`/cmd_vel`은 mux(실차) 또는 adapter(시뮬)만 발행합니다. lane_ws는 `/mission/lane`까지만 냅니다.

---

## 0. 빌드

처음 한 번, 그리고 `git pull`이나 `config/*.yaml`을 수정한 뒤에 다시 합니다.

```bash
sudo apt install ros-humble-ackermann-msgs ros-humble-cv-bridge libopencv-dev libyaml-cpp-dev qtbase5-dev  # 처음 한 번
cd ~/2026-AutoRace/lane_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --packages-select autorace_lane_lite autorace_lane_pid --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## 1. 실차

### 1-1. 통신 설정 (차량 PC와 노트북, 처음 한 번)

두 PC가 같은 공유기에 있고 **같은 도메인**을 써야 노트북에서 튜너를 띄울 수 있습니다. 이 공유기는 기기 간 자동 탐색(멀티캐스트)이 안 돼서 IP를 직접 적은 `fastdds_peers.xml`을 씁니다.

1. 두 PC 모두 `~/fastdds_peers.xml`에 설정 파일을 둡니다.
   ```bash
   cp ~/2026-AutoRace/lane_ws/fastdds_peers.xml ~/
   ```
   파일 안의 IP(차량 `192.168.50.97`, 노트북 `192.168.50.170`)가 바뀌면 두 PC의 파일을 모두 고칩니다.
2. 두 PC의 `~/.bashrc` **맨 끝**에 추가합니다.
   ```bash
   export ROS_DOMAIN_ID=31
   export ROS_LOCALHOST_ONLY=0
   export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_peers.xml
   ```
3. **새 터미널(새 ssh)** 에서 확인합니다. 이미 떠 있던 노드는 예전 설정이므로 모두 껐다가 다시 띄웁니다.
   ```bash
   printenv | grep -E "ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|FASTRTPS"   # 31 / 0 / fastdds_peers.xml
   ```
4. 통신 테스트: 차에서 `ros2 run demo_nodes_cpp talker`, 노트북에서 `ros2 run demo_nodes_cpp listener`를 실행해 `I heard`가 나오면 성공입니다.

> 노트북의 이 터미널에서는 `simul/ros_env.sh`를 source하지 않습니다. 시뮬용이라 도메인 42·localhost 전용으로 바꿉니다.

### 1-2. 실행 순서 (차량 PC, ssh 터미널 3개)

**터미널 1: 센서**
```bash
cd ~/2026-AutoRace/sensor_ws && source install/setup.bash
ros2 launch autorace_sensor_bringup sensors.launch.py
```

**터미널 2: 차선 인식 + PID 제어**
```bash
source /opt/ros/humble/setup.bash
source ~/2026-AutoRace/lane_ws/install/setup.bash
ros2 launch autorace_lane_pid lane_pid.launch.py profile:=real
```
둘을 따로 띄우려면 아래처럼 실행합니다.
```bash
ros2 launch autorace_lane_lite lane_lite.launch.py profile:=real            # 차선 인식
ros2 launch autorace_lane_pid lane_pid.launch.py profile:=real lane:=false  # PID 제어만
```
**한 번만** 실행합니다. 두 번 띄우면 `/mission/lane` 발행자가 둘이 됩니다.

**노트북: PID 튜너를 띄우고 ■ 주행 정지 누르기** (1-3 참고)

**터미널 3: mux** (이걸 켜야 차가 움직입니다. 반드시 마지막에)
```bash
cd ~/2026-AutoRace/mux_ws && source install/setup.bash
ros2 launch autorace_mux mux.launch.py
```

`config/real/pid.yaml`은 `active: 1.0`이라, mux를 켜는 순간 차선을 잡고 있으면 바로 출발합니다. 처음에는 바퀴를 띄워 조향 방향을 확인한 뒤 바닥에서 시험합니다.

**종료**는 mux부터 `Ctrl+C`로 끄고 나머지를 끕니다.

### 1-3. 노트북에서 튜너 띄우기

```bash
source /opt/ros/humble/setup.bash
source ~/2026-AutoRace/lane_ws/install/setup.bash
ros2 topic list --no-daemon --spin-time 3 | grep -E "camera|lane"   # 차 토픽이 보이는지

ros2 run autorace_lane_pid lane_pid_tuner --profile real    # PID 튜너 (차의 제어기에 바로 적용)
ros2 run autorace_lane_lite lane_lite_tuner --profile real  # 차선 튜너 (차 카메라 실시간)
```

- PID 창: 슬라이더를 움직이면 차에서 도는 제어기에 바로 적용됩니다. **■ 주행 정지 / ▶ 주행 시작** 버튼이 있습니다.
- 차선 창: 값을 바꾼 뒤 **Apply to running lane_lite_node**를 눌러야 차의 인식 노드에 적용됩니다.
- **Save YAML은 노트북의 소스 파일에 저장**됩니다. 차에도 반영하려면 커밋·푸시 후 차에서 `git pull`하고 0단계 빌드를 하거나, `scp`로 복사한 뒤 차에서 빌드합니다.
  ```bash
  scp ~/2026-AutoRace/lane_ws/src/autorace_lane_pid/config/real/pid.yaml \
      wego@192.168.50.97:~/2026-AutoRace/lane_ws/src/autorace_lane_pid/config/real/
  ```

ssh X11 포워딩(`ssh -Y`)으로 차에서 창을 띄울 수도 있지만, 차선 튜너는 영상 때문에 매우 느립니다. 노트북에서 실행하는 쪽을 권합니다.

### 1-4. 토픽 주기(hz) 점검 (차량 PC, GUI 모두 끈 상태)

| 토픽 | 기대값 | 낮으면 |
| --- | --- | --- |
| `/camera/color/image_raw/compressed` | 약 30 Hz | 카메라 드라이버·USB 문제 |
| `/lane_lite/target` | 카메라와 거의 같게 | 차량 PC가 인식 처리를 못 따라감(CPU) |
| `/mission/lane` | 50 Hz | lane_pid 노드가 안 떠 있음 |
| `/cmd_vel` | 30 Hz | mux가 안 떠 있음 |
| `/odom` | 수십 Hz | `limo_base`·시리얼 문제 |
| `/scan` | 약 10 Hz | 라이다 (주차·라바콘 등에서 사용) |

```bash
ros2 topic hz /camera/color/image_raw/compressed
ros2 topic hz /lane_lite/target
ros2 topic hz /mission/lane
ros2 topic hz /cmd_vel
ros2 topic info /lane_lite/debug     # debug:=false면 "Unknown topic"이 정상
top                                  # lane_lite_node, limo_base CPU % 확인
```

`/lane_lite/debug`가 필요하면(노트북에서 `rqt_image_view`로 볼 때만) `debug:=true`로 띄웁니다. 노트북 차선 튜너는 카메라 영상을 직접 받아 그리므로 `debug`와 상관없습니다.

### 1-5. 상태 확인 명령 (차량 PC)

```bash
ros2 topic hz /camera/color/image_raw/compressed   # 카메라 (~30 Hz)
ros2 topic echo /lane_lite/target --once           # 첫 값 1 = 차선 잡음
ros2 topic hz /mission/lane                        # 약 50 Hz면 제어 출력 중
ros2 topic info /mission/lane -v                   # 발행자 1개(lane_pid_controller)인지
ros2 topic echo /cmd_vel --once                    # mux 출력 (linear.x, angular.z)
ros2 topic echo /lane_pid/state --once             # [추종, 오차, P, I, D, FF, 목표조향, 출력조향, 속도, ...]
```

---

## 2. 시뮬

터미널 3개를 씁니다.

```bash
# 터미널 1: 시뮬
cd ~/2026-AutoRace && ./simul.sh

# 터미널 2: 차선 인식 + PID + PID 튜너 창 (시뮬 DDS 도메인 42를 먼저 불러옴)
source ~/2026-AutoRace/simul/ros_env.sh
source ~/2026-AutoRace/lane_ws/install/setup.bash
ros2 launch autorace_lane_pid lane_pid.launch.py profile:=sim
#   차선 튜너 창도 같이: ... profile:=sim lane_gui:=true

# 터미널 3: /mission/lane -> 시뮬 차량 /cmd_vel
cd ~/2026-AutoRace && ./simul/adapter.sh --no-gui --topic /mission/lane
```

`lane_ws/lane.sh sim|real`, `simul/lane.sh`는 위 launch를 대신 실행하는 단축 스크립트입니다.

---

## 3. launch 인자

| 인자 | 기본값 | 의미 |
| --- | --- | --- |
| `profile` | `sim` | `sim` / `real`. 인식·제어 설정 폴더(`config/<profile>/`)와 `use_sim_time`을 함께 선택 |
| `lane` | `true` | `false`면 인식 노드 없이 PID 제어만 |
| `gui` | sim `true`, real `false` | PID 튜너 창 |
| `lane_gui` | `false` | 차선 튜너 창 |
| `debug` | `false` | `true`면 `/lane_lite/debug`(BEV 검출 영상) 발행. 끄면 토픽도 안 만들고 시각화 계산도 생략(인식 CPU 약 1/3 절약) |
| `enable_vehicle_output` | `true` | `false`면 `/mission/lane` 대신 `/lane_pid/command` 미리보기만 |
| `image_topic` | `/camera/color/image_raw/compressed` | 카메라 토픽 (`CompressedImage`) |

---

## 4. 실차 조향 튜닝 메모

- 실차는 BEV가 짧아 목표점이 약 0.68 m 앞입니다(시뮬은 약 1.2 m). 그래서 시뮬 게인을 그대로 쓰면 커브에서 조향이 절반밖에 안 나옵니다.
- 현재 실차 값(`config/real/pid.yaml`): `kp 1.0`, `lateral_weight 0.3`, `heading_weight 0.15`, `lookahead_weight 1.0`, `kd 0.12`, `max_steering 0.40`, `speed 0.15`. 2026-11-04 실차 bag 기준으로 커브 프레임의 약 절반이 최대 조향 0.40 rad에 닿습니다. 폐루프 주행으로는 아직 확인하지 않았습니다.
- **kp를 1.0보다 올려도 더 꺾이지 않습니다.** 0.40 rad가 LIMO 조향 한계(중심 조향 약 0.41 rad) 근처라서입니다. 최대로 꺾어도 커브를 못 돌면 게인보다 아래를 의심합니다.
  - `speed`를 낮춥니다.
  - 바퀴를 띄운 상태에서 `/cmd_vel`의 `angular.z`가 나오는지, 실제 바퀴가 꺾이는지 확인합니다.
  - 차량 드라이버가 Ackermann 모드인지 확인합니다(`source sensor_ws/install/setup.bash` 후 `ros2 topic echo /limo_status --once`, `motion_mode: 1`).
- 직선에서 좌우로 흔들리면 `kp`를 0.1씩 낮추거나 `kd`를 올립니다. 커브 안쪽을 깎으면 `lookahead_weight`를 낮춥니다.

---

## 5. 자주 막히는 곳

| 증상 | 원인 / 해결 |
| --- | --- |
| `Package 'autorace_lane_lite' not found` | 빌드 실패, 또는 그 터미널에서 `source ~/2026-AutoRace/lane_ws/install/setup.bash`를 안 함 |
| 노트북 `ros2 topic list`에 `/parameter_events`, `/rosout`만 보임 | 도메인·`ROS_LOCALHOST_ONLY`·peers 파일이 두 PC에서 다름. 차 노드를 새 설정으로 다시 띄웠는지 확인 |
| 일부 노드만 보이고 나중에 띄운 노드(mux·lane)는 안 보임 | `fastdds_peers.xml`에 `maxInitialPeersRange`가 없는 예전 파일. 저장소의 `fastdds_peers.xml`로 교체 후 모든 노드 재시작 |
| mux를 켰는데 `/cmd_vel`이 안 나옴 | mux가 다른 설정으로 떠 있음(위 두 줄). mux는 입력이 없어도 정지 명령을 계속 발행하므로 echo가 비어 있으면 통신 문제 |
| `/cmd_vel`은 나오는데 속도 0 | PID 창이 ■ 주행 정지 상태, 차선을 못 잡음(`/lane_pid/state` 첫 값 0), 또는 우선순위가 높은 미션(주차 등)이 정지 명령을 내는 중 |
| `limo_msgs/msg/LimoStatus is invalid` | 그 터미널에서 `source ~/2026-AutoRace/sensor_ws/install/setup.bash`를 안 함 |
| `./simul.sh`가 "Conda의 autorace-simul 환경을 활성화…" 출력 | conda가 PATH에 없음. `simul.sh`는 `~/miniforge3` 등을 자동으로 찾도록 수정됨. 다른 위치면 `conda init bash` |

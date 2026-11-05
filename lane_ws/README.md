# lane_ws — 차선 인식과 차로 유지 제어

| 패키지 | 역할 |
| --- | --- |
| [`autorace_lane_lite`](src/autorace_lane_lite/README.md) | 차선 인식 전용. `/camera/color/image_raw/compressed` → `/lane_lite/target`. 인식 튜너 `lane_lite_tuner` |
| [`autorace_lane_pid`](src/autorace_lane_pid/README.md) | 오차 기반 PID 제어. `/lane_lite/target` → `/mission/lane`. PID 튜너 `lane_pid_tuner` |

인식과 제어는 노드, 설정 파일, 튜너 창이 모두 따로 있습니다. 차선 튜너에는 제어 파라미터가 없고, PID 튜너에는 인식 파라미터가 없습니다.

## 토픽

| 방향 | 토픽 | 타입 | 의미 |
| --- | --- | --- | --- |
| 구독 | `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 카메라 입력. 실차(Orbbec)와 시뮬이 같은 토픽에 JPEG로 발행 |
| 내부 | `/lane_lite/target` | `std_msgs/msg/Float32MultiArray` | 인식 결과(횡오차·헤딩·곡률·신뢰도) |
| 발행 | `/mission/lane` | `ackermann_msgs/msg/AckermannDriveStamped` | 기본 주행 명령. `enable_vehicle_output:=true`일 때만 발행 |
| 발행 | `/lane_pid/command`, `/lane_pid/state` | | 제어 미리보기·상태(튜너 창 표시용) |

카메라 입력은 `CompressedImage` 전용입니다. 토픽 이름을 바꿔야 할 때만 `image_topic:=/다른/토픽`을 지정합니다.
`/mission/lane`은 [팀 아키텍처](../docs/architecture.md)의 기본 주행 명령(m/s·rad, 좌회전 +, `frame_id=base_link`)이며 한 노드만 발행해야 합니다.

---

## 실행 설명서 (시뮬 → 차선 → 제어)

아래 명령은 저장소를 `~/2026-AutoRace`에 clone했다고 가정합니다. 위치가 다르면 경로만 바꾸면 됩니다.

### 0. 처음 한 번: 준비와 빌드

```bash
# 시뮬 Conda 환경(autorace-simul)이 없으면 먼저 simul/docs/SETUP.md 대로 설치
sudo apt install ros-humble-ackermann-msgs ros-humble-cv-bridge libopencv-dev libyaml-cpp-dev qtbase5-dev

cd ~/2026-AutoRace/lane_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --packages-select autorace_lane_lite autorace_lane_pid \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

코드를 받거나(`git pull`) `config/*.yaml`을 고친 뒤에는 이 빌드를 다시 실행합니다.

### 1. 시뮬 실행 (터미널 1)

```bash
cd ~/2026-AutoRace
./simul.sh
```

시뮬 창이 뜨고 출발선에 차가 놓입니다. 카메라 영상이 나오는지 확인하려면:

```bash
./simul/ros2.sh topic hz /camera/color/image_raw/compressed   # 약 30 Hz
```

### 2. 차선 인식 (터미널 2)

**인식만 먼저 확인할 때** (차는 움직이지 않음):

```bash
cd ~/2026-AutoRace/lane_ws
source ../simul/ros_env.sh && source install/setup.bash   # 시뮬과 같은 DDS 도메인(42) 사용
ros2 launch autorace_lane_lite lane_lite.launch.py profile:=sim use_sim_time:=true gui:=true
```

- `gui:=true`이면 차선 튜너 창(`lane_lite_tuner`)이 함께 뜹니다. ROI·BEV·색상 등 인식 값을 여기서 조절합니다.
- 인식 결과 확인: `ros2 topic echo /lane_lite/target` (첫 값 `1`이면 차선을 잡은 상태)
- 영상으로 확인: `ros2 run rqt_image_view rqt_image_view /lane_lite/debug`

`ros2 topic`/`ros2 run` 명령은 위 두 `source`를 한 터미널에서 실행해야 시뮬 토픽이 보입니다.

확인이 끝나면 `Ctrl+C`로 종료하고 3단계로 넘어갑니다. 3단계의 `lane.sh`가 인식 노드를 다시 함께 띄웁니다.

### 3. 제어 (터미널 2, 3)

**터미널 2: 인식 + PID 제어 + PID 튜너 창**

```bash
cd ~/2026-AutoRace
./simul/lane.sh                     # 인식 + PID, /mission/lane 발행
# ./simul/lane.sh lane_gui:=true    # 차선 튜너 창도 같이
# ./simul/lane.sh gui:=false        # PID 튜너 창 없이
```

**터미널 3: `/mission/lane`을 시뮬 차량(`/cmd_vel`)으로 전달**

```bash
cd ~/2026-AutoRace
./simul/adapter.sh --no-gui --topic /mission/lane
```

이제 차가 차선을 따라 주행합니다. PID 튜너 창에서

- 게인·속도 슬라이더를 움직이면 주행 중인 제어기에 바로 적용됩니다.
- **■ 주행 정지 / ▶ 주행 시작** 버튼으로 차를 세우거나 다시 출발시킵니다.
- **Save YAML**로 저장하면 `src/autorace_lane_pid/config/sim/pid.yaml`에 기록됩니다. 다음 실행부터 반영하려면 0단계 빌드를 다시 합니다.

mux를 거쳐 다른 미션과 함께 돌릴 때는 터미널 3 대신 mux를 실행합니다. 이 경우에도 `/mission/lane`은 `lane.sh` 하나만 발행해야 합니다.

### 종료

각 터미널에서 `Ctrl+C`를 누릅니다. 시뮬은 창을 닫아도 됩니다.

### 자주 막히는 곳

| 증상 | 확인 |
| --- | --- |
| `lane.sh`가 "먼저 colcon build…"를 출력 | 0단계 빌드를 안 했거나 실패함 |
| `/lane_lite/target` 첫 값이 계속 `0` | 시뮬이 떠 있는지, 1단계 `topic hz`로 카메라가 나오는지 확인 |
| 인식은 되는데 차가 안 움직임 | 터미널 3(`adapter.sh`)을 실행했는지, PID 창이 "주행 정지" 상태인지 확인 |
| `ros2 topic list`에 시뮬 토픽이 안 보임 | 그 터미널에서 `source ~/2026-AutoRace/simul/ros_env.sh`를 안 함 (시뮬은 `ROS_DOMAIN_ID=42`, localhost 전용) |

---

`build/`, `install/`, `log/`, `artifacts/`는 Git에서 제외됩니다. 루트의 `lane*.yaml`은 과거 GUI로 저장한 인식 설정이며 `lane_lite_tuner`의 Load YAML로 ROI·BEV·차로 폭·색상만 가져올 수 있습니다. `docs/`의 검증 문서는 삭제된 레거시 패키지(`autorace_lane`, `autorace_lane_single`)의 기록입니다.

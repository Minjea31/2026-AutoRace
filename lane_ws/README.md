# lane_ws — 차선 인식과 차로 유지 제어

| 패키지 | 역할 |
| --- | --- |
| [`autorace_lane_lite`](src/autorace_lane_lite/README.md) | 차선 인식 전용. `/camera/color/image_raw/compressed` → `/lane_lite/target`. 인식 튜너 `lane_lite_tuner` |
| [`autorace_lane_pid`](src/autorace_lane_pid/README.md) | 오차 기반 PID 제어. `/lane_lite/target` → `/mission/lane`. PID 튜너 `lane_pid_tuner` |

**실행 방법은 [RUN.md](RUN.md)에 정리돼 있습니다.** 인식과 제어는 노드, 설정 파일, 튜너 창이 모두 따로 있습니다. 차선 튜너에는 제어 파라미터가 없고, PID 튜너에는 인식 파라미터가 없습니다.

## 알고리즘

가까운 경로의 횡오차·방향 오차와 전방 목표점 오차를 결합해 PID로 조향합니다. 조향각·변화율·속도를 제한하며, 현재 deep 실행에서는 `/lane_deep/target`을 받습니다. HSV 인식은 별도 opencv 모드입니다. [1][2]

![lane_ws 알고리즘 개념 그림](../docs/logic/lane_ws.svg)

## References

- [1] K. J. Åström, R. M. Murray, **Feedback Systems**, 2008, Chapter 10: PID Control. [저자 공개 자료](https://www.cds.caltech.edu/~murray/FBS/PID_Control.html). PID·미분 필터·적분 포화 처리의 배경이며 오차 결합과 주차 상태 처리는 자체 구현입니다.
- [2] OpenCV, [HSV 범위 임계값](https://docs.opencv.org/4.x/da/d97/tutorial_threshold_inRange.html), [모폴로지 연산](https://docs.opencv.org/4.x/d9/d61/tutorial_py_morphological_ops.html). HSV 인식의 기본 연산 자료입니다.

## 명령어 모음 (복사해서 쓰기)

저장소를 `~/2026-AutoRace`에 clone했다고 가정합니다. 위치가 다르면 경로만 바꿉니다.

### 빌드 (처음 한 번, `git pull`이나 `config/*.yaml` 수정 후 다시)

```bash
sudo apt install ros-humble-ackermann-msgs ros-humble-cv-bridge libopencv-dev libyaml-cpp-dev qtbase5-dev  # 처음 한 번
cd ~/2026-AutoRace/lane_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --packages-select autorace_lane_lite autorace_lane_pid --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 실차에서 주행 (`ros2 launch`)

```bash
source /opt/ros/humble/setup.bash
source ~/2026-AutoRace/lane_ws/install/setup.bash

# 인식 + 제어 한 번에 (/mission/lane 발행 → mux가 차량으로 전달)
ros2 launch autorace_lane_pid lane_pid.launch.py profile:=real

# 또는 따로 띄우기
ros2 launch autorace_lane_lite lane_lite.launch.py profile:=real           # 차선 인식
ros2 launch autorace_lane_pid lane_pid.launch.py profile:=real lane:=false # PID 제어만
```

- 실차(`profile:=real`)에서는 PID 튜너 창이 기본으로 안 뜹니다(ssh에는 화면이 없음). 노트북에서 `ros2 run autorace_lane_pid lane_pid_tuner --profile real`로 띄웁니다.
- 실차 PID(`config/real/pid.yaml`)는 폐루프 검증 전 값입니다(0.15 m/s). 노트북 PID 창의 **■ 주행 정지** 상태에서 mux를 켜고, 인식을 확인한 뒤 출발하세요.

### 시뮬에서 주행 (터미널 3개)

```bash
cd ~/2026-AutoRace && ./simul.sh                                          # 1: 시뮬
source ~/2026-AutoRace/simul/ros_env.sh                                   # 2: 시뮬 DDS 도메인(42) 먼저
source ~/2026-AutoRace/lane_ws/install/setup.bash
ros2 launch autorace_lane_pid lane_pid.launch.py profile:=sim             #    인식 + PID + PID 튜너 창
cd ~/2026-AutoRace && ./simul/adapter.sh --no-gui --topic /mission/lane   # 3: /mission/lane -> 시뮬 차량
```

`lane_ws/lane.sh sim|real`, `simul/lane.sh`는 위 `ros2 launch`를 대신 실행해 주는 단축 스크립트입니다(그대로 동작).

### 자주 쓰는 launch 인자

| 인자 | 의미 |
| --- | --- |
| `profile:=sim` / `profile:=real` | 인식·제어 설정 폴더와 `use_sim_time`을 함께 선택 |
| `lane:=false` | 인식 노드 없이 PID 제어만 |
| `lane_gui:=true` | 차선 인식 튜너 창도 같이 열기 |
| `gui:=true` / `gui:=false` | PID 튜너 창 열기/닫기 (기본: sim 켬, real 끔) |
| `enable_vehicle_output:=false` | `/mission/lane` 대신 `/lane_pid/command` 미리보기만 발행 (기본 true) |
| `image_topic:=/다른/토픽` | 카메라 토픽 변경 (`CompressedImage`) |

### 인식만 확인 (차는 안 움직임)

```bash
cd ~/2026-AutoRace/lane_ws && source install/setup.bash   # 시뮬이면 먼저: source ../simul/ros_env.sh
ros2 launch autorace_lane_lite lane_lite.launch.py profile:=real gui:=true   # 시뮬: profile:=sim
ros2 topic echo /lane_lite/target                         # 첫 값 1 = 차선 잡음
ros2 run rqt_image_view rqt_image_view /lane_lite/debug   # 검출 영상
```

### 튜닝

```bash
cd ~/2026-AutoRace/lane_ws && source install/setup.bash
ros2 run autorace_lane_lite lane_lite_tuner --profile real                     # 인식 (실행 중인 카메라)
ros2 run autorace_lane_lite lane_lite_tuner --profile real --bag <bag 폴더>    # 인식 (bag 재생)
ros2 run autorace_lane_pid lane_pid_tuner --profile real                       # 제어 (실행 중인 제어기)
```

**Save YAML**은 `src/<패키지>/config/<profile>/`의 소스 파일에 저장합니다. launch에 반영하려면 다시 빌드합니다. 실행 중인 노드에는 저장 없이 바로 적용됩니다.

### bag 일괄 평가 · 테스트

```bash
cd ~/2026-AutoRace/lane_ws && source install/setup.bash
ros2 run autorace_lane_lite lane_lite_eval <bag 폴더> src/autorace_lane_lite/config/real/lane.yaml /tmp/lane_eval --every 150
# /tmp/lane_eval/frames.csv(프레임별 상태·횡오차·탈락 이유) + frame_N.jpg + 요약 1줄
./build/autorace_lane_lite/test_lane && ./build/autorace_lane_pid/test_pid
```

## 시뮬 / 실차 전환

두 패키지 모두 `config/sim/`과 `config/real/`을 따로 두고, **`profile` 인자 하나**로 인식·제어 설정과 `use_sim_time`을 같이 바꿉니다.

```bash
ros2 launch autorace_lane_pid lane_pid.launch.py profile:=sim    # 시뮬: use_sim_time (먼저 source simul/ros_env.sh)
ros2 launch autorace_lane_pid lane_pid.launch.py profile:=real   # 실차
```

## 토픽

| 방향 | 토픽 | 타입 | 의미 |
| --- | --- | --- | --- |
| 구독 | `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 카메라 입력. 실차(Orbbec)와 시뮬이 같은 토픽에 JPEG로 발행 |
| 내부 | `/lane_lite/target` | `std_msgs/msg/Float32MultiArray` | 인식 결과(횡오차·헤딩·곡률·신뢰도) |
| 발행 | `/mission/lane` | `ackermann_msgs/msg/AckermannDriveStamped` | 기본 주행 명령. 기본 발행(`enable_vehicle_output:=false`면 안 함) |
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
ros2 launch autorace_lane_lite lane_lite.launch.py profile:=sim gui:=true
```

- `gui:=true`이면 차선 튜너 창(`lane_lite_tuner`)이 함께 뜹니다. ROI·BEV·색상 등 인식 값을 여기서 조절합니다.
- 인식 결과 확인: `ros2 topic echo /lane_lite/target` (첫 값 `1`이면 차선을 잡은 상태)
- 영상으로 확인: `ros2 run rqt_image_view rqt_image_view /lane_lite/debug`

`ros2 topic`/`ros2 run` 명령은 위 두 `source`를 한 터미널에서 실행해야 시뮬 토픽이 보입니다.

확인이 끝나면 `Ctrl+C`로 종료하고 3단계로 넘어갑니다. 3단계의 launch가 인식 노드를 다시 함께 띄웁니다.

### 3. 제어 (터미널 2, 3)

**터미널 2: 인식 + PID 제어 + PID 튜너 창**

```bash
cd ~/2026-AutoRace
source simul/ros_env.sh && source lane_ws/install/setup.bash
ros2 launch autorace_lane_pid lane_pid.launch.py profile:=sim                  # 인식 + PID, /mission/lane 발행
# ros2 launch autorace_lane_pid lane_pid.launch.py profile:=sim lane_gui:=true # 차선 튜너 창도 같이
# ros2 launch autorace_lane_pid lane_pid.launch.py profile:=sim gui:=false     # PID 튜너 창 없이
```

**터미널 3: `/mission/lane`을 시뮬 차량(`/cmd_vel`)으로 전달**

```bash
cd ~/2026-AutoRace
./simul/adapter.sh --no-gui --topic /mission/lane
```

이제 차가 차선을 따라 주행합니다. PID 튜너 창에서

- 게인·속도 슬라이더를 움직이면 주행 중인 제어기에 바로 적용됩니다.
- **■ 주행 정지 / ▶ 주행 시작** 버튼으로 차를 세우거나 다시 출발시킵니다.
- **Save YAML**로 저장하면 `src/autorace_lane_pid/config/sim/pid.yaml`(실차는 `config/real/pid.yaml`)에 기록됩니다. 다음 실행부터 반영하려면 0단계 빌드를 다시 합니다.

mux를 거쳐 다른 미션과 함께 돌릴 때는 터미널 3 대신 mux를 실행합니다. 이 경우에도 `/mission/lane`은 이 launch 하나만 발행해야 합니다.

### 종료

각 터미널에서 `Ctrl+C`를 누릅니다. 시뮬은 창을 닫아도 됩니다.

### 자주 막히는 곳

| 증상 | 확인 |
| --- | --- |
| `Package 'autorace_lane_pid' not found` | 0단계 빌드를 안 했거나 실패함, 또는 그 터미널에서 `source lane_ws/install/setup.bash`를 안 함 |
| `/lane_lite/target` 첫 값이 계속 `0` | 시뮬이 떠 있는지, 1단계 `topic hz`로 카메라가 나오는지 확인 |
| 인식은 되는데 차가 안 움직임 | 터미널 3(`adapter.sh`)을 실행했는지, PID 창이 "주행 정지" 상태인지 확인 |
| `ros2 topic list`에 시뮬 토픽이 안 보임 | 그 터미널에서 `source ~/2026-AutoRace/simul/ros_env.sh`를 안 함 (시뮬은 `ROS_DOMAIN_ID=42`, localhost 전용) |

---

`build/`, `install/`, `log/`, `artifacts/`는 Git에서 제외됩니다. 루트의 `lane*.yaml`은 과거 GUI로 저장한 인식 설정이며 `lane_lite_tuner`의 Load YAML로 ROI·BEV·차로 폭·색상만 가져올 수 있습니다. `docs/`의 검증 문서는 삭제된 레거시 패키지(`autorace_lane`, `autorace_lane_single`)의 기록입니다.

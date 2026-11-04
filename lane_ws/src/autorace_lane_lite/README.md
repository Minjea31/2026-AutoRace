# autorace_lane_lite

**간결한 단일 차로 인식 패키지**입니다. 카메라 영상에서 차선을 검출해 목표점(`/lane_lite/target`)을 내고, 선택적으로 Pure Pursuit 제어기가 `/mission/lane`을 발행합니다.

## 팀원용 빠른 실행 (GUI 없음, 시뮬)

이 패키지만 있으면 **인식 노드 + 제어기만 빌드**됩니다(튜너 GUI·bag 평가 도구는 개발 PC 전용 `autorace_lane_single` 소스를 재사용하므로 그 패키지가 없으면 자동으로 빠집니다).

```bash
# 빌드 (한 번)
cd ~/2026-AutoRace/lane_ws          # 각자 clone 위치
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-ackermann-msgs ros-humble-cv-bridge libopencv-dev libyaml-cpp-dev  # 없을 때만
PYTHONNOUSERSITE=1 colcon build --packages-select autorace_lane_lite --cmake-args -DCMAKE_BUILD_TYPE=Release

# 터미널 1: 시뮬
cd ~/2026-AutoRace && ./simul.sh

# 터미널 2: 차선 인식 (+ 제어기, /mission/lane 발행)
source ~/2026-AutoRace/simul/ros_env.sh
source ~/2026-AutoRace/lane_ws/install/setup.bash
ros2 launch autorace_lane_lite lane_lite.launch.py profile:=sim use_sim_time:=true \
  controller:=true enable_vehicle_output:=true

# 터미널 3: /mission/lane -> /cmd_vel (mux 대신 시뮬로 바로 보낼 때)
cd ~/2026-AutoRace && ./simul/adapter.sh --no-gui --topic /mission/lane
```

- **차선 검출만** 필요하면 터미널 2에서 `controller:=true enable_vehicle_output:=true`를 빼고 `/lane_lite/target`만 구독합니다(`ros2 topic echo /lane_lite/target`).
- 검출 결과 영상 확인(선택): `ros2 run rqt_image_view rqt_image_view /lane_lite/debug`
- 자기 미션 노드와 함께 돌릴 때는 터미널 3 대신 mux를 실행합니다. `/mission/lane`은 이 노드만 발행해야 합니다.

| 파일 | 역할 | 줄 수 |
| --- | --- | --- |
| `src/lane.cpp` | 설정(41개 파라미터) + 인식 코어 + 시각화 | 약 860 |
| `src/node.cpp` | 카메라 → `/lane_lite/target` (인식 전용, 주행 명령 없음) | 약 180 |
| `src/controller.cpp` | Pure Pursuit 제어기 (ROS 파라미터, GUI와 분리) | 약 90 |
| `src/tuner.cpp` | Qt 튜너 (`autorace_lane_single`이 있을 때만 빌드) | 약 680 |
| `src/eval.cpp` | bag 일괄 평가 / 단일 이미지 확인 (`autorace_lane_single`이 있을 때만 빌드) | 약 100 |

## 처리 단계

1. ROI ∩ BEV 사다리꼴 → BEV 투영 (경계는 `BORDER_REPLICATE`로 가짜 에지 방지)
2. BEV에서 HSV 노랑 / 흰색. 흰색은 **Top-hat**(주변 노면보다 밝은 얇은 띠)만 인정 → 넓은 반사광·조명 변화 제거
3. 색상별 **띠 두께 필터**(행 또는 열 방향 run ≤ `line_width_max`×1.45) + 최소 면적 → 유광 바닥 반사 덩어리 제거
4. 시작점: 이전 추적 경계의 가까운 끝, 없으면 하단 히스토그램 피크 중 **차로 폭에 맞는 좌우 쌍**
5. **차선 방향을 따라가는 슬라이딩 윈도우**(세로/가로 무관, 시작점에서 양방향). 관측된 윈도우 중심만 사용. 빈 윈도우가 `max_misses`개 미만이면 예측 방향으로 계속 진행하므로 **흰 점선**(시뮬 3차선: 칠 0.20 m / 빈칸 0.15 m ≈ 윈도우 1.5개)도 한 경계로 이어집니다. 피팅 후 경로는 항상 **가까운 쪽 → 먼 쪽** 순서로 맞춥니다(뒤집히면 heading이 ±π가 되고 중앙 오프셋이 반대편으로 감)
6. 현의 회전 좌표계에서 3차 곡선 피팅(짧은 관측일수록 굽힘 억제), 안 맞으면 이상치 1개 제거 → 먼 쪽부터 잘라냄
7. 새로 잡는 **흰 가로선**(가까운 1/3이 진행 방향에서 60° 넘게 누운 흰 경계: 정지선·횡단보도 테두리)은 경계로 채택하지 않고, 추적 중인 경계가 그쪽으로 옮겨 가지도 않음. 이미 추적 중인 경계가 커브를 따라 점점 눕는 것은 허용(커브 유지). 좌우 쌍 폭 검사(불일치 시 추적과 더 어긋난 쪽 제외; 차로 폭 학습은 `lane_width_px`의 ±10% 안에서, 설정값에 가까운 쌍으로만 → 노면 글자·화살표와 짝지어져 폭이 줄어드는 것 방지), 차량 반대편 경계 역할 교정, 추적(평활·3프레임 확인·색상 일관성·`track_memory` 동안 유지)
8. 중앙 경로: 양쪽이면 중점, 한쪽이면 법선 방향 `lane_width_px/2` 오프셋 → 목표점은 **차량에서 같은 거리만큼 떨어진 경로 위 점**(직선에선 `lookahead_row`, 곡선에선 자동으로 안쪽)

## 토픽

| 방향 | 토픽 | 내용 |
| --- | --- | --- |
| 구독 | `/camera/color/image_raw/compressed` (기본) | `image_topic`, `transport:=raw` 변경 가능 |
| 발행 | `/lane_lite/target` | `Float32MultiArray` `[valid, lateral_m, heading_rad, lookahead_m, curvature, confidence, left_state, right_state]` (좌측 +, state 0=LOST 1=HELD 2=DETECTED) |
| 발행 | `/lane_lite/debug` | BEV 경계·경로 영상 (구독자가 있을 때만 생성) |
| 발행 | `/lane_lite/command` | 제어기 미리보기 `AckermannDriveStamped` |
| 발행 | `/mission/lane` | `enable_vehicle_output:=true`일 때만 |

## 설정 파일 — 시뮬과 실차를 폴더로 분리

```
config/
├── sim/                      시뮬 전용 (폐루프 검증된 값, 기본 카메라 기준)
│   ├── lane.yaml             ./simul.sh 기본 카메라. 3차선 흰 점선 때문에 흰색 사용
│   ├── lane_camera.yaml      급커브용 카메라(pitch 0.35) 시뮬 옵션용. 현재 main 시뮬에는 해당 옵션 없음
│   └── controller.yaml       제어기 게인 (시뮬)
└── real/                     실차 전용
    ├── lane.yaml             2026-11-04 bag(lane_20261104_192410) BEV, lane_width_px 470
    └── controller.yaml       제어기 게인 (실차, 시작값은 시뮬과 동일)
```

- launch와 튜너 모두 `profile`(sim / real)로 폴더를 고릅니다. 한쪽을 튜닝해도 다른 쪽은 바뀌지 않습니다.
- 튜너를 `--profile`로 열면 **Save YAML의 기본 저장 위치가 해당 프로필의 소스 파일**(`src/autorace_lane_lite/config/<profile>/lane.yaml`)입니다. install 폴더로 열어도 소스 경로로 저장합니다. 창 제목에 현재 파일이 표시됩니다. 저장 후 다시 빌드해야 launch(설치 경로)에 반영됩니다. 실행 중인 노드에는 **Apply to running lane_lite_node**로 바로 적용할 수 있습니다.
- `real/lane.yaml`의 **`bev_near_m`/`bev_far_m`는 자리표시값**입니다. 실차 폐루프 전에 실측해야 합니다.
- 코드 기본값(YAML 없이 실행할 때)은 시뮬 값과 같습니다(`lane_width_px` 240 등). 실차는 반드시 `--profile real`로 엽니다.

기존 GUI로 저장한 YAML(`lane:` / `sliding_window:`)도 Load YAML로 열 수 있습니다. ROI·BEV·차로 폭·HSV 색 임계값만 가져오고(HLS로 저장된 색은 무시) 나머지는 기본값을 씁니다.

튜닝 순서: **BEV(직선에서 두 선이 평행·세로) → `lane_width_px`(BEV를 바꾸면 반드시 다시 측정: 직선에서 두 선 사이 픽셀) → 색상(Binary 비교 화면) → `line_width_max`/`white_tophat`(반사광) → 윈도우 → 추적**.
`lane_width_px`가 실제보다 작으면 반대쪽 선이 폭 검사에서 탈락하고, 중앙선이 한쪽 선에서 `lane_width_px/2`만큼만 떨어진 곳(도로의 1/4 지점)에 생깁니다. 진단 창의 "차로 폭 학습값"과 BEV에서 두 선 사이 거리를 비교하세요.

## 빌드

```bash
cd /home/a/dev/2026-AutoRace/lane_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --packages-select autorace_lane_lite --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
./build/autorace_lane_lite/test_lane          # 합성 영상 단위 테스트 8개
```

## 아래는 개발 PC 전용 (튜너 GUI 필요: `autorace_lane_single`을 `lane_ws/src`에 함께 둔 경우)

GUI 튜너를 띄우려면 launch에 `gui:=true`를 추가합니다(기본값 false).

## 검증 1 — bag (GUI)

```bash
cd /home/a/dev/2026-AutoRace/lane_ws && source install/setup.bash
ros2 run autorace_lane_lite lane_lite_tuner --profile real \
  --bag /mnt/Ubuntu_Data/autorace_bags/lane_1104_ctrack/lane_20261104_192410
# 특정 프레임부터: --frame 1189   /  시뮬 값: --profile sim   /  임의 파일: --config 경로
```

Play/Pause, 이전/다음 프레임, 슬라이더 탐색, ROI·BEV 드래그, 슬라이더 조정이 기존 튜너와 같습니다. 아래 진단 창에서 좌/우 상태(DETECTED/HELD/LOST), 관측 윈도우 수, RMS, 탈락 이유, 제어기로 나가는 값(읽기 전용)을 확인합니다. 조정한 값은 **Save YAML**로 저장합니다(기본 위치 = 프로필 소스 파일).

## 검증 2 — bag 일괄 수치

```bash
CFG=$(ros2 pkg prefix autorace_lane_lite)/share/autorace_lane_lite/config
ros2 run autorace_lane_lite lane_lite_eval /mnt/Ubuntu_Data/autorace_bags/lane_1104_ctrack/lane_20261104_192410 \
  $CFG/real/lane.yaml /tmp/lane_lite_eval --every 150
# frames.csv(프레임별 상태·횡오차) + frame_N.jpg(원본|특징|탐색|경로) + 요약 1줄
ros2 run autorace_lane_lite lane_lite_eval some_image.png $CFG/sim/lane.yaml out.png   # 이미지 한 장
```

## 검증 3 — 시뮬 폐루프 (터미널 3개)

```bash
# 터미널 1: 시뮬 (기본 카메라). --lane-camera를 붙이면 급커브용 카메라
cd /home/a/dev/2026-AutoRace && source simul/env.sh && ./simul.sh
#   (conda가 PATH에 없으면 ./simul.sh만으로는 "autorace-simul 환경을 활성화" 오류가 납니다)

# 터미널 2: 인식 + 제어기 (/mission/lane 발행). 튜너도 띄우려면 gui:=true
source /home/a/dev/2026-AutoRace/simul/ros_env.sh
source /home/a/dev/2026-AutoRace/lane_ws/install/setup.bash
ros2 launch autorace_lane_lite lane_lite.launch.py profile:=sim \
  use_sim_time:=true controller:=true enable_vehicle_output:=true
#   --lane-camera 시뮬이면 뒤에 추가:
#   config_file:=$(ros2 pkg prefix autorace_lane_lite)/share/autorace_lane_lite/config/sim/lane_camera.yaml

# 터미널 3: /mission/lane -> /cmd_vel 어댑터
cd /home/a/dev/2026-AutoRace && ./simul/adapter.sh --no-gui --topic /mission/lane
```

다른 출발 위치: 터미널 1에서 `./simul.sh --mission signal` 등.

**3차선 흰 점선만 확인**(정차 차량 없는 맵, 터미널 1만 바꿈):
```bash
cd /home/a/dev/2026-AutoRace && source simul/env.sh
sed 's/^  occupied: .*/  occupied: {}/' simul/src/limo_parking_sim/config/parking_map.yaml > /tmp/empty_map.yaml
./simul.sh --map /tmp/empty_map.yaml --spawn 8.0 0.94 3.14159   # 가운데 차로(양쪽 점선). 아래 0.42 / 위 1.46
```
실행 중 제어 게인 변경: `ros2 param set /lane_lite_controller speed 0.3`. 인식 값 출력 확인: `ros2 topic echo /lane_lite/target`.
기존 `lane.sh`(lane_pd)와 동시에 실행하면 둘 다 `/mission/lane`을 발행하므로 하나만 실행합니다.

## 검증 4 — 실차 (주행 출력 없이 먼저)

```bash
ros2 launch autorace_lane_lite lane_lite.launch.py profile:=real transport:=compressed
# 다른 터미널: ros2 run rqt_image_view rqt_image_view /lane_lite/debug
```

`real/lane.yaml`의 `bev_near_m`/`bev_far_m`를 실측해 넣기 전에는 `controller:=true enable_vehicle_output:=true`를 쓰지 않습니다.

## 검증 결과 (2026-11-04, 이 PC)

- 2026-11-04 bag `lane_20261104_192410` 5051프레임(`real/lane.yaml`): `lane_width_px` 240 → 470으로 양쪽 경계 5% → 48%, 흰 가로선 채택 제거. 직진·횡단보도·정지선·3차선 점선 구간은 양쪽 경계로 도로 중앙을 냄. 나머지 단일 경계 구간은 급커브·라바콘·손으로 든 장면.
- 이전 bag 3311프레임(수동 주행, 차선 밖·벽 장면 포함, 당시 BEV 기준): 유효 99.5%, 양쪽 경계 66%, CV 평균 7.6 ms. 같은 BEV를 준 legacy는 유효 80%·정지 23%·CV 19 ms. 유광 바닥의 넓은 반사광이 일부 남아 단일 경계 구간에서 횡오차가 튀는 프레임(>5 cm)이 약 5% 있습니다.
- 시뮬 폐루프(headless, 별도 DDS 도메인): `track` 출발 → 우상단 코너 → 우측 직선 → 우하단 코너를 기본/`--lane-camera` 두 카메라 모두 주행. `signal` 출발 → 합류 S자 → 우측 지그재그 경계 → 코너 주행. 세 경우 모두 하단 **장애물 미션 구간의 정차 LIMO**에 막혀 정지(장애물 회피·갈림길은 이 패키지 범위 밖).
- 3차선 흰 점선(2026-11-04 추가 확인, 정차 차량 없는 맵): `track` 출발 → 우하단 코너 → 3차선 아래 차로를 노랑+점선 양쪽 DETECTED로 통과(반복 3회), 가운데 차로(양쪽 점선)·위 차로 출발도 점선 구간을 횡오차 0.01 m 이내로 주행. `signal` 출발·`--lane-camera`도 같은 구간까지 통과. 좌하단 코너 (0.7, 1.9)에서 정지하는 문제는 흰색 켬/끔과 무관하게 남아 있습니다.
- 실차 폐루프는 검증하지 않았습니다.

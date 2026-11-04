# autorace_lane_lite

**간결한 단일 차로 인식 패키지**입니다. 카메라 영상에서 차선을 검출해 목표점을 `/lane_lite/target`으로 발행합니다. **주행 명령(`/mission/lane`)은 발행하지 않습니다** — 제어기는 아직 포함하지 않았습니다.

## 실행 (GUI 없음, 시뮬)

모든 명령은 **저장소 루트(clone한 `2026-AutoRace` 폴더)**에서 실행합니다.

```bash
# 빌드 (한 번)
cd lane_ws
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-cv-bridge libopencv-dev libyaml-cpp-dev  # 없을 때만
PYTHONNOUSERSITE=1 colcon build --packages-select autorace_lane_lite --cmake-args -DCMAKE_BUILD_TYPE=Release
cd ..

# 터미널 1: 시뮬
./simul.sh

# 터미널 2: 차선 인식
source simul/ros_env.sh
source lane_ws/install/setup.bash
ros2 launch autorace_lane_lite lane_lite.launch.py profile:=sim use_sim_time:=true

# 터미널 3: 확인 (같은 두 줄 source 후)
ros2 topic echo /lane_lite/target
ros2 run rqt_image_view rqt_image_view /lane_lite/debug   # 검출 결과 영상(선택)
```

- 기본 profile은 `sim`입니다(`config/sim/lane.yaml`).
- 이 패키지만으로는 **인식 노드(`lane_lite_node`)만 빌드**됩니다. 튜너 GUI·bag 평가 도구(`src/tuner.cpp`, `src/eval.cpp`)는 개발 PC 전용 패키지의 소스를 재사용하므로 그 패키지가 있을 때만 빌드되고, 그때만 `gui:=true`를 쓸 수 있습니다.

## 토픽

| 방향 | 토픽 | 내용 |
| --- | --- | --- |
| 구독 | `/camera/color/image_raw/compressed` (기본) | `image_topic`, `transport:=raw` 변경 가능 |
| 발행 | `/lane_lite/target` | `std_msgs/Float32MultiArray` |
| 발행 | `/lane_lite/debug` | BEV 경계·경로 영상 (구독자가 있을 때만 생성) |

`/lane_lite/target` 배열 순서:

| 인덱스 | 이름 | 의미 |
| --- | --- | --- |
| 0 | `valid` | 1이면 유효 |
| 1 | `lateral_m` | 목표점의 횡방향 위치 (m, 좌측 +) |
| 2 | `heading_rad` | 목표점에서 경로 방향 (rad, 좌측 +) |
| 3 | `lookahead_m` | 목표점까지 전방 거리 (m) |
| 4 | `curvature` | 경로 곡률 |
| 5 | `confidence` | 0~1 |
| 6, 7 | `left_state`, `right_state` | 0=LOST, 1=HELD, 2=DETECTED |

## 파일

| 파일 | 역할 |
| --- | --- |
| `src/lane.cpp` | 설정(41개 파라미터) + 인식 코어 + 시각화 |
| `src/node.cpp` | 카메라 → `/lane_lite/target` |
| `src/tuner.cpp` | Qt 튜너 (개발 PC 전용 패키지가 있을 때만 빌드) |
| `src/eval.cpp` | bag 일괄 평가 / 단일 이미지 확인 (위와 같음) |
| `config/sim/lane.yaml` | 시뮬 기본 카메라용 (`./simul.sh`) |
| `config/sim/lane_camera.yaml` | 급커브용 카메라(pitch 0.35) 설정. 현재 main 시뮬에는 해당 카메라 옵션 없음 |

다른 설정 파일: `config_file:=/절대/경로/lane.yaml`. 실행 중 파라미터 변경은 `ros2 param set /lane_lite_node <이름> <값>`.

## 처리 단계

1. ROI ∩ BEV 사다리꼴 → BEV 투영 (경계는 `BORDER_REPLICATE`로 가짜 에지 방지)
2. BEV에서 HSV 노랑 / 흰색. 흰색은 **Top-hat**(주변 노면보다 밝은 얇은 띠)만 인정 → 넓은 반사광·조명 변화 제거
3. 색상별 **띠 두께 필터**(행 또는 열 방향 run ≤ `line_width_max`×1.45) + 최소 면적 → 유광 바닥 반사 덩어리 제거
4. 시작점: 이전 추적 경계의 가까운 끝, 없으면 하단 히스토그램 피크 중 **차로 폭에 맞는 좌우 쌍**
5. **차선 방향을 따라가는 슬라이딩 윈도우**(세로/가로 무관, 시작점에서 양방향). 관측된 윈도우 중심만 사용. 빈 윈도우가 `max_misses`개 미만이면 예측 방향으로 계속 진행하므로 **흰 점선**(시뮬 3차선: 칠 0.20 m / 빈칸 0.15 m ≈ 윈도우 1.5개)도 한 경계로 이어집니다. 피팅 후 경로는 항상 **가까운 쪽 → 먼 쪽** 순서로 맞춥니다(뒤집히면 heading이 ±π가 되고 중앙 오프셋이 반대편으로 감)
6. 현의 회전 좌표계에서 3차 곡선 피팅(짧은 관측일수록 굽힘 억제), 안 맞으면 이상치 1개 제거 → 먼 쪽부터 잘라냄
7. 새로 잡는 **흰 가로선**(가까운 1/3이 진행 방향에서 60° 넘게 누운 흰 경계: 정지선·횡단보도 테두리)은 경계로 채택하지 않고, 추적 중인 경계가 그쪽으로 옮겨 가지도 않음. 이미 추적 중인 경계가 커브를 따라 점점 눕는 것은 허용(커브 유지). 좌우 쌍 폭 검사(불일치 시 추적과 더 어긋난 쪽 제외; 차로 폭 학습은 `lane_width_px`의 ±10% 안에서, 설정값에 가까운 쌍으로만 → 노면 글자·화살표와 짝지어져 폭이 줄어드는 것 방지), 차량 반대편 경계 역할 교정, 추적(평활·3프레임 확인·색상 일관성·`track_memory` 동안 유지)
8. 중앙 경로: 양쪽이면 중점, 한쪽이면 법선 방향 `lane_width_px/2` 오프셋 → 목표점은 **차량에서 같은 거리만큼 떨어진 경로 위 점**(직선에선 `lookahead_row`, 곡선에선 자동으로 안쪽)

## 테스트

```bash
./build/autorace_lane_lite/test_lane   # 합성 영상 단위 테스트 8개
```

## 검증 결과 (2026-11-04, 시뮬)

- 시뮬 `config/sim/lane.yaml`: 트랙 코너·3차선 흰 점선 구간에서 경계 검출 확인.
- 장애물 미션 출발점(정지 상태): 차로 중앙·좌우 0.12~0.22 m 치우침·±0.3 rad 틀어짐 모두 좌우 경계 DETECTED, 횡오차 부호·크기 일치. 단, 차가 틀어져 옆 차로가 정면에 보이면 그 차로를 따라갑니다(원래 차로를 기억하지 않음).

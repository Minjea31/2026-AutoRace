# lane_fusion_ws — 차선 인식 퓨전 버전 (OpenCV 주 검출 + U-Net 비동기 후보 퓨전)

`lane_ws`(HSV 차선 인식 + PID)와 `lanedeep_ws`(U-Net 차선 분할)를 **복사해서 합친 별도 워크스페이스**입니다. 원본 두 워크스페이스는 수정하지 않았습니다. 패키지·실행 파일·토픽 이름을 모두 바꿔서 원본과 같은 셸에서 source해도 섞이지 않습니다. 단, 둘 다 `/mission/lane`을 내므로 **주행은 둘 중 하나만** 띄웁니다.

```text
/camera/color/image_raw/compressed
  ├─ lane_fusion_deep_mask (U-Net) ── /lane_fusion/deep_mask  (mono8 0=배경 1=노랑 2=흰 점선, 카메라 header 그대로)
  └─ lane_fusion_node ── HSV 매 프레임 + 딥(odom 정합) 후보 퓨전 ── /lane_fusion/target
                                                        └─ lane_fusion_pid_controller ── /mission/lane → mux
```

| 패키지 | 원본 | 실행 파일 |
| --- | --- | --- |
| `autorace_lane_fusion` | `lane_ws/src/autorace_lane_lite` 기반, 퓨전 파이프라인으로 재설계 | `lane_fusion_node`, `lane_fusion_tuner`, `lane_fusion_eval` |
| `autorace_lane_fusion_pid` | `lane_ws/src/autorace_lane_pid` (입력만 `/lane_fusion/target`) | `lane_fusion_pid_controller`, `lane_fusion_pid_tuner` |
| `autorace_lane_fusion_deep` | `lanedeep_ws`의 추론 코드 + 마스크 전용 노드 | `lane_fusion_deep_mask` |
| `autorace_lane_deep_lite` | (보관용) 딥 마스크만 쓰는 실험 패키지. 주행에 쓰지 않음 | — |

모델: `src/autorace_lane_fusion_deep/model/best.pt`(lanedeep 학습 결과, 640×480)와 ONNX 두 가지. `lane_640_mask.onnx`(기본)는 정규화·softmax·argmax를 모델 안에 넣어 Jetson에서 후처리가 GPU로 갑니다.

## 알고리즘

현재 실차의 `lane_mode:=deep` 경로는 **딥 마스크 → deep_lite 기하 처리 → lane_ws PID**입니다. 아래의 `fusion` 모드와 구분합니다.

| 단계 | 현재 deep 모드의 동작 |
| --- | --- |
| 딥 분할 | MobileNetV3-Small 인코더와 depthwise separable U-Net 디코더로 카메라 픽셀을 배경·노랑·흰색으로 분류합니다. TensorRT/ONNX/PyTorch 실행은 모델 경로에 따라 선택합니다. [1][2] |
| 경계·중앙 경로 | 클래스 마스크를 BEV로 변환하고 경계 탐색·시간 추적을 수행합니다. 양쪽 경계의 중앙 또는 한쪽 경계와 차로 폭으로 중앙 경로·목표점·횡오차·방향·곡률·신뢰도를 계산합니다. 유실 시 odom으로 이전 경로를 짧게 유지합니다. |
| 지그재그 | 노란 윤곽의 convex hull과 convexity defects로 반복된 홈을 찾습니다. 0=없음, 1=한쪽, 2=양쪽을 출력하는 자체 형상 판정입니다. [3] |
| 조향 제어 | `/lane_deep/target`을 `lane_ws`의 PID가 구독합니다. 이 워크스페이스가 직접 차량 명령을 만드는 단계는 아닙니다. |

U-Net과 MobileNetV3는 모델 구조의 참고문헌입니다. 차선 데이터 학습, 두 구조의 결합, 경계 추적과 미션 연결은 저장소 구현입니다.

`lane_mode:=fusion`에서는 별도 `lane_fusion_node`가 HSV 후보와 딥 후보를 차량 이동량으로 정합하고, 기하·차로 폭·추적 연속성으로 선택/융합합니다. 이 후보 융합은 자체 휴리스틱입니다. `autorace_lane_fusion_pid`가 `/lane_fusion/target`을 제어하며 현재 deep 실행에는 두 노드가 뜨지 않습니다.

구현: [분할 모델](src/autorace_lane_fusion_deep/autorace_lane_fusion_deep/model.py), [mask_node.py](src/autorace_lane_fusion_deep/autorace_lane_fusion_deep/mask_node.py), [deep_lite](src/autorace_lane_deep_lite/src/deep_lane.cpp), [지그재그](src/autorace_lane_deep_lite/src/zigzag.cpp), [fusion 후보 선택](src/autorace_lane_fusion/src/lane.cpp).

## 입력·출력 토픽

### 현재 deep 모드

| 방향 / 노드 | 토픽 | 타입 | 내용 / 연결 |
| --- | --- | --- | --- |
| 입력 · deep_mask | `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 카메라 영상 |
| 출력 · deep_mask → deep_lite·objavoid | `/lane_fusion/deep_mask` | `sensor_msgs/msg/Image` | `mono8`: 0=배경, 1=노랑, 2=흰색 |
| 입력 · deep_lite | `/odom` | `nav_msgs/msg/Odometry` | 이전 경로의 이동 보정 |
| 출력 · deep_lite | `/lane_deep/target` | `std_msgs/msg/Float32MultiArray` | 차선 기하·목표점 → lane_ws PID |
| 출력 · deep_lite | `/lane_deep/zigzag` | `std_msgs/msg/UInt8` | 0/1/2 지그재그 판정 → slow |
| 출력 · deep_lite | `/lane_deep/debug` | `sensor_msgs/msg/Image` | BEV 관측, `debug=true`일 때 |

`/lane_fusion/deep_mask`는 시각화 전용이 아니라 차선·장애물 회피의 실제 인지 입력입니다. PID 입출력은 [lane_ws](../lane_ws/README.md#입력출력-토픽)에 정리합니다.

### fusion 모드에서만 추가되는 연결

| 방향 / 노드 | 토픽 | 타입 | 내용 |
| --- | --- | --- | --- |
| 입력 · fusion 인식 | `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | HSV 인식 영상 |
| 입력 · fusion 인식 | `/lane_fusion/deep_mask` | `sensor_msgs/msg/Image` | 딥 경계 후보 |
| 입력 · fusion 인식 | `/odom`, `/imu` | `nav_msgs/msg/Odometry`, `sensor_msgs/msg/Imu` | 지연된 마스크·유지 경로 이동 보정 |
| 입력 · fusion 인식 | `/cone_lane/exit_lane_active` | `std_msgs/msg/Bool` | 콘 탈출 시 경계 역할 판단 보조 |
| 출력 · fusion 인식 → fusion PID | `/lane_fusion/target` | `std_msgs/msg/Float32MultiArray` | 융합 차선 기하 |
| 출력 · fusion 인식 | `/lane_fusion/yellow_boundary` | `nav_msgs/msg/Path` | 주차가 구독하는 노란 경계 |
| 출력 · fusion 인식 | `/lane_fusion/debug` | `sensor_msgs/msg/Image` | `debug=true` 관측 영상 |
| 입력 · fusion PID | `/parking_start` | `std_msgs/msg/Bool` | 주차 준비 정지 |
| 입력 · fusion PID | `/parking/ready` | `std_msgs/msg/Header` | 주차 준비 완료 |
| 출력 · fusion PID | `/mission/lane`, `/lane_fusion_pid/command` | `ackermann_msgs/msg/AckermannDriveStamped` | 차량 후보 / 관측 명령, 전자는 차량 출력 활성화 시 |
| 출력 · fusion PID | `/lane_fusion_pid/state` | `std_msgs/msg/Float32MultiArray` | 제어 상태 |

현재 deep 모드에서는 `/lane_fusion/target`, `/lane_fusion/yellow_boundary`, `/lane_fusion/debug`의 발행자가 없습니다. `/lane_fusion/fork_debug`도 현재 인식 노드에서 발행하지 않습니다.

## References

- [1] O. Ronneberger, P. Fischer, T. Brox, **U-Net: Convolutional Networks for Biomedical Image Segmentation**, MICCAI, 2015. [논문](https://arxiv.org/abs/1505.04597). Skip connection을 사용하는 분할 디코더의 배경입니다.
- [2] A. Howard et al., **Searching for MobileNetV3**, ICCV, 2019. [논문](https://arxiv.org/abs/1905.02244). 코드의 `mobilenet_v3_small` 인코더에 해당합니다.
- [3] OpenCV, [Contour Features](https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html). 윤곽·convex hull·convexity defects의 기본 연산 자료입니다.

## 퓨전 파이프라인 (2026-11-10 재설계: 비동기 후보 퓨전)

이전 구조(딥 우선 → 딥이 약할 때만 HSV, `hsv_penalty`, 셀 체인 후보, 프레임-마스크 동기 대기, 출력 게이트)를 걷어내고 `lane_ws`의 검증된 OpenCV 검출·추적을 바탕으로 다시 만들었습니다.

1. **OpenCV(HSV) = 주 검출기, 매 프레임**: `lane_ws`와 같은 노이즈 필터(Top-hat·횡단보도 두께·흰 실선 길이·두께 띠·면적)와 같은 검출(추적 시작점/히스토그램 차로 쌍 → 방향 추종 슬라이딩 윈도우 → 3차 피팅 → 정지선·역할 교정).
2. **딥 = 독립 검출기, 비동기**: 마스크 노드는 최신 프레임만 `max_rate`(기본 10 Hz)까지 처리합니다. 인식 노드는 마스크를 기다리지 않고, 프레임마다 **촬영 시각이 `deep_max_age`(0.2 s) 이내인 최신 마스크를 그 사이 차량 이동량(odom)으로 현재 프레임에 정합**해 1과 같은 검출을 돌립니다. odom이 없으면 0.04 s 넘은 마스크, 나이 초과 마스크는 쓰지 않습니다.
3. **후보 퓨전**: 좌우 후보(HSV 2 + 딥 2)를 출처가 아니라 기하(추적 연속성, 차량 기준 위치, 차로 폭, 방향, 피팅 품질)로 비교해 고릅니다. 둘이 같은 선이면 겹치는 구간을 평균합니다. 한쪽 검출기만 맞게 잡아도 그 후보를 씁니다.
4. **트래커 하나 + 센터패스**: 좌우 추적(평활, 3프레임 점프 확인, 유지), 대응점 중점 또는 폭 절반 오프셋, 한쪽만 보이면 역할 확인 후. 다항식 재피팅이 없어 커브가 펴지지 않고, `track_jump_px`보다 큰 실제 변화는 평활 없이 바로 반영됩니다.
5. **유실**: odom으로 옮긴 직전 중앙 경로를 `track_memory`(0.5 s) 유지 → 그 뒤 무효(PID 정지).

코드: `src/candidates.cpp`(HSV 특징·한 마스크 검출), `src/lane.cpp`(퓨전·추적·센터패스·시각화), `src/geometry.cpp`(경로 기하·센터라인·곡률), `include/autorace_lane_fusion/odometry.hpp`(odom 이력), `deep_latest.hpp`(마스크 선택).

### 갈림길 A/B: 제거 (2026-11-12)

갈림길 로직(섬 감지·표지판 대기, `/yolo/LR_sign`·`/yolo/LR_detail` 구독, 차로 변경 조향, 분기 후 재추적, `/lane_fusion/fork_debug` 스냅샷)은 다시 설계하기 위해 차선 노드에서 뺐습니다. 갈림길에서도 보이는 차선만 따라갑니다. 예전 `fork_*` 키가 있는 YAML도 그대로 읽힙니다(무시). 이전 구현은 커밋 `65bcbb6` 이전 기록에 있습니다.

### 실시간 처리 (노드)

- 워커는 **항상 가장 최근 카메라 프레임**을 처리합니다(밀린 프레임은 큐 길이 1로 버림). 기본은 따라갈 수 있는 만큼 매 프레임 처리합니다. bag에서 20 Hz로 줄이면 커브 검출이 약 10%p 떨어졌으므로(lane_ws도 같음) CPU 경합이 있을 때만 `max_rate`(런치 `lane_rate:=20`)로 제한합니다(30 fps 카메라 → 3장 중 2장, 도착 즉시 처리).
- 디버그 영상은 `debug:=true`이고 구독자가 있을 때만 만듭니다.
- 회전량은 `/imu` 자이로(`imu_topic`, 없으면 `/odom` angular.z), 전진량은 `/odom`을 씁니다. 06번 차 bag에서 `/odom` angular.z가 IMU의 약 2배(누적 -14.9 vs -6.4 rad)라, 그대로 쓰면 이전 딥 마스크가 두 배로 회전돼 선이 두 줄로 겹쳤습니다. IMU로 바꾼 뒤 234303 Δ>5cm 88 → 78, 191317 74 → 62.
- odom·마스크·프레임 시각은 모두 노드 수신 시각(steady clock) 기준이라 카메라와 odom 시계가 달라도 됩니다. 물리적으로 불가능한 odom(1.5 m/s·후진 0.5 m/s·4 rad/s 초과, 예: 발산한 EKF)은 쓰지 않습니다.
- 노드 로그(5초마다): `Hz | valid % | deep used % (평균 나이, 제공 %) | decode + CV ms | receive->publish ms (max) | capture->publish | network latency`.

## 빌드

```bash
cd ~/2026-AutoRace/lane_fusion_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 실차 (Jetson)

1. TensorRT 엔진을 **Jetson에서** 패키지 model 폴더로 빌드하고 다시 colcon build합니다(처음 한 번, 모델이 바뀔 때마다). `*.engine`은 git에 올리지 않습니다.
   ```bash
   python3 tools/build_tensorrt.py --onnx src/autorace_lane_fusion_deep/model/lane_640_mask.onnx \
     --output src/autorace_lane_fusion_deep/model/lane_640_mask.engine --workspace-mib 1024
   ```
2. 센서와 mux는 기존 방법으로 띄우고, 차선은 이것 하나만 실행합니다(`lane_ws`의 lane과 동시에 띄우지 않음).
   ```bash
   source ~/2026-AutoRace/lane_fusion_ws/install/setup.bash
   ros2 launch autorace_lane_fusion_pid lane_fusion_pid.launch.py profile:=real
   ```
   - `/mission/lane`을 기본으로 발행합니다(2026-11-07 06번 차 주행 확인). 미리보기만 보려면 `enable_vehicle_output:=false`(`/lane_fusion_pid/command`만).
   - `deep:=false`: 딥 노드 없이 실행(HSV만), `lane_gui:=true` / `gui:=true`: 인식 / PID 튜너
   - `deep_rate:=10`(딥 최대 Hz, 5~10에서 시작해 로그의 ms/frame을 보고 조절), `lane_rate:=20`(인식 최대 Hz, 기본 0 = 매 프레임)
   - `model_path:=/abs/model.engine`, `deep_device:=cpu|cuda`: 모델·장치 직접 지정

## 튜너·오프라인 비교

- 튜너 Live: 튜너는 odom을 받지 않으므로 약 1프레임(0.04 s) 이내 마스크만 씁니다(노드는 odom으로 더 오래된 마스크도 정합). 상세창 첫 줄에 마스크 상태와 사용 여부, 선택된 경계 출처가 나옵니다. BEV 창 "후보" 보기: 보라 = 딥 후보, 주황 점선 = HSV 후보, 초록 = 둘이 같은 선, 초록/하늘 굵은 선 = 선택된 왼쪽/오른쪽. 영상 표시(화면 갱신)와 인식 처리는 다른 스레드이고, 보이는 보기만 변환합니다.
- bag: 프레임별 마스크를 먼저 만들고(`tools/bag_masks.py BAG MODEL - --topic T --masks DIR`), 튜너는 `<bag 상위>/deep_masks/<bag 이름>`을 자동으로 읽습니다.
  ```bash
  ros2 run autorace_lane_fusion lane_fusion_tuner --profile real --bag ~/AutoRace_bag/<bag>
  ros2 run autorace_lane_fusion lane_fusion_eval BAG src/autorace_lane_fusion/config/real/lane.yaml OUT_DIR \
    --deep-masks DIR [--deep-rate 10 --deep-latency 0.05] [--rate 20] [--no-hsv] [--no-views] [--every 60]
  ```
  `--deep-rate/--deep-latency`: 비동기 딥 노드 흉내(최신 프레임, S초 뒤 결과) + bag의 odom으로 정합, `--rate`: 노드의 `max_rate`, `--odom-topic T`/`--no-odom`, `--no-hsv`: 딥만, `--deep-masks` 없이: HSV만. 요약 줄: valid·held 비율, 횡오차 흔들림, 5 cm 튐, 곡률 흔들림, 좌우 뒤바뀜, 딥 사용 비율·평균 나이, 처리 시간.

## 검증 (2026-11-10, bag 4개 + 프레임별 딥 마스크, 이 PC)

같은 bag·설정(real)·지표 코드로 `lane_ws`, 기존 퓨전(main 101ec33), 개선 퓨전을 비교했습니다. 커브 = odom 요레이트 |ω| ≥ 0.3 rad/s 프레임, 신규검출 = 이번 프레임 검출로 중앙 경로를 만든 비율(held 제외), Δ>5cm = 목표점 횡오차가 출력 사이에 5 cm 넘게 바뀐 횟수(실제 커브 이동 포함). `fast`/`slow`는 `/odometry/filtered`가 발산한 EKF(전진 평균 -1.56 m/s)라 세 방식 모두 odom 없이, `234303`(최대 0.75 m/s)/`191317`은 `/odom` 사용. 비동기 = 딥 10 Hz·지연 50 ms 흉내.

| bag | 방식 | 출력 Hz | 커브 신규검출 | Δ>5cm | 좌우 뒤바뀜 | 최장 무효 |
| --- | --- | --- | --- | --- | --- | --- |
| 234303 (1731) | lane_ws HSV | 29.8 | 94.6% | 75 | 0 | 398 ms |
| | 기존 퓨전, 매 프레임 마스크(이상적) | 29.8 | 82.4% | 90 | 2 | 398 ms |
| | 기존 퓨전, 실제 동작(딥 10 Hz 동기 대기) | 8.4 | 51.6% | 91 | 1 | 707 ms |
| | **개선, 매 프레임 마스크** | 29.8 | **94.8%** | 82 | 0 | 397 ms |
| | **개선, 딥 10 Hz 비동기**(20 Hz 처리) | 19.6 | 92.9% | **67** | 0 | 411 ms |
| 191317 (1118) | lane_ws HSV | 29.9 | 98.6% | 52 | 3 | 396 ms |
| | 기존 퓨전, 이상적 / 실제 | 29.9 / 8.5 | 82.9% / 48.8% | 94 / 99 | 2 / 2 | 398 / 669 ms |
| | **개선, 매 프레임 / 비동기** | 29.9 / 19.2 | **99.5%** / 88.9% | 65 / 71 | 2 / 0 | 397 / 396 ms |
| slow (2982) | lane_ws / 기존(이상적) / 기존(실제) | 29.9 / 29.9 / 7.6 | 89.5 / 86.3 / 64.5% | 112 / 32 / 39 | 0 / 1 / 0 | 166 / 330 / 1455 ms |
| | **개선, 매 프레임** | 29.9 | **91.7%** | 111 | 1 | 229 ms |
| fast (1781) | lane_ws / 기존(이상적) / 기존(실제) | 29.9 / 29.9 / 8.5 | 89.5 / 87.7 / 54.1% | 90 / 49 / 40 | 0 / 0 / 1 | 68 / 70 / 1833 ms |
| | **개선, 매 프레임** | 29.9 | **90.0%** | 74 | 1 | 139 ms |

- 기존 퓨전의 실제 동작(프레임-자기 마스크 대기)은 출력이 딥 속도로 떨어지고(10 Hz 딥 → 8.5 Hz) 지연에 딥 지연이 더해집니다(50 ms 딥 → 출력 지연 약 55 ms). 개선안은 딥과 무관하게 매 프레임 출력합니다.
- 처리 빈도 영향: 같은 검출이라도 20 Hz로 줄이면 커브 신규검출이 약 10%p 떨어집니다(fast lane_ws 89.5 → 80.2%). 그래서 노드 기본값은 매 프레임 처리입니다.
- 오래된(정합한) 딥 마스크를 HSV와 평균하면 흔들림이 늘어서, 0.04 s 넘은 마스크는 같은 선 확인과 HSV가 놓친 쪽 보충에만 쓰고 품질을 나이만큼 낮춥니다. 비동기 딥의 이득은 234303에서 HSV 단독 대비 Δ>5cm 72 → 67, 커브 90.8 → 92.9%, 191317에서는 거의 없음(67 → 71)입니다. 딥 이득은 마스크가 신선할수록 큽니다.
- 기존 퓨전은 slow bag 10 Hz 조건에서 `fitPath`(8 px 미만 중앙 경로) segfault로 죽었습니다. 새 코드에는 이 재피팅이 없습니다.
- **실시간 재생(이 PC, 234303 raw → JPEG 재발행, 딥 ONNX CPU 10 Hz)**: 개선 노드 29.9 Hz, 수신→발행 약 10 ms(max 15.5), 딥 사용 97~100%(평균 나이 90 ms). 기존 노드는 딥 10 Hz에서 9.7 Hz, 딥이 28 Hz로 돌 때 28.4 Hz(수신→발행 ≈ 딥 지연 38 ms + 처리).
- **Jetson(06번 차) 처리 시간·지연과 0.8 m/s 실주행은 측정하지 않았습니다**(이번에 차 연결 불가). bag 결과만으로 실차 안정성이 검증된 것은 아닙니다.
- 단위 테스트 30개(HSV 회귀, 출처별 단독 검출, 딥·HSV 상호 확인, odom 정합/미정합 마스크, odom 이상값) 통과.


### 주기 성능 로그

`lane_fusion_node`의 5초 구간 처리율이 **20Hz 이하**일 때만 성능 요약을 WARN으로 출력한다.
20Hz를 넘는 구간은 출력하지 않으며 통계 구간은 계속 초기화한다.
`lane_fusion_deep_mask`의 주기 `masks/s` 로그는 기본적으로 끈다.
필요할 때 해당 노드에 `-p log_performance:=true`를 주면 다시 표시한다.
모델 로딩 결과와 경고·오류 로그는 유지한다.

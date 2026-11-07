# lane_fusion_ws — 차선 인식 퓨전 버전 (lanedeep U-Net 우선 + HSV 보완)

`lane_ws`(HSV 차선 인식 + PID)와 `lanedeep_ws`(U-Net 차선 분할)를 **복사해서 합친 별도 워크스페이스**입니다. 원본 두 워크스페이스는 수정하지 않았습니다. 패키지·실행 파일·토픽 이름을 모두 바꿔서 원본과 같은 셸에서 source해도 섞이지 않습니다. 단, 둘 다 `/mission/lane`을 내므로 **주행은 둘 중 하나만** 띄웁니다.

```text
/camera/color/image_raw/compressed
  ├─ lane_fusion_deep_mask (U-Net) ── /lane_fusion/deep_mask  (mono8 0=배경 1=노랑 2=흰 점선, 카메라 header 그대로)
  └─ lane_fusion_node ── 딥 후보 우선, 필요할 때만 HSV 후보 ── /lane_fusion/target
                                                        └─ lane_fusion_pid_controller ── /mission/lane → mux
```

| 패키지 | 원본 | 실행 파일 |
| --- | --- | --- |
| `autorace_lane_fusion` | `lane_ws/src/autorace_lane_lite` 기반, 퓨전 파이프라인으로 재설계 | `lane_fusion_node`, `lane_fusion_tuner`, `lane_fusion_eval` |
| `autorace_lane_fusion_pid` | `lane_ws/src/autorace_lane_pid` (입력만 `/lane_fusion/target`) | `lane_fusion_pid_controller`, `lane_fusion_pid_tuner` |
| `autorace_lane_fusion_deep` | `lanedeep_ws`의 추론 코드 + 마스크 전용 노드 | `lane_fusion_deep_mask` |
| `autorace_lane_deep_lite` | (보관용) 딥 마스크만 쓰는 실험 패키지. 주행에 쓰지 않음 | — |

모델: `src/autorace_lane_fusion_deep/model/best.pt`(lanedeep 학습 결과, 640×480)와 ONNX 두 가지. `lane_640_mask.onnx`(기본)는 정규화·softmax·argmax를 모델 안에 넣어 Jetson에서 후처리가 GPU로 갑니다.

## 퓨전 파이프라인 (하나, 모드 없음)

2026-11-07 재설계: 예전 `deep_mode 0~3` 분기, 마스크 OR/AND(gated), 프레임 짝짓기 대기를 없애고 하나의 파이프라인으로 합쳤습니다.

1. **딥 후보**: 클래스 마스크 → BEV → 격자(`cell`) 칸마다 클래스별 중심점 → 방향을 유지하며 체인으로 연결(`max_gap`으로 흰 점선 빈칸 연결, 끊긴 체인 잇기) → 각 체인을 시작점으로 **슬라이딩 윈도우**가 선을 따라가며 정제(윈도우가 지그재그 이빨보다 넓어 평균선을 따라감) → 곡선 피팅 → 같은 띠의 중복 제거.
2. **HSV 후보 (필요할 때만)**: 딥 마스크가 없거나(`deep_max_age`보다 오래됨), 한쪽 경계가 없거나, 고른 경계의 품질이 `hsv_min_quality`보다 낮을 때만 계산합니다. HSV 특징(Top-hat·두께·흰 실선 길이·두께 띠·면적 필터)도 1과 같은 방식으로 후보가 됩니다. **딥 후보와 겹치는 HSV 후보는 그 딥 후보를 확인**(출처 `deep+hsv`)하고, 나머지는 `hsv_penalty`를 더한 채 선택에 참여합니다. 딥이 충분한 프레임은 HSV도, JPEG 디코딩도 하지 않습니다.
3. **좌우 선택**: 모든 (왼쪽, 오른쪽, 없음) 조합을 한 비용으로 비교합니다 — 이전 트랙과의 거리(연속성), 차량이 경계 안쪽에 있는지(진행 방향 기준, 화면 x가 아님), 차로 폭, 좌우 방향 차이(노랑+노랑은 급커브라 제외), 출처, 길이. **추적 중인 선은 차량이 그 선을 넘어가도(차로 폭의 절반까지) 역할을 유지**해서 급커브·지그재그에서 좌우가 뒤바뀌지 않습니다. 진행 방향에 가로로 누운 흰 선(정지선·횡단보도)은 새 경계가 될 수 없습니다.
4. **추적**: 평활(`smoothing`), 급격한 이동·클래스 변경은 3프레임 확인, 놓치면 `track_memory` 동안 유지.
5. **센터패스**: 기준 경계 **하나**에서 만듭니다(반대쪽이 보이면 대응점 사이 중점 = 측정 폭으로 보정, 안 보이면 학습 폭의 절반만큼 법선 방향). 기준 쪽은 보이는 동안 바꾸지 않아서 양쪽↔한쪽 전환에 중앙이 튀지 않습니다. **한쪽만 보이면 그 선을 `role_frames` 이상 추적했거나 차량이 분명히 안쪽에 있을 때만** 중앙 경로를 만듭니다. 커브 안쪽으로는 반지름의 80%까지만 옮겨 접히지 않게 하고, 한 번 더 매끄럽게 피팅합니다.
6. **출력 검증**: 가까운 점이 차로 밖(`0.9 × lane_width_m` 초과)이거나 목표점 횡오차가 한 프레임에 `max_target_jump_m`보다 크게 바뀌면, 3프레임 연속 확인 전까지 직전 출력을 유지(`held`)합니다. 곡률은 중앙 경로 `curvature_span_m` 구간의 원 맞춤(`max_curvature`로 제한).

코드: `src/candidates.cpp`(HSV 특징·후보 생성·윈도우 정제), `src/geometry.cpp`(경로 기하·센터라인·피팅·곡률), `src/lane.cpp`(설정·선택·추적·센터패스·검증·시각화), `include/autorace_lane_fusion/deep_latest.hpp`(마스크 선택).

### 비동기 처리 (노드·튜너 Live 공용)

- 워커는 **항상 가장 최근 카메라 프레임**을 처리하고 딥 결과를 기다리지 않습니다(밀린 프레임은 버림).
- 그 프레임 자신의 마스크가 이미 왔으면 그것, 아니면 **촬영 시각이 `deep_max_age`(기본 0.12 s) 이내인 가장 최근 마스크**를 씁니다. 더 오래됐거나 미래 시각인 마스크는 쓰지 않고 그 프레임은 HSV만으로 처리합니다.
- 노드 로그(5초마다): `Hz | valid % | mask own / older (평균 나이) / none | HSV % | JPEG decoded % | 처리 ms (max) | network latency`.

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
   - `model_path:=/abs/model.engine`, `deep_device:=cpu|cuda`: 모델·장치 직접 지정

## 튜너·오프라인 비교

- 튜너 Live: 가장 최근 프레임과 `deep_max_age` 이내 마스크를 노드와 같은 방식으로 씁니다. 상세창 첫 줄에 마스크 상태(이 프레임 것 / 이전 프레임 것 n ms 전 / 너무 오래됨), HSV 후보 계산 여부, 선택된 경계 출처가 나옵니다. BEV 창 "후보" 보기: 보라 = 딥 후보, 주황 점선 = HSV 단독 후보, 초록/하늘 = 선택된 왼쪽/오른쪽. 영상 표시(화면 갱신)와 인식 처리는 다른 스레드이고, 보이는 보기만 변환합니다.
- bag: 프레임별 마스크를 먼저 만들고(`tools/bag_masks.py BAG MODEL - --topic T --masks DIR`), 튜너는 `<bag 상위>/deep_masks/<bag 이름>`을 자동으로 읽습니다.
  ```bash
  ros2 run autorace_lane_fusion lane_fusion_tuner --profile real --bag ~/AutoRace_bag/<bag>
  ros2 run autorace_lane_fusion lane_fusion_eval BAG src/autorace_lane_fusion/config/real/lane.yaml OUT_DIR \
    --deep-masks DIR [--no-hsv] [--mask-delay N] [--no-views] [--every 60] [--video out.mp4 --tracks]
  ```
  `--no-hsv`: 딥만, `--deep-masks` 없이: HSV만, `--mask-delay N`: N프레임 묵은 마스크(노드에서 딥이 늦을 때). 요약 줄: valid·held 비율, 횡오차 흔들림, 5 cm 튐, 곡률 흔들림, 좌우 뒤바뀜, 출력 보류 횟수, HSV·이미지 사용 비율, 처리 시간.

## 검증 (2026-11-07, 06번 차 BEV, bag 4개 + 프레임별 딥 마스크)

5 cm 튐 = 목표점 횡오차가 한 프레임에 5 cm 넘게 바뀐 횟수(커브에서 실제 이동도 포함). 좌우 뒤바뀜 = 한쪽 경계가 직전 프레임 반대쪽 경계 자리로 넘어간 횟수. 처리 시간은 JPEG 디코딩 제외.

| bag (프레임) | 방식 | valid | 흔들림 mm/프레임 | 5 cm 튐 | 좌우 뒤바뀜 | HSV 사용 | PC ms |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 191317 (1118) | 이전 퓨전 mode 3 | 97.9% | 10.4 | 22 | – | – | 5.8 |
| | lane_ws HSV | 100% | 14.6 | 53 | – | 100% | 8.5 |
| | **새 퓨전** | 97.6% | **9.9** | 24 | 2 | 66% | **4.3** |
| 234303 (1731) | 이전 퓨전 mode 3 | 99.1% | 6.5 | 35 | – | – | 3.7 |
| | lane_ws HSV | 100% | 9.3 | 76 | – | 100% | 8.7 |
| | **새 퓨전** | 99.5% | 6.8 | **26** | 1 | 29% | **2.3** |
| slow (2982) | 이전 퓨전 mode 3 | 98.8% | 5.2 | 27 | – | – | 5.3 |
| | lane_ws HSV | 99.8% | 8.6 | 111 | – | 100% | 8.9 |
| | **새 퓨전** | 98.9% | 6.2 | 38 | 1 | 40% | **3.1** |
| fast (1781) | 이전 퓨전 mode 3 | 98.3% | 8.8 | 40 | – | – | 5.5 |
| | lane_ws HSV | 100% | 12.3 | 90 | – | 100% | 8.8 |
| | **새 퓨전** | 99.3% | 9.7 | 48 | 0 | 38% | **3.1** |

- 같은 파이프라인의 단독 결과: 딥만 0.8~1.0 ms(5 cm 튐 24/26/34/42), HSV만 5.2~5.8 ms(32/44/37/56). 새 파이프라인의 HSV만도 lane_ws HSV보다 튐이 절반 이하입니다.
- 마스크 1~2 프레임 지연(`--mask-delay 1|2`): valid·튐 거의 같음(예: 234303 26 → 30 / 37).
- **Jetson(06번 차) fast bag**: 이전 퓨전 mode 3 **21.3 ms** → 새 퓨전 **11.8 ms**(max 32.9, HSV를 돌린 프레임), 딥만 9.3 → **3.1 ms**, HSV만 22.1 → 24.2 ms. JPEG 디코딩은 HSV가 필요한 38% 프레임에서만.
- 지그재그 구간(slow #790~#840): 체인만 쓰던 중간 버전은 지그재그 이빨에서 선이 끊겨 한쪽을 잃었고, 윈도우 정제 후 평균선을 따라갑니다.
- 2026-11-07 06번 차 저속 주행 확인. 남은 과제: 커브에서 한쪽만 보일 때 목표점이 5~8 cm씩 움직이는 프레임(실제 이동 포함), Jetson에서 HSV 경로가 무거움(약 24 ms), 퓨전 PID 게인·`curvature_ff`를 새 BEV 축척에 맞춰 재튜닝.
- 단위 테스트 26개(기존 HSV 회귀 + 퓨전·출력 검증·마스크 선택) 통과.

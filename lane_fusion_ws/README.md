# lane_fusion_ws — 차선 인식 퓨전 버전 (HSV + lanedeep U-Net)

`lane_ws`(HSV 차선 인식 + PID)와 `lanedeep_ws`(U-Net 차선 분할)를 **복사해서 합친 별도 워크스페이스**입니다. 원본 두 워크스페이스는 수정하지 않았습니다. 패키지·실행 파일·토픽 이름을 모두 바꿔서 원본과 같은 셸에서 source해도 섞이지 않습니다. 단, 둘 다 `/mission/lane`을 내므로 **주행은 둘 중 하나만** 띄웁니다.

```text
/camera/color/image_raw/compressed
  ├─ lane_fusion_deep_mask (U-Net) ── /lane_fusion/deep_mask  (mono8 0=배경 1=노랑 2=흰 점선, 카메라 header 그대로)
  └─ lane_fusion_node ── 같은 stamp의 mask와 짝지어 HSV ∧ 딥(gated) ── /lane_fusion/target
                                                        └─ lane_fusion_pid_controller ── /mission/lane → mux
```

| 패키지 | 원본 | 실행 파일 |
| --- | --- | --- |
| `autorace_lane_fusion` | `lane_ws/src/autorace_lane_lite` + 딥 퓨전 | `lane_fusion_node`, `lane_fusion_tuner`, `lane_fusion_eval` |
| `autorace_lane_fusion_pid` | `lane_ws/src/autorace_lane_pid` (입력만 `/lane_fusion/target`) | `lane_fusion_pid_controller`, `lane_fusion_pid_tuner` |
| `autorace_lane_fusion_deep` | `lanedeep_ws`의 추론 코드(`runtime.py`, `model.py`, `constants.py`) + 마스크 전용 노드 | `lane_fusion_deep_mask` |

모델: `src/autorace_lane_fusion_deep/model/best.pt`(lanedeep 학습 결과, 640×480, 3 epoch)와 ONNX 두 가지.
- `lane_640_mask.onnx` (기본): 정규화·softmax·argmax를 모델 안에 넣어 `label`/`prob`를 바로 출력합니다. Jetson에서 픽셀 단위 후처리가 GPU로 가고 CPU는 JPEG 디코딩·`blobFromImage`·임계값 비교만 합니다. `tools/export_mask_onnx.py`로 만들며 실제 프레임 20장에서 logits 방식과 mask가 100% 같습니다.
- `lane_640.onnx`: lanedeep 원래 형식(`rgb` → `logits`, CPU 후처리). 런타임은 두 형식을 모두 지원합니다. 설정 YAML 형식은 원본과 같아 `lane_ws` 튜닝 값을 그대로 옮길 수 있습니다.

## 퓨전 방식

`config/<profile>/lane.yaml`의 `deep_mode`:

| 값 | 의미 |
| --- | --- |
| `0` | HSV만 (원본 lane_lite와 동일) — sim 기본 |
| `1` | **gated**: HSV 특징 중 딥도 차선이라고 한 픽셀만 사용(딥 mask를 `deep_dilate` px 넓힘). 노면 글자·마름모·횡단보도·흰 실선 제거 — **real 기본** |
| `2` | 딥 mask만 |

- 프레임은 자기 mask를 최대 `deep_wait`(0.15 s) 기다립니다. 출력 주기는 딥 속도를 따릅니다.
- mask가 `deep_timeout`(0.5 s) 동안 없으면 기다리지 않고 HSV만으로 동작합니다. 딥 노드가 죽어도 주행은 계속됩니다.
- 노드 로그에 5초마다 `deep-fused N / HSV-only M`이 찍힙니다. `deep_mode`는 실행 중 튜너 또는 `ros2 param set /lane_fusion_node deep_mode 2.0`으로 바꿀 수 있습니다.

## 빌드

```bash
cd ~/2026-AutoRace/lane_fusion_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 실차 (Jetson)

1. TensorRT 엔진을 **Jetson에서** 패키지 model 폴더로 빌드하고, 다시 colcon build해서 설치합니다(처음 한 번, 모델이 바뀔 때마다). `*.engine`은 git에 올리지 않습니다.
   ```bash
   cd ~/2026-AutoRace/lane_fusion_ws
   python3 tools/build_tensorrt.py --onnx src/autorace_lane_fusion_deep/model/lane_640_mask.onnx \
     --output src/autorace_lane_fusion_deep/model/lane_640_mask.engine --workspace-mib 1024
   PYTHONNOUSERSITE=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
2. 센서와 mux는 기존 방법으로 띄우고, 차선은 이것 하나만 실행합니다(`lane_ws`의 lane과 동시에 띄우지 않음).
   ```bash
   source ~/2026-AutoRace/lane_fusion_ws/install/setup.bash
   ros2 launch autorace_lane_fusion_pid lane_fusion_pid.launch.py profile:=real
   ```
   딥 마스크 노드·퓨전 인식·PID가 한 번에 뜹니다. 모델은 설치된 것 중 `lane_640_mask.engine` > `lane_640.engine` > `lane_640_mask.onnx` > `lane_640.onnx` 순으로 자동 선택합니다(로그 첫 줄 `tensorrt (mask output) ...`로 확인).
   - `deep:=false`: 딥 노드 없이 실행(HSV만)
   - `lane_gui:=true` / `gui:=true`: 인식 / PID 튜너
   - `enable_vehicle_output:=false`: `/mission/lane` 대신 `/lane_fusion_pid/command` 미리보기만
   - `model_path:=/abs/model.engine`, `deep_device:=cpu|cuda`: 모델·장치 직접 지정
   - 따로 띄우기: `ros2 launch autorace_lane_fusion_deep deep_mask.launch.py`, `ros2 launch autorace_lane_fusion lane_fusion.launch.py profile:=real`
   - `./fusion.sh real`은 위 launch를 감싼 단축 스크립트입니다.
3. 로그에서 `lane_fusion: xx Hz, deep-fused ...`와 `lane_fusion_deep_mask: xx masks/s`를 확인합니다. YOLO와 함께 돌릴 때 15 Hz보다 낮으면 YOLO 입력 크기나 512×288 모델을 검토합니다.

## 튜너·오프라인 비교

- 튜너 Live: `/lane_fusion/deep_mask`를 자동으로 짝짓습니다. Binary 창의 "딥 마스크" 보기와 상세창 첫 줄(`딥 퓨전: gated | 딥 마스크 적용`)로 확인합니다.
- bag: 먼저 프레임별 mask를 만듭니다. 그다음 튜너가 `<bag 상위>/deep_masks/<bag 이름>`을 자동으로 읽습니다(다른 위치는 `--deep-masks DIR`).
  ```bash
  source /opt/ros/humble/setup.bash
  python3 tools/bag_masks.py ~/AutoRace_bag/<bag> src/autorace_lane_fusion_deep/model/lane_640.onnx \
    ~/AutoRace_bag/videos/<bag>_deep.mp4 --masks ~/AutoRace_bag/deep_masks/<bag>
  source install/setup.bash
  ros2 run autorace_lane_fusion lane_fusion_tuner --profile real --bag ~/AutoRace_bag/<bag>
  ros2 run autorace_lane_fusion lane_fusion_eval ~/AutoRace_bag/<bag> \
    src/autorace_lane_fusion/config/real/lane.yaml --topic /camera/color/image_raw \
    --deep-masks ~/AutoRace_bag/deep_masks/<bag> --deep-mode 1 /tmp/eval --video out.mp4 --tracks
  ```
  `.pt` 모델은 torch가 있는 Python에서, `.onnx`는 onnxruntime이 있는 Python에서 실행합니다. 출력 영상은 원본 위에 인식된 노란선/흰 점선을 칠하고, `--tracks`면 추적 경계(빨강)와 중앙 경로(파랑)를 겹칩니다.

## 확인한 것 (2026-11-07, PC RTX 4060 / bag `lane_20261106_234303`)

- 통합 launch: ONNX CPU 30 ms/frame → `lane_fusion` **29.9 Hz, 150/150 프레임 딥 퓨전**, PID 50 Hz. 딥 노드가 없으면 30 Hz HSV만.
- 오프라인 gated: #452 흰 실선을 경계로 잡던 것, #880 굵은 흰 선, #1456~1472 "천천히" 글자의 획을 오른쪽 경계로 잡던 것이 사라짐. 전체 valid·both 비율은 HSV와 비슷.
- 테스트 28개 통과. Jetson TensorRT 속도와 실주행은 아직 확인하지 않았습니다.
- 남은 과제: 급한 헤어핀에서 안쪽 경계가 화각 밖으로 나가면 heading 부호가 흔들립니다. 근처 다른 노란선과 헷갈리는 문제도 있습니다. 둘 다 추적 문제라 퓨전으로 해결되지 않으며, odom 기반 이동 보정으로 다룰 예정입니다.

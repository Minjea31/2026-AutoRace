# yolo_ws — 표지판·신호등 인식

카메라 영상에서 표지판과 신호등을 검출해 각 미션에 결과를 전달합니다. 현재 실차는 `yolo_detect` 노드와 `model/best.engine`을 사용합니다.

## 알고리즘

1. **YOLO26 객체 검출**: 최신 카메라 영상을 640×640 추론 입력으로 처리해 클래스·박스·confidence를 얻습니다. 모델 구조는 YOLO26 계열이며 TensorRT 엔진, ONNX, PyTorch 모델 경로를 선택할 수 있습니다. [1]
2. **후처리**: 박스를 원본 영상 좌표로 제한한 뒤 클래스별 confidence·면적 기준을 적용합니다. 같은 클래스를 서로 다른 새 영상에서 2회 연속 확인해야 확정합니다.
3. **미션 신호**: slow/crosswalk/parking은 존재 여부를 Bool로 발행합니다. 좌우 방향과 신호등은 확정한 새 결과만 발행하며, 양쪽 방향이나 빨강·초록이 함께 확정된 모호한 상황은 발행하지 않습니다.
4. **최신성**: 같은 영상을 반복 추론하지 않고 0.3초 이상 지난 영상·미수신·추론 오류에서는 presence를 false로 초기화합니다. 목표 처리 주기는 10 Hz이며 실제 주기는 추론 시간에 따라 달라집니다.

표지판 데이터 학습, 면적·연속 확인·최신성 처리는 이 프로젝트의 구현입니다. 현재 `bump` 클래스는 검출/표시할 수 있지만 `/yolo/bump_sign` 발행은 꺼져 있습니다.

구현: [yolo_detect.py](src/yolo_detect.py), [모델 학습](train/train_baseline_model.py), [모델 최적화](train/optimize_model.py).

## 입력·출력 토픽

| 방향 | 토픽 | 타입 | 내용 / 연결 |
| --- | --- | --- | --- |
| 입력 · 기본 | `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 카메라 압축 영상 |
| 입력 · 선택 | `/camera/color/image_raw` | `sensor_msgs/msg/Image` | `image_transport=raw`와 `image_topic` 변경 시 |
| 출력 | `/yolo/slow_sign` | `std_msgs/msg/Bool` | 서행 표지 존재 → slow |
| 출력 | `/yolo/crosswalk_sign` | `std_msgs/msg/Bool` | 횡단보도 표지 존재 → crosswalk·slow |
| 출력 | `/yolo/parking_sign` | `std_msgs/msg/Bool` | 주차 표지 존재 → parking·lane PID |
| 출력 | `/yolo/LR_sign` | `std_msgs/msg/Bool` | false=왼쪽/A, true=오른쪽/B → objavoid |
| 출력 | `/yolo/LR_detail` | `std_msgs/msg/Float32MultiArray` | 확정한 좌우 표지 박스 → objavoid |
| 출력 | `/yolo/signal` | `std_msgs/msg/Bool` | false=빨강, true=초록 → signal |
| 출력 · 관측 | `/yolo/debug/compressed` | `sensor_msgs/msg/CompressedImage` | 검출 박스 영상, 구독자가 있을 때 |

slow/crosswalk/parking은 미검출 시 false를 발행합니다. `/yolo/LR_sign`과 `/yolo/signal`은 미검출·모호함·영상 만료 시 메시지를 보내지 않습니다. 신호등의 false는 미검출이 아니라 빨간불입니다. `/yolo/bump_sign`은 현재 발행하지 않습니다.

`LR_detail` 배열은 `[right(1)/left(0), 면적px², 폭, 높이, confidence, x0, y0, x1, y1, stamp_sec, stamp_nanosec]`이며 좌표는 원본 영상 픽셀입니다.

## 검출 설정

현재 [CLASS_THRESHOLDS](src/yolo_detect.py)를 기준으로 합니다. 최소 면적은 추론 입력 비율이 아니라 원본 영상 박스의 픽셀 면적입니다.

| 클래스 | 최소 confidence | 최소 면적(px²) |
| --- | ---: | ---: |
| bump (토픽 발행 비활성) | 0.55 | 1800 |
| slow | 0.55 | 1500 |
| crosswalk | 0.60 | 2000 |
| parking | 0.60 | 2000 |
| left / right | 0.65 | 2000 |
| red / green | 0.60 | 3000 |

기본 엔진은 고정 입력 1×3×640×640, batch 1입니다. 다른 GPU·TensorRT 환경에서는 대상 장치에서 엔진을 다시 생성합니다. CPU에서는 `.pt` 모델과 `device:=cpu`를 사용합니다. 모델 클래스는 slow/bump/crosswalk/left/right/parking/red/green이며 OCR로 방향을 판단하지 않습니다.

## 빌드와 실행

```bash
source /opt/ros/humble/setup.bash
cd ~/2026-AutoRace/yolo_ws
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install -e ./yolo26
colcon build --symlink-install --packages-select yolo_detect
source install/setup.bash
export ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=0
ros2 run yolo_detect yolo_detect --ros-args -p show_image:=false
```

PyTorch·Ultralytics는 노드를 실행하는 Python 환경에 설치되어 있어야 합니다. 기존 `dl_env`를 사용한다면 ROS 환경과 해당 환경을 source한 뒤 실행합니다. 통합 실행은 `tensor_bringup`이 `yolo26` 경로와 모델을 설정합니다.

```bash
# CPU 모델 사용
ros2 run yolo_detect yolo_detect --ros-args \
  -p model_path:=$HOME/2026-AutoRace/yolo_ws/model/best.pt -p device:=cpu

# 원본 ROS Image 사용
ros2 run yolo_detect yolo_detect --ros-args \
  -p image_transport:=raw -p image_topic:=/camera/color/image_raw
```

`show_image=true`는 검출 창을 켭니다. 디버그 영상은 실제 미션 신호를 발행한 박스를 클래스 색으로, 조건 미충족 박스를 회색으로 표시합니다. 좌회전 false·빨간불 false도 유효한 발행으로 표시합니다. 카메라 미수신·처리 실패는 ROS 로그로 보고합니다.

## 모델 학습

데이터 설정은 `train/dataset.yaml`, 데이터 폴더는 `train/dataset/{train,valid,test}/{images,labels}`입니다. 데이터·학습 산출물은 Git에서 제외합니다.

```bash
conda activate yolo
python yolo_ws/train/train_baseline_model.py
```

기본 시작 모델은 `yolo_ws/yolo26n.pt`, 학습 결과는 `train/checkpoints/`를 사용합니다. 설치본에 새 모델·소스를 반영하려면 다시 빌드하고 노드를 재시작합니다.

## References

- [1] G. Jocher et al., **Ultralytics YOLO26: Unified Real-Time End-to-End Vision Models**, 2026. [논문](https://arxiv.org/abs/2606.03748), [공식 문서](https://docs.ultralytics.com/models/yolo26/). 검출기 계열의 구조 자료이며, 본 저장소의 표지판 후처리와 미션 토픽 정책은 별도 구현입니다.

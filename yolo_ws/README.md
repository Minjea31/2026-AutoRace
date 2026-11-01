# yolo_ws

`/camera/color/image_raw/compressed` (`sensor_msgs/msg/CompressedImage`)를 구독하고 검출됐을 때만 다음 Bool 인지 결과를 발행합니다.

| 토픽 | 구독 ws | false / true |
| --- | --- | --- |
| `/yolo/slow_sign` | `slow_ws` | false 미발행 / 방지턱·서행 표지판 감지 |
| `/yolo/crosswalk_sign` | `crosswalk_ws` | false 미발행 / 보행자 표지판 감지 |
| `/yolo/LR_sign` | `lane_ws` | 좌/A / 우/B |
| `/yolo/parking_sign` | `parking_ws` | false 미발행 / 주차 표지판 감지 |

모든 출력 타입은 `std_msgs/msg/Bool`입니다. 모든 토픽에서 미검출은 발행하지 않습니다. 인지 결과의 시간적 확인과 영상 시각 검사를 수행한 뒤 발행합니다. 갈림길 주행 동작은 `lane_ws`가 처리합니다.

`src/yolo_detect.py`는 `model/test_in_e87.pt`를 사용하는 ROS 2 인지 노드입니다.

## YOLO 노드 실행

```bash
conda activate yolo
source /opt/ros/humble/setup.bash
python yolo_ws/src/yolo_detect.py
```

카메라와 같은 `ROS_DOMAIN_ID`를 사용합니다. 시뮬레이션에서는 시뮬레이터의 도메인으로 맞추고 `--ros-args -p use_sim_time:=true`를 추가합니다. GPU가 있으면 GPU 0, 없으면 CPU를 사용합니다. 직접 지정하려면 `--ros-args -p device:=cpu` 또는 `-p device:=0`을 추가합니다. `model_path`, `image_topic`도 ROS 파라미터로 변경할 수 있습니다.

`yolo_detect.py` 상단의 **`CLASS_THRESHOLDS`**에서 6개 클래스 각각의 `confidence`와 `min_area_ratio`를 조정합니다. 크기는 원본 영상의 바운딩 박스 면적 비율이며, 예를 들어 640×480에서 `0.002`는 약 614 픽셀 면적입니다. 초기값은 실차에 맞춰 조정해야 합니다. `CONFIRM_FRAMES=2`로 서로 다른 연속 영상 2장에서 확인합니다.

| 모델 클래스 | 발행 토픽 | Bool 의미 |
| --- | --- | --- |
| `bump` 또는 `slow` | `/yolo/slow_sign` | 검출됐을 때만 true |
| `crosswalk` | `/yolo/crosswalk_sign` | 검출됐을 때만 true |
| `parking` | `/yolo/parking_sign` | 검출됐을 때만 true |
| `left`, `right` | `/yolo/LR_sign` | 왼쪽 false / 오른쪽 true |

최신 압축 영상을 0.1초 타이머로 처리하고 **검출된 클래스의 토픽만 최대 10 Hz**로 발행합니다. 모든 토픽에서 미검출은 발행을 생략하며, 미검출 false를 보내지 않습니다. `false`가 발행되는 경우는 왼쪽을 인지한 `/yolo/LR_sign`뿐입니다. 수신 노드는 마지막 true를 계속 유효하게 두지 말고 수신 유효 시간을 적용해야 합니다.

새 영상이 없거나 획득 시각이 0.3초보다 오래되면 발행하지 않습니다. 추론 후에도 영상 나이를 다시 검사합니다. 10 Hz 유지에는 입력 영상이 10 FPS 이상이고 장치가 추론을 100 ms 안에 처리할 수 있어야 합니다. `/yolo/LR_sign`은 좌우 동시 검출 시에도 발행하지 않습니다.

**방지턱과 지그재그 구간은 모두 `/yolo/slow_sign`을 사용합니다.** `bump` 또는 `slow` 클래스가 검출되면 같은 토픽으로 true를 발행합니다. 별도의 지그재그 토픽이나 OR 조건은 사용하지 않습니다. 현재 모델에는 신호등 클래스가 없으므로 `/yolo/signal`은 발행하지 않습니다. 각 미션의 재진입·완료 처리는 미션 노드에서 관리합니다.

## Baseline 모델 학습

데이터 설정은 `train/dataset.yaml`, 데이터 폴더는 `train/dataset/{train,valid,test}/{images,labels}`입니다. `dataset/`은 Git에서 제외합니다.

```bash
conda activate yolo
python yolo_ws/train/train_baseline_model.py
```

기본 모델은 `yolo_ws/yolo26n.pt`, 결과 저장 경로는 `yolo_ws/train/checkpoints/`입니다. 기존 코드 구조와 학습 옵션은 유지했습니다. `checkout/`과 `train/checkpoints/`는 Git에서 제외합니다.

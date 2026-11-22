# Camera OpenCV compatibility overlay

카메라와 ROS 영상 패키지의 OpenCV 버전을 맞추는 호환성 워크스페이스입니다.

## 알고리즘과 역할

카메라 인식 알고리즘이 아니라 **OpenCV 호환성 오버레이**입니다. `cv_bridge`와 `image_transport` 플러그인을 시스템 OpenCV에 맞춰 빌드합니다.

- `cv_bridge`: ROS 영상과 OpenCV 행렬 사이의 형식·인코딩 변환.
- `compressed_image_transport`: JPEG/PNG 압축·복원. 실제 토픽 발행·구독은 이를 사용하는 카메라 노드가 수행합니다.
- 별도의 미션 판단·경로 계획·차량 제어는 없습니다.

## 입력·출력 토픽

이 워크스페이스 자체에는 독립 실행되는 주행 노드나 고정 토픽이 없습니다. 현재 시스템에서 라이브러리를 사용하는 영상 흐름은 다음과 같습니다.

| 구분 | 토픽 / 데이터 | 타입 | 역할 |
| --- | --- | --- | --- |
| 영상 원본 | `/camera/color/image_raw` | `sensor_msgs/msg/Image` | 카메라 노드의 ROS 영상 ↔ OpenCV 변환 |
| 압축 전송 | `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 카메라 노드가 발행하고 인지 노드가 복원 |

## References

학습·주행 알고리즘 논문 대신 사용하는 라이브러리의 원본을 참고합니다.

- [ROS perception: cv_bridge](https://github.com/ros-perception/vision_opencv/tree/humble/cv_bridge) — ROS/OpenCV 영상 변환.
- [ROS perception: image_transport_plugins](https://github.com/ros-perception/image_transport_plugins/tree/humble) — 압축 영상 전송 플러그인.

## 빌드와 환경 적용

The camera uses system OpenCV 4.8; ROS binaries used OpenCV 4.5, causing mixed libraries and camera aborts. This workspace rebuilds cv_bridge and image transport plugins against OpenCV 4.8, without replacing system packages.

Sources: ros-perception/vision_opencv and ros-perception/image_transport_plugins, humble branch. Git checkouts retain exact revisions.

```bash
source /opt/ros/humble/setup.bash
cd ~/2026-AutoRace/camera_compat_ws
colcon build --packages-select cv_bridge compressed_image_transport compressed_depth_image_transport theora_image_transport --cmake-args -DOpenCV_DIR=/usr/lib/cmake/opencv4 -DCMAKE_BUILD_TYPE=Release --parallel-workers 2
source install/setup.bash
```
Sensor and tensor_bringup setup files were regenerated with this overlay as an underlay. Re-source setup in existing terminals before restarting sensors.

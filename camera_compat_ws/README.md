# camera_compat_ws — 카메라 호환성

ROS 영상과 OpenCV 사이의 변환·압축 전송을 지원합니다. 자체 주행 노드는 없습니다.

![카메라 호환성](../docs/logic/camera_compat_ws.svg)

## 입력·출력 토픽

<details>
<summary>토픽 이름·메시지 타입 펼쳐보기</summary>

| 구분 | 토픽 / 데이터 | 타입 | 역할 |
| --- | --- | --- | --- |
| 영상 원본 | `/camera/color/image_raw` | `sensor_msgs/msg/Image` | 카메라 노드의 ROS 영상 ↔ OpenCV 변환 |
| 압축 전송 | `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 카메라 노드가 발행하고 인지 노드가 복원 |

</details>

## References

- [ROS perception: cv_bridge](https://github.com/ros-perception/vision_opencv/tree/humble/cv_bridge)
- [ROS perception: image_transport_plugins](https://github.com/ros-perception/image_transport_plugins/tree/humble)

[통합 실행](../tensor_bringup/README.md) · [전체 토픽](../docs/architecture.md)

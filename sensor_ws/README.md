# sensor_ws

카메라와 라이다 드라이버를 담당합니다. 도면에서는 이 워크스페이스 안에 `camera`, `lidar` 박스를 배치하고 각각의 센서 토픽을 연결합니다.

| 내부 구성 요소 | 토픽 | 타입 | 연결선 색 |
| --- | --- | --- | --- |
| camera | `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 파랑 |
| lidar | `/scan` | `sensor_msgs/msg/LaserScan` | 주황 |

두 내부 박스는 별도 워크스페이스가 아닙니다. parking_ws·objavoid_ws는 두 토픽을 각각 구독합니다. 전체 토픽·색상 계약은 [architecture.md](../docs/architecture.md)를 참고합니다.

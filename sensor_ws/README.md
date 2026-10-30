# sensor_ws

카메라와 라이다 드라이버를 담당합니다. 도면에서는 이 워크스페이스 안에 `camera`, `lidar` 박스를 배치하고 각각의 센서 토픽을 연결합니다.

| 내부 구성 요소 | 토픽 | 타입 | 연결선 색 |
| --- | --- | --- | --- |
| camera | `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 파랑 |
| lidar | `/scan` | `sensor_msgs/msg/LaserScan` | 주황 |

카메라 원본 컬러 영상은 `/camera/color/image_raw` (`sensor_msgs/msg/Image`),
압축 컬러 영상은 `/camera/color/image_raw/compressed` (`sensor_msgs/msg/CompressedImage`)이다.
깊이 영상은 `/camera/depth/image_raw` (`sensor_msgs/msg/Image`)이다.
기본 `camera_name:=camera` 기준이며, 통합 실행의 센서 수신 로그는 원본 컬러·깊이 토픽을 감시한다.

두 내부 박스는 별도 워크스페이스가 아닙니다. parking_ws·objavoid_ws는 두 토픽을 각각 구독합니다. 전체 토픽·색상 계약은 [architecture.md](../docs/architecture.md)를 참고합니다.


## 센서 통합 실행과 드라이버

`src/autorace_sensor_bringup`과 LIMO·YDLIDAR·Orbbec·EKF 드라이버를 이 워크스페이스에 포함했다.
기존 `wego_ws`를 source할 필요가 없다. 빌드 및 실행:

```bash
cd /home/wego/2026-AutoRace/sensor_ws
./build_standalone.sh
source install/setup.bash
ros2 launch autorace_sensor_bringup sensors.launch.py
```

종료는 Ctrl+C. 시스템 의존성과 기능별 활성화·로그 확인은
[패키지 README](src/autorace_sensor_bringup/README.md)를 참고한다.

센서 좌표계는 기본 LIMO Pro 장착 기준으로 맞췄다. 토픽별 `frame_id`, 장착 위치,
공식 출처와 기존 보정 선택 방법은 [FRAMES.md](src/autorace_sensor_bringup/FRAMES.md)를 참고한다.

RViz 설정은 저장소 최상위 `sensors.rviz`에 있다:

```bash
rviz2 -d /home/wego/2026-AutoRace/sensors.rviz
```

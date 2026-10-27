# yolo_ws

`/camera/image_raw/compressed` (`sensor_msgs/msg/CompressedImage`)를 구독하고 다음 Bool 인지 결과를 발행합니다.

| 토픽 | 구독 ws | false / true |
| --- | --- | --- |
| `/yolo/slow_sign` | `slow_ws` | 미검출 / 서행 표지판 감지 |
| `/yolo/LR_sign` | `lane_ws` | 좌/A / 우/B |
| `/yolo/signal` | `signal_ws` | red / green |
| `/yolo/parking_sign` | `parking_ws` | 미검출 / 주차 표지판 감지 |

모든 출력 타입은 `std_msgs/msg/Bool`입니다. LR_sign과 signal은 판별한 프레임에서만 발행하며, 미검출을 false로 발행하지 않습니다. 인지 결과의 시간적 확인과 영상 시각 검사를 수행한 뒤 발행합니다. 갈림길 주행 동작은 `lane_ws`가 처리합니다.

현재는 인터페이스 명세이며 `src/init.txt`는 빈 src 폴더 유지용입니다.

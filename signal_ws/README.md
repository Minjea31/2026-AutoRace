# signal_ws — 신호등 정지 제어

빨강 확인 후 정지하고, 초록 확인으로 해제합니다. 자동 10초 해제는 없습니다.

![신호등 · 빨강 정지 / 초록 해제](../docs/logic/signal_ws.svg)

## 입력·출력 토픽

<details>
<summary>토픽 이름·메시지 타입 펼쳐보기</summary>

| 방향 | 토픽 | 타입 | 내용 / 연결 |
| --- | --- | --- | --- |
| 입력 | `/yolo/signal` | `std_msgs/msg/Bool` | 빨강 false / 초록 true |
| 입력 | `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 새 흰색 바닥 ROI |
| 입력 | `/mission/lane` | `ackermann_msgs/msg/AckermannDriveStamped` | 정지 중 유지할 조향 |
| 출력 | `/mission/signal/command_02` | `ackermann_msgs/msg/AckermannDriveStamped` | 정지 명령 → mux |
| 출력 | `/start_signal` | `std_msgs/msg/Bool` | 첫 정지 시작 true → slow |
| 출력 · 관측 | `/signal/white_roi/detected` | `std_msgs/msg/Bool` | 흰색 ROI 판정 |
| 출력 · 관측 | `/signal/white_roi/ratio` | `std_msgs/msg/Float32` | 흰색 픽셀 비율 |
| 출력 · 관측 | `/signal/white_roi/debug/compressed` | `sensor_msgs/msg/CompressedImage` | ROI 미리보기, 구독자가 있을 때 |

</details>

## References

- [1] OpenCV, [Thresholding Operations using inRange](https://docs.opencv.org/4.x/da/d97/tutorial_threshold_inRange.html)

[통합 실행](../tensor_bringup/README.md) · [전체 토픽](../docs/architecture.md)

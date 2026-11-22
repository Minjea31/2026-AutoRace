# yolo_ws — 표지판·신호등 인식

카메라에서 표지판과 신호등을 인식해 미션에 결과를 전달합니다.

![YOLO · 표지판과 신호등](../docs/logic/yolo_ws.svg)

`LR_sign`·`signal`은 미검출 시 발행하지 않습니다. `/yolo/bump_sign` 발행은 현재 꺼져 있습니다.

## 입력·출력 토픽

<details>
<summary>토픽 이름·메시지 타입 펼쳐보기</summary>

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

</details>

## References

- [1] G. Jocher et al., **Ultralytics YOLO26: Unified Real-Time End-to-End Vision Models**, 2026. [논문](https://arxiv.org/abs/2606.03748), [공식 문서](https://docs.ultralytics.com/models/yolo26/)

[통합 실행](https://github.com/Minjea31/2026-AutoRace/blob/main/tensor_bringup/README.md) · [전체 토픽](../docs/architecture.md)

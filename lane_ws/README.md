# lane_ws

카메라 기반 차선 유지, 지그재그 구간 조향, 갈림길 A/B 주행을 담당합니다. 갈림길의 진입·경로 선택·속도·조향·완료 및 재검출 방지 상태를 이 워크스페이스에서 함께 관리합니다.

| 방향 | 토픽 | 타입 | 의미 |
| --- | --- | --- | --- |
| 구독 | `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 차선·구간 특징 인식 |
| 구독 예정 | `/current_lane` | `std_msgs/msg/Bool` | `false(0)=좌측 추종`, `true(1)=우측 추종`. 기본값은 좌측 |
| 구독 | `/yolo/LR_sign` | `std_msgs/msg/Bool` | `false=좌/A`, `true=우/B` |
| 발행 | `/mission/lane` | `ackermann_msgs/msg/AckermannDriveStamped` | 기본 주행 및 갈림길을 반영한 속도·조향 명령 |

기본 명령은 `mux_ws`, `slow_ws`, `crosswalk_ws`, `signal_ws`가 구독합니다. 세 미션은 `drive.steering_angle`을 보존하고 `drive.speed`만 바꿔 각각의 command 토픽으로 발행합니다.

추종 차선의 초기값은 `false`이며 좌측을 추종합니다. `/current_lane`을 수신하면 값에 따라 좌측 또는 우측 차선을 추종합니다. 아직 발행 워크스페이스는 정하지 않았으며, 도면에는 발행자 없이 열린 시작점에서 lane_ws로 들어오는 점선으로 표시합니다. 이 차선 선택 입력과 `/yolo/LR_sign`의 갈림길 A/B 인지는 별도 입력입니다.

미검출은 좌측 인지 결과가 아니므로 `/yolo/LR_sign`의 초기 false를 방향 결정으로 사용하지 않습니다. 유효한 인지 시각·구간 특징·내부 상태로 분기 방향을 선택하고 통과 후 상태를 정리합니다. 미션 command가 없으면 mux는 `/mission/lane`을 기본 출력으로 선택합니다.

현재는 인터페이스와 폴더 구조를 정리한 상태이며, `src/init.txt`는 빈 src 디렉터리 유지용입니다. 전체 설계는 [architecture.md](../docs/architecture.md)를 참고합니다.

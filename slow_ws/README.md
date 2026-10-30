# slow_ws

방지턱 서행과 지그재그 서행 유도선 통과 미션을 함께 담당합니다. 서행 표지판과 카메라 영상으로 각 구간의 진입·종료를 판단하고, `lane_ws` 명령의 조향을 유지하면서 속도만 감속합니다. 지그재그 구간의 조향 계산은 `lane_ws`가 담당합니다.

| 방향 | 토픽 | 타입 | 의미 |
| --- | --- | --- | --- |
| 구독 | `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 지그재그 서행 유도선의 진입·종료 판단 |
| 구독 | `/yolo/slow_sign` | `std_msgs/msg/Bool` | `true=서행 표지판 감지`, `false=미검출` |
| 구독 | `/mission/lane` | `ackermann_msgs/msg/AckermannDriveStamped` | 기본 속도·조향 명령 |
| 발행 | `/mission/slow/command_08` | `ackermann_msgs/msg/AckermannDriveStamped` | 방지턱·지그재그 구간의 감속 명령, mux 우선순위 08 |

최신 lane 메시지를 복사해 `drive.speed`만 lane 속도의 절반 수준, 최대 0.2 m/s 이하로 제한합니다. 조향각과 나머지 drive 필드는 보존하며, lane 입력이 만료되면 오래된 명령을 새 명령으로 발행하지 않습니다.

방지턱과 지그재그 구간의 수행·완료·재검출 방지 상태를 각각 관리합니다. 감속이 필요한 구간에서만 하나의 command 토픽을 주기 발행하고, 구간을 통과하면 발행을 중단해 mux가 기본 차선 주행으로 복귀하도록 합니다. 표지판이 화면에서 사라졌다는 이유만으로 진행 중인 미션을 종료하지 않습니다.

현재는 인터페이스와 폴더 구조를 정리한 상태이며, `src/init.txt`는 빈 src 디렉터리 유지용입니다. 전체 설계는 [architecture.md](../docs/architecture.md)를 참고합니다.

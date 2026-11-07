# mux_ws

유효한 미션 command 중 숫자가 가장 작은 토픽을 선택하고, 선택 결과를 `/cmd/selected`로 `control_ws`에 전달합니다. 주행 명령 타입은 `ackermann_msgs/msg/AckermannDriveStamped`입니다.

| 순서 | 입력 토픽 |
| --- | --- |
| 01 | `/mission/crosswalk/command_01` |
| 02 | `/mission/signal/command_02` |
| 03 | `/mission/parking/command_03` |
| 04 | `/mission/cone/command_04` |
| 05 | `/mission/objavoid/command_05` |
| 08 | `/mission/slow/command_08` |
| DEFAULT | `/mission/lane` |

1. 유효한 미션 command가 있으면 가장 작은 번호를 선택합니다.
2. 수신된 미션 command가 없거나 모두 만료되면 `/mission/lane`의 최신 유효 명령을 기본값으로 선택합니다.
3. 기본 명령도 없거나 만료되면 속도 0의 정지 명령을 출력합니다.

`lane_ws`만 `/mission/lane`을 발행합니다. mux는 이 토픽을 구독하고 선택한 내용을 `/cmd/selected`로 전달하며, 기본 토픽 자체를 다시 발행하지 않습니다. 미션 노드로 보내는 상태·활성화 토픽은 없습니다. 갈림길은 lane에서 처리하므로 별도 우선순위 06 입력을 두지 않습니다. 방지턱과 지그재그 서행은 `slow_ws`의 `/mission/slow/command_08` 입력으로 함께 처리합니다.

후보 timeout은 초기 제안 0.25 s이며 실측 후 조정합니다. 현재는 노드 구현 전 인터페이스 명세이며 전체 계약은 [architecture.md](../docs/architecture.md)에 있습니다.

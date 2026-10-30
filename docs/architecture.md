# 2026 AutoRace 통합 워크스페이스·토픽 설계

2026-10-30 수정. draw.io의 **`전체 흐름` 한 페이지**에 센서·인지·미션·우선순위 mux·차량 제어를 모두 연결한다. 11개의 `*_ws` 박스와 sensor_ws 내부의 camera·lidar 박스 2개, 발행자 미정 입력선 1개를 포함한 26개 선에 토픽명과 타입을 기재했다. 이 저장소는 아직 실행 노드가 없는 상태이므로 도면과 본 문서는 구현할 인터페이스 명세이다.

갈림길의 진입·경로 선택·속도·조향·완료 처리는 `lane_ws`에 통합한다. YOLO 인지 토픽 `/yolo/LR_sign`은 `lane_ws`가 직접 구독한다. 기본 주행 토픽은 `/mission/lane`이며, mux는 유효한 미션 command가 없으면 이 입력을 기본값으로 선택한다.

- [편집 가능한 draw.io 파일](2026_AutoRace.drawio)
- [전체 흐름 PNG](2026_AutoRace.png) / [SVG](2026_AutoRace.svg)
- [기계 판독용 토픽 연결 명세](architecture-topics.json)

## 글자 표시

도면의 제목·본문·토픽·메시지 타입·범례·주석은 모두 굵은 글씨로 표시한다. 크기는 전체 제목 44, 워크스페이스 이름 32, 내부 센서 이름 26, 설명 영역 제목 24, 본문·토픽명 20, 메시지 타입 16, 주석 18을 사용한다. 센서 박스의 메시지 타입과 발행자 미정 표시는 16이다. 기존 연결선 색상·토픽·메시지 타입·추종 차선 기본값과 연결 구조는 유지한다.

## 사용자 지정 입력

| 발행 ws | 토픽 | 타입 | 구독 ws |
| --- | --- | --- | --- |
| `sensor_ws` | `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | `yolo_ws`, `lane_ws`, `crosswalk_ws`, `signal_ws`, `parking_ws`, `objavoid_ws`, `slow_ws` |
| `sensor_ws` | `/scan` | `sensor_msgs/msg/LaserScan` | `parking_ws`, `cone_ws`, `objavoid_ws` |

센서는 카메라와 라이다만 구독한다. TF, odom, IMU, CameraInfo 토픽은 이 설계에 없다. 카메라 소비자는 압축 JPEG/PNG를 내부에서 디코딩하며 별도의 Image 토픽을 다시 발행할 필요는 없다. 정지선·주차선 거리 환산에 필요한 보정값은 설정 파일에서 읽는다. LaserScan은 센서 설치 방향을 반영한 각도·거리 설정으로 처리한다.

`sensor_ws`는 센서 드라이버를 실행하고, 차량의 base driver는 최종 출력 위치인 `control_ws`에 둔다. 드라이버가 제공하는 다른 텔레메트리 토픽이 있더라도 본 미션에서는 구독하지 않는다.

도면에서는 `sensor_ws`를 컨테이너로 두고 내부에 `camera`와 `lidar`를 나눈다. 카메라 선은 camera 박스에서, `/scan` 선은 lidar 박스에서 출발한다. parking_ws·objavoid_ws로 향하는 카메라와 라이다 입력은 각각 별도 연결선으로 표시한다. 두 내부 박스는 sensor_ws에 속한 구성 요소이며 별도 워크스페이스 폴더를 만들지 않는다. 토픽 JSON의 source는 내부 셀 ID, source_workspace는 sensor_ws로 기록한다.

## 연결선 색상

도면 상단 범례와 각 선의 라벨은 같은 색상을 사용한다.

| 구분 | 색상 | 토픽 |
| --- | --- | --- |
| 카메라 | 파랑 `#1976D2` | `/camera/color/image_raw/compressed` |
| 라이다 | 주황 `#E65100` | `/scan` |
| YOLO 인지 | 보라 `#8E44AD` | `/yolo/*` Bool |
| 기본 차선 명령 | 황금색 `#AD7A00` | `/mission/lane` |
| 미션 명령 | 초록 `#238363` | `/mission/*/command_XX` |
| mux 선택 출력 | 청록 `#00695C` | `/cmd/selected` |
| 추종 차선 선택 | 회청색 점선 `#455A64` | `/current_lane` — 발행자 미정 |

slow_ws·parking_ws·signal_ws 박스에도 `/yolo/slow_sign`, `/yolo/parking_sign`, `/yolo/signal`이라는 실제 토픽명과 Bool 의미를 적는다.

## 추종 차선 선택 입력

`lane_ws`는 `/current_lane` (`std_msgs/msg/Bool`)을 입력으로 받도록 설계한다.

| 값 | 동작 |
| --- | --- |
| `false` / `0` | 좌측 차선 추종 |
| `true` / `1` | 우측 차선 추종 |
| 초기값·아직 미수신 | `false`, 좌측 차선 추종 |

아직 발행 워크스페이스는 정하지 않는다. 도면에서는 source가 없는 열린 시작점에서 `lane_ws`로 화살표가 들어오도록 하고, 회청색 점선에 토픽·타입·0/1 의미·기본값·발행자 미정을 적는다. 토픽 JSON도 source=null, publisher_status=unassigned로 기록한다. 추종할 차선 선택과 YOLO의 갈림길 A/B 인지는 별도 입력이며, 이 선을 임의의 기존 워크스페이스에 연결하지 않는다.

## YOLO Bool 계약

| 발행 ws → 구독 ws | 토픽 | 타입 | false(0) | true(1) |
| --- | --- | --- | --- | --- |
| `yolo_ws → slow_ws` | `/yolo/slow_sign` | `std_msgs/msg/Bool` | 서행 표지판 미검출 | 서행 표지판 감지 |
| `yolo_ws → lane_ws` | `/yolo/LR_sign` | `std_msgs/msg/Bool` | 왼쪽 / A | 오른쪽 / B |
| `yolo_ws → signal_ws` | `/yolo/signal` | `std_msgs/msg/Bool` | red | green |
| `yolo_ws → parking_ws` | `/yolo/parking_sign` | `std_msgs/msg/Bool` | 주차 표지판 미검출 | 주차 표지판 감지 |

`LR_sign`은 요청한 대소문자를 유지한다. **LR_sign과 signal은 인지한 프레임에서만 발행한다.** 미검출을 false로 보내면 왼쪽이나 적색이라는 의미와 섞이므로, 미검출·판별 불가·황색에서는 해당 Bool을 발행하지 않는다. 수신 노드는 수신 시각과 유효 시간으로 값의 신선도를 판단하며, 초기값 false를 인지 결과로 사용하지 않는다. 활성 신호등 미션은 신호가 불확실해져도 정지 명령을 계속 유지하고, 유효한 green 확인 후에 출발한다.

`slow_sign`과 `parking_sign`은 처리한 프레임에서 감지 여부를 true/false로 보낸다. 주차 시작 후 표지판이 화면 밖으로 나갔다고 미션을 종료하지 않고, 주차 노드 내부 상태를 유지한다. YOLO 결과에는 연속 프레임 확인·신뢰도 임계값·재검출 방지 조건을 적용한다. Bool에는 원본 stamp가 없으므로 `yolo_ws`에서 압축 영상 header의 획득 시각을 검사하고 오래된 영상의 결과를 새 인지로 발행하지 않는다.

**`crosswalk_ws`에는 YOLO 연결이 없다.** 카메라로 횡단보도와 정지선을 인식하고, `lane_ws` 명령의 조향은 유지하면서 속도만 바꿔 정지·3초 대기·재출발을 수행한다.

## 미션 명령과 우선순위

모든 주행 후보와 선택 결과는 **`ackermann_msgs/msg/AckermannDriveStamped`**로 통일한다. 이 표준 타입은 속도 `drive.speed`와 조향각 `drive.steering_angle`이 분리되어 있어, 요청한 "속도만 변경"을 그대로 표현한다. 미션의 숫자는 토픽 이름의 **`command_XX`**에 붙이며 작을수록 우선순위가 높다. 기본 주행 토픽 `/mission/lane`에는 command 접미사를 붙이지 않는다. 다음 배치는 현재 설계 제안이고 실제 트랙의 주행 순서와 별개이다.

| 우선순위 | 발행 ws | 토픽 | 입력 |
| --- | --- | --- | --- |
| 01 | `crosswalk_ws` | `/mission/crosswalk/command_01` | lane 명령 + 카메라 |
| 02 | `signal_ws` | `/mission/signal/command_02` | lane 명령 + 카메라 + YOLO signal Bool |
| 03 | `parking_ws` | `/mission/parking/command_03` | 카메라 + `/scan` + YOLO parking_sign Bool |
| 04 | `cone_ws` | `/mission/cone/command_04` | `/scan` |
| 05 | `objavoid_ws` | `/mission/objavoid/command_05` | 카메라 + `/scan` |
| 08 | `slow_ws` | `/mission/slow/command_08` | lane 명령 + 카메라 + YOLO slow_sign Bool |
| DEFAULT | `lane_ws` | `/mission/lane` | 카메라 + YOLO LR_sign Bool |

`lane_ws`는 `/mission/lane`을 mux와 **slow_ws, crosswalk_ws, signal_ws**에 함께 발행한다. 이 세 미션은 최신 lane 메시지를 복사하고 **`drive.speed`만 수정한다. `drive.steering_angle`, 조향각 변화율, 가속도·jerk 설정 등 다른 drive 필드는 유지한다.** 헤더 시각은 생성 시각으로 설정하되, lane 입력이 만료되면 오래된 입력을 새 명령으로 되살리지 않는다. `drive.speed` 단위는 m/s, `drive.steering_angle`은 rad이다. 좌측 조향은 양수이다.

정지에서는 speed=0으로 유지하면서 lane 조향각은 그대로 보존한다. slow_ws는 방지턱·지그재그 구간에서 감속하고 crosswalk/signal은 정지·재출발 속도를 지정한다. 갈림길 통과 속도와 조향은 lane_ws가 계산한다. parking/cone/objavoid는 자기 센서로 속도와 조향을 모두 계산할 수 있다. `header.frame_id=base_link`는 차량 기준 방향 표기이며 TF 구독을 추가하지 않는다.

갈림길 주행은 **`lane_ws`가 `/yolo/LR_sign`을 직접 구독해 좌/우 경로·속도·조향을 선택하고 구간 종료까지 처리**한다. 갈림길을 위한 별도 command 토픽이나 mux 우선순위 06 입력은 없다. 지그재그 서행은 slow_ws에 통합하여 별도 우선순위 07 입력 없이 기존 우선순위 08을 사용한다. 지그재그 구간의 영상 기반 조향 처리도 lane_ws가 담당하고 slow_ws는 두 서행 미션의 구간 판단과 감속을 담당한다.

## mux와 차량 출력

`mux_ws`는 미션 노드에 상태·활성화·완료를 보내지 않는다. 미션 상태 토픽과 이전 REQUEST/ACTIVE/DONE 메시지 계약은 제거했다. 미션 노드는 각각 자기 실행·완료·재검출 방지 상태를 관리한다.

1. **미션 비활성:** 명령을 발행하지 않는다. 비활성이라고 0 명령을 반복 발행하면 높은 우선순위가 기본 주행을 막으므로 사용하지 않는다.
2. **미션 수행 중:** 해당 command_XX를 주기 발행한다. 속도 조정 미션의 정지 중에는 drive.speed=0과 lane의 조향각을 계속 발행해 제어권을 유지한다.
3. **미션 완료:** 명령 발행을 중단한다. 기존 후보가 timeout되면 다음 유효한 우선순위 후보가 선택된다.
4. **주차 완료:** 예외적으로 정지 명령을 계속 발행해 차량을 유지한다.
5. **mux 선택:** 등록된 입력 중 timeout되지 않고 수치가 유효한 후보의 가장 작은 번호를 선택한다. 우선순위는 구독 토픽 설정에 명시하며 도착 순서로 결정하지 않는다.
6. **lane DEFAULT:** 미션 command가 수신되지 않았거나 모든 후보가 만료되면 `/mission/lane`의 최신 유효 명령을 기본값으로 선택한다. 기본 입력도 없거나 만료되면 속도 0의 정지 명령을 출력한다.

| 발행 ws → 구독 ws | 토픽 | 타입 |
| --- | --- | --- |
| `mux_ws → control_ws` | `/cmd/selected` | `ackermann_msgs/msg/AckermannDriveStamped` |
| `control_ws` 내부 → LIMO base driver | `/cmd_vel` | `geometry_msgs/msg/Twist` |

사용자의 “mux는 받기만”은 **미션 방향으로의 역방향 출력을 없애는 것**으로 적용했다. 선택 결과가 차량에 도달하도록 control_ws로 향하는 최종 출력 1개는 유지했다. 센서와 YOLO 결과는 mux에 직접 보내지 않는다.

`lane_ws`가 `/mission/lane`의 유일한 발행자이다. mux는 이 토픽을 구독하고, 선택한 내용을 `/cmd/selected`로 전달한다. `/mission/lane`을 mux에서 다시 발행하는 구조가 아니다.

초기 제안 주기는 미션/lane 후보 20 Hz, mux/control 출력 30 Hz이며, 후보 유효 시간은 0.25 s이다. 이 숫자는 대회 규정이 아니라 실측 후 조정할 설정값이다. mux는 선택된 후보의 stamp를 보존해 오래된 후보를 현재 명령처럼 갱신하지 않는다. 후보가 전혀 없을 때 생성하는 정지 명령은 현재 시각을 사용한다. `control_ws`는 입력 timeout이면 0을 주기 발행한다. 주행 명령 QoS는 Reliable/Volatile/KeepLast(1)를 제안하고, 과거 명령을 latch하지 않는다. Bool도 Volatile/KeepLast(1)로 다룬다. 센서 구독은 실제 드라이버와 QoS를 맞춘다.

주차의 후진 후보는 `/mission/parking/command_03`에서만 허용하도록 mux 입력 검증 조건을 둔다. control_ws는 속도·조향각·가감속 범위를 제한하고, LIMO 드라이버용 `/cmd_vel`로 변환한다. `linear.x=v`, `angular.z=v*tan(steering_angle)/wheelbase`이며 **v=0이면 angular.z도 0**이다. 축간거리와 조향 한계는 실차 설정값을 사용한다. 세 미션이 조향각을 보존하는 것과 최종 드라이버 출력 변환을 구분한다.

## 센서 제한에 맞춘 미션 판단

- `crosswalk_ws`: 영상의 횡단보도·정지선 위치로 접근/정지를 판단하고 lane 속도만 수정한다. 정지 유지와 영상 안정 확인 후 3초 대기한다.
- `signal_ws`: Bool의 red/green과 카메라 정지선을 함께 사용하고 lane 속도만 수정한다. 실제 신호등 영역과 광고판을 구분한다.
- `lane_ws`: 카메라와 YOLO Bool로 갈림길 진입·좌/우 방향·속도·조향·완료를 관리하고 `/mission/lane`으로 발행한다.
- `parking_ws`: 표지판 Bool로 미션을 시작하고 영상의 주차선·LiDAR 여유 공간으로 진행한다. 영상과 센서 설치 치수는 고정 보정값을 사용한다.
- `slow_ws`: 서행 표지판으로 방지턱 미션을 시작하고, 영상으로 지그재그 구간 진입·종료를 판단한다. 두 구간의 수행·완료·재검출 방지 상태를 각각 관리하며, lane 조향각을 유지하고 속도만 lane 속도의 절반 수준, 최대 0.2 m/s 이하로 제한한다. 두 미션 모두 `/mission/slow/command_08`로 발행한다.
- `objavoid_ws`, `cone_ws`: LiDAR 거리·배열과 필요한 영상 특징으로 진입·회피·출구를 판단한다.

미션 종료는 각 노드가 받는 카메라 특징·Bool·LiDAR 패턴·시간 조건을 조합한다. odom 적분 거리 기반 종료 조건은 쓰지 않는다. 표지판이 없는 구간의 진입 범위·종료 조건·재검출 금지 시간은 각 미션에서 트랙을 보고 보정한다. 트랙 순서는 slow → zigzag → crosswalk → objavoid → 갈림길(lane) → cone → signal → parking이다.

## 커스텀 타입과 검증

**이번 구조에는 커스텀 메시지 타입이 필요 없다.** CompressedImage, LaserScan, Bool, AckermannDriveStamped, Twist를 사용한다. 이전 설계의 커스텀 메시지 정의는 현재 명세에서 제거했다.

통합본은 11개 워크스페이스 박스와 센서 내부 박스 2개, 25개 기존 연결 및 발행자 미정 입력선 1개, 15개 고유 외부 토픽이다. control_ws 내부의 `/cmd_vel`은 별도 내부 인터페이스로 표시한다. MCP로 다시 읽어 페이지가 하나인지, 센서 셀의 parent가 sensor_ws인지, 카메라 7개·라이다 3개 연결이 분리되어 올바른 색인지, 세 미션과 mux가 `/mission/lane`을 받는지, `/current_lane` 선은 발행자 없이 lane_ws에 연결되는지, 모델과 토픽 명세가 일치하는지 확인하고 PNG로 배치·라벨을 확인한다. 실제 워크스페이스 폴더 11개도 도면과 대조한다. 현재 검증 범위는 폴더·도면·인터페이스 정합성이며 ROS 노드·실차 검증은 포함하지 않는다.

## 근거

작년 `/home/a/Downloads/AutoRace2025`의 사각형·화살표 구성을 참고했다. 올해 assets의 규정집(2026-10-08 초안_수정본), 미션 상세(2026-09-28 초안), 트랙 도면·지도에서 8개 미션과 주행 순서를 확인했다. 센서·YOLO Bool 의미·단일 페이지·명령 우선순위 규칙은 이번 사용자 지시를 반영했다.

- [ROS CompressedImage 정의](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/CompressedImage.msg)
- [ROS Bool 정의](https://github.com/ros2/common_interfaces/blob/humble/std_msgs/msg/Bool.msg)
- [ROS AckermannDriveStamped 정의](https://github.com/ros-drivers/ackermann_msgs/blob/ros2/msg/AckermannDriveStamped.msg), [AckermannDrive 정의](https://github.com/ros-drivers/ackermann_msgs/blob/ros2/msg/AckermannDrive.msg)

표기는 기존 도면의 ROS 2 형식을 유지했다. ROS 1을 쓰면 타입 표기의 `/msg/`를 제거하고 해당 버전에 맞게 설정한다.

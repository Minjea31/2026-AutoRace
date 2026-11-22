# parking_ws

주차칸을 찾고 경로를 계획·추종해 주차와 출차를 수행합니다.

![주차 · 경로 계획과 추종](../docs/logic/parking_ws.svg)

현재 실차는 곡선 후진 완료로 출차를 끝냅니다. 조건부 입력은 아래 표에 구분했습니다.

## 입력·출력 토픽

<details>
<summary>토픽 이름·메시지 타입 펼쳐보기</summary>

| 방향 | 토픽 | 타입 | 내용 / 연결 |
| --- | --- | --- | --- |
| 입력 | `/yolo/parking_sign` | `std_msgs/msg/Bool` | 주차 표지 |
| 입력 | `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 흰색 바닥 표시, floor gate 활성화 시 |
| 입력 | `/scan` | `sensor_msgs/msg/LaserScan` | 장애물·빈 칸·정합 |
| 입력 | `/odom` | `nav_msgs/msg/Odometry` | 원시 이동·정지 검사·출차 추적 |
| 입력 | `/localization/imu` | `sensor_msgs/msg/Imu` | 정지·방향 검사 |
| 입력 | `/odometry/filtered` | `nav_msgs/msg/Odometry` | EKF 융합 위치, fused 모드 |
| 입력 | `/localization/health` | `std_msgs/msg/String` | 융합 준비·최신성, fused 모드 |
| 입력 | `/parking/approach_start` | `std_msgs/msg/Header` | lane 접근 시작 시각, fused 모드 |
| 입력 · 조건부 | `/mission/lane` | `ackermann_msgs/msg/AckermannDriveStamped` | `exit_on_lane_reacquired=true`에서만; 현재 실차 false |
| 입력 | `/mux/active_source` | `std_msgs/msg/String` | 제어 주체 |
| 입력 · 조건부 | `/limo_status` | `limo_msgs/msg/LimoStatus` | `manual_mode_guard=true`일 때만; 현재 실차 true |
| 입력 · 조건부 | `/lane_fusion/target` | `std_msgs/msg/Float32MultiArray` | `exit_on_lane_reacquired=true`에서만; 현재 false, deep 발행자도 없음 |
| 입력 · deep에서 미연결 | `/lane_fusion/yellow_boundary` | `nav_msgs/msg/Path` | 노란 경계; 현재 deep 발행자 없음 |
| 출력 | `/mission/parking/command_03` | `ackermann_msgs/msg/AckermannDriveStamped` | 주차·정지·출차 명령 → mux |
| 출력 | `/parking/control_state` | `std_msgs/msg/String` | 제어 준비·소유·인계 상태 → mux·PID |
| 출력 | `/parking/active` | `std_msgs/msg/Bool` | 주차 활성 → PID·관측 |
| 출력 | `/parking/approach_ready` | `std_msgs/msg/Header` | 접근 준비 → lane PID |
| 출력 | `/parking/lane_handoff` | `std_msgs/msg/Header` | 출차 후 lane 인계 → mux |
| 출력 | `/parking_start`, `/parking/ready` | `std_msgs/msg/Bool`, `std_msgs/msg/Header` | 별도 fusion PID의 준비 정지/해제; deep PID는 구독하지 않음 |
| 출력 · 관측 | `/parking/localization/odometry`, `/parking/approach_odometry` | `nav_msgs/msg/Odometry` | 주차 위치·접근 기준 상대 위치 |
| 출력 · 관측 | `/parking/status` | `std_msgs/msg/String` | 주차 상태·진단 |
| 출력 · 관측 | `/parking/path` | `nav_msgs/msg/Path` | 주차·출차 경로 |
| 출력 · 관측 | `/parking/obstacle_memory` | `sensor_msgs/msg/PointCloud2` | 누적 장애물 지도 |
| 출력 · 관측 | `/parking/obstacles`, `/parking/target`, `/parking/slot_map`, `/parking/candidate_roi`, `/parking/lookahead` | `visualization_msgs/msg/MarkerArray` | 장애물·목표·칸·ROI·추종점 |
| 출력 · 관측 | `/parking/ldf` | `std_msgs/msg/Float32` | 추종 전방 거리 |

| 서비스 | 방향 | 타입 | 내용 |
| --- | --- | --- | --- |
| `/parking/reset` | 제공 | `std_srvs/srv/Trigger` | 미션 초기화 |
| `/set_pose` | 호출 | `robot_localization/srv/SetPose` | 시작 시 EKF 자세 설정 |
| `/localization/calibrate_gyro` | 조건부 호출 | `std_srvs/srv/Trigger` | `parking_calibration_stop` 활성화 시 정지 자이로 보정 |
| `/<steering_driver_node>/get_parameters` | 선택 조회 | `rcl_interfaces/srv/GetParameters` | 드라이버 조향 보정 설정 조회 |

</details>

## References

- [1] Y. Chen, G. Medioni, **Object Modeling by Registration of Multiple Range Images**, ICRA, 1991. [논문](https://graphics.stanford.edu/courses/cs164-09-spring/Handouts/paper_chenmedioni.pdf)
- [2] D. Dolgov, S. Thrun, M. Montemerlo, J. Diebel, **Practical Search Techniques in Path Planning for Autonomous Driving**, AAAI Workshop, 2008. [논문](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)
- [3] R. C. Coulter, **Implementation of the Pure Pursuit Path Tracking Algorithm**, CMU-RI-TR-92-01, 1992. [CMU 원문](https://www.ri.cmu.edu/publication_view.html?pub_id=278)

[통합 실행](../tensor_bringup/README.md) · [전체 토픽](../docs/architecture.md)

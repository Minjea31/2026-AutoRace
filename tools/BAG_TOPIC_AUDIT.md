# Bag 토픽 재대조 (2026-11-18)

기준: 원격 저장소의 현재 소스와 tensor_bringup 실차 기본 launch/config. C++/Python 노드의 발행·구독, 파라미터 기본 토픽명, launch의 명령 토픽 변경을 대조했습니다. 선택 토픽 98개는 중복 없이 모드 합집합과 일치합니다.

수정: EKF가 구독하는 `geometry_msgs/msg/PoseWithCovarianceStamped` 토픽 `/set_pose`를 sensor/all에 추가했습니다. `ros_filter.cpp`에는 같은 이름의 서비스도 있지만 서비스 호출은 일반 토픽 녹화에 포함되지 않습니다.

주행 노드가 실행되지 않은 상태여서 현재 그래프의 실제 발행 주기·데이터 유효성은 이번 정적 검사 범위에 포함되지 않습니다. 표의 근거는 토픽 사용 코드 또는 해당 표준 ROS 인터페이스를 시작하는 launch 위치입니다. 임의 remap, 다른 namespace/camera_name, 기본 비활성 depth/IR/point cloud/accel 옵션은 별도 목록 수정이 필요합니다.

미션 모드 obstacle/cone/parking/crosswalk/signal/slow는 lane 전체 토픽을 자동 포함합니다. 속도 조정 미션의 조향 원인과 다른 미션의 차선 진입·복귀를 함께 분석하기 위한 구성입니다. yolo/sensor에는 lane 전체를 자동 포함하지 않습니다. 공통 제어·TF·clock·로그와 관련 미션 의존성, 디버그 출력, 단독 실행용 출력도 포함합니다. all은 ROS 전체 그래프가 아니라 이 목록의 합집합입니다.

| 모드 | 공통 포함 토픽 수 | 주요 코드 |
|---|---:|---|
| `lane` | 32 | `lane_ws/src/autorace_lane_pid/src/node.cpp` |
| `obstacle` | 48 | `objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp` |
| `cone` | 44 | `cone_ws/src/cone_lane_follower/cpp/roi_node.cpp` |
| `parking` | 50 | `parking_ws/src/autorace_parking/autorace_parking/node.py` |
| `crosswalk` | 38 | `crosswalk_ws/src/autorace_crosswalk/autorace_crosswalk/node.py` |
| `signal` | 38 | `signal_ws/src/autorace_signal/autorace_signal/node.py` |
| `slow` | 41 | `slow_ws/src/autorace_slow/autorace_slow/node.py` |
| `yolo` | 16 | `yolo_ws/src/yolo_detect.py` |
| `sensor` | 20 | `sensor_ws/src/autorace_sensor_bringup/scripts/sensor_fusion.py` |

명령 번호: crosswalk 01 / signal 02 / parking 03 / slow 04 / objavoid 05 / cone 06. `mux_ws/src/autorace_mux/autorace_mux/core.py`와 일치합니다. 장애물 단독 출력 `/objavoid_v3/cmd`는 통합 launch에서 `/mission/objavoid/command_05`로 지정됩니다.

| 토픽 | 포함 모드 (all 제외) | 코드/실행 근거 |
|---|---|---|
| `/mission/lane` | lane, obstacle, cone, parking, crosswalk, signal, slow, yolo, sensor | [lane_ws/src/autorace_lane_pid/src/node.cpp:71](../lane_ws/src/autorace_lane_pid/src/node.cpp#L71) |
| `/cmd_vel` | lane, obstacle, cone, parking, crosswalk, signal, slow, yolo, sensor | [sensor_ws/src/limo_ros2/limo_base/src/limo_driver.cpp:93](../sensor_ws/src/limo_ros2/limo_base/src/limo_driver.cpp#L93) |
| `/mux/active_source` | lane, obstacle, cone, parking, crosswalk, signal, slow, yolo, sensor | [objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp:169](../objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp#L169) |
| `/tf` | lane, obstacle, cone, parking, crosswalk, signal, slow, yolo, sensor | [sensor_ws/src/autorace_sensor_bringup/launch/sensors.launch.py:212](../sensor_ws/src/autorace_sensor_bringup/launch/sensors.launch.py#L212) |
| `/tf_static` | lane, obstacle, cone, parking, crosswalk, signal, slow, yolo, sensor | [sensor_ws/src/autorace_sensor_bringup/launch/camera_tf.launch.py:27](../sensor_ws/src/autorace_sensor_bringup/launch/camera_tf.launch.py#L27) |
| `/clock` | lane, obstacle, cone, parking, crosswalk, signal, slow, yolo, sensor | ROS 2 Node use_sim_time 기본 인터페이스 |
| `/rosout` | lane, obstacle, cone, parking, crosswalk, signal, slow, yolo, sensor | ROS 2 Node 로깅 기본 인터페이스 |
| `/camera/color/image_raw/compressed` | lane, obstacle, cone, parking, crosswalk, signal, slow, yolo, sensor | [lane_ws/src/autorace_lane_lite/src/node.cpp:29](../lane_ws/src/autorace_lane_lite/src/node.cpp#L29) |
| `/odom` | lane, obstacle, cone, parking, crosswalk, signal, slow, sensor | [lane_ws/src/autorace_lane_lite/src/node.cpp:33](../lane_ws/src/autorace_lane_lite/src/node.cpp#L33) |
| `/imu` | lane, obstacle, cone, parking, crosswalk, signal, slow, sensor | [lane_fusion_ws/src/autorace_lane_fusion/src/node.cpp:57](../lane_fusion_ws/src/autorace_lane_fusion/src/node.cpp#L57) |
| `/odometry/filtered` | lane, obstacle, cone, parking, crosswalk, signal, slow, sensor | [parking_ws/src/autorace_parking/autorace_parking/node.py:369](../parking_ws/src/autorace_parking/autorace_parking/node.py#L369) |
| `/yolo/parking_sign` | lane, obstacle, cone, parking, crosswalk, signal, slow, yolo | [lane_ws/src/autorace_lane_pid/src/node.cpp:40](../lane_ws/src/autorace_lane_pid/src/node.cpp#L40) |
| `/parking_start` | lane, obstacle, cone, parking, crosswalk, signal, slow | [lane_fusion_ws/src/autorace_lane_fusion_pid/src/node.cpp:93](../lane_fusion_ws/src/autorace_lane_fusion_pid/src/node.cpp#L93) |
| `/parking/ready` | lane, obstacle, cone, parking, crosswalk, signal, slow | [lane_fusion_ws/src/autorace_lane_fusion_pid/src/node.cpp:100](../lane_fusion_ws/src/autorace_lane_fusion_pid/src/node.cpp#L100) |
| `/parking/active` | lane, obstacle, cone, parking, crosswalk, signal, slow | [lane_ws/src/autorace_lane_pid/src/node.cpp:128](../lane_ws/src/autorace_lane_pid/src/node.cpp#L128) |
| `/parking/approach_start` | lane, obstacle, cone, parking, crosswalk, signal, slow | [lane_ws/src/autorace_lane_pid/src/node.cpp:74](../lane_ws/src/autorace_lane_pid/src/node.cpp#L74) |
| `/parking/approach_ready` | lane, obstacle, cone, parking, crosswalk, signal, slow | [lane_ws/src/autorace_lane_pid/src/node.cpp:116](../lane_ws/src/autorace_lane_pid/src/node.cpp#L116) |
| `/parking/control_state` | lane, obstacle, cone, parking, crosswalk, signal, slow | [lane_ws/src/autorace_lane_pid/src/node.cpp:122](../lane_ws/src/autorace_lane_pid/src/node.cpp#L122) |
| `/parking/lane_handoff` | lane, obstacle, cone, parking, crosswalk, signal, slow | [parking_ws/src/autorace_parking/autorace_parking/node.py:361](../parking_ws/src/autorace_parking/autorace_parking/node.py#L361) |
| `/cone_lane/exit_lane_active` | lane, obstacle, cone, parking, crosswalk, signal, slow | [lane_fusion_ws/src/autorace_lane_fusion/src/node.cpp:78](../lane_fusion_ws/src/autorace_lane_fusion/src/node.cpp#L78) |
| `/lane_lite/target` | lane, obstacle, cone, parking, crosswalk, signal, slow | [lane_ws/src/autorace_lane_pid/src/node.cpp:37](../lane_ws/src/autorace_lane_pid/src/node.cpp#L37) |
| `/lane_lite/debug` | lane, obstacle, cone, parking, crosswalk, signal, slow | [lane_ws/src/autorace_lane_lite/src/node.cpp:50](../lane_ws/src/autorace_lane_lite/src/node.cpp#L50) |
| `/lane_pid/command` | lane, obstacle, cone, parking, crosswalk, signal, slow | [lane_ws/src/autorace_lane_pid/src/node.cpp:68](../lane_ws/src/autorace_lane_pid/src/node.cpp#L68) |
| `/lane_pid/state` | lane, obstacle, cone, parking, crosswalk, signal, slow | [lane_ws/src/autorace_lane_pid/src/node.cpp:73](../lane_ws/src/autorace_lane_pid/src/node.cpp#L73) |
| `/lane_deep/target` | lane, obstacle, cone, parking, crosswalk, signal, slow | [lane_fusion_ws/src/autorace_lane_deep_lite/src/node.cpp:30](../lane_fusion_ws/src/autorace_lane_deep_lite/src/node.cpp#L30) |
| `/lane_deep/debug` | lane, obstacle, cone, parking, crosswalk, signal, slow | [lane_fusion_ws/src/autorace_lane_deep_lite/src/node.cpp:33](../lane_fusion_ws/src/autorace_lane_deep_lite/src/node.cpp#L33) |
| `/lane_fusion/deep_mask` | lane, obstacle, cone, parking, crosswalk, signal, slow | [lane_fusion_ws/src/autorace_lane_fusion/src/node.cpp:50](../lane_fusion_ws/src/autorace_lane_fusion/src/node.cpp#L50) |
| `/lane_fusion/target` | lane, obstacle, cone, parking, crosswalk, signal, slow | [lane_fusion_ws/src/autorace_lane_fusion/src/node.cpp:89](../lane_fusion_ws/src/autorace_lane_fusion/src/node.cpp#L89) |
| `/lane_fusion/debug` | lane, obstacle, cone, parking, crosswalk, signal, slow | [lane_fusion_ws/src/autorace_lane_fusion/src/node.cpp:92](../lane_fusion_ws/src/autorace_lane_fusion/src/node.cpp#L92) |
| `/lane_fusion/yellow_boundary` | lane, obstacle, cone, parking, crosswalk, signal, slow | [lane_fusion_ws/src/autorace_lane_fusion/src/node.cpp:88](../lane_fusion_ws/src/autorace_lane_fusion/src/node.cpp#L88) |
| `/lane_fusion_pid/command` | lane, obstacle, cone, parking, crosswalk, signal, slow | [lane_fusion_ws/src/autorace_lane_fusion_pid/src/node.cpp:58](../lane_fusion_ws/src/autorace_lane_fusion_pid/src/node.cpp#L58) |
| `/lane_fusion_pid/state` | lane, obstacle, cone, parking, crosswalk, signal, slow | [lane_fusion_ws/src/autorace_lane_fusion_pid/src/node.cpp:63](../lane_fusion_ws/src/autorace_lane_fusion_pid/src/node.cpp#L63) |
| `/scan` | obstacle, cone, parking, sensor | [objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp:62](../objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp#L62) |
| `/yolo/LR_sign` | obstacle, yolo | [objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp:82](../objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp#L82) |
| `/yolo/LR_detail` | obstacle, yolo | [objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp:83](../objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp#L83) |
| `/slow/mission_active` | obstacle, slow | [objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp:134](../objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp#L134) |
| `/mission/slow/command_04` | obstacle, slow | [objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp:137](../objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp#L137) |
| `/objavoid_v3/activate` | obstacle | [objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp:139](../objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp#L139) |
| `/objavoid_v3/remap` | obstacle | [objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp:141](../objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp#L141) |
| `/objavoid_v3/active` | obstacle | [objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp:166](../objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp#L166) |
| `/objavoid_v3/entrance_armed` | obstacle | [objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp:167](../objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp#L167) |
| `/objavoid_v3/has_control` | obstacle | [objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp:168](../objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp#L168) |
| `/objavoid_v3/state` | obstacle | [objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp:171](../objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp#L171) |
| `/objavoid_v3/done` | obstacle | [objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp:172](../objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp#L172) |
| `/objavoid_v3/markers` | obstacle | [objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp:173](../objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp#L173) |
| `/objavoid_v3/path` | obstacle | [objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp:174](../objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp#L174) |
| `/objavoid_v3/cmd` | obstacle | [objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp:61](../objavoid_ws/src/autorace_objavoid_v3_cpp/src/node.cpp#L61) |
| `/mission/objavoid/command_05` | obstacle | [tensor_bringup/launch/tensor_bringup.launch.py:125](../tensor_bringup/launch/tensor_bringup.launch.py#L125) |
| `/cone_lane/roi_marker` | cone | [cone_ws/src/cone_lane_follower/cpp/roi_node.cpp:253](../cone_ws/src/cone_lane_follower/cpp/roi_node.cpp#L253) |
| `/cone_lane/roi_points` | cone | [cone_ws/src/cone_lane_follower/cpp/roi_node.cpp:255](../cone_ws/src/cone_lane_follower/cpp/roi_node.cpp#L255) |
| `/cone_lane/clusters` | cone | [cone_ws/src/cone_lane_follower/cpp/roi_node.cpp:256](../cone_ws/src/cone_lane_follower/cpp/roi_node.cpp#L256) |
| `/cone_lane/cone_centers` | cone | [cone_ws/src/cone_lane_follower/cpp/roi_node.cpp:257](../cone_ws/src/cone_lane_follower/cpp/roi_node.cpp#L257) |
| `/cone_lane/boundaries` | cone | [cone_ws/src/cone_lane_follower/cpp/roi_node.cpp:258](../cone_ws/src/cone_lane_follower/cpp/roi_node.cpp#L258) |
| `/cone_lane/control_points` | cone | [cone_ws/src/cone_lane_follower/cpp/roi_node.cpp:259](../cone_ws/src/cone_lane_follower/cpp/roi_node.cpp#L259) |
| `/cone_lane/center_path` | cone | [cone_ws/src/cone_lane_follower/cpp/roi_node.cpp:260](../cone_ws/src/cone_lane_follower/cpp/roi_node.cpp#L260) |
| `/cone_lane/center_spot` | cone | [cone_ws/src/cone_lane_follower/cpp/roi_node.cpp:261](../cone_ws/src/cone_lane_follower/cpp/roi_node.cpp#L261) |
| `/cone_lane/backup_path` | cone | [cone_ws/src/cone_lane_follower/cpp/roi_node.cpp:262](../cone_ws/src/cone_lane_follower/cpp/roi_node.cpp#L262) |
| `/cone_lane/control_status` | cone | [cone_ws/src/cone_lane_follower/cpp/roi_node.cpp:254](../cone_ws/src/cone_lane_follower/cpp/roi_node.cpp#L254) |
| `/mission/cone/command_06` | cone | [cone_ws/src/cone_lane_follower/cpp/roi_node.cpp:263](../cone_ws/src/cone_lane_follower/cpp/roi_node.cpp#L263) |
| `/localization/wheel` | parking, sensor | [sensor_ws/src/autorace_sensor_bringup/scripts/sensor_fusion.py:57](../sensor_ws/src/autorace_sensor_bringup/scripts/sensor_fusion.py#L57) |
| `/localization/imu` | parking, sensor | [parking_ws/src/autorace_parking/autorace_parking/start_sequence.py:39](../parking_ws/src/autorace_parking/autorace_parking/start_sequence.py#L39) |
| `/localization/health` | parking, sensor | [parking_ws/src/autorace_parking/autorace_parking/node.py:371](../parking_ws/src/autorace_parking/autorace_parking/node.py#L371) |
| `/limo_status` | parking, sensor | [parking_ws/src/autorace_parking/autorace_parking/node.py:353](../parking_ws/src/autorace_parking/autorace_parking/node.py#L353) |
| `/parking/status` | parking | [parking_ws/src/autorace_parking/autorace_parking/node.py:349](../parking_ws/src/autorace_parking/autorace_parking/node.py#L349) |
| `/parking/path` | parking | [parking_ws/src/autorace_parking/autorace_parking/node.py:347](../parking_ws/src/autorace_parking/autorace_parking/node.py#L347) |
| `/parking/localization/odometry` | parking | [parking_ws/src/autorace_parking/autorace_parking/node.py:356](../parking_ws/src/autorace_parking/autorace_parking/node.py#L356) |
| `/parking/approach_odometry` | parking | [parking_ws/src/autorace_parking/autorace_parking/node.py:357](../parking_ws/src/autorace_parking/autorace_parking/node.py#L357) |
| `/parking/slot_map` | parking | [parking_ws/src/autorace_parking/autorace_parking/node.py:345](../parking_ws/src/autorace_parking/autorace_parking/node.py#L345) |
| `/parking/obstacle_memory` | parking | [parking_ws/src/autorace_parking/autorace_parking/node.py:341](../parking_ws/src/autorace_parking/autorace_parking/node.py#L341) |
| `/parking/obstacles` | parking | [parking_ws/src/autorace_parking/autorace_parking/node.py:342](../parking_ws/src/autorace_parking/autorace_parking/node.py#L342) |
| `/parking/target` | parking | [parking_ws/src/autorace_parking/autorace_parking/node.py:344](../parking_ws/src/autorace_parking/autorace_parking/node.py#L344) |
| `/parking/candidate_roi` | parking | [parking_ws/src/autorace_parking/autorace_parking/node.py:346](../parking_ws/src/autorace_parking/autorace_parking/node.py#L346) |
| `/parking/ldf` | parking | [parking_ws/src/autorace_parking/autorace_parking/node.py:365](../parking_ws/src/autorace_parking/autorace_parking/node.py#L365) |
| `/parking/lookahead` | parking | [parking_ws/src/autorace_parking/autorace_parking/node.py:366](../parking_ws/src/autorace_parking/autorace_parking/node.py#L366) |
| `/parking/robot_description` | parking | [parking_ws/src/autorace_parking/autorace_parking/robot_visual.py:63](../parking_ws/src/autorace_parking/autorace_parking/robot_visual.py#L63) |
| `/mission/parking/command_03` | parking | [parking_ws/src/autorace_parking/autorace_parking/node.py:355](../parking_ws/src/autorace_parking/autorace_parking/node.py#L355) |
| `/yolo/crosswalk_sign` | crosswalk, yolo | [crosswalk_ws/src/autorace_crosswalk/autorace_crosswalk/node.py:83](../crosswalk_ws/src/autorace_crosswalk/autorace_crosswalk/node.py#L83) |
| `/start_crosswalk` | crosswalk, slow | [crosswalk_ws/src/autorace_crosswalk/autorace_crosswalk/node.py:81](../crosswalk_ws/src/autorace_crosswalk/autorace_crosswalk/node.py#L81) |
| `/mission/crosswalk/command_01` | crosswalk | [crosswalk_ws/src/autorace_crosswalk/autorace_crosswalk/node.py:80](../crosswalk_ws/src/autorace_crosswalk/autorace_crosswalk/node.py#L80) |
| `/crosswalk/white_roi/debug/compressed` | crosswalk | [crosswalk_ws/src/autorace_crosswalk/autorace_crosswalk/node.py:91](../crosswalk_ws/src/autorace_crosswalk/autorace_crosswalk/node.py#L91) |
| `/crosswalk/white_roi/ratio` | crosswalk | [crosswalk_ws/src/autorace_crosswalk/autorace_crosswalk/node.py:92](../crosswalk_ws/src/autorace_crosswalk/autorace_crosswalk/node.py#L92) |
| `/crosswalk/white_roi/detected` | crosswalk | [crosswalk_ws/src/autorace_crosswalk/autorace_crosswalk/node.py:93](../crosswalk_ws/src/autorace_crosswalk/autorace_crosswalk/node.py#L93) |
| `/yolo/signal` | signal, yolo | [signal_ws/src/autorace_signal/autorace_signal/node.py:139](../signal_ws/src/autorace_signal/autorace_signal/node.py#L139) |
| `/start_signal` | signal, slow | [signal_ws/src/autorace_signal/autorace_signal/node.py:137](../signal_ws/src/autorace_signal/autorace_signal/node.py#L137) |
| `/mission/signal/command_02` | signal | [signal_ws/src/autorace_signal/autorace_signal/node.py:136](../signal_ws/src/autorace_signal/autorace_signal/node.py#L136) |
| `/signal/white_roi/debug/compressed` | signal | [signal_ws/src/autorace_signal/autorace_signal/node.py:159](../signal_ws/src/autorace_signal/autorace_signal/node.py#L159) |
| `/signal/white_roi/ratio` | signal | [signal_ws/src/autorace_signal/autorace_signal/node.py:160](../signal_ws/src/autorace_signal/autorace_signal/node.py#L160) |
| `/signal/white_roi/detected` | signal | [signal_ws/src/autorace_signal/autorace_signal/node.py:161](../signal_ws/src/autorace_signal/autorace_signal/node.py#L161) |
| `/yolo/bump_sign` | slow, yolo | [slow_ws/src/autorace_slow/autorace_slow/node.py:82](../slow_ws/src/autorace_slow/autorace_slow/node.py#L82) |
| `/yolo/slow_sign` | slow, yolo | [slow_ws/src/autorace_slow/autorace_slow/node.py:82](../slow_ws/src/autorace_slow/autorace_slow/node.py#L82) |
| `/slow/white_roi/debug/compressed` | slow | [slow_ws/src/autorace_slow/autorace_slow/node.py:103](../slow_ws/src/autorace_slow/autorace_slow/node.py#L103) |
| `/slow/white_roi/ratio` | slow | [slow_ws/src/autorace_slow/autorace_slow/node.py:104](../slow_ws/src/autorace_slow/autorace_slow/node.py#L104) |
| `/slow/white_roi/detected` | slow | [slow_ws/src/autorace_slow/autorace_slow/node.py:105](../slow_ws/src/autorace_slow/autorace_slow/node.py#L105) |
| `/yolo/debug/compressed` | yolo | [yolo_ws/src/yolo_detect.py:224](../yolo_ws/src/yolo_detect.py#L224) |
| `/joint_states` | sensor | [sensor_ws/src/autorace_sensor_bringup/launch/sensors.launch.py:206](../sensor_ws/src/autorace_sensor_bringup/launch/sensors.launch.py#L206) |
| `/robot_description` | sensor | [sensor_ws/src/autorace_sensor_bringup/launch/sensors.launch.py:206](../sensor_ws/src/autorace_sensor_bringup/launch/sensors.launch.py#L206) |
| `/diagnostics` | sensor | [sensor_ws/src/robot_localization/src/ros_filter.cpp:2044](../sensor_ws/src/robot_localization/src/ros_filter.cpp#L2044) |
| `/set_pose` | sensor | [parking_ws/src/autorace_parking/autorace_parking/start_sequence.py:35](../parking_ws/src/autorace_parking/autorace_parking/start_sequence.py#L35) |

# 센서 frame_id와 장착 TF

사용자가 기본 LIMO Pro 장착 상태임을 확인하여 `tf_profile:=wego_limo_pro`를 기본으로 적용했다.
실측 캘리브레이션이 아니라 공식 소스의 명목 장착값이다. 토픽 이름과 frame_id는 서로 다르다.
길이는 m, 회전은 rad이며 차체 좌표는 x 전방, y 좌측, z 위쪽이다.

## 토픽의 좌표계

| 토픽 | header.frame_id | child_frame_id | 발행자 |
|---|---|---|---|
| /scan | laser_link | 해당 없음 | YDLIDAR |
| /imu | imu_link | 해당 없음 | limo_base 내장 IMU |
| /odom | odom | base_link | limo_base |
| /odometry/filtered | odom | base_link | EKF |
| /camera/color/image_raw | camera_color_optical_frame | 해당 없음 | Orbbec |
| /camera/color/image_raw/compressed | camera_color_optical_frame | 해당 없음 | image_transport |
| /camera/depth/image_raw | camera_depth_optical_frame | 해당 없음 | Orbbec |

`camera_name:=front_camera`라면 카메라 토픽 접두사와 카메라 프레임 접두사도 front_camera로 바뀐다.
위 표는 기본 astra_stereo_u3.launch.py, depth_registration=false 기준이다.
등록된 깊이 영상이나 다른 카메라 launch를 사용하면 드라이버의 frame_id를 다시 확인한다.
/limo_status는 상태 메시지이므로 공간 센서 좌표계를 지정하지 않는다.

## 공식 기본 배치

| TF 부모 → 자식 | xyz | rpy |
|---|---|---|
| base_link → base_footprint | (0, 0, -0.15) | (0, 0, 0) |
| base_link → laser_link | (0.103, 0, -0.034) | (0, 0, 0) |
| base_link → imu_link | (0, 0, -0.1) | (0, 0, 0) |
| base_link → camera_mount | (0.103, 0, 0.06) | (0, 0, 0) |
| camera_mount → camera_rotate | (0, 0, 0) | (0, degree, 0) |
| camera_rotate → camera_link | (0.03, -0.05, 0) | (0, 0, 0) |

degree=0일 때 base_link에서 camera_link까지 합성한 위치는 (0.133, -0.05, 0.06)이다.
카메라 광학 좌표는 x 오른쪽, y 아래쪽, z 전방이다.
camera_link 아래 color/depth/optical 프레임의 위치·방향은 Orbbec SDK의 내부 보정을 사용한다.
이를 차체와 같은 방향의 임의 TF로 덮어쓰지 않는다.
통합 launch는 Orbbec의 tf_publish_rate=0.0으로 내부 보정 TF를 /tf_static에 발행한다.
장착 오프셋은 그대로이고, 카메라가 직접 계산한 내부 보정값 역시 유지된다.

공식 Xacro는 base_footprint→base_link를 z=+0.15로 표현한다.
이 패키지는 기존 limo_base/EKF의 odom→base_link 계약을 유지하기 위해
동일한 두 프레임 관계를 역방향 base_link→base_footprint, z=-0.15로 표현했다.
따라서 base_link가 TF 루트이며 이 변환은 별도 static_transform_publisher로 중복 발행하지 않는다.

레이저·IMU·차체 TF는 robot_state_publisher만, 카메라 장착 TF는 camera_tf.launch.py만,
카메라 내부 TF는 Orbbec만 발행한다. EKF 활성화 시 odom→base_link는 EKF만 발행한다.
EKF를 끄면 limo_base가 해당 odometry TF를 발행한다.

## 이전 로컬 보정

```bash
ros2 launch autorace_sensor_bringup sensors.launch.py tf_profile:=existing
```

이 프로파일은 이전 로컬 TF를 유지한다:
base_link→base_footprint=(0.1,0,0.1), 레이저·IMU의 부모는 base_footprint,
각각 (0.103,0,-0.034), (0,0,-0.1).
카메라 마운트는 base_link 기준 (0.2,0.1,0.06)이며 마지막 오프셋과 degree 회전은 동일하다.
두 프로파일의 기준 프레임이 다르므로 숫자를 섞어서 사용하면 안 된다.
주행 모드, /cmd_vel, use_mcnamu=false 및 Ackermann odometry C++ 수정은 변경하지 않았다.

## 출처

WeGo-Robotics/limo_pro, commit `7ef355cd515e806b26f498d38bfe2abe4f3a9728`:

- [공식 Ackermann 모델의 레이저·IMU·차체 기준](https://github.com/WeGo-Robotics/limo_pro/blob/7ef355cd515e806b26f498d38bfe2abe4f3a9728/limo_ros2/limo_description/urdf/limo_ackerman.xacro)
- [공식 카메라 장착 TF](https://github.com/WeGo-Robotics/limo_pro/blob/7ef355cd515e806b26f498d38bfe2abe4f3a9728/wego/launch/camera_tilt_launch.py)

공식 저장소의 별도 depth_camera_link 고정 프레임은 추가하지 않았다.
현재 사용하는 Orbbec SDK 카메라의 실제 내부 프레임을 연결하여 TF 중복·가짜 광학 보정을 피한다.
차체 시각 모델과 바퀴의 기존 크기·트랙은 이번 센서 TF 작업에서 교체하지 않았다.

## 검증

하드웨어를 켜지 않고 두 TF 프로파일, 비영점 카메라 pitch, camera_name 변경,
static TF 발행자 수·이름과 자식별 부모 일관성를 검사한다:

```bash
cd /home/wego/2026-AutoRace/sensor_ws
source install/setup.bash
ROS_DOMAIN_ID=180 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$PWD/log/tf_verification" \
  /usr/bin/python3 src/autorace_sensor_bringup/tests/tf_check.py
```

센서가 이미 실행 중일 때 실제 메시지 헤더와 TF를 읽기 전용으로 확인한다:

```bash
ros2 topic echo /scan --field header --once
ros2 topic echo /imu --field header --once
ros2 topic echo /camera/color/image_raw --field header --once
ros2 topic echo /camera/depth/image_raw --field header --once
ros2 run tf2_ros tf2_echo base_link laser_link
ros2 run tf2_ros tf2_echo base_link imu_link
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

이번 검증은 하드웨어 없는 TF 검증이며 실제 센서 헤더·실측 장착 오차는 현장 확인 대상이다.

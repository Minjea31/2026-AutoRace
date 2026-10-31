# 치수·센서·인터페이스 근거

## 저장소 원본

- [트랙 도면](../../assets/2026Track%20도면.pdf): CAD 도면의 10,000×6,000 mm 외곽, 주차장 1,580×1,240 mm, 내부 500×600 mm, 테이프 20 mm. `tools/extract_track.py`가 `pdftocairo` SVG에서 도로 엔티티 276개를 추출하고 치수선/도곽을 제외한다. 좌표 원점은 매트 왼쪽 아래이다. CAD export 반올림은 약 0.4 mm 이내이다. 도로 가장자리는 2.5 mm/pixel로 래스터화하고, 글자·횡단보도 등은 1.25 mm/pixel로 복원해 런타임 8,000×4,800 텍스처에 저장한다.
- [규정집](../../assets/Autorace2026_규정집.pdf): 주차 양옆 정차 LIMO 최소 2대. 차량과 표지판의 구체적인 위치는 도면에 없어 연습 배치로 분리했다. 사용자 요청에 따라 점수 기능은 구현하지 않았다.
- [토픽 문서](../../docs/architecture.md), [토픽 JSON](../../docs/architecture-topics.json): `/camera/color/image_raw/compressed`, `/scan`, `/mission/parking/command_03`, `/cmd_vel` 및 타입. YOLO Bool은 실제 YOLO 노드가 발행한다.
- [카메라 launch](../../sensor_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/astra_stereo_u3.launch.py): RGB 640×480@30, Depth 640×400@30, depth_registration=false, camera_name=camera, default Reliable camera QoS.
- [라이다 YAML](../../sensor_ws/src/autorace_sensor_bringup/config/ydlidar.yaml): sample_rate=4 kHz, frequency=10, angle_min/max=−110/+110, range_min/max=.01/64, invalid_range_is_inf=false.
- [센서 TF 명세](../../sensor_ws/src/autorace_sensor_bringup/FRAMES.md), [카메라 TF launch](../../sensor_ws/src/autorace_sensor_bringup/launch/camera_tf.launch.py): `wego_limo_pro` 프로파일, base_link 지상고 .15 m, laser_link/imu_link/camera_mount 및 카메라 연결.
- [실차 드라이버 상수](../../sensor_ws/src/limo_ros2/limo_base/include/limo_base/limo_driver.h): wheelbase=.2, track=.172, max_inner_angle=.48869. 중앙 조향으로 변환한 실제 제한을 시뮬레이터에 적용한다.
- [실차 Twist 처리와 적분](../../sensor_ws/src/limo_ros2/limo_base/src/limo_driver.cpp): Twist의 angular.z는 yaw rate. 시뮬레이션은 조향각과 혼동하지 않는다. 앞바퀴 조향을 내부적으로 계산하고 v=0에서는 회전하지 않는다.
- [P 표지판 이미지](../../assets/signs/parking.png)를 시뮬레이션 텍스처로 복사했다.
- [미션별 원본 표지판](../../assets/signs/)의 방지턱·서행·보행자·갈림길 A/B 이미지도 사용한다. 신호등은 실제 켜지는 세 개의 원형 등을 3D로 만들고, 광고판은 혼동을 일으킬 수 있는 적색·녹색 화면과 문자를 배치했다.
- [로컬 LIMO 차체 메시](../../sensor_ws/src/limo_ros2/limo_description/meshes/limo_base.dae), [바퀴 메시](../../sensor_ws/src/limo_ros2/limo_description/meshes/limo_wheel.dae)를 `tools/build_limo_model.py`로 Panda3D BAM에 변환했다. 원본 재질 색상·차체·스크린·타이어 트레드를 보존하고 작은 삼각형을 단순화했다. 차체는 Xacro와 같은 Z축 90° 회전 후 길이 322 mm / 높이 251 mm로 맞췄고, 바퀴는 반지름·폭 45 mm로 맞췄다. 원본 해시와 변환 정보는 [limo_pro_model.json](../src/limo_parking_sim/assets/limo_pro_model.json)에 있다.

기존 `sensor_ws`, 미션 워크스페이스와 토픽 설계 문서는 변경하지 않았다.

## 공식 외부 자료

2026-10-31 확인. 온라인 제품 사양은 실측 보정값을 대체하지 않는다.

- [WEGO 공식 limo_pro 저장소](https://github.com/WeGo-Robotics/limo_pro), [Ackermann Xacro](https://github.com/WeGo-Robotics/limo_pro/blob/7ef355cd515e806b26f498d38bfe2abe4f3a9728/limo_ros2/limo_description/urdf/limo_ackerman.xacro), [카메라 장착 launch](https://github.com/WeGo-Robotics/limo_pro/blob/7ef355cd515e806b26f498d38bfe2abe4f3a9728/wego/launch/camera_tilt_launch.py): 저장소의 기존 명목 TF 근거와 대조했다. Xacro의 간략 충돌 박스/구형 윤거를 차량 전체 외곽으로 사용하지 않았다.
- [AgileX LIMO Pro 공식 매뉴얼](https://github.com/agilexrobotics/limo_pro_doc/blob/master/Limo%20Pro%20user%20manual(EN).md), 1.3/5.1/7.1절: 차량 외곽·축간·센서 모델과 기본 광학/거리 사양을 확인했다.
- [YDLIDAR T-mini Pro 제조사 설명](https://www.ydlidar.com/news/hot-product-recommendation-t-mini-pro-tof-lidar-for-precision-sensing-capturing-everything-in-sight): 물리적 12 m, 360° 기기이며 현재 실차 드라이버 설정은 전방만 출력한다.
- [Panda3D render-to-texture](https://docs.panda3d.org/1.10/python/programming/render-to-texture/low-level-render-to-texture): 실제 RGB/깊이 렌더 버퍼의 구현 근거.

## 명목값과 가정값

차량 외곽 322×220×251 mm, 축간 200 mm, 윤거 175 mm, 최고속도 1 m/s는 제조사 명목값이다. 28° 안쪽 조향 및 172 mm 변환 윤거는 현재 로컬 드라이버의 값이므로, 제품 명목 최소 회전반경보다 더 보수적인 유효 회전반경 약 0.462 m가 된다.

카메라 RGB 화각은 제조사 고해상도 모드 명목값을 낮은 해상도에도 임시 적용했다. 실제 640×480 모드의 K/D, 각 센서 광학 중심의 정확한 오프셋은 장치 SDK 측정이 필요하다. 기본 RGB/Depth 내부 오프셋 0은 보정 완료를 뜻하지 않는다. `intrinsics`, `xyz`, `rpy`를 통해 투영과 TF를 함께 바꿀 수 있다. 렌즈 왜곡, 노출·블러, 구조광 결측은 구현하지 않았다.

라이다는 한 시각의 장면을 스캔하므로 `time_increment=0`이다. 원시 장치의 회전 위상, 포인트별 시간차와 신호 강도를 정밀하게 재현하지 않는다. 기본 거리는 이상적이고 설정으로 Gaussian 잡음과 누락률을 추가할 수 있다. 강도는 시각적 재질 모델이 아닌 유효 거리 표시값이다. 장애물의 충돌/라이다 외곽은 실차 전체 외곽 크기의 회전 직육면체이며, RGB는 로컬 차체·바퀴 메시를 사용한다. 메시 단순화 해상도는 2 mm이다.

구동은 평면 운동 모델이다. 질량은 사양 기록용이며 힘·마찰을 계산하는 동역학 파라미터로 쓰지 않는다. 가속도 1 m/s², 중앙 조향 속도 1.5 rad/s, 주차 차량/표지판 위치는 연습용 설정값이다. 물체 접촉 시 이동을 막고 충돌을 표시하며, 시설물을 밀거나 넘어뜨리지 않는다.

사용자 요청으로 앞줄 양옆 2대와 안쪽 줄 3대를 배치했다. 가운데 앞줄은 주차 목표 공간이다. 표지판 판면은 y=4.570188 m의 흰 선을 넘어 차로에 돌출되지 않게 했다. 정차 차량·표지판 세부 위치는 확정된 실측 배치가 아니다. 마우스 제어는 관찰 화면만 이동하며, 라이다 광선과 카메라 미리보기는 기본으로 숨긴다. ROS 센서 발행은 유지한다.

추가 사용자 지정 위치에 따라 왼쪽 세로 구간의 두 정지선 사이에 연속 S자 통로의 양쪽을 따라 콘 48개를 배치했다. 하단 장애물 LIMO는 사용자 마지막 그림의 위 3·8번 / 가운데 4·5·7번 / 아래 2·6번 배치로 변경했다. 위치 메타데이터만 8칸으로 나누며 화면에는 격자·번호·색상을 표시하지 않는다. 섬의 오른쪽 끝 L/R 교체 표지판과 각 미션의 흰 기준선 옆 표지판은 유지한다. 18 mm 방지턱은 차체 지상고 아래이고, 콘의 높이별 원뿔 반지름은 라이다와 차체 충돌 계산에 사용한다. 구체적인 콘 크기와 세부 좌표는 연습 설정값이다.

후속 사용자 요청으로 콘과 차량의 양쪽 흰 차선 침범을 허용하고, S자 진폭을 6 cm에서 20 cm로 키웠다. 이어서 ‘S자 두 개가 연속되는 모양’이라는 명시적 답변과 오른쪽 차선 밖 사용 허용을 반영해 두 정지선 사이에 2주기 파형을 배치했다. 콘 줄은 최대 좌우 이동 폭 40 cm, 가로 간격 64 cm, 양쪽 24개씩이다. 급한 굴곡에서 수직 오프셋이 접혀 겹치지 않도록 가로 방향으로 줄을 나누고 각 줄의 호 길이에 따라 균일하게 배치한다. 검증 차량은 통로 여유 공간을 활용하는 별도 경로(진폭 4.8 cm, 최소 회전반경 약 51.7 cm)를 사용한다. 콘 줄 자체의 가운데를 추종할 수 있다고 가정하지 않는다. 원래 흰 도로 선은 콘 미션의 충돌 경계가 아니다.

사용자가 제공한 표지판 도면과 후속 치수 설명으로 전체 높이 200 mm가 **지름 90 mm의 원판을 포함**함을 확인했다. 지면부터 판 중심 155 mm, 원판 아래 110 mm, 원판 두께 10 mm와 인쇄부 지름 80 mm로 반영했다. 직사각형 흰 종이 대신 실제 원판과 테두리를 렌더링하며, 삼각형 표지판은 원래 모양을 유지한다. `tools/prepare_sign_prints.py`는 원본 이미지의 종이 여백을 제거해 인쇄부에 맞추며, 인쇄 그림은 같은 원본을 쓴다. 라이다는 원형 표지판과 스캔 높이가 만나는 실제 현의 폭으로 교차를 계산한다.

원본 CAD에서 횡단보도 줄 16개와 경계선 4개를 따로 채워 검정 간격을 보존한다. 한글은 시스템 NanumGothicBold로 생성 시 래스터화하며 런타임에는 폰트가 필요 없다. 마름모 내부·A/B·체커 출발선도 따로 복원한다. 재생성 도구에는 OpenCV가 필요하지만 사용자 Conda 실행 환경에는 추가 설치하지 않았다.

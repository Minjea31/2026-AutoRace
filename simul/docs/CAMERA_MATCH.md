# 실차 카메라와 시뮬레이터 비교

2026-11-01 확인 결과: **출력 해상도와 설정 FPS는 같지만, 실제 카메라 화질과 동일하다고 볼 수 없습니다.** 실제 카메라 영상과 장치별 보정값을 수신하지 못했으므로 실측 화질 비교는 아직 하지 못했습니다.

| 항목 | 실차 드라이버 기본 설정 | 시뮬레이터 | 판단 |
| --- | --- | --- | --- |
| 컬러 영상 | 640×480, 30 FPS | 640×480, 30 FPS | 설정 일치. 실제 수신 FPS는 미측정 |
| 깊이 영상 | 640×400, 30 FPS | 640×400, 30 FPS | 설정 일치. 실제 센서 노이즈·결측은 미재현 |
| 컬러 압축 | 센서 입력 MJPG → 드라이버 디코딩 → image_transport 압축 | 렌더링 RGB → Pillow JPEG quality=90 | 동일 압축 품질을 확인한 상태가 아님 |
| 자동 노출·화이트밸런스 | 둘 다 true | 없음 | 밝기·색감이 실차와 달라질 수 있음 |
| 렌즈 왜곡 | 장치 보정값을 CameraInfo에 반영 | D=0 | 실차 raw 영상과 다름 |
| 화각·내부 보정 | 장치/해상도별 K 필요 | color.intrinsics=null, 명목 화각 71°×43.7° | 640×480 모드와 일치 여부 미확인 |
| 흐림·노이즈·조명 변화 | 실물 센서/환경 영향 | 실측에 맞춘 모델 없음 | 같은 해상도가 같은 인식 난이도를 뜻하지 않음 |

근거 파일: [실차 기본 launch](../../sensor_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/astra_stereo_u3.launch.py), [실차 카메라 드라이버](../../sensor_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp), [시뮬레이터 설정](../src/limo_parking_sim/config/limo_pro.yaml), [시뮬레이터 CameraInfo/JPEG 발행](../src/limo_parking_sim/limo_parking_sim/ros_bridge.py).

현재 PC에서는 Orbbec USB vendor ID `2bc5`가 확인되지 않았습니다. 감지된 `5986`의 `HD Camera`를 LIMO 카메라로 취급하지 않았습니다. localhost의 ROS 도메인 0과 42에서 각각 5초간 확인했으나 컬러 압축 영상·컬러/깊이 CameraInfo 발행자가 없었습니다. 다른 PC나 다른 도메인에 카메라가 있는지는 이 검사로 판단할 수 없습니다. 재현 가능한 검사 결과는 `simul/captures/camera-check/report.json`에 있습니다.

실물 카메라를 켠 뒤 같은 환경/ROS 도메인에서 다음을 실행하면 실제 컬러/깊이 K·D·해상도, JPEG 영상과 메시지 타임스탬프 기준 FPS를 수집합니다. 시뮬레이터 발행자는 제외하며, 장치 설정은 변경하지 않습니다.

```bash
conda activate autorace-simul
source simul/env.sh
python simul/tools/check_camera_match.py --live --domains 0 --seconds 10
```

다른 PC의 카메라를 검사하려면 해당 PC에서 실행하거나 ROS 네트워크 연결을 별도로 준비해야 합니다. `simul/env.sh`는 localhost 통신을 사용합니다. 수집한 CameraInfo의 `[K[0], K[4], K[2], K[5]]`를 `camera.color/depth.intrinsics`에 반영하면 투영을 맞출 수 있습니다. 왜곡 D, 센서 노이즈, 노출·색감·모션 블러까지 맞추려면 같은 조명과 장면에서 촬영한 실물 영상의 추가 보정이 필요합니다. 임의의 노이즈를 넣고 동일한 화질이라고 간주하지 않았습니다.

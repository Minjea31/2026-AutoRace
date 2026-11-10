# AutoRace Web Viewer

플랫폼이 ROS 영상을 수신해 HTTP로 제공합니다. 같은 네트워크의 PC에서 브라우저만으로 접속합니다.

```bash
cd ~/2026-AutoRace/web_viewer
./start.sh
# 종료
./stop.sh
```

접속 주소: `http://192.168.50.97:8080` (플랫폼 IP가 바뀌면 새 IP 사용).
기본 ROS_DOMAIN_ID는 96입니다. 센서와 같은 도메인을 사용하세요.

```bash
ROS_DOMAIN_ID=96 PORT=8080 FPS=10 ./start.sh
# 터미널에서 직접 실행하려면 ./run.sh
```

| 화면 | 수신 토픽 |
| --- | --- |
| 카메라 원본 | `/camera/color/image_raw/compressed` |
| YOLO bbox | `/yolo/debug/compressed` |
| 차선 마스크 | `/lane_fusion/deep_mask` |
| 차선 디버그 | `/lane_fusion/debug` |
| 서행·방지턱 바닥 ROI | `/slow/white_roi/debug/compressed` |

YOLO 화면은 bbox 아래에 클래스, 가로×세로, 면적(px²)을 작게 표시합니다.
해당 프레임에서 실제 검출 신호 발행 조건을 만족한 bbox는 클래스별 색상, 나머지는 회색입니다.
좌회전 토픽의 False도 정상 발행이므로 색상으로 표시합니다.

차선 노드가 꺼져 있으면 해당 화면은 대기 상태입니다. 웹 서버는 미션 노드를 시작하거나 차량 제어 명령을 발행하지 않습니다.
JPEG 카메라는 재인코딩 없이 전달하고, raw 마스크·디버그는 프레임당 한 번 JPEG로 변환합니다. 기본 전송 상한은 각 영상 10FPS이며, 여러 브라우저가 인코딩 결과를 공유합니다.
이미지 저장, 전체화면, 영상별 보기, 수신 상태 및 자동 재연결을 지원합니다.

ROS Humble의 시스템 Python(`/usr/bin/python3`), rclpy, sensor_msgs, NumPy, OpenCV를 사용합니다. pip 설치나 colcon 빌드는 필요하지 않습니다.
로그: `.run/viewer.log`. PID: `.run/viewer.pid`.
LAN용 HTTP 서버이며 별도 로그인은 없습니다. 자동 부팅 등록은 하지 않습니다.

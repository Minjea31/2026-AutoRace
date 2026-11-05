# 검증 결과와 남은 항목

이 문서는 초기 구현과 ROI 개선 기록이다. 이후 다차로 확장은 [최신 검증 보고서](multilane_validation.md)에 정리했다. 반사광·Center Path 변경의 전후 결과는 [추가 검증 보고서](path_validation.md)를 참고한다.

작성일: 2026-11-02 (Asia/Seoul). 로그에 표시된 호스트 시각과 클라이언트 날짜가 다를 수 있다.

## 변경 범위와 환경

- 기준 commit: `5a9d6a835850d0ead37a66ce1520a16591698ce5`, 로컬 브랜치 `lane_ws`.
- 구현·설정·문서·테스트는 `lane_ws/` 아래에 작성했다. 다른 팀 워크스페이스와 팀 토픽 설계는 수정하지 않았다. push/merge는 하지 않았다.
- 재개 시 별도로 존재한 `.vscode/colcon.env`는 이번 구현 결과에서 제외하고 보존했다.
- Ubuntu 22.04.5, ROS 2 Humble, Intel Core i7-13620H, OpenCV 4.5.4, Qt 5.15.3, C++17 Release 빌드. OpenCV 내부 스레드 수는 1.
- 결과 파일은 `lane_ws/artifacts/`, 빌드·ROS 로그는 `lane_ws/log/`에 보관한다. 원본 Bag은 다른 SSD에서 읽기 전용으로 열었고 저장소에 복사하지 않았다.

## 빌드와 실행

| 항목 | 결과 | 근거 |
| --- | --- | --- |
| `colcon build` | 통과 | `artifacts/final_build.txt` |
| C++ 단위 테스트 | 9개 통과 | `artifacts/colcon_test.txt`, `build/autorace_lane/test_results/` |
| `colcon test-result` | errors/failures/skipped 모두 0 | CTest 래퍼까지 포함하면 10 tests로 집계 |
| 실제 ROS pub/sub | 통과 | `artifacts/ros_check.txt` |
| launch 시작·SIGINT 종료 | 정상 | `artifacts/launch_check.txt` |
| 실제 Bag 전체 순차 처리·역방향 seek | 통과 | `artifacts/final_default/`, `artifacts/final_lab/` |
| Bag GUI 탐색·이전 프레임·정지 중 재튜닝·2× 재생 | 통과 | `artifacts/bag_gui_check.txt` |
| 실시간 원본/JPEG GUI | 기능 통과, 원본 전송 손실 관찰 | `artifacts/live_gui_check.txt`, `artifacts/gui_live_*.csv` |

GTest는 설정 검증·YAML 왕복, 직선·좌우 단일 차선, 곡선 조향 부호·이상점·변화율, 소실 정지·재탐색, 시간 역행, 유효한 A/B와 조향 방향·방향 고정·완료, 평행선의 갈림길 오인 방지, HLS 노란색·마스크 비활성, 지그재그 조향 제한을 검사한다.

ROS 테스트는 별도 domain 87과 `/lane_test/*` 토픽을 사용한다. startup 정지, 20 Hz 출력, base_link, 신선한 JPEG 주행, 입력 중단, 오래된/미래 stamp, 손상 이미지, 잘못된 파라미터의 원자적 거부와 복구, `/clock` 정지 시 wall watchdog을 확인했다. 실차·mux·다른 팀 노드를 실행한 테스트가 아니다.

최종 빌드는 경고 없이 통과했다. 앞선 재설정 빌드의 stderr에는 배포판 `gtest_vendor`의 오래된 CMake 호환성 선언에 관한 경고가 있었으며 컴파일 오류는 아니었다. Python 테스트에는 사용자 NumPy 2 ABI 충돌을 피하도록 `PYTHONNOUSERSITE=1 /usr/bin/python3`를 사용했다.

## 실제 Bag 측정

입력: `/mnt/Ubuntu_Data/autorace_bags/lane_20261012_125620`

- SQLite3 약 2.9GB, 카메라 `/camera/color/image_raw`, `sensor_msgs/msg/Image`, RGB8 640×480.
- 카메라 3,311프레임, 첫/마지막 카메라 간격 162.583초. 전체 Bag metadata duration은 162.772초이다.
- 카메라 평균 기록 빈도 약 20.37 FPS. 원본 timestamp 간격을 재생 스케줄에 사용한다.
- CameraInfo/IMU도 들어 있으나 차선 처리에는 사용하지 않았다. `/current_lane`, `/yolo/LR_sign` 기록은 없다.

| 전체 프레임 처리 | 기본 `lane.yaml` | 탐색용 `lab_bag.yaml` |
| --- | ---: | ---: |
| 인덱스 구성 | 0.539초 | 0.562초 |
| 전체 처리 시간 | 16.101초 | 14.570초 |
| 오프라인 처리량 | 205.6 FPS | 227.3 FPS |
| 읽기+디코딩+CV 평균 | 4.84ms | 4.38ms |
| 읽기+디코딩+CV p95 | 6.47ms | 5.74ms |
| 좌우 모두 검출 | 549 | 752 |
| 한쪽만 검출 | 2,233 | 1,749 |
| 검출 없음 | 529 | 810 |
| 계산된 속도 0 | 339 | 579 |

기본 설정 측정의 최대 RSS는 약 86.1MiB, swap 0이었다 (`artifacts/benchmark_resources.txt`). 파일시스템 입력 카운터가 0이므로 OS 페이지 캐시가 따뜻한 실행이다. SSD의 cold-read 성능이라고 해석하면 안 된다. JPEG 시각화 파일 저장 시간도 전체 처리 시간에 포함되며 프레임별 CSV의 total_ms는 해당 저장 시간을 제외한다.

검출 유무는 정답 라벨과 비교한 정확도가 아니다. 실제 Bag에는 트랙 밖 의자·가구를 보는 구간, 급곡선, 강한 반사광이 있다. 예를 들어 700번 프레임은 차선 대신 의자를 향한다. 단일 차선으로 계산된 곡선이 반사광·배경에 붙는 경우도 시각적으로 확인했다. `lab_bag.yaml`은 흰 외곽선에 대한 별도 투영 예시이며 기본 설정보다 좋다는 결론을 내리지 않는다. 차량 위치·노출·도로의 선 의미에 맞춘 현장 보정과 라벨링이 필요하다.

## GUI 성능 해석

Qt offscreen 플랫폼에서 실제 QWidget 생성·이벤트 루프·네 화면 렌더링을 실행했다. Bag 테스트는 재생 도중 이전 프레임, 중간 seek, 정지 상태의 임계값 변경, 재개·2×를 자동 조작한다. `artifacts/final_gui.png`에 화면을 저장했다. 실제 모니터 compositor·GPU 출력의 부드러움까지 검증한 것은 아니다.

진단용 `--screenshot`을 포함한 실행에서는 최장 Qt timer 간격 81.13ms를 관찰했다 (`artifacts/gui_with_screenshot.csv`). PNG 저장도 Qt 메인 스레드에서 실행되므로 일반 GUI 성능과 구분하기 위해 최종 시간 측정에서는 해당 옵션을 제외했다. 화면 PNG는 별도 실행에서 확보했다. 측정 FPS는 Bag의 1×/2× 전환과 탐색을 합친 평균이며 고정 30 FPS 시험이 아니다.

최종 Bag GUI 독립 실행 (`artifacts/final_gui.csv`, screenshot 저장 제외):

| 항목 | 측정 |
| --- | ---: |
| 처리 결과 / 화면 표시 | 441 / 440 |
| 조작을 포함한 평균 화면 FPS | 29.02 |
| 결과 준비→렌더 평균 | 8.14ms |
| 최장 Qt timer 간격 | 16.92ms |
| 최대 RSS | 약 123.5MiB |
| 프로세스 CPU (`time -v`, 한 코어 100% 기준) | 35% |
| 이전 프레임 / seek / 정지 중 튜닝 | 모두 통과 |

실시간 테스트는 영상 생성기와 GUI를 별도 프로세스로 실행했다. 최종 실행에서 (JPEG는 별도 토픽 인자 없이 기본 입력 경로 검증):

| 전송 | 송신 | GUI ROS 콜백 수신 | 처리 결과 | 화면 표시 | 표시 FPS |
| --- | ---: | ---: | ---: | ---: | ---: |
| 원본 640×480 BGR8 | 154 | 58 | 59 | 58 | 11.54 |
| JPEG 640×480 | 146 | 140 | 141 | 139 | 29.08 |

처리 결과가 수신보다 하나 많은 것은 파라미터 변경으로 같은 프레임을 재계산했기 때문이다. 최초 발견·종료 구간도 송신 수에 포함된다. 원본 저속 실행에서는 수신과 처리 수는 일치하고, 표시 단계에서는 최신 슬롯 때문에 일부 결과를 건너뛴다. 대부분의 손실은 ROS 콜백에 도착하기 전에 발생했다. 앞선 동일 구조 실행의 raw 표시 빈도는 5.56 FPS였으므로 raw 수신 성능의 변동성도 관찰했다. 정확한 DDS/커널 원인은 확정하지 않았다. 시스템 네트워크 설정을 변경하지 않았다. 실시간 기본 경로는 아키텍처와 같은 compressed 토픽이며, raw Bag은 직접 읽어 이 경로를 우회한다. 임의 raw 토픽도 지원하지만 이 PC의 해당 전송 경로에서 30 FPS를 보장하지 않는다.

원본/JPEG 실행 모두 준비→렌더 평균 약 8ms, 최장 timer 간격 약 17ms였다. 최신 결과 슬롯이 있으므로 처리보다 표시가 느린 상황에서는 표시 프레임을 건너뛰며 오래된 결과를 쌓지 않는다. 읽기·CV·변환 시간과 화면 FPS는 서로 다른 지표이다.

## 미완료·실차 적용 전 필요한 검증

1. 실제 A/B 갈림길과 지그재그 주행 영상에 대한 검증. 발견한 Bag에는 해당 Bool과 확정된 미션 라벨이 없어 이 부분은 합성 테스트까지 확인했다.
2. 실차 카메라 설치에 맞는 BEV·차로 폭·lookahead 거리 보정과 저속 폐루프 주행. 픽셀 좌표를 물리량으로 바꾸는 현재 설정은 보정된 카메라 모델을 대체하지 않는다.
3. 반사광·교차선·가구가 만드는 거짓 경계와 트랙 이탈 상황에 대한 정답 데이터 평가. 기록 영상에서 검출됨은 안전한 주행 경로임을 뜻하지 않는다.
4. 영상만 사용하는 지그재그 평균 진행선 추종의 실제 헤딩 유지 성능. IMU/odom은 팀 계약에 따라 추가 구독하지 않았다.
5. 실제 카메라 드라이버·네트워크·모니터 환경에서 장시간 실행과 rqt/image_view/RViz의 동일 조건 A/B 측정. rqt의 기존 끊김 원인은 확정하지 않았다.
6. MCAP·대규모 다중 파일 Bag 검증. 구현은 rosbag2 storage plugin을 사용하지만 실측은 위 SQLite3 Bag에 한정한다.

실차 주행 완료나 대회 미션 성공을 주장하지 않는다. 재현 명령과 튜닝 방법은 [README](../README.md), 구조와 rqt 소스 분석은 [설계 기록](design.md)에 있다.


## ROI GUI 개선 검증 (2026-11-02)

- `roi_left/right`와 독립 BEV 높이 추가 후 `colcon build` 통과 (`artifacts/roi_build.txt`).
- C++ 코어 12개 + Qt 마우스 테스트 3개 통과. CTest 래퍼를 포함한 `colcon test-result`는 17 tests, 0 errors/failures/skipped (`artifacts/roi_tests.txt`).
- 네 경계·모서리 드래그, 가로/세로 letterbox, 창 크기 변경, 화면 밖 드래그의 clamp, 경계 역전 방지, 여백/내부 클릭 무시를 실제 Qt mouse event로 확인했다.
- 기존 YAML의 BEV 높이 이관, 신규 좌우 ROI와 BEV의 YAML 왕복, 원본 픽셀 보존, 사각 ROI의 상하좌우 마스크, BEV와 겹치지 않는 ROI의 정지 처리를 확인했다.
- ROS 노드에 `roi_left=0.4`, `roi_right=0.6`을 atomic 서비스로 적용해 합성 좌우 차선을 제외하고 정지하는 것을 확인했다. 역전된 좌우 값은 거부했고, 0/1로 복원하면 BEV 변경 없이 주행 계산이 복구됐다 (`artifacts/roi_ros_check.txt`).
- 실제 Bag GUI에서 이전 프레임·seek·정지 중 파라미터 변경·**마우스 ROI 드래그**·재생 재개가 통과했다. `roi_drag=1`은 spin box 동기화, BEV 높이 불변, 정지 프레임 재계산과 ROI 왼쪽 바깥 Binary=0까지 확인한 결과다 (`artifacts/roi_gui_check.txt`). 화면: `artifacts/roi_gui.png`.
- 기본 ROI(좌우 0/1)로 실제 Bag 3,311프레임을 다시 처리해 이전 구현의 프레임/시간/좌우 검출/신뢰도/속도/조향/상태 CSV와 비교했다. 차이는 **0개**였다 (`artifacts/roi_regression/comparison.txt`). 기존 검출 동작을 보존했다.

이 절의 GUI 검증은 screenshot PNG 저장을 포함하므로 timer 최대 간격을 일반 재생 성능 수치로 사용하지 않는다. 이전에 명시한 실차·실제 A/B·지그재그 검증 한계는 동일하다.

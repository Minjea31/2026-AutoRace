# 2026 AUTO RACE 스케일카 자율주행 경진대회

## Team

<table>
  <tr>
    <td align="center" width="150">
      <a href="https://github.com/Minjea31">
        <img src="https://github.com/Minjea31.png?size=200" width="100" height="100" alt="김민재 GitHub 프로필 사진" />
      </a>
    </td>
    <td align="center" width="150">
      <a href="https://github.com/ysjee0229">
        <img src="https://github.com/ysjee0229.png?size=200" width="100" height="100" alt="지연수 GitHub 프로필 사진" />
      </a>
    </td>
    <td align="center" width="150">
      <a href="https://github.com/mini0909-web">
        <img src="https://github.com/mini0909-web.png?size=200" width="100" height="100" alt="서민영 GitHub 프로필 사진" />
      </a>
    </td>
    <td align="center" width="150">
      <a href="https://github.com/ms-c-h">
        <img src="https://github.com/ms-c-h.png?size=200" width="100" height="100" alt="최민성 GitHub 프로필 사진" />
      </a>
    </td>
  </tr>
  <tr>
    <td align="center"><a href="https://github.com/Minjea31">김민재</a></td>
    <td align="center"><a href="https://github.com/ysjee0229">지연수</a></td>
    <td align="center"><a href="https://github.com/mini0909-web">서민영</a></td>
    <td align="center"><a href="https://github.com/ms-c-h">최민성</a></td>
  </tr>
  <tr>
    <td align="center">충북대학교</td>
    <td align="center">충북대학교</td>
    <td align="center">충북대학교</td>
    <td align="center">충북대학교</td>
  </tr>
</table>

## Poster

![2026 AUTO RACE 스케일카 자율주행 경진대회 포스터](assets/poster.jpg)

## Node Flowchart

노드 구조와 노드 간 데이터 흐름은 [노드 구조 Flowchart (Google Drive)](https://drive.google.com/file/d/1IbPikYiX3ACrdugrASQr_ySAeMO6wz26/view?usp=sharing)에서 확인할 수 있습니다.

로컬 [draw.io 원본](docs/2026_AutoRace.drawio), [Flowchart 이미지](docs/2026_AutoRace.png)·[SVG](docs/2026_AutoRace.svg), [토픽·우선순위 명세](docs/architecture.md)도 함께 관리합니다.

## Workspaces

핵심 로직은 [알고리즘 그림 모음](docs/algorithms.md)에서 볼 수 있습니다. 현재 deep 차선은 `lane_fusion_ws`의 인식과 `lane_ws`의 PID를 사용하고, 갈림길은 `objavoid_ws`가 처리합니다.

| 구분 | 워크스페이스 | 담당 기능 |
| --- | --- | --- |
| 미션 | [signal_ws](signal_ws/) | 신호등 정지·발진. 방해 광고판이 있는 환경에서 신호 인식 |
| 미션 | [crosswalk_ws](crosswalk_ws/) | YOLO 보행자 표지판 Bool로 정지에 진입하고 lane 명령의 속도만 바꿔 정지·3초 대기·출발 |
| 미션 | [parking_ws](parking_ws/) | 지정 주차 구역에 T자 주차 |
| 미션 | [slow_ws](slow_ws/) (도면 이름 `slow`) | YOLO 서행 표지·deep 지그재그 판정으로 서행. lane 조향 유지 |
| 미션 | [cone_ws](cone_ws/) | 라바콘 회피 주행 |
| 미션 | [objavoid_ws](objavoid_ws/) | 정적 장애물 인식 및 차선 변경을 통한 회피 |
| 공통 | [lane_ws](lane_ws/) | 차로 유지 PID. deep 또는 HSV 인식 결과로 `/mission/lane` 발행 |
| 공통 | [lane_fusion_ws](lane_fusion_ws/) | Deep 분할·경계 추적 및 별도 HSV/딥 후보 fusion 모드 |
| 공통 | [mux_ws](mux_ws/) | 미션 command 우선순위 선택. 유효한 미션 command가 없으면 `/mission/lane` 선택. 최종 차량 제어 `/cmd_vel` 발행 |
| 공통 | [sensor_ws](sensor_ws/) | 카메라·라이다 등 센서 데이터 수집 및 처리 |
| 공통 | [yolo_ws](yolo_ws/) | 서행·횡단보도·갈림길·신호등·주차 인지 결과 발행 |

알고리즘은 [그림 모음](docs/algorithms.md), 입력·출력 토픽은 [워크스페이스별 정리](docs/architecture.md#워크스페이스별-알고리즘), 미사용 코드 삭제 전후 결과는 [검증 기록](docs/logic-cleanup.md)에 정리했습니다.

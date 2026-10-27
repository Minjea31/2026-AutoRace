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

로컬 [draw.io 도면](docs/2026_AutoRace.drawio)과 [토픽·우선순위 명세](docs/architecture.md)도 함께 관리합니다.

## Workspaces

8종 미션을 7개 미션 워크스페이스와 5개 공통 워크스페이스에서 처리합니다. 갈림길 A/B 주행은 `lane_ws`에 통합하며, `/yolo/LR_sign`의 Bool 결과로 좌·우 경로를 선택합니다.

| 구분 | 워크스페이스 | 담당 기능 |
| --- | --- | --- |
| 미션 | [signal_ws](signal_ws/) | 신호등 정지·발진. 방해 광고판이 있는 환경에서 신호 인식 |
| 미션 | [crosswalk_ws](crosswalk_ws/) | 카메라로 횡단보도·정지선을 인식하고 lane 명령의 속도만 바꿔 정지·3초 대기·출발 |
| 미션 | [parking_ws](parking_ws/) | 지정 주차 구역에 T자 주차 |
| 미션 | [slow_ws](slow_ws/) | 방지턱 앞 서행 표지판 인식, 감속 및 방지턱 통과 |
| 미션 | [zigzag_ws](zigzag_ws/) | 지그재그 서행 유도선 구간 통과 |
| 미션 | [cone_ws](cone_ws/) | 라바콘 회피 주행 |
| 미션 | [objavoid_ws](objavoid_ws/) | 정적 장애물 인식 및 차선 변경을 통한 회피 |
| 공통 | [control_ws](control_ws/) | 차량 조향 및 속도 제어 |
| 공통 | [lane_ws](lane_ws/) | 차선 인식·차로 유지 및 갈림길 A/B 주행. 기본 명령 `/mission/lane` 발행 |
| 공통 | [mux_ws](mux_ws/) | 미션 command 우선순위 선택. 유효한 미션 command가 없으면 `/mission/lane` 선택 |
| 공통 | [sensor_ws](sensor_ws/) | 카메라·라이다 등 센서 데이터 수집 및 처리 |
| 공통 | [yolo_ws](yolo_ws/) | 서행·갈림길·신호등·주차 인지 결과를 Bool 토픽으로 발행 |

모든 워크스페이스는 아래와 같은 폴더 구조를 사용합니다. `src/init.txt`는 빈 `src` 폴더를 Git에 유지하기 위한 파일입니다.

```text
<workspace>/
├── README.md
└── src/
    └── init.txt
```

대회 규정집, 미션 상세 문서 및 트랙 도면은 [assets](assets/) 폴더에 있습니다.

지도 순서에 따른 미션 수행 내용, 판정 기준 및 표지판은 [미션별 정리](misson.md)에서 확인할 수 있습니다.

# 2026 AUTO RACE 스케일카 자율주행 경진대회

## Team

<table>
  <tr>
    <td align="center" width="150">
      <a href="https://github.com/Minjea31">
        <img src="https://github.com/Minjea31.png?size=200" width="100" height="100" alt="김민재 GitHub 프로필 사진" />
      </a>
      <br />
      <a href="https://github.com/Minjea31">김민재</a>
    </td>
    <td align="center" width="150">
      <a href="https://github.com/ysjee0229">
        <img src="https://github.com/ysjee0229.png?size=200" width="100" height="100" alt="지연수 GitHub 프로필 사진" />
      </a>
      <br />
      <a href="https://github.com/ysjee0229">지연수</a>
    </td>
    <td align="center" width="150">
      <a href="https://github.com/mini0909-web">
        <img src="https://github.com/mini0909-web.png?size=200" width="100" height="100" alt="서민영 GitHub 프로필 사진" />
      </a>
      <br />
      <a href="https://github.com/mini0909-web">서민영</a>
    </td>
    <td align="center" width="150">
      <a href="https://github.com/ms-c-h">
        <img src="https://github.com/ms-c-h.png?size=200" width="100" height="100" alt="최민성 GitHub 프로필 사진" />
      </a>
      <br />
      <a href="https://github.com/ms-c-h">최민성</a>
    </td>
  </tr>
</table>

## Poster

![2026 AUTO RACE 스케일카 자율주행 경진대회 포스터](info/poster.jpg)

## Workspaces

8개 미션 워크스페이스와 4개 공통 워크스페이스로 구성합니다.

| 구분 | 워크스페이스 | 담당 기능 |
| --- | --- | --- |
| 미션 | [signal_ws](signal_ws/) | 신호등 정지·발진. 방해 광고판이 있는 환경에서 신호 인식 |
| 미션 | [LR_sign_ws](LR_sign_ws/) | 좌·우 표지판 인식 및 갈림길 A/B 경로 선택 |
| 미션 | [crosswalk_ws](crosswalk_ws/) | 보행자 표지판 인식, 횡단보도 앞 정지 및 3초 이상 대기 후 출발 |
| 미션 | [parking_ws](parking_ws/) | 지정 주차 구역에 T자 주차 |
| 미션 | [slow_ws](slow_ws/) | 방지턱 앞 서행 표지판 인식, 감속 및 방지턱 통과 |
| 미션 | [zigzag_ws](zigzag_ws/) | 지그재그 서행 유도선 구간 통과 |
| 미션 | [cone_ws](cone_ws/) | 라바콘 회피 주행 |
| 미션 | [objavoid_ws](objavoid_ws/) | 정적 장애물 인식 및 차선 변경을 통한 회피 |
| 공통 | [control_ws](control_ws/) | 차량 조향 및 속도 제어 |
| 공통 | [lane_ws](lane_ws/) | 차선 인식 및 차로 유지 주행 |
| 공통 | [mux_ws](mux_ws/) | 주행·미션별 제어 명령 선택 및 전환 |
| 공통 | [sensor_ws](sensor_ws/) | 카메라·라이다 등 센서 데이터 수집 및 처리 |

모든 워크스페이스는 아래와 같은 폴더 구조를 사용합니다. `src/init.txt`는 빈 `src` 폴더를 Git에 유지하기 위한 파일입니다.

```text
<workspace>/
├── README.md
└── src/
    └── init.txt
```

대회 규정집, 미션 상세 문서 및 트랙 도면은 [info](info/) 폴더에 있습니다.

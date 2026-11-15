// 자동 시작 판정: 흰 정지선(가로로 얇고 넓은 띠) + 그 너머 흰 점선(차로 경계 위치의 얇은 조각, 3차선에만 있음)
// + 전방 장애물. 입력은 모두 차 기준(base_link: x 전방, y 왼쪽).
#pragma once
#include <vector>

#include "autorace_objavoid_v3_cpp/lane.hpp"
#include "autorace_objavoid_v3_cpp/params.hpp"
#include "autorace_objavoid_v3_cpp/perception.hpp"

namespace objavoid {

struct GateResult
{
  bool line = false;
  double line_x = 0, line_span = 0;
  int line_pts = 0;
  int dashes_l = 0, dashes_r = 0;
  int obstacles = 0;
  int line_after = 0;
  int stripes = 0;
  bool pair = false;  // 좌우 점선 간격 = 차로 폭  // 같은 거리에 나란한 얇은 조각 최대 수(횡단보도 판별)
  bool ok = false;
};

// white: 흰색 바닥 점, boxes: LiDAR 검출 박스(차 기준, 실측 폭 start_obs_min_width 미만은 호출 전에 제외)
GateResult eval_start_gate(const std::vector<XY> & white, const std::vector<Box> & boxes, const Params & p);

// cmd_emulate_calibration: 센서 드라이버 보정이 꺼져 있어도 보정 켰을 때와 같은 서보 값이 되도록 중심 조향각 변환
double emulate_calibration(double delta, const Params & p);

}  // namespace objavoid

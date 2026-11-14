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
  int clutter = 0;
  bool ok = false;
};

// white: 흰색 바닥 점, boxes: LiDAR 검출 박스(차 기준)
GateResult eval_start_gate(const std::vector<XY> & white, const std::vector<Box> & boxes, const Params & p);

}  // namespace objavoid

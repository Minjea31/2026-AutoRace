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

struct EntranceResult {
  bool ok=false, left=false, right=false;
  double x=0, center=0, yaw=0;
  int support=0;
};
// Narrow (~0.50m) -> diagonal widening -> wide (~1.54m) yellow boundary.
EntranceResult eval_entrance_gate(const std::vector<XY> & yellow, bool dash_pair, const Params & p);

constexpr double kEntranceFreshness = 0.30;

// A fixed observation window also requires current confirmation. A lost/stale
// sighting cannot lend its remaining deadline to unrelated later obstacles.
class EntranceWindow {
public:
  double deadline=-1e9, last_time=-1e9, last_confirmed=-1e9;
  bool seen=false;
  bool armed(double t) const {return t>=last_time && t<=deadline && t-last_confirmed<=kEntranceFreshness;}
  void reset() {deadline=last_time=last_confirmed=-1e9;seen=false;}
  bool update(double t,bool detected,int obstacles,double seconds=5.0,int required=2,bool in_front=true) {
    if(t<last_time) reset();
    last_time=t;
    if(!in_front) {reset();last_time=t;return false;}
    if(!detected && t-last_confirmed>kEntranceFreshness) {reset();last_time=t;return false;}
    if(detected && !seen && !armed(t)) deadline=t+seconds;
    if(detected) last_confirmed=t;
    seen=detected;
    return detected && armed(t) && obstacles>=required;
  }
};

inline bool entrance_ahead(double x,double y,double vehicle_x,double vehicle_y,double yaw) {
  return (x-vehicle_x)*std::cos(yaw)+(y-vehicle_y)*std::sin(yaw)>0.10;
}

// Detect a new entrance episode, not repeated frames of the same entrance.
class EntranceReentry {
public:
  double last_reset=-1e9,absent_since=-1,last_time=-1e9;
  bool latched=false;int hits=0;EntranceResult previous;
  void note_start(double t) {last_reset=t;last_time=t;latched=true;hits=0;absent_since=-1;previous={};}
  bool update(double t,const EntranceResult & result,bool white_line,int confirm=3) {
    if(t<last_time) note_start(t);
    last_time=t;
    if(!result.ok) {
      hits=0;previous={};if(absent_since<0) absent_since=t;
      if(t-absent_since>=1.0) latched=false;
      return false;
    }
    absent_since=-1;
    const bool consistent=previous.ok && std::abs(previous.x-result.x)<0.2 &&
      std::abs(previous.center-result.center)<0.15 && std::abs(previous.yaw-result.yaw)<0.15;
    hits=consistent?hits+1:1;previous=result;
    if(!white_line) {hits=0;previous={};return false;}
    if(latched || t-last_reset<5.0 || hits<confirm) return false;
    note_start(t);return true;
  }
};

// cmd_emulate_calibration: 센서 드라이버 보정이 꺼져 있어도 보정 켰을 때와 같은 서보 값이 되도록 중심 조향각 변환
double emulate_calibration(double delta, const Params & p);

}  // namespace objavoid

#include "autorace_objavoid_v3_cpp/start_gate.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

namespace objavoid {

namespace {

struct Blob {double x0 = kInf, x1 = -kInf, y0 = kInf, y1 = -kInf; int n = 0;};

// 3 cm 격자 8방향 연결 덩어리
std::vector<Blob> blobs(const std::vector<XY> & pts, double cell = 0.03)
{
  std::map<std::pair<int, int>, std::vector<int>> grid;
  for (size_t k = 0; k < pts.size(); ++k) {
    grid[{static_cast<int>(std::floor(pts[k].x / cell)), static_cast<int>(std::floor(pts[k].y / cell))}].push_back(
      static_cast<int>(k));
  }
  std::map<std::pair<int, int>, bool> done;
  std::vector<Blob> out;
  for (auto & [key, idx] : grid) {
    if (done[key]) {continue;}
    Blob b;
    std::vector<std::pair<int, int>> stack{key};
    done[key] = true;
    while (!stack.empty()) {
      const auto c = stack.back();
      stack.pop_back();
      for (int k : grid[c]) {
        b.x0 = std::min(b.x0, pts[k].x);
        b.x1 = std::max(b.x1, pts[k].x);
        b.y0 = std::min(b.y0, pts[k].y);
        b.y1 = std::max(b.y1, pts[k].y);
        ++b.n;
      }
      for (int di = -1; di <= 1; ++di) {
        for (int dj = -1; dj <= 1; ++dj) {
          const std::pair<int, int> nb{c.first + di, c.second + dj};
          if (grid.count(nb) && !done[nb]) {
            done[nb] = true;
            stack.push_back(nb);
          }
        }
      }
    }
    out.push_back(b);
  }
  return out;
}

}  // namespace

GateResult eval_start_gate(const std::vector<XY> & white, const std::vector<Box> & boxes, const Params & p)
{
  GateResult g;
  // 1) 정지선: 전방 x 범위에서 앞뒤 두께 start_line_thick 띠 안에 좌우로 넓게 이어진 점(가장 가까운 것).
  //    띠 바로 뒤는 비어 있어야 함(라바콘·벽 같은 두꺼운 흰 영역 거부).
  std::vector<XY> near;
  for (const XY & q : white) {
    if (q.x >= p.start_line_x_min && q.x <= p.start_line_x_max + p.start_line_thick) {near.push_back(q);}
  }
  std::sort(near.begin(), near.end(), [](const XY & a, const XY & b) {return a.x < b.x;});
  for (size_t i = 0; i < near.size() && !g.line && near[i].x <= p.start_line_x_max; ++i) {
    const double xa = near[i].x, xb = xa + p.start_line_thick;
    double ymin = kInf, ymax = -kInf, sx = 0;
    int n = 0;
    size_t j = i;
    for (; j < near.size() && near[j].x <= xb; ++j) {
      ymin = std::min(ymin, near[j].y);
      ymax = std::max(ymax, near[j].y);
      sx += near[j].x;
      ++n;
    }
    if (n < p.start_line_min_pts || ymax - ymin < p.start_line_span) {continue;}
    const int nb = static_cast<int>((ymax - ymin) / 0.05) + 1;
    std::vector<char> occ(nb, 0);
    for (size_t k = i; k < j; ++k) {occ[std::min(nb - 1, static_cast<int>((near[k].y - ymin) / 0.05))] = 1;}
    const double fill = static_cast<double>(std::count(occ.begin(), occ.end(), 1)) / nb;
    if (fill < p.start_line_fill) {continue;}
    int after = 0;
    for (const XY & q : white) {
      after += q.x > xb && q.x <= xb + p.start_line_thick && q.y >= ymin && q.y <= ymax;
    }
    if (after > p.start_line_after * n) {continue;}
    g.line = true;
    g.line_x = sx / n;
    g.line_span = ymax - ymin;
    g.line_pts = n;
    g.line_after = after;
  }
  // 2) 정지선 너머 흰 덩어리: 앞뒤로 길고 좌우로 얇은 조각 = 점선(차 위치와 무관하게 |y| <= start_dash_max_y).
  //    같은 거리에 나란한 얇은 조각이 start_stripe_max 보다 많으면 횡단보도.
  // start_need_line false: 정지선 없이 차 앞 start_line_x_min 부터 start_dash_dist 안에서 점선을 찾음
  if (g.line || !p.start_need_line) {
    std::vector<XY> beyond;
    const double base = g.line ? g.line_x + p.start_line_thick : p.start_line_x_min;
    const double x0 = base, x1 = base + p.start_dash_dist;
    for (const XY & q : white) {
      if (q.x > x0 && q.x < x1 && std::abs(q.y) <= p.start_dash_max_y + 0.1) {beyond.push_back(q);}
    }
    std::vector<Blob> thin;
    std::vector<double> lefts, rights;
    for (const Blob & b : blobs(beyond)) {
      const double len = b.x1 - b.x0, wid = b.y1 - b.y0, cy = 0.5 * (b.y0 + b.y1);
      if (b.n < 3 || wid > p.start_dash_max_width || len < p.start_dash_min_len) {continue;}
      thin.push_back(b);  // 얇은 세로 조각(점선 또는 횡단보도 줄무늬)
      if (len <= p.start_dash_max_len && std::abs(cy) <= p.start_dash_max_y) {
        (cy > 0 ? g.dashes_l : g.dashes_r) += 1;
        (cy > 0 ? lefts : rights).push_back(cy);
      }
    }
    for (double l : lefts) {
      for (double r : rights) {g.pair = g.pair || std::abs(l - r - p.start_lane_width) <= p.start_lane_width_tol;}
    }
    for (const Blob & a : thin) {
      int over = 0;
      for (const Blob & b : thin) {over += b.x0 <= a.x1 && a.x0 <= b.x1;}
      g.stripes = std::max(g.stripes, over);
    }
  }
  // 3) 전방 장애물
  for (const Box & b : boxes) {
    // 정지선을 찾았으면 그 너머(도로 위) 장애물만
    const double from = g.line ? g.line_x + p.start_line_thick : 0.3;
    if (b.s1 > from && b.s0 < p.start_obs_x_max && b.d1 > -p.start_obs_y && b.d0 < p.start_obs_y) {++g.obstacles;}
  }
  g.ok = (g.line || !p.start_need_line) && g.dashes_l + g.dashes_r >= p.start_dash_min &&
    (!p.start_dash_both || (g.dashes_l > 0 && g.dashes_r > 0 && g.pair)) && g.stripes <= p.start_stripe_max &&
    g.obstacles >= p.start_obs_min;
  return g;
}

double emulate_calibration(double delta, const Params & p)
{
  if (!p.cmd_emulate_calibration || std::abs(delta) < 1e-6) {return delta;}
  const double L = p.cmd_wheelbase, T = p.cmd_track, a = std::abs(delta);
  // 드라이버와 같은 변환: 중심각 -> 안쪽 바퀴각 -> (보정 켜짐) 서보 값
  const double inner = std::atan(2 * L * std::sin(a) / (2 * L * std::cos(a) - T * std::sin(a)));
  const double wire = std::min(p.cmd_cal_limit, (inner + p.cmd_cal_deadband) / p.cmd_cal_gain);
  // 보정 꺼진 드라이버가 같은 서보 값을 내도록: 안쪽 바퀴각 = 서보 * legacy_scale -> 중심각
  const double inner2 = std::min(wire * p.cmd_legacy_scale, 1.5);
  const double r = L / std::tan(inner2) + T / 2;
  return std::copysign(std::atan(L / r), delta);
}

}  // namespace objavoid

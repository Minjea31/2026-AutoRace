#include "autorace_objavoid_v3_cpp/start_gate.hpp"

#include <algorithm>
#include <cmath>

namespace objavoid {

GateResult eval_start_gate(const std::vector<XY> & white, const std::vector<Box> & boxes, const Params & p)
{
  GateResult g;
  // 1) 정지선: 전방 x 범위에서, 앞뒤 두께 start_line_thick 띠 안에 좌우로 넓게 퍼진 점들(가장 가까운 것)
  std::vector<XY> near;
  for (const XY & q : white) {
    if (q.x >= p.start_line_x_min && q.x <= p.start_line_x_max) {near.push_back(q);}
  }
  std::sort(near.begin(), near.end(), [](const XY & a, const XY & b) {return a.x < b.x;});
  for (size_t i = 0; i < near.size() && !g.line; ++i) {
    // near[i]에서 시작하는 두께 창
    double ymin = kInf, ymax = -kInf, sx = 0;
    int n = 0;
    for (size_t j = i; j < near.size() && near[j].x <= near[i].x + p.start_line_thick; ++j) {
      ymin = std::min(ymin, near[j].y);
      ymax = std::max(ymax, near[j].y);
      sx += near[j].x;
      ++n;
    }
    if (n < p.start_line_min_pts || ymax - ymin < p.start_line_span) {continue;}
    // 좌우로 이어져 있어야 함: 5 cm 칸 중 점이 있는 칸 비율
    const int nb = static_cast<int>((ymax - ymin) / 0.05) + 1;
    std::vector<char> occ(nb, 0);
    for (size_t j = i; j < near.size() && near[j].x <= near[i].x + p.start_line_thick; ++j) {
      occ[std::min(nb - 1, static_cast<int>((near[j].y - ymin) / 0.05))] = 1;
    }
    const double fill = static_cast<double>(std::count(occ.begin(), occ.end(), 1)) / nb;
    if (fill >= p.start_line_fill) {
      g.line = true;
      g.line_x = sx / n;
      g.line_span = ymax - ymin;
      g.line_pts = n;
    }
  }
  // 2) 점선: 정지선 너머(정지선 띠 바깥) start_dash_dist 안, |y| ≈ lane_dash_offset 의 얇고 짧은 조각
  if (g.line) {
    for (int side : {1, -1}) {
      std::vector<XY> c;
      for (const XY & q : white) {
        if (q.x > g.line_x + p.start_line_thick && q.x < g.line_x + p.start_dash_dist &&
          std::abs(side * q.y - p.lane_dash_offset) <= p.start_dash_tol)
        {
          c.push_back(q);
        }
      }
      std::sort(c.begin(), c.end(), [](const XY & a, const XY & b) {return a.x < b.x;});
      size_t a = 0;
      for (size_t k = 1; k <= c.size(); ++k) {
        if (k == c.size() || c[k].x - c[k - 1].x > 0.05) {  // 점선 사이 틈에서 조각을 나눔
          if (k > a) {
            double ymin = kInf, ymax = -kInf;
            for (size_t q = a; q < k; ++q) {ymin = std::min(ymin, c[q].y); ymax = std::max(ymax, c[q].y);}
            const double len = c[k - 1].x - c[a].x;
            if (k - a >= 3 && len >= p.start_dash_min_len && len <= p.start_dash_max_len &&
              ymax - ymin <= p.start_dash_max_width)
            {
              (side > 0 ? g.dashes_l : g.dashes_r) += 1;
            }
          }
          a = k;
        }
      }
    }
  }
  // 2b) 정지선 너머 점선 자리 밖 흰 점(횡단보도 줄무늬·넓은 흰 영역)이 많으면 거부
  if (g.line) {
    for (const XY & q : white) {
      if (q.x > g.line_x + p.start_line_thick && q.x < g.line_x + p.start_dash_dist &&
        std::abs(q.y) < p.start_clutter_y && std::abs(std::abs(q.y) - p.lane_dash_offset) > p.start_dash_tol)
      {
        ++g.clutter;
      }
    }
  }
  // 3) 전방 장애물
  for (const Box & b : boxes) {
    if (b.s1 > 0.3 && b.s0 < p.start_obs_x_max && b.d1 > -p.start_obs_y && b.d0 < p.start_obs_y) {++g.obstacles;}
  }
  g.ok = g.line && g.dashes_l + g.dashes_r >= p.start_dash_min && g.obstacles >= p.start_obs_min &&
    g.clutter <= p.start_clutter_max;
  return g;
}

}  // namespace objavoid

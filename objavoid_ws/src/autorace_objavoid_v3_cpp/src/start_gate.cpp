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

namespace {
struct EntranceLine {
  double slope=0, error=0;
  bool valid=false;
};

EntranceLine entrance_line(const std::vector<XY> & points) {
  if(points.size()<4) return {};
  double x=0,y=0,xx=0,xy=0,lo=kInf,hi=-kInf;
  for(const auto & q:points) {x+=q.x;y+=q.y;xx+=q.x*q.x;xy+=q.x*q.y;lo=std::min(lo,q.x);hi=std::max(hi,q.x);}
  const double n=points.size(),den=xx-x*x/n;
  if(hi-lo<0.15 || den<1e-8) return {};
  const double a=(xy-x*y/n)/den,b=(y-a*x)/n;
  double error=0;
  for(const auto & q:points) error+=std::pow(q.y-a*q.x-b,2);
  return {a,error,true};
}

// Both legs must belong to one connected boundary and actually change direction.
// Pooling unrelated paint fragments or fitting a straight line to two short
// portions of a widening template must never count as an entrance corner.
bool entrance_corner(const std::vector<XY> & points,double start,double length,double center,
  double narrow,double growth,int side) {
  const double sign=side?1:-1,corner=start+length;
  std::vector<XY> matched;
  for(const auto & q:points) {
    if(q.x<start+0.03 || q.x>corner+0.45 || (q.y>=center?1:0)!=side) continue;
    const double prediction=center+sign*(narrow+growth*std::clamp((q.x-start)/length,0.0,1.0));
    if(std::abs(q.y-prediction)<=0.035) matched.push_back(q);
  }
  std::vector<char> visited(matched.size(),0);
  for(size_t seed=0;seed<matched.size();++seed) {
    if(visited[seed]) continue;
    std::vector<size_t> queue{seed};visited[seed]=1;
    std::vector<XY> diagonal,straight;
    for(size_t j=0;j<queue.size();++j) {
      const auto & q=matched[queue[j]];
      if(q.x<corner-0.03) diagonal.push_back(q);
      else if(q.x>=corner) straight.push_back(q);
      for(size_t k=0;k<matched.size();++k)
        if(!visited[k] && std::hypot(q.x-matched[k].x,q.y-matched[k].y)<=0.10) {
          visited[k]=1;queue.push_back(k);
        }
    }
    const auto a=entrance_line(diagonal),b=entrance_line(straight);
    if(!a.valid || !b.valid || sign*a.slope<0.45 || std::abs(b.slope)>0.25 ||
      sign*(a.slope-b.slope)<0.45) continue;
    std::vector<XY> combined=diagonal;combined.insert(combined.end(),straight.begin(),straight.end());
    const auto single=entrance_line(combined);
    const double error=a.error+b.error;
    if(single.valid && error<=0.025*0.025*combined.size() && error<0.49*single.error) return true;
  }
  return false;
}
}  // namespace

EntranceResult eval_entrance_gate(const std::vector<XY> & yellow,bool dash_pair,const Params & p) {
  (void)dash_pair;  // A single yellow boundary corner now arms the separate obstacle window.
  EntranceResult best;
  std::vector<XY> pts;pts.reserve(128);
  const size_t stride=std::max(size_t(1),yellow.size()/128);
  for(size_t i=0;i<yellow.size() && pts.size()<128;i+=stride)
    if(yellow[i].x>0.1 && yellow[i].x<2.0 && std::abs(yellow[i].y)<1.3) pts.push_back(yellow[i]);
  if(pts.size()<12) return best;
  const double narrow=p.start_lane_width/2, growth=(p.fork_road_width-p.start_lane_width)/2;
  for(double yaw=-0.3;yaw<=0.301;yaw+=0.1) {
    const double c=std::cos(yaw),sn=std::sin(yaw);
    std::vector<XY> rotated;rotated.reserve(pts.size());
    for(const auto & q:pts) rotated.push_back({c*q.x+sn*q.y,-sn*q.x+c*q.y});
    for(double start=0.1;start<=1.201;start+=0.1) for(double length:{0.4,0.52,0.64})
      for(double center=-0.3;center<=0.301;center+=0.1) {
        int diagonal[2]={},straight[2]={};double dlo[2]={kInf,kInf},dhi[2]={-kInf,-kInf};
        double slo[2]={kInf,kInf},shi[2]={-kInf,-kInf};int hits=0;
        for(const auto & q:rotated) {
          if(q.x<start-0.25 || q.x>start+length+0.45) continue;
          const int side=q.y>=center?1:0;const double sign=side?1:-1;
          const double prediction=center+sign*(narrow+growth*std::clamp((q.x-start)/length,0.0,1.0));
          if(std::abs(q.y-prediction)>0.035) continue;
          ++hits;
          if(q.x>start+0.03 && q.x<start+length-0.03) {
            ++diagonal[side];dlo[side]=std::min(dlo[side],q.x);dhi[side]=std::max(dhi[side],q.x);
          } else if(q.x>=start+length) {
            ++straight[side];slo[side]=std::min(slo[side],q.x);shi[side]=std::max(shi[side],q.x);
          }
        }
        bool support[2];for(int k=0;k<2;++k) support[k]=diagonal[k]>=4 && straight[k]>=4 &&
          dhi[k]-dlo[k]>0.12 && shi[k]-slo[k]>0.12;
        for(int k=0;k<2;++k) if(support[k])
          support[k]=entrance_corner(rotated,start,length,center,narrow,growth,k);
        const bool ok=support[0] || support[1];
        if(ok && hits>best.support) best={true,support[1],support[0],start,center,yaw,hits};
      }
  }
  return best;
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

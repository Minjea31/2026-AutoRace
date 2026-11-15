#include "autorace_objavoid_v3_cpp/perception.hpp"

#include <cmath>
#include <set>

namespace objavoid {

void correct_extent(double & lo, double & hi, double sensor, double min_size, bool occ_lo, bool occ_hi)
{
  if (hi - lo >= min_size) {return;}
  if (occ_lo && !occ_hi) {lo = hi - min_size; return;}
  if (occ_hi && !occ_lo) {hi = lo + min_size; return;}
  if (sensor <= lo) {hi = lo + min_size; return;}
  if (sensor >= hi) {lo = hi - min_size; return;}
  const double c = 0.5 * (lo + hi);
  lo = c - min_size / 2;
  hi = c + min_size / 2;
}

static bool occluded(const std::vector<double> & r, const std::vector<char> & ok, long j, double rr)
{
  if (j < 0 || j >= static_cast<long>(r.size())) {return false;}
  return ok[j] && r[j] < rr - 0.08;
}

void detect(const std::vector<float> & ranges, double angle_min, double angle_inc, const LidarPose & lp,
  const RoadFrame & frame, const Boundary & boundary, double s_vehicle, const Params & p,
  std::vector<Box> & boxes, std::vector<Pt2> & pts)
{
  boxes.clear();
  pts.clear();
  const size_t n = ranges.size();
  std::vector<double> r(n);
  std::vector<char> ok(n);
  std::vector<long> sel;
  for (size_t i = 0; i < n; ++i) {
    r[i] = ranges[i];
    ok[i] = std::isfinite(r[i]) && r[i] > p.scan_range_min;
    if (!ok[i] || !(r[i] < p.scan_range_max)) {continue;}
    const double th = angle_min + i * angle_inc + lp.yaw;
    double s, d;
    frame.to_road(lp.x + r[i] * std::cos(th), lp.y + r[i] * std::sin(th), s, d);
    double right, left;
    boundary.limits(s, right, left);
    if (s > s_vehicle - 0.6 && s < p.section_end + p.roi_beyond_end && d > right - p.roi_margin &&
      d < left + p.roi_margin)
    {
      sel.push_back(static_cast<long>(i));
      pts.push_back({s, d});
    }
  }
  if (sel.empty()) {return;}
  double ls, ld;
  frame.to_road(lp.x, lp.y, ls, ld);
  // 각도 순서 인접 점 간격으로 분할(ROI 밖 빔이 여러 개 끼면 분리)
  std::vector<size_t> starts{0};
  for (size_t k = 1; k < sel.size(); ++k) {
    const bool jump = std::hypot(pts[k].s - pts[k - 1].s, pts[k].d - pts[k - 1].d) > p.cluster_jump;
    if (jump || sel[k] - sel[k - 1] > 3) {starts.push_back(k);}
  }
  starts.push_back(sel.size());
  for (size_t g = 0; g + 1 < starts.size(); ++g) {
    const size_t a = starts[g], b = starts[g + 1];
    if (static_cast<int>(b - a) < p.cluster_min_points) {continue;}
    double s0 = kInf, s1 = -kInf, d0 = kInf, d1 = -kInf;
    for (size_t k = a; k < b; ++k) {
      s0 = std::min(s0, pts[k].s); s1 = std::max(s1, pts[k].s);
      d0 = std::min(d0, pts[k].d); d1 = std::max(d1, pts[k].d);
    }
    // 클러스터 양 끝 바로 바깥 빔이 더 가까우면 그쪽이 가려진 것
    const long i0 = sel[a], i1 = sel[b - 1];
    const bool occ0 = occluded(r, ok, i0 - 1, r[i0]);
    const bool occ1 = occluded(r, ok, i1 + 1, r[i1]);
    const bool first_low = pts[a].d <= pts[b - 1].d;
    const bool occ_lo = first_low ? occ0 : occ1;
    const bool occ_hi = first_low ? occ1 : occ0;
    if (p.obstacle_fixed_size) {
      // Keep the observed near face when viewing from the front/rear.
      if (ls <= s0) {s1 = s0 + p.obstacle_min_length;}
      else if (ls >= s1) {s0 = s1 - p.obstacle_min_length;}
      else {const double c = (s0 + s1) / 2; s0 = c - p.obstacle_min_length / 2; s1 = c + p.obstacle_min_length / 2;}
      correct_extent(d0, d1, ld, p.obstacle_min_width, occ_lo, occ_hi);
      const double center = (d0 + d1) / 2;
      d0 = center - p.obstacle_min_width / 2;
      d1 = center + p.obstacle_min_width / 2;
    } else if (ls < s0) {
      // 앞에서 볼 때만 최소 크기 보정(앞면이 실제로 보일 때만 뒤로 확장)
      int nf = 0;
      double fmin = kInf, fmax = -kInf;
      for (size_t k = a; k < b; ++k) {
        if (pts[k].s < s0 + 0.03) {
          ++nf; fmin = std::min(fmin, pts[k].d); fmax = std::max(fmax, pts[k].d);
        }
      }
      if (nf >= 2 && fmax - fmin >= 0.08) {s1 = std::max(s1, s0 + p.obstacle_min_length);}
      correct_extent(d0, d1, ld, p.obstacle_min_width, occ_lo, occ_hi);
    }
    boxes.push_back({s0, s1, d0, d1});
  }
}

// ------------------------------------------------------------ Tracker
void Tracker::update(const std::vector<Box> & boxes, double t, double s_rear,
  const std::vector<float> * ranges, const Beam * beam)
{
  const Params & p = p_;
  std::set<int> used;
  for (const Box & b : boxes) {
    long best = -1;
    double best_d = kInf;
    for (size_t k = 0; k < tracks.size(); ++k) {
      const Track & tr = tracks[k];
      const double dc = std::hypot(tr.box.cs() - b.cs(), tr.box.cd() - b.cd());
      if ((tr.box.gap_to(b, 0.0) < 0.06 || dc < p.assoc_distance) && dc < best_d) {
        best = static_cast<long>(k);
        best_d = dc;
      }
    }
    if (best < 0) {
      if (b.s1 + p.pass_margin < s_rear) {continue;}  // 이미 지나간 차의 뒷면 재검출
      Track nt;
      nt.id = next_id++;
      nt.box = b;
      nt.hits = 1;
      nt.first_seen = nt.last_seen = t;
      nt.hist = {b};
      tracks.push_back(nt);
      continue;
    }
    Track & tr = tracks[best];
    used.insert(tr.id);
    tr.hist.push_back(b);
    if (static_cast<int>(tr.hist.size()) > p.track_window) {
      tr.hist.erase(tr.hist.begin(), tr.hist.end() - p.track_window);
    }
    // 추정: 최근 track_window회 관측의 합집합
    Box est = tr.hist[0];
    for (size_t k = 1; k < tr.hist.size(); ++k) {est = est.unite(tr.hist[k]);}
    if (est.s1 - est.s0 > p.obstacle_max_length || est.d1 - est.d0 > p.obstacle_max_width) {est = b;}
    tr.box = p.obstacle_fixed_size ? b : est;
    tr.hits += 1;
    tr.last_seen = t;
    tr.ghost_time = 0.0;
    if (!tr.confirmed && tr.hits >= p.confirm_hits) {
      tr.confirmed = true;
      ++version;
    }
  }
  std::vector<Track> keep;
  for (Track & tr : tracks) {
    if (s_rear > tr.box.s1 + p.pass_margin) {
      if (tr.confirmed && tr.box.s0 < p.section_end) {
        ++passed;
        passed_ids.push_back(tr.id);
        ++version;
      }
      continue;
    }
    if (!tr.confirmed && t - tr.last_seen > p.tentative_timeout) {continue;}
    if (tr.confirmed && beam != nullptr && ranges != nullptr && !used.count(tr.id) &&
      see_through(tr, *ranges, *beam))
    {
      tr.ghost_time += beam->dt;
      if (tr.ghost_time > p.ghost_time) {
        ++version;
        continue;
      }
    }
    keep.push_back(tr);
  }
  tracks.swap(keep);
}

bool Tracker::see_through(const Track & tr, const std::vector<float> & ranges, const Beam & beam) const
{
  double x, y;
  beam.frame.to_odom(tr.box.cs(), tr.box.cd(), x, y);
  const double dist = std::hypot(x - beam.lidar.x, y - beam.lidar.y);
  if (dist > 2.5 || dist < 0.2) {return false;}
  const double a = wrap(std::atan2(y - beam.lidar.y, x - beam.lidar.x) - beam.lidar.yaw);
  const long i = std::lround((a - beam.angle_min) / beam.angle_inc);
  const long n = static_cast<long>(ranges.size());
  if (i < 3 || i >= n - 3) {return false;}
  int far = 0;
  for (long k = i - 2; k <= i + 2; ++k) {
    const double w = ranges[k];
    if (!std::isfinite(w) || w <= p_.scan_range_min || w > dist + 0.20) {++far;}
  }
  return far >= 4;
}

std::vector<Box> Tracker::confirmed_boxes() const
{
  std::vector<Box> out;
  for (const Track & tr : tracks) {
    if (tr.confirmed) {out.push_back(tr.box);}
  }
  return out;
}

int Tracker::confirmed_count() const
{
  int n = 0;
  for (const Track & tr : tracks) {n += tr.confirmed;}
  return n;
}

int Tracker::section_count() const
{
  int n = 0;
  for (const Track & tr : tracks) {n += (tr.confirmed && tr.box.s0 < p_.section_end);}
  return n;
}

// ------------------------------------------------------------ VisibilityGrid
VisibilityGrid::VisibilityGrid(const Params & p, double smin, double smax, double d_half)
: res(p.vis_res), s_min(smin), d_min(-d_half), p_(&p)
{
  ns = static_cast<int>(std::ceil((smax - smin) / res));
  nd = static_cast<int>(std::ceil(2 * d_half / res));
  seen.assign(static_cast<size_t>(ns) * nd, 0);
  for (double t = 0.06; t < p.vis_range; t += res * 0.8) {steps_.push_back(t);}
}

void VisibilityGrid::update(const std::vector<float> & ranges, double angle_min, double angle_inc,
  const LidarPose & lp, const RoadFrame & frame)
{
  const Params & p = *p_;
  const long n = static_cast<long>(ranges.size());
  const long step = std::max(1L, std::lround((p.vis_beam_step_deg * M_PI / 180.0) / std::max(angle_inc, 1e-6)));
  for (long i = 0; i < n; i += step) {
    const double rr = ranges[i];
    if (!std::isfinite(rr) || rr <= p.scan_range_min) {continue;}  // 무반사 빔은 미관측
    const double free = std::min(rr - 0.04, p.vis_range);
    const double th = angle_min + i * angle_inc + lp.yaw;
    const double c = std::cos(th), sn = std::sin(th);
    for (double t : steps_) {
      if (t >= free) {break;}
      double s, d;
      frame.to_road(lp.x + c * t, lp.y + sn * t, s, d);
      const int a = static_cast<int>((s - s_min) / res);
      const int b = static_cast<int>((d - d_min) / res);
      // Python astype(int)는 0 쪽으로 자름: 음수 근처도 같은 칸 판정
      if ((s - s_min) / res > -1.0 && (d - d_min) / res > -1.0 && a >= 0 && a < ns && b >= 0 && b < nd) {
        seen[static_cast<size_t>(a) * nd + b] = 1;
      }
    }
  }
}

VisibilityGrid VisibilityGrid::reframed(const RoadFrame & old_f, const RoadFrame & new_f) const
{
  VisibilityGrid g = *this;
  for (int a = 0; a < ns; ++a) {
    for (int b = 0; b < nd; ++b) {
      const double S = (a + 0.5) * res + s_min, D = (b + 0.5) * res + d_min;
      double x, y, os, od;
      new_f.to_odom(S, D, x, y);
      old_f.to_road(x, y, os, od);
      g.seen[static_cast<size_t>(a) * nd + b] = is_seen(os, od);
    }
  }
  return g;
}

void VisibilityGrid::mark_free(double s0, double s1, double d0, double d1)
{
  const int a0 = static_cast<int>(std::floor((s0 - s_min) / res));
  const int a1 = static_cast<int>(std::floor((s1 - s_min) / res));
  const int b0 = static_cast<int>(std::floor((d0 - d_min) / res));
  const int b1 = static_cast<int>(std::floor((d1 - d_min) / res));
  for (int a = std::max(a0, 0); a < std::min(std::max(a1 + 1, 0), ns); ++a) {
    for (int b = std::max(b0, 0); b < std::min(std::max(b1 + 1, 0), nd); ++b) {
      seen[static_cast<size_t>(a) * nd + b] = 1;
    }
  }
}

bool VisibilityGrid::is_seen(double s, double d) const
{
  const int a = static_cast<int>(std::floor((s - s_min) / res));
  const int b = static_cast<int>(std::floor((d - d_min) / res));
  if (a < 0 || a >= ns || b < 0 || b >= nd) {return false;}
  return seen[static_cast<size_t>(a) * nd + b] != 0;
}

}  // namespace objavoid

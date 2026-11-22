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
  std::vector<Box> & boxes, std::vector<Pt2> & pts, std::vector<ClusterInfo> * diagnostics)
{
  boxes.clear();
  pts.clear();
  if (diagnostics) {diagnostics->clear();}
  const bool shape_filter = p.obstacle_fixed_size && p.obstacle_shape_filter;
  const size_t n = ranges.size();
  std::vector<double> r(n);
  std::vector<char> ok(n), in_roi;
  std::vector<long> sel;
  std::vector<Pt2> cloud;
  for (size_t i = 0; i < n; ++i) {
    r[i] = ranges[i];
    ok[i] = std::isfinite(r[i]) && r[i] > p.scan_range_min;
    if (!ok[i] || !(r[i] < p.scan_range_max)) {continue;}
    const double th = angle_min + i * angle_inc + lp.yaw;
    double s, d;
    frame.to_road(lp.x + r[i] * std::cos(th), lp.y + r[i] * std::sin(th), s, d);
    double right, left;
    boundary.limits(s, right, left);
    const bool inside = s > s_vehicle - 0.6 && s < p.section_end + p.roi_beyond_end &&
      d > right - p.roi_margin && d < left + p.roi_margin;
    if (inside) {pts.push_back({s, d});}
    // Keep neighbouring scan returns outside the planning ROI as shape context.
    // Cropping first can make a long wall look like a 20cm robot face.
    if (inside || shape_filter) {
      sel.push_back(static_cast<long>(i));
      cloud.push_back({s, d});
      in_roi.push_back(inside);
    }
  }
  if (sel.empty()) {return;}
  double ls, ld;
  frame.to_road(lp.x, lp.y, ls, ld);
  std::vector<size_t> starts{0};
  for (size_t k = 1; k < sel.size(); ++k) {
    const bool jump = std::hypot(cloud[k].s - cloud[k - 1].s, cloud[k].d - cloud[k - 1].d) > p.cluster_jump;
    if (jump || sel[k] - sel[k - 1] > 3) {starts.push_back(k);}
  }
  starts.push_back(sel.size());
  for (size_t g = 0; g + 1 < starts.size(); ++g) {
    const size_t a = starts[g], b = starts[g + 1];
    int roi_count = 0;
    Box roi{kInf, -kInf, kInf, -kInf};
    double s0 = kInf, s1 = -kInf, d0 = kInf, d1 = -kInf;
    for (size_t k = a; k < b; ++k) {
      s0 = std::min(s0, cloud[k].s); s1 = std::max(s1, cloud[k].s);
      d0 = std::min(d0, cloud[k].d); d1 = std::max(d1, cloud[k].d);
      if (in_roi[k]) {
        ++roi_count;
        roi.s0 = std::min(roi.s0, cloud[k].s); roi.s1 = std::max(roi.s1, cloud[k].s);
        roi.d0 = std::min(roi.d0, cloud[k].d); roi.d1 = std::max(roi.d1, cloud[k].d);
      }
    }
    if (roi_count < p.cluster_min_points) {continue;}
    ClusterInfo info;
    info.observed = {s0, s1, d0, d1};
    info.points = static_cast<int>(b - a);
    if (shape_filter || diagnostics) {
      double ms = 0, md = 0;
      for (size_t k = a; k < b; ++k) {ms += cloud[k].s; md += cloud[k].d;}
      const double count = static_cast<double>(b - a);
      ms /= count; md /= count;
      double xx = 0, xy = 0, yy = 0;
      for (size_t k = a; k < b; ++k) {
        const double x = cloud[k].s - ms, y = cloud[k].d - md;
        xx += x * x; xy += x * y; yy += y * y;
      }
      xx /= count; xy /= count; yy /= count;
      const double disc = std::hypot(xx - yy, 2 * xy);
      const double major = std::max(0.0, 0.5 * (xx + yy + disc));
      const double minor = std::max(0.0, 0.5 * (xx + yy - disc));
      const double angle = 0.5 * std::atan2(2 * xy, xx - yy);
      const double vx = std::cos(angle), vy = std::sin(angle);
      double lo = kInf, hi = -kInf;
      for (size_t k = a; k < b; ++k) {
        const double projection = cloud[k].s * vx + cloud[k].d * vy;
        lo = std::min(lo, projection); hi = std::max(hi, projection);
      }
      info.span = hi - lo;
      info.eigen_ratio = minor / std::max(major, 1e-12);
      info.line_rmse = std::sqrt(minor);
      info.wall = shape_filter && info.span >= p.obstacle_wall_min_length &&
        info.eigen_ratio <= p.obstacle_wall_max_ratio && info.line_rmse <= p.obstacle_wall_max_rmse;
    }
    const long i0 = sel[a], i1 = sel[b - 1];
    const bool occ0 = occluded(r, ok, i0 - 1, r[i0]);
    const bool occ1 = occluded(r, ok, i1 + 1, r[i1]);
    const bool first_low = cloud[a].d <= cloud[b - 1].d;
    const bool occ_lo = first_low ? occ0 : occ1;
    const bool occ_hi = first_low ? occ1 : occ0;
    if (p.obstacle_fixed_size) {
      const double long_size = std::max(p.obstacle_min_length, p.obstacle_min_width);
      const double short_size = std::min(p.obstacle_min_length, p.obstacle_min_width);
      auto fit_axis = [](double lo, double hi, double sensor, double size) {
          if (sensor <= lo) {return std::pair<double, double>{lo, lo + size};}
          if (sensor >= hi) {return std::pair<double, double>{hi - size, hi};}
          const double center = 0.5 * (lo + hi);
          return std::pair<double, double>{center - size / 2, center + size / 2};
        };
      Box best;
      double best_score = kInf, best_error = kInf;
      for (bool transverse : {false, true}) {
        const double length = transverse ? short_size : long_size;
        const double width = transverse ? long_size : short_size;
        const auto sb = fit_axis(s0, s1, ls, length), db = fit_axis(d0, d1, ld, width);
        const Box candidate{sb.first, sb.second, db.first, db.second};
        double score = 0;
        for (size_t k = a; k < b; ++k) {
          const auto & pt = cloud[k];
          const double ds = std::max({candidate.s0 - pt.s, 0.0, pt.s - candidate.s1});
          const double dd = std::max({candidate.d0 - pt.d, 0.0, pt.d - candidate.d1});
          double face = kInf;
          if (ls <= candidate.s0) {face = std::min(face, std::abs(pt.s - candidate.s0));}
          if (ls >= candidate.s1) {face = std::min(face, std::abs(pt.s - candidate.s1));}
          if (ld <= candidate.d0) {face = std::min(face, std::abs(pt.d - candidate.d0));}
          if (ld >= candidate.d1) {face = std::min(face, std::abs(pt.d - candidate.d1));}
          if (!std::isfinite(face)) {face = 0;}
          score += 4 * (ds * ds + dd * dd) + face * face;
        }
        score /= (b - a);
        const double error = std::sqrt(score);
        if (!occ_lo && !occ_hi) {score += 0.25 * std::pow((d1 - d0) - width, 2);}
        if (score < best_score) {best_score = score; best = candidate; best_error = error;}
      }
      info.fit_rmse = best_error;
      // A short robot face can have a poor axis-aligned fit when close/rotated.
      // Reject by fit residual only when its observed extent also exceeds the
      // robot diagonal plus a sensor/motion tolerance; straightness alone is
      // likewise insufficient to reject a short frontal robot face.
      const bool oversized = info.span > std::hypot(long_size, short_size) + p.obstacle_shape_extent_margin;
      if (shape_filter && (info.wall || (oversized && best_error > p.obstacle_fit_max_rmse))) {
        // Do not erase physical obstacles. Preserve their measured ROI footprint
        // for tracking/planning, without relabeling them as fixed-size robots.
        roi.robot_candidate = false;
        boxes.push_back(roi);
      } else {
        boxes.push_back(best);
      }
    } else {
      if (ls < s0) {
        int nf = 0;
        double fmin = kInf, fmax = -kInf;
        for (size_t k = a; k < b; ++k) {
          if (cloud[k].s < s0 + 0.03) {
            ++nf; fmin = std::min(fmin, cloud[k].d); fmax = std::max(fmax, cloud[k].d);
          }
        }
        if (nf >= 2 && fmax - fmin >= 0.08) {s1 = std::max(s1, s0 + p.obstacle_min_length);}
        correct_extent(d0, d1, ld, p.obstacle_min_width, occ_lo, occ_hi);
      }
      boxes.push_back({s0, s1, d0, d1});
    }
    if (diagnostics) {info.output = boxes.back(); diagnostics->push_back(info);}
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
      if (tr.box.robot_candidate != b.robot_candidate) {continue;}
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
      if (tr.confirmed && tr.box.robot_candidate && tr.box.s0 < p.section_end) {
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
  for (const Track & tr : tracks) {n += (tr.confirmed && tr.box.robot_candidate && tr.box.s0 < p_.section_end);}
  return n;
}

int Tracker::structure_count() const
{
  int n = 0;
  for (const Track & tr : tracks) {n += tr.confirmed && !tr.box.robot_candidate;}
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

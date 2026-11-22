#include "autorace_objavoid_v3_cpp/lane.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <random>
#include <set>
#include <stdexcept>

#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace objavoid {

// ------------------------------------------------------------ GroundProjection
static double ycfg(const YAML::Node & c, const char * k, double def)
{
  return c[k] ? c[k].as<double>() : def;
}

GroundProjection::GroundProjection(const std::string & cfg_path, double mr, double ml,
  const std::vector<double> & homography, bool full_image)
: max_range(mr), max_lateral(ml)
{
  const YAML::Node root = YAML::LoadFile(cfg_path);
  const YAML::Node c = root["lane_lite"];
  if (!c) {throw std::runtime_error("lane_lite section missing in " + cfg_path);}
  w = c["width"].as<int>();
  h = c["height"].as<int>();
  const double W = w - 1, H = h - 1;
  const double top_y = c["bev_top_y"].as<double>() * H, bot_y = c["bev_bottom_y"].as<double>() * H;
  std::vector<cv::Point2f> src{{static_cast<float>(c["bev_top_left"].as<double>() * W), static_cast<float>(top_y)},
    {static_cast<float>(c["bev_top_right"].as<double>() * W), static_cast<float>(top_y)},
    {static_cast<float>(c["bev_bottom_right"].as<double>() * W), static_cast<float>(bot_y)},
    {static_cast<float>(c["bev_bottom_left"].as<double>() * W), static_cast<float>(bot_y)}};
  std::vector<cv::Point2f> dst{{0, 0}, {static_cast<float>(W), 0}, {static_cast<float>(W), static_cast<float>(H)},
    {0, static_cast<float>(H)}};
  cv::Mat Hm = cv::getPerspectiveTransform(src, dst);
  const double mpx = c["lane_width_m"].as<double>() / c["lane_width_px"].as<double>();
  const double near_m = c["bev_near_m"].as<double>(), far_m = c["bev_far_m"].as<double>();
  const double mrow = (far_m - near_m) / H;
  const double cu = ycfg(c, "vehicle_center_u", 0.5) * W;
  cv::Matx33d M(0, -mrow, near_m + H * mrow, -mpx, 0, cu * mpx, 0, 0, 1);
  cv::Matx33d Hx;
  for (int r = 0; r < 3; ++r) {
    for (int q = 0; q < 3; ++q) {Hx(r, q) = Hm.at<double>(r, q);}
  }
  G_ = M * Hx;
  Hbev_ = Hx;
  bool any = false;
  for (double v : homography) {any = any || v != 0.0;}
  if (any && homography.size() == 9) {
    G_ = cv::Matx33d(homography.data());
  }
  Ginv_ = G_.inv();
  roi_ = {ycfg(c, "roi_left", 0.0), ycfg(c, "roi_right", 1.0), ycfg(c, "roi_top", 0.0), ycfg(c, "roi_bottom", 1.0)};
  if(full_image) roi_={0,1,0,1};
  // 영상 ROI 전체(거리 max_range까지)를 관측 영역으로
  near = kInf;
  for (double x = 0.2; x < max_range; x += 0.03) {
    for (double y = -max_lateral; y < max_lateral; y += 0.03) {
      double u, v;
      if (!to_image(x, y, u, v)) {continue;}
      if (u >= roi_[0] * W && u <= roi_[1] * W && v >= roi_[2] * H && v <= roi_[3] * H) {
        view_points.push_back({x, y});
        near = std::min(near, x);
      }
    }
  }
}

void GroundProjection::set_scale(double sx, double sy)
{
  for (XY & q : view_points) {
    q.x = q.x / sx_ * sx;
    q.y = q.y / sy_ * sy;
  }
  sx_ = sx;
  sy_ = sy;
}

void GroundProjection::to_ground(double u, double v, double & x, double & y) const
{
  const double X = G_(0, 0) * u + G_(0, 1) * v + G_(0, 2);
  const double Y = G_(1, 0) * u + G_(1, 1) * v + G_(1, 2);
  const double Z = G_(2, 0) * u + G_(2, 1) * v + G_(2, 2);
  x = X / Z;
  y = Y / Z;
}

bool GroundProjection::to_image(double x, double y, double & u, double & v) const
{
  const double U = Ginv_(0, 0) * x + Ginv_(0, 1) * y + Ginv_(0, 2);
  const double V = Ginv_(1, 0) * x + Ginv_(1, 1) * y + Ginv_(1, 2);
  const double Z = Ginv_(2, 0) * x + Ginv_(2, 1) * y + Ginv_(2, 2);
  if (Z == 0) {return false;}
  u = U / Z;
  v = V / Z;
  return std::isfinite(u) && std::isfinite(v);
}

double GroundProjection::class_ratio_bev(const cv::Mat & mask, int cls) const
{
  if (mask.empty() || w <= 1 || h <= 1) {return 0.0;}
  // mask pixel -> camera image pixel (mask may be smaller), image -> BEV, evaluated on a half-size BEV.
  const cv::Matx33d S(static_cast<double>(w) / mask.cols, 0, 0, 0, static_cast<double>(h) / mask.rows, 0, 0, 0, 1);
  const cv::Matx33d D(0.5, 0, 0, 0, 0.5, 0, 0, 0, 1);
  const cv::Matx33d T = D * Hbev_ * S;
  cv::Mat bev;
  cv::warpPerspective(mask, bev, cv::Mat(T), cv::Size(std::max(1, w / 2), std::max(1, h / 2)),
    cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
  return static_cast<double>(cv::countNonZero(bev == cls)) / static_cast<double>(bev.total());
}

std::vector<XY> GroundProjection::class_points(const cv::Mat & mask, int cls,
  int step) const
{
  std::vector<XY> out;
  const int mh = mask.rows, mw = mask.cols;
  const int sh = (mh + step - 1) / step, sw = (mw + step - 1) / step;
  const double W = w - 1, H = h - 1;
  for (int vv = 0; vv < sh; ++vv) {
    const uchar * mrow = mask.ptr<uchar>(vv * step);
    for (int uu = 0; uu < sw; ++uu) {
      if (mrow[uu * step] != cls) {continue;}
      const double u = uu * step * (static_cast<double>(w) / mw);
      const double v = vv * step * (static_cast<double>(h) / mh);
      if (!(u >= roi_[0] * W && u <= roi_[1] * W && v >= roi_[2] * H && v <= roi_[3] * H)) {continue;}
      double x, y;
      to_ground(u, v, x, y);
      x *= sx_;
      y *= sy_;
      if (std::isfinite(x) && std::isfinite(y) && x >= std::max(0.05, near - 0.02) && x <= max_range && std::abs(y) <= max_lateral) {out.push_back({x, y});}
    }
  }
  return out;
}

// ------------------------------------------------------------ YellowMap
YellowMap::YellowMap(const Params & p, double cx, double cy, double half)
: res(p.lane_cell), x0(cx - half), y0(cy - half), p_(&p)
{
  n = static_cast<int>(2 * half / res);
  views.assign(static_cast<size_t>(n) * n, 0);
  hits.assign(static_cast<size_t>(n) * n, 0);
}

XY YellowMap::to_odom(const XY & pt, double x, double y, double yaw)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  return {x + c * pt.x - s * pt.y, y + s * pt.x + c * pt.y};
}

void YellowMap::update(const std::vector<XY> & yellow_base, const std::vector<XY> & view_base, double x, double y,
  double yaw)
{
  auto cells = [&](const std::vector<XY> & pts) {
      std::set<long> out;
      for (const XY & q : pts) {
        const XY o = to_odom(q, x, y, yaw);
        const double fi = (o.x - x0) / res, fj = (o.y - y0) / res;
        const int i = static_cast<int>(fi), j = static_cast<int>(fj);  // astype(int): 0 쪽으로 자름
        if (i >= 0 && i < n && j >= 0 && j < n) {out.insert(static_cast<long>(i) * n + j);}
      }
      return out;
    };
  for (long c : cells(view_base)) {views[c] = static_cast<int16_t>(views[c] + 1);}
  if (!yellow_base.empty()) {
    for (long c : cells(yellow_base)) {hits[c] = static_cast<int16_t>(hits[c] + 1);}
  }
  ++frames;
  ++version;
}

bool YellowMap::hit_confirmed(int i, int j) const
{
  const size_t c = static_cast<size_t>(i) * n + j;
  return hits[c] >= p_->lane_confirm && hits[c] >= p_->lane_ratio * std::max<int>(views[c], 1);
}

std::vector<XY> YellowMap::confirmed() const
{
  std::vector<XY> out;
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      if (hit_confirmed(i, j)) {out.push_back({x0 + (i + 0.5) * res, y0 + (j + 0.5) * res});}
    }
  }
  return out;
}

// ------------------------------------------------------------ LaneBoundary
LaneBoundary::LaneBoundary(const PriorBoundary & pr, const Params & p)
: prior(pr), p_(&p)
{
  s_min = prior.s_min;
  s_max = prior.s_max;
  // np.arange(s_min, s_max + 1e-9, bin)과 같은 개수·위치
  const long cnt = static_cast<long>(std::ceil((s_max + 1e-9 - s_min) / bin));
  for (long k = 0; k < cnt; ++k) {s_bins.push_back(s_min + k * bin);}
  right.resize(s_bins.size());
  left.resize(s_bins.size());
  for (size_t k = 0; k < s_bins.size(); ++k) {prior.limits(s_bins[k], right[k], left[k]);}
  known_r.assign(s_bins.size(), 0);
  known_l.assign(s_bins.size(), 0);
}

void LaneBoundary::limits(double s, double & r, double & l) const
{
  r = interp(s, s_bins, right);
  l = interp(s, s_bins, left);
}

bool LaneBoundary::known(double s, int side) const
{
  const auto & k = side > 0 ? known_l : known_r;
  long i = std::lround((s - s_min) / bin);
  i = std::clamp(i, 0L, static_cast<long>(s_bins.size()) - 1);
  return k[i] != 0;
}

void LaneBoundary::rebuild(const std::vector<Pt2> & cells)
{
  const Params & p = *p_;
  for (size_t k = 0; k < s_bins.size(); ++k) {prior.limits(s_bins[k], right[k], left[k]);}
  std::fill(known_r.begin(), known_r.end(), 0);
  std::fill(known_l.begin(), known_l.end(), 0);
  cells_sd = cells;
  if (cells.empty()) {return;}
  const double half = p.lane_line_half;
  std::vector<double> lv(s_bins.size(), kInf), rv(s_bins.size(), -kInf);
  for (const Pt2 & c : cells) {
    const long k = std::lround((c.s - s_min) / bin);
    if (k < 0 || k >= static_cast<long>(s_bins.size()) || !(std::abs(c.d) < p.lane_max_d)) {continue;}
    if (c.d > p.lane_center_gap) {lv[k] = std::min(lv[k], c.d - half);}
    if (c.d < -p.lane_center_gap) {rv[k] = std::max(rv[k], c.d + half);}
  }
  for (size_t k = 0; k < s_bins.size(); ++k) {
    if (std::isfinite(lv[k])) {known_l[k] = 1; left[k] = lv[k];}
    if (std::isfinite(rv[k])) {known_r[k] = 1; right[k] = rv[k];}
  }
}

// ------------------------------------------------------------ line_segments / fit_road
std::vector<LineSeg> line_segments(const std::vector<Pt2> & points, int max_lines, double tol, int iters,
  unsigned seed, double max_gap)
{
  std::vector<Pt2> pts = points;
  std::mt19937 rng(seed);
  std::vector<LineSeg> out;
  for (int line = 0; line < max_lines; ++line) {
    if (pts.size() < 8) {break;}
    std::uniform_int_distribution<size_t> pick(0, pts.size() - 1);
    int best_cnt = -1;
    double bax = 0, bay = 0, bnx = 0, bny = 0;
    bool any = false;
    for (int it = 0; it < iters; ++it) {
      const Pt2 a = pts[pick(rng)], b = pts[pick(rng)];
      const double dx = b.s - a.s, dy = b.d - a.d, L = std::hypot(dx, dy);
      if (!(L > 0.1)) {continue;}
      any = true;
      const double nx = -dy / L, ny = dx / L;
      int cnt = 0;
      for (const Pt2 & q : pts) {cnt += std::abs((q.s - a.s) * nx + (q.d - a.d) * ny) < tol;}
      if (cnt > best_cnt) {best_cnt = cnt; bax = a.s; bay = a.d; bnx = nx; bny = ny;}
    }
    if (!any) {break;}
    std::vector<size_t> inl;
    for (size_t k = 0; k < pts.size(); ++k) {
      if (std::abs((pts[k].s - bax) * bnx + (pts[k].d - bay) * bny) < tol) {inl.push_back(k);}
    }
    // 주성분 방향(2x2 공분산 고유벡터)
    double ms = 0, md = 0;
    for (size_t k : inl) {ms += pts[k].s; md += pts[k].d;}
    ms /= inl.size();
    md /= inl.size();
    double sxx = 0, sxy = 0, syy = 0;
    for (size_t k : inl) {
      const double a = pts[k].s - ms, b = pts[k].d - md;
      sxx += a * a; sxy += a * b; syy += b * b;
    }
    const double th = 0.5 * std::atan2(2 * sxy, sxx - syy);
    double dvx = std::cos(th), dvy = std::sin(th);
    if (dvx < 0) {dvx = -dvx; dvy = -dvy;}
    std::vector<std::pair<double, size_t>> proj;
    for (size_t k : inl) {proj.push_back({(pts[k].s - ms) * dvx + (pts[k].d - md) * dvy, k});}
    std::sort(proj.begin(), proj.end());
    // 가장 긴 연속 구간(축 방향 max_gap 넘게 끊기면 분리)
    size_t bs = 0, be = 1, cs = 0;
    for (size_t q = 1; q <= proj.size(); ++q) {
      if (q == proj.size() || proj[q].first - proj[q - 1].first > max_gap) {
        if (q - cs > be - bs) {bs = cs; be = q;}
        cs = q;
      }
    }
    double rs = 0, rd = 0;
    for (size_t q = bs; q < be; ++q) {rs += pts[proj[q].second].s; rd += pts[proj[q].second].d;}
    const double cnt = static_cast<double>(be - bs);
    rs /= cnt;
    rd /= cnt;
    double pmin = kInf, pmax = -kInf, var = 0, mean_n = 0;
    std::vector<double> nn;
    for (size_t q = bs; q < be; ++q) {
      const Pt2 & r = pts[proj[q].second];
      const double pr = (r.s - rs) * dvx + (r.d - rd) * dvy;
      pmin = std::min(pmin, pr);
      pmax = std::max(pmax, pr);
      nn.push_back((r.s - rs) * (-dvy) + (r.d - rd) * dvx);
    }
    for (double v : nn) {mean_n += v;}
    mean_n /= nn.size();
    for (double v : nn) {var += (v - mean_n) * (v - mean_n);}
    const double thick = std::sqrt(var / nn.size());
    const double ang = std::atan2(dvy, dvx);
    const double nr = -rs * std::sin(ang) + rd * std::cos(ang);
    out.push_back({ang, pmax - pmin, nr, rs, thick, static_cast<int>(be - bs), 'y'});
    std::vector<char> rm(pts.size(), 0);
    for (size_t q = bs; q < be; ++q) {rm[proj[q].second] = 1;}
    std::vector<Pt2> keep;
    for (size_t k = 0; k < pts.size(); ++k) {
      if (!rm[k]) {keep.push_back(pts[k]);}
    }
    pts.swap(keep);
  }
  return out;
}

std::optional<RoadFit> fit_road(const std::vector<Pt2> & yellow_sd, const Params & p, double s_from, double s_to,
  const Boundary * prior, const std::vector<double> * widths, const std::vector<Pt2> & white_sd)
{
  auto pick = [&](const std::vector<Pt2> & pts, char kind, double gap) {
      std::vector<LineSeg> out;
      if (pts.empty()) {return out;}
      std::vector<Pt2> sel;
      for (const Pt2 & q : pts) {
        if (q.s > s_from && q.s < s_to && std::abs(q.d) < p.lane_max_d) {sel.push_back(q);}
      }
      for (LineSeg g : line_segments(sel, 5, 0.02, 150, 0, gap)) {
        if (g.len >= p.lane_seg_min_len && g.thick <= p.lane_seg_max_thick && std::abs(g.ang) <= p.lane_align_max) {
          g.kind = kind;
          out.push_back(g);
        }
      }
      return out;
    };
  std::vector<LineSeg> segs = pick(yellow_sd, 'y', 0.10);
  for (const LineSeg & g : pick(white_sd, 'w', p.lane_white_gap)) {segs.push_back(g);}
  std::stable_sort(segs.begin(), segs.end(), [](const LineSeg & a, const LineSeg & b) {return a.len > b.len;});
  std::vector<std::vector<LineSeg>> groups;
  for (const LineSeg & g : segs) {
    bool placed = false;
    for (auto & grp : groups) {
      if (std::abs(g.ang - grp[0].ang) < 0.06) {grp.push_back(g); placed = true; break;}
    }
    if (!placed) {groups.push_back({g});}
  }
  if (groups.empty()) {return std::nullopt;}
  auto total_of = [](const std::vector<LineSeg> & grp) {
      double t = 0;
      for (const LineSeg & q : grp) {t += q.len;}
      return t;
    };
  size_t gi = 0;
  for (size_t q = 1; q < groups.size(); ++q) {
    if (total_of(groups[q]) > total_of(groups[gi])) {gi = q;}
  }
  const auto & grp = groups[gi];
  const double total = total_of(grp);
  if (total < p.lane_align_min_len) {return std::nullopt;}
  double a = 0;
  for (const LineSeg & g : grp) {a += g.ang * g.len;}
  a /= total;
  struct Line {double d, sm, len; char kind;};
  std::vector<Line> lines;
  for (const LineSeg & g : grp) {
    const double dm = (g.nrm + g.sm * std::sin(g.ang)) / std::cos(g.ang);
    lines.push_back({-g.sm * std::sin(a) + dm * std::cos(a), g.sm, g.len, g.kind});
  }
  std::optional<double> center;
  const std::vector<double> wpair{2 * p.lane_dash_offset};
  const std::vector<double> & ypair = widths != nullptr ? *widths : p.lane_pair_widths;
  for (size_t i = 0; i < lines.size(); ++i) {
    for (size_t j = i + 1; j < lines.size(); ++j) {
      if (lines[i].kind != lines[j].kind) {continue;}
      const double wd = std::abs(lines[j].d - lines[i].d);
      const auto & pw = lines[i].kind == 'w' ? wpair : ypair;
      bool match = false;
      for (double ww : pw) {match = match || std::abs(wd - ww) < p.lane_width_tol;}
      if (match) {
        const double c = 0.5 * (lines[i].d + lines[j].d);
        if (std::abs(c) <= p.lane_center_max && (!center || std::abs(c) < std::abs(*center))) {center = c;}
      }
    }
  }
  if (!center && !lines.empty()) {
    const Line * bw = nullptr;
    for (const Line & q : lines) {
      if (q.kind == 'w' && (bw == nullptr || q.len > bw->len)) {bw = &q;}
    }
    if (bw != nullptr) {
      const double c = bw->d;
      const double c1 = c - p.lane_dash_offset, c2 = c + p.lane_dash_offset;
      const double dc = std::abs(c1) <= std::abs(c2) ? c1 : c2;
      if (std::abs(dc) <= p.lane_single_tol) {center = dc;}
    } else if (prior != nullptr) {
      const Line * bl = &lines[0];
      for (const Line & q : lines) {
        if (q.len > bl->len) {bl = &q;}
      }
      double r, l;
      prior->limits(bl->sm, r, l);
      const double c1 = bl->d - (l + p.lane_line_half), c2 = bl->d - (r - p.lane_line_half);
      const double dc = std::abs(c1) <= std::abs(c2) ? c1 : c2;
      if (std::abs(dc) <= p.lane_single_tol) {center = dc;}
    }
  }
  RoadFit fit{a, center, {}};
  for (const Line & q : lines) {fit.lines.push_back(q.d);}
  return fit;
}

}  // namespace objavoid

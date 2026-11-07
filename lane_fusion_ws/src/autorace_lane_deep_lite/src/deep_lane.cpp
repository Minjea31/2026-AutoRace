#include "autorace_lane_deep_lite/deep_lane.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace deep_lite {
const std::vector<Spec> &specs() {
  static const std::vector<Spec> s = {
      {"width", "입력", "처리 크기(px). 딥 mask를 이 크기로 맞추고 BEV도 같은 크기", 640, 160, 1280, 1},
      {"height", "입력", "처리 높이(px)", 480, 120, 960, 1},
      {"bev_top_y", "BEV", "BEV 사다리꼴 위쪽 높이(0-1)", .567, 0, 1, .001},
      {"bev_bottom_y", "BEV", "BEV 사다리꼴 아래쪽 높이", .962, 0, 1, .001},
      {"bev_top_left", "BEV", "BEV 위 왼쪽 x", .126, -2, 1, .001},
      {"bev_top_right", "BEV", "BEV 위 오른쪽 x", .892, 0, 3, .001},
      {"bev_bottom_left", "BEV", "BEV 아래 왼쪽 x(화면 밖 가능)", -1.001, -3, 1, .001},
      {"bev_bottom_right", "BEV", "BEV 아래 오른쪽 x(화면 밖 가능)", 1.959, 0, 4, .001},
      {"cell", "체인", "중심점을 뽑는 격자 크기(BEV px). 선 두께보다 크게", 16, 4, 64, 1},
      {"min_cell_pixels", "체인", "격자 한 칸을 차선으로 인정할 최소 픽셀 수", 6, 1, 500, 1},
      {"max_gap", "체인",
       "체인에서 이어 붙일 최대 간격(BEV px). 흰 점선 빈칸(0.15 m)보다 크게, 차로 폭보다 작게",
       110, 10, 400, 1},
      {"max_turn_deg", "체인", "체인이 한 번에 꺾일 수 있는 최대 각도(도)", 35, 5, 90, 1},
      {"min_length", "체인", "경계로 인정할 체인 최소 길이(BEV px)", 60, 10, 1000, 1},
      {"max_residual", "체인", "체인 곡선 맞춤 RMS 상한(px)", 6, 1, 60, .5},
      {"lane_width_px", "차로·추적", "BEV에서 좌우 경계(선 중심) 사이 차로 폭(px)", 277, 40, 1200, 1},
      {"width_tolerance", "차로·추적", "차로 폭 허용 오차 비율", .35, .05, .9, .01},
      {"vehicle_center_u", "차로·추적", "BEV에서 차량 중심선 가로 위치(0-1)", .5, 0, 1, .001},
      {"lookahead_row", "차로·추적", "목표점 BEV 세로 위치(0=먼 곳, 1=차량 앞)", .55, .05, .98, .01},
      {"smoothing", "차로·추적", "20 Hz 기준 새 관측 반영 비율(1=필터 없음)", .6, .05, 1, .05},
      {"track_jump_px", "차로·추적", "한 프레임 경계 이동 허용치(px). 넘으면 3프레임 연속일 때만 교체",
       35, 5, 400, 1},
      {"track_memory", "차로·추적", "관측이 끊겨도 마지막 경계를 유지하는 시간(s)", .5, 0, 3, .05},
      {"lane_width_m", "거리 보정", "실제 차로 폭(m, 선 중심 사이)", .52, .1, 3, .01},
      {"bev_near_m", "거리 보정", "BEV 맨 아래 행의 base_link 전방 거리(m), 실측", .49, 0, 5, .01},
      {"bev_far_m", "거리 보정", "BEV 맨 위 행의 전방 거리(m), 실측", 1.39, .05, 10, .01},
      {"curvature_span_m", "거리 보정",
       "곡률을 원 맞춤할 중앙 경로 길이(m, 가까운 쪽부터). 짧으면 잡음, 길면 늦음", .6, .15, 3, .05},
      {"max_curvature", "거리 보정", "발행 곡률 상한(1/m). 차량 최소 회전반경의 역수 근처", 3, .2, 20, .1},
  };
  return s;
}
Config::Config() {
  for (const auto &s : specs())
    values[s.name] = s.value;
}
void Config::validate() const {
  for (const auto &s : specs()) {
    const double v = (*this)[s.name];
    if (!std::isfinite(v) || v < s.min || v > s.max || (s.step == 1 && v != std::floor(v)))
      throw std::invalid_argument(std::string("Invalid parameter: ") + s.name);
  }
  const auto &c = *this;
  if (c["bev_top_y"] + .05 >= c["bev_bottom_y"] || c["bev_top_left"] + .05 >= c["bev_top_right"] ||
      c["bev_bottom_left"] + .05 >= c["bev_bottom_right"] || c["bev_near_m"] + .05 >= c["bev_far_m"])
    throw std::invalid_argument("Inverted BEV/distance limits");
}
Config Config::load(const std::string &path) {
  Config c;
  const auto root = YAML::LoadFile(path);
  const bool own = bool(root["lane_deep_lite"]);
  const auto n = own ? root["lane_deep_lite"] : root["lane_lite"];
  if (!n || !n.IsMap())
    throw std::invalid_argument("YAML needs a lane_deep_lite (or lane_lite) map: " + path);
  for (const auto &kv : n) {
    const auto key = kv.first.as<std::string>();
    if (c.values.count(key))
      c.values[key] = kv.second.as<double>();
    else if (own)
      throw std::invalid_argument("Unknown lane_deep_lite parameter: " + key);
  }
  c.validate();
  return c;
}
void Config::save(const std::string &path) const {
  validate();
  YAML::Emitter e;
  e << YAML::BeginMap << YAML::Key << "lane_deep_lite" << YAML::Value << YAML::BeginMap;
  for (const auto &s : specs())
    e << YAML::Key << s.name << YAML::Value << values.at(s.name);
  e << YAML::EndMap << YAML::EndMap;
  std::ofstream f(path);
  f << e.c_str() << '\n';
  if (!f)
    throw std::runtime_error("Cannot save YAML: " + path);
}
const char *name(LineState state) {
  return state == LineState::Detected ? "DETECTED" : state == LineState::Held ? "HELD" : "LOST";
}

namespace {
float length(cv::Point2f v) { return std::sqrt(v.dot(v)); }
cv::Point2f unit(cv::Point2f v, cv::Point2f fallback = {0, -1}) {
  const float n = length(v);
  return n > 1e-6f ? v * (1.f / n) : fallback;
}
// Right-hand normal of a travel direction (up in BEV -> +x).
cv::Point2f rightOf(cv::Point2f d) { return {-d.y, d.x}; }
cv::Point2f tangent(const Path &p, size_t i) {
  return unit(p[std::min(i + 1, p.size() - 1)] - p[i ? i - 1 : 0]);
}
size_t nearest(const Path &p, cv::Point2f q, float *distance = nullptr) {
  size_t best = 0;
  float d = 1e9f;
  for (size_t i = 0; i < p.size(); ++i)
    if (length(p[i] - q) < d)
      d = length(p[i] - q), best = i;
  if (distance)
    *distance = d;
  return best;
}
// Signed distance of q from the path, positive when q is RIGHT of the
// near -> far direction (local tangent, so it also holds in a bend).
float sideOf(const Path &p, cv::Point2f q) {
  const size_t i = nearest(p, q);
  return (q - p[i]).dot(rightOf(tangent(p, i)));
}
bool interior(const Path &p, size_t i) { return i > 0 && i + 1 < p.size(); }
// Overlap distances from a to b, positive when b lies to the right of a.
std::vector<float> separation(const Path &a, const Path &b) {
  std::vector<float> out;
  for (size_t i = 0; i < a.size(); ++i) {
    float d;
    const size_t j = nearest(b, a[i], &d);
    if (interior(b, j))
      out.push_back((b[j] - a[i]).dot(rightOf(tangent(a, i))) >= 0 ? d : -d);
  }
  return out;
}
float median(std::vector<float> v) {
  if (v.empty())
    return NAN;
  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  return v[v.size() / 2];
}
// How far path a is from path b: median overlap distance, else near-end gap.
float distanceTo(const Path &a, const Path &b) {
  auto s = separation(a, b);
  for (auto &v : s)
    v = std::abs(v);
  if (!s.empty())
    return median(s);
  float d;
  nearest(b, a.front(), &d);
  return d;
}
Path blend(const Path &fresh, const Path &old, double alpha, float gate) {
  Path out = fresh;
  for (auto &p : out) {
    float d;
    const size_t j = nearest(old, p, &d);
    if (d < gate && interior(old, j))
      p += (old[j] - p) * float(1 - alpha);
  }
  return out;
}
cv::Point2f closestPoint(const Path &p, cv::Point2f q) {
  cv::Point2f best = p.front();
  float d = length(q - best);
  for (size_t j = 1; j < p.size(); ++j) {
    const cv::Point2f a = p[j - 1], e = p[j] - a;
    const float u = std::clamp((q - a).dot(e) / std::max(1e-6f, e.dot(e)), 0.f, 1.f);
    const cv::Point2f x = a + e * u;
    if (length(q - x) < d)
      d = length(q - x), best = x;
  }
  return best;
}
// Centre between `base` and the boundary on its `sign` side (+1 right, -1
// left), one point per base vertex: the midpoint where the base normal meets
// the other boundary, else to the closest other point at a plausible width,
// else the normal offset by half the nearest measured (or learnt) width,
// capped at 80 % of the local radius on the inside of a bend.
Path centreLine(const Path &base, const Path &other, float sign, float width, float tol) {
  const size_t n = base.size();
  std::vector<float> across(n, NAN);
  std::vector<cv::Point2f> mid(n);
  std::vector<int> kind(n, 3);
  for (size_t i = 0; i < n; ++i) {
    const cv::Point2f p = base[i], d = rightOf(tangent(base, i)) * sign;
    for (size_t j = 1; j < other.size(); ++j) {
      const cv::Point2f a = other[j - 1], e = other[j] - a, q = a - p;
      const float den = d.x * -e.y - d.y * -e.x;
      if (std::abs(den) < 1e-6f)
        continue;
      const float t = (q.x * -e.y - q.y * -e.x) / den, u = (d.x * q.y - d.y * q.x) / den;
      if (u >= 0 && u <= 1 && t > width * (1 - tol) && t < width * (1 + tol) &&
          (std::isnan(across[i]) || t < across[i]))
        across[i] = t;
    }
    if (!std::isnan(across[i])) {
      kind[i] = 1, mid[i] = p + d * (.5f * across[i]);
    } else if (!other.empty()) {
      const cv::Point2f q = closestPoint(other, p);
      const float w = length(q - p);
      if (w > width * (1 - tol) && w < width * (1 + tol) && (q - p).dot(d) > 0)
        kind[i] = 2, mid[i] = (p + q) * .5f, across[i] = w;
    }
  }
  for (size_t i = 0; i < n; ++i) {
    if (kind[i] != 3)
      continue;
    float w = width;
    for (size_t k = 1; k < n; ++k)
      if ((i >= k && !std::isnan(across[i - k])) || (i + k < n && !std::isnan(across[i + k]))) {
        w = i >= k && !std::isnan(across[i - k]) ? across[i - k] : across[i + k];
        break;
      }
    float half = .5f * w;
    const size_t a = i > 3 ? i - 3 : 0, b = std::min(i + 3, n - 1);
    const cv::Point2f ta = tangent(base, a), tb = tangent(base, b);
    const float turn = std::atan2(ta.x * tb.y - ta.y * tb.x, ta.dot(tb)) * sign;
    float arc = 0;
    for (size_t k = a; k < b; ++k)
      arc += length(base[k + 1] - base[k]);
    if (turn > 1e-3f)
      half = std::min(half, .8f * arc / turn);
    mid[i] = base[i] + rightOf(tangent(base, i)) * (sign * half);
  }
  Path out;
  for (size_t i = 0; i < n; ++i) {
    cv::Point2f sum;
    int count = 0;
    for (size_t k = i > 2 ? i - 2 : 0; k < std::min(n, i + 3); ++k)
      sum += mid[k], ++count;
    const cv::Point2f c = sum * (1.f / count);
    if (out.empty() || (c - out.back()).dot(tangent(base, i)) > 0)
      out.push_back(c);
  }
  return out;
}

struct Point {
  cv::Point2f p;
  float weight;
  int cls;
  bool used = false;
};

// 2. One centroid per grid cell and class (the network mask is already clean).
std::vector<Point> cellCentroids(const cv::Mat &bev, int cell, int min_pixels) {
  const int gx = (bev.cols + cell - 1) / cell, gy = (bev.rows + cell - 1) / cell;
  std::vector<cv::Vec3f> acc(size_t(gx) * gy * 2, cv::Vec3f(0, 0, 0));
  for (int y = 0; y < bev.rows; ++y) {
    const uchar *row = bev.ptr<uchar>(y);
    const int cy = y / cell;
    for (int x = 0; x < bev.cols; ++x)
      if (const uchar k = row[x]; k == 1 || k == 2) {
        auto &a = acc[(size_t(cy) * gx + x / cell) * 2 + (k - 1)];
        a[0] += x, a[1] += y, a[2] += 1;
      }
  }
  std::vector<Point> out;
  for (size_t i = 0; i < acc.size(); ++i)
    if (acc[i][2] >= min_pixels)
      out.push_back({{acc[i][0] / acc[i][2], acc[i][1] / acc[i][2]}, acc[i][2], int(i % 2) + 1});
  return out;
}

// 3. Chains: from the unused centroid nearest the vehicle, step to the nearest
// same-class centroid within max_gap that keeps the direction (|turn| <=
// max_turn), both ways. Neighbours within 1.25 cells (a thick or ragged band
// is two cells wide) are absorbed into the same chain. A
// dash gap is just a longer step; a line of another class never joins.
std::vector<std::vector<int>> chainPoints(std::vector<Point> &pts, cv::Point2f vehicle,
                                          float cell, float max_gap, float max_turn_deg) {
  const float cos_turn = std::cos(max_turn_deg * float(CV_PI / 180));
  std::vector<int> order(pts.size());
  for (size_t i = 0; i < pts.size(); ++i)
    order[i] = int(i);
  std::sort(order.begin(), order.end(), [&](int a, int b) {
    return length(pts[a].p - vehicle) < length(pts[b].p - vehicle);
  });
  auto absorb = [&](int k, int cls) {
    for (auto &q : pts)
      if (!q.used && q.cls == cls && length(q.p - pts[k].p) < 1.25f * cell)
        q.used = true;
  };
  auto extend = [&](std::vector<int> &seq) {
    while (true) {
      const int n = int(seq.size());
      const cv::Point2f last = pts[seq.back()].p;
      // Direction over the last ~2 cells (single steps jitter by a cell).
      cv::Point2f dir(0, 0);
      for (int k = n - 2; k >= 0; --k)
        if (length(last - pts[seq[k]].p) >= 1.5f * cell || k == 0) {
          dir = unit(last - pts[seq[k]].p, {0, 0});
          break;
        }
      int best = -1;
      float best_score = 1e9f;
      for (size_t j = 0; j < pts.size(); ++j) {
        const auto &q = pts[j];
        if (q.used || q.cls != pts[seq.back()].cls)
          continue;
        const float d = length(q.p - last);
        if (d > max_gap || d < 1e-3f)
          continue;
        float score = d;
        if (dir.x != 0 || dir.y != 0) {
          const float c = unit(q.p - last).dot(dir);
          if (c < cos_turn)
            continue;
          score = d * (1 + 3 * (1 - c));
        }
        if (score < best_score)
          best_score = score, best = int(j);
      }
      if (best < 0)
        return;
      pts[best].used = true;
      absorb(best, pts[best].cls);
      seq.push_back(best);
    }
  };
  std::vector<std::vector<int>> chains;
  for (int start : order) {
    if (pts[start].used)
      continue;
    pts[start].used = true;
    absorb(start, pts[start].cls);
    std::vector<int> seq = {start};
    extend(seq);
    std::reverse(seq.begin(), seq.end());
    extend(seq);
    chains.push_back(seq);
  }
  return chains;
}

// 4. Parametric fit x(t), y(t) over arc length (degree 2, 3 for long chains),
// weighted by cell pixel counts. If it does not fit: drop the worst point
// once, then trim the far end. Interpolates across dash gaps, never extrapolates.
bool fitChain(const Path &points, const std::vector<float> &weights_in, double max_residual,
              Path &out, double &rms) {
  Path pts = points;
  auto weights = weights_in;
  auto power = [](double t, int degree, double *row) {
    for (int k = 0; k <= degree; ++k)
      row[k] = k ? row[k - 1] * t : 1;
  };
  cv::Mat cx, cy;
  int degree = 0;
  double arc = 0;
  rms = 1e9;
  for (int attempt = 0; pts.size() >= 3; ++attempt) {
    const int n = int(pts.size());
    std::vector<double> tau(n, 0);
    for (int i = 1; i < n; ++i)
      tau[i] = tau[i - 1] + length(pts[i] - pts[i - 1]);
    arc = tau.back();
    if (arc < 8)
      break;
    for (auto &t : tau)
      t = 2 * t / arc - 1;
    degree = std::min(n - 1, n <= 6 ? 2 : 3);
    cv::Mat a(n, degree + 1, CV_64F), bx(n, 1, CV_64F), by(n, 1, CV_64F);
    for (int i = 0; i < n; ++i) {
      const double w = std::sqrt(weights[i]);
      double *row = a.ptr<double>(i);
      power(tau[i], degree, row);
      for (int k = 0; k <= degree; ++k)
        row[k] *= w;
      bx.at<double>(i) = w * pts[i].x;
      by.at<double>(i) = w * pts[i].y;
    }
    if (!cv::solve(a, bx, cx, cv::DECOMP_QR) || !cv::solve(a, by, cy, cv::DECOMP_QR))
      return false;
    double sum = 0, worst_e = -1, row[6];
    size_t worst = 0;
    for (int i = 0; i < n; ++i) {
      power(tau[i], degree, row);
      double x = 0, y = 0;
      for (int k = 0; k <= degree; ++k)
        x += cx.at<double>(k) * row[k], y += cy.at<double>(k) * row[k];
      const double e = std::hypot(pts[i].x - x, pts[i].y - y);
      sum += e * e;
      if (e > worst_e)
        worst_e = e, worst = size_t(i);
    }
    rms = std::sqrt(sum / n);
    if (rms <= max_residual)
      break;
    const size_t drop = attempt == 0 ? worst : pts.size() - 1;
    pts.erase(pts.begin() + long(drop));
    weights.erase(weights.begin() + long(drop));
  }
  if (rms > max_residual)
    return false;
  const int samples = std::max(8, std::min(64, int(arc / 15) + 1));
  double row[6];
  out.clear();
  for (int k = 0; k <= samples; ++k) {
    power(2. * k / samples - 1, degree, row);
    double x = 0, y = 0;
    for (int j = 0; j <= degree; ++j)
      x += cx.at<double>(j) * row[j], y += cy.at<double>(j) * row[j];
    out.emplace_back(float(x), float(y));
  }
  return true;
}

// Signed curvature (1/m, left positive) of metric points (x forward, y left):
// algebraic circle fit; ~0 when the points are nearly straight.
double circleCurvature(const std::vector<cv::Point2d> &p) {
  if (p.size() < 5)
    return 0;
  const auto a = p.front(), b = p.back();
  const double chord = cv::norm(b - a);
  if (chord < .1)
    return 0;
  double bulge = 0, side = 0;
  for (const auto &q : p) {
    const double c = ((b - a).x * (q - a).y - (b - a).y * (q - a).x) / chord;
    if (std::abs(c) > std::abs(bulge))
      bulge = c;
    side += c;
  }
  if (std::abs(bulge) < .003)
    return 0;
  cv::Mat A(int(p.size()), 3, CV_64F), r(int(p.size()), 1, CV_64F), s;
  for (int i = 0; i < int(p.size()); ++i) {
    A.at<double>(i, 0) = p[i].x, A.at<double>(i, 1) = p[i].y, A.at<double>(i, 2) = 1;
    r.at<double>(i) = -(p[i].x * p[i].x + p[i].y * p[i].y);
  }
  if (!cv::solve(A, r, s, cv::DECOMP_SVD))
    return 0;
  const double D = s.at<double>(0), E = s.at<double>(1), F = s.at<double>(2);
  const double r2 = D * D / 4 + E * E / 4 - F;
  if (!(r2 > 0))
    return 0;
  // Points left of the chord (positive cross) bulge left: a right turn.
  return (side > 0 ? -1 : 1) / std::sqrt(r2);
}

void drawPath(cv::Mat &image, const Path &p, cv::Scalar color, int thickness, bool dashed = false) {
  for (size_t i = 1; i < p.size(); ++i)
    if (!dashed || (i / 2) % 2 == 0)
      cv::line(image, p[i - 1], p[i], color, thickness, cv::LINE_AA);
}
} // namespace

Pipeline::Pipeline(const Config &config) { configure(config); }
void Pipeline::configure(const Config &config) {
  config.validate();
  config_ = config;
  const auto &c = config_;
  const int w = c["width"], h = c["height"];
  const std::vector<cv::Point2f> src = {
      {float(c["bev_top_left"] * (w - 1)), float(c["bev_top_y"] * (h - 1))},
      {float(c["bev_top_right"] * (w - 1)), float(c["bev_top_y"] * (h - 1))},
      {float(c["bev_bottom_right"] * (w - 1)), float(c["bev_bottom_y"] * (h - 1))},
      {float(c["bev_bottom_left"] * (w - 1)), float(c["bev_bottom_y"] * (h - 1))}};
  const std::vector<cv::Point2f> dst = {
      {0, 0}, {float(w - 1), 0}, {float(w - 1), float(h - 1)}, {0, float(h - 1)}};
  H_ = cv::getPerspectiveTransform(src, dst);
  // Image area inside the BEV; the part outside the camera view stays empty.
  cv::warpPerspective(cv::Mat(h, w, CV_8U, cv::Scalar(255)), bev_valid_, H_, {w, h},
                      cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
  cv::erode(bev_valid_, bev_valid_, cv::Mat(), {-1, -1}, 2);
  reset();
}
void Pipeline::reset() {
  tracks_[0] = tracks_[1] = {};
  center_.clear();
  center_seen_ = last_time_ = -1;
  center_confidence_ = 0;
  width_ = config_["lane_width_px"];
}

Result Pipeline::process(const cv::Mat &classes, double time, bool views) {
  const auto start_clock = std::chrono::steady_clock::now();
  if (classes.empty() || classes.type() != CV_8U || !std::isfinite(time))
    throw std::invalid_argument("Expected a CV_8U class mask and finite time");
  const auto &c = config_;
  if (last_time_ >= 0 && (time <= last_time_ || time - last_time_ > std::max(.5, c["track_memory"])))
    reset();
  const double dt = last_time_ < 0 ? .05 : std::clamp(time - last_time_, .001, .2);
  last_time_ = time;
  const int w = c["width"], h = c["height"];
  const double cu = c["vehicle_center_u"] * (w - 1), tol = c["width_tolerance"];
  const float cell = float(c["cell"]), jump_gate = float(c["track_jump_px"]);
  const cv::Point2f vehicle(float(cu), float(h - 1));
  Result r;

  // 1. Class mask -> BEV (nearest keeps the class ids).
  cv::Mat sized = classes;
  if (classes.cols != w || classes.rows != h)
    cv::resize(classes, sized, {w, h}, 0, 0, cv::INTER_NEAREST);
  cv::warpPerspective(sized, r.bev_classes, H_, {w, h}, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
  r.bev_classes.setTo(0, bev_valid_ == 0);

  // 2-4. Cell centroids -> chains -> fitted candidates.
  auto pts = cellCentroids(r.bev_classes, int(cell), int(c["min_cell_pixels"]));
  struct Seq {
    Path p;
    std::vector<float> w;
    int cls;
  };
  std::vector<Seq> seqs;
  for (const auto &ids : chainPoints(pts, vehicle, cell, float(c["max_gap"]), float(c["max_turn_deg"]))) {
    Seq q{{}, {}, pts[ids.front()].cls};
    for (int k : ids)
      q.p.push_back(pts[k].p), q.w.push_back(pts[k].weight);
    if (length(q.p.back() - vehicle) < length(q.p.front() - vehicle)) { // near -> far
      std::reverse(q.p.begin(), q.p.end());
      std::reverse(q.w.begin(), q.w.end());
    }
    seqs.push_back(q);
  }
  // A ragged spot can split one line: join a far end to another chain's near
  // end when the gap is bridgeable and both directions agree.
  const float join_cos = std::cos(c["max_turn_deg"] * CV_PI / 180);
  auto endDir = [](const Path &p, bool far) {
    const size_t n = p.size(), k = std::min<size_t>(3, n - 1);
    return far ? unit(p[n - 1] - p[n - 1 - k]) : unit(p[k] - p[0]);
  };
  for (bool merged = true; merged;) {
    merged = false;
    for (size_t a = 0; a < seqs.size() && !merged; ++a)
      for (size_t b = 0; b < seqs.size() && !merged; ++b) {
        if (a == b || seqs[a].cls != seqs[b].cls || seqs[a].p.size() < 2 || seqs[b].p.size() < 2)
          continue;
        const cv::Point2f gap = seqs[b].p.front() - seqs[a].p.back();
        const cv::Point2f da = endDir(seqs[a].p, true), db = endDir(seqs[b].p, false);
        if (length(gap) > c["max_gap"] || da.dot(db) < join_cos ||
            (length(gap) > .5f * cell && (unit(gap).dot(da) < join_cos || unit(gap).dot(db) < join_cos)))
          continue;
        seqs[a].p.insert(seqs[a].p.end(), seqs[b].p.begin(), seqs[b].p.end());
        seqs[a].w.insert(seqs[a].w.end(), seqs[b].w.begin(), seqs[b].w.end());
        seqs.erase(seqs.begin() + long(b));
        merged = true;
      }
  }
  for (const auto &q : seqs) {
    Chain ch;
    ch.cls = q.cls;
    ch.points = q.p;
    for (size_t i = 1; i < ch.points.size(); ++i)
      ch.length += length(ch.points[i] - ch.points[i - 1]);
    if (ch.length < c["min_length"] || ch.points.size() < 3)
      continue;
    if (fitChain(ch.points, q.w, c["max_residual"], ch.path, ch.residual))
      r.chains.push_back(ch);
  }
  // A second chain along the same band (thick paint) is a duplicate: keep the
  // longest of chains that lie within 1.5 cells of each other.
  std::sort(r.chains.begin(), r.chains.end(),
            [](const Chain &a, const Chain &b) { return a.length > b.length; });
  {
    std::vector<Chain> kept;
    for (const auto &ch : r.chains) {
      bool duplicate = false;
      for (const auto &k : kept)
        duplicate |= distanceTo(ch.path, k.path) < 1.5f * cell;
      if (!duplicate)
        kept.push_back(ch);
    }
    r.chains = kept;
  }

  // 5. Choose the left/right boundaries: on each side the line nearest the
  // vehicle with the vehicle on its inner side (the network mask has no
  // clutter to confuse "nearest"); the line that continues the track gets a
  // bonus of 0.3 lane widths so a side does not flicker between two lines.
  // If the two sides do not form one lane, the nearer side stays and the
  // other is re-chosen at the lane width from it.
  Path prior[2];
  for (int side = 0; side < 2; ++side)
    if (!tracks_[side].path.empty() && time - tracks_[side].seen <= c["track_memory"])
      prior[side] = tracks_[side].path;
  std::vector<float> inner(r.chains.size()); // >0: vehicle right of chain (a left boundary)
  for (size_t i = 0; i < r.chains.size(); ++i)
    inner[i] = sideOf(r.chains[i].path, vehicle);
  auto fitsSide = [&](size_t i, int side) { return side == 0 ? inner[i] > 0 : inner[i] < 0; };
  auto widthOf = [&](int a, int b) { // a left, b right
    return median(separation(r.chains[a].path, r.chains[b].path));
  };
  // Continuing the track: a line straddled by the vehicle keeps its side
  // until the vehicle is clearly past it (no left/right flip on the line).
  auto continues = [&](size_t i, int side) {
    return !prior[side].empty() && distanceTo(r.chains[i].path, prior[side]) < 1.5 * jump_gate;
  };
  auto choose = [&](int side, int exclude, int anchor) {
    int best_i = -1;
    double best = 1e9;
    for (size_t i = 0; i < r.chains.size(); ++i) {
      if (int(i) == exclude || int(i) == anchor)
        continue;
      const bool track = continues(i, side);
      if (!fitsSide(i, side) && !(track && std::abs(inner[i]) < .25 * width_))
        continue;
      if (anchor >= 0) {
        const float m = side == 0 ? widthOf(int(i), anchor) : widthOf(anchor, int(i));
        if (std::isnan(m) || std::abs(m - width_) > tol * width_)
          continue; // must overlap the anchor at one lane width
      }
      const double score = std::abs(inner[i]) - (track ? .3 * width_ : 0);
      if (score < best)
        best = score, best_i = int(i);
    }
    return best_i;
  };
  int pick[2] = {choose(0, -1, -1), -1};
  pick[1] = choose(1, pick[0], -1);
  if (pick[0] >= 0 && pick[1] >= 0) {
    const float m = widthOf(pick[0], pick[1]);
    if (std::isnan(m) || std::abs(m - width_) > tol * width_) {
      // Keep the side that continues its track, else the longer line, else
      // the nearer one: a fragment glimpsed for one frame must not replace a
      // boundary followed for many.
      const bool t0 = continues(pick[0], 0), t1 = continues(pick[1], 1);
      const double l0 = r.chains[pick[0]].length, l1 = r.chains[pick[1]].length;
      const int keep = t0 != t1                            ? (t0 ? 0 : 1)
                       : std::abs(l0 - l1) > .25 * std::max(l0, l1) ? (l0 > l1 ? 0 : 1)
                       : std::abs(inner[pick[0]]) <= std::abs(inner[pick[1]]) ? 0 : 1;
      pick[1 - keep] = choose(1 - keep, -1, pick[keep]);
    }
  }
  Line *lines[2] = {&r.left, &r.right};
  for (int side = 0; side < 2; ++side) {
    if (pick[side] < 0) {
      lines[side]->note = "후보 없음";
      continue;
    }
    const auto &ch = r.chains[pick[side]];
    lines[side]->path = ch.path;
    lines[side]->cls = ch.cls;
    lines[side]->state = LineState::Detected;
    lines[side]->quality = std::min(1., ch.length / (.5 * h));
  }

  // Corridor check of a tracked pair (a fresh pair was chosen by width).
  if (r.left.state == LineState::Detected && r.right.state == LineState::Detected) {
    auto ws = separation(r.left.path, r.right.path);
    if (ws.empty())
      ws.push_back((r.right.path.front() - r.left.path.front()).dot(rightOf(tangent(r.left.path, 0))));
    const auto [mn, mx] = std::minmax_element(ws.begin(), ws.end());
    if (*mn < width_ * (1 - tol) || *mx > width_ * (1 + tol)) {
      auto disagreement = [&](int side) {
        return prior[side].empty() ? 1e6 - lines[side]->quality
                                   : double(distanceTo(lines[side]->path, prior[side]));
      };
      const int drop = disagreement(0) > disagreement(1) ? 0 : 1;
      lines[drop]->path.clear();
      lines[drop]->state = LineState::Lost;
      lines[drop]->note = "차로 폭 " + std::to_string(int(*mn)) + "~" + std::to_string(int(*mx)) +
                          " px 불일치로 제외";
    } else {
      const double base = c["lane_width_px"], m = median(ws);
      if (std::abs(m - base) <= .5 * tol * base)
        width_ = std::clamp(width_ + .1 * (m - width_), .9 * base, 1.1 * base);
    }
  }

  // Tracking: smooth, confirm jumps / class changes over 3 frames, or hold.
  const double alpha = 1 - std::pow(1 - c["smoothing"], dt / .05);
  for (int side = 0; side < 2; ++side) {
    auto &line = *lines[side];
    auto &track = tracks_[side];
    bool accept = line.state == LineState::Detected, relock = false;
    if (accept && !prior[side].empty()) {
      double jump = distanceTo(line.path, prior[side]);
      if (track.cls && track.cls != line.cls)
        jump = std::max(jump, c["track_jump_px"] + 1);
      if (jump > c["track_jump_px"]) {
        relock = ++track.jumps >= 3;
        if (!relock) {
          accept = false;
          line.note = "급격한 이동 " + std::to_string(int(jump)) + " px, 확인 " +
                      std::to_string(track.jumps) + "/3";
        }
      } else {
        track.jumps = 0;
      }
    }
    if (accept) {
      track.path = !prior[side].empty() && !relock ? blend(line.path, prior[side], alpha, jump_gate)
                                                   : line.path;
      track.seen = time;
      track.quality = line.quality;
      track.cls = line.cls;
      if (relock)
        track.jumps = 0;
      line.path = track.path;
      line.state = LineState::Detected;
    } else if (!prior[side].empty()) {
      line.path = prior[side];
      line.state = LineState::Held;
      line.age = time - track.seen;
      line.quality = .5 * track.quality * (1 - line.age / std::max(.01, c["track_memory"]));
    } else {
      track = {};
      line.path.clear();
      line.state = LineState::Lost;
    }
  }

  // Centre path along the longer detection (held boundaries never mix in).
  const bool dl = r.left.state == LineState::Detected, dr = r.right.state == LineState::Detected;
  Path center;
  double confidence = 0;
  if (dl || dr) {
    const bool left_base = dl && (!dr || r.left.path.size() >= r.right.path.size());
    const Line &base = left_base ? r.left : r.right, &other = left_base ? r.right : r.left;
    center = centreLine(base.path, dl && dr ? other.path : Path(), left_base ? 1.f : -1.f,
                        float(width_), float(tol));
    // The offset of a ragged boundary zigzags; one smooth fit of the centre
    // keeps the lookahead point and the curvature from jumping.
    if (center.size() >= 4) {
      Path smooth;
      double rms;
      if (fitChain(center, std::vector<float>(center.size(), 1.f), 1e9, smooth, rms))
        center = smooth;
    }
    confidence = dl && dr ? (r.left.quality + r.right.quality) / 2 : .7 * base.quality;
    r.state = dl && dr ? "both" : dl ? "left only" : "right only";
    if (!center_.empty() && time - center_seen_ <= c["track_memory"])
      center = blend(center, center_, alpha, jump_gate);
    center_ = center;
    center_seen_ = time;
    center_confidence_ = confidence;
  } else if (!center_.empty() && time - center_seen_ <= c["track_memory"]) {
    center = center_;
    confidence = center_confidence_ * (1 - (time - center_seen_) / std::max(.01, c["track_memory"]));
    r.state = "held";
  } else {
    center_.clear();
    r.state = "lost";
  }
  r.lane_width_px = width_;

  // Lookahead target and metric values (same definitions as lane_fusion);
  // curvature from a circle fit over curvature_span_m of the centre path.
  const double m_px = c["lane_width_m"] / c["lane_width_px"];
  const double m_row = (c["bev_far_m"] - c["bev_near_m"]) / (h - 1);
  auto metric = [&](cv::Point2f p) {
    return cv::Point2d(c["bev_near_m"] + (h - 1 - p.y) * m_row, (cu - p.x) * m_px);
  };
  double span = 0;
  for (size_t i = 1; i < center.size(); ++i)
    span += length(center[i] - center[i - 1]);
  if (center.size() >= 3 && span >= .1 * h) {
    const double want = c["bev_near_m"] + (1 - c["lookahead_row"]) * (h - 1) * m_row;
    size_t t = 0;
    while (t + 1 < center.size() && cv::norm(metric(center[t])) < want)
      ++t;
    const auto a = metric(center[t ? t - 1 : 0]), b = metric(center[std::min(t + 1, center.size() - 1)]);
    const auto target = metric(center[t]);
    r.valid = true;
    r.center_path = center;
    r.confidence = std::clamp(confidence, 0., 1.);
    r.target_x = center[t].x;
    r.target_y = center[t].y;
    r.lateral_m = target.y;
    r.lookahead_m = target.x;
    r.heading = std::atan2(b.y - a.y, b.x - a.x);
    std::vector<cv::Point2d> arc = {metric(center[0])};
    for (size_t i = 1; i < center.size(); ++i) {
      const auto q = metric(center[i]);
      if (cv::norm(q - arc.front()) > c["curvature_span_m"])
        break;
      arc.push_back(q);
    }
    r.curvature = std::clamp(circleCurvature(arc), -c["max_curvature"], c["max_curvature"]);
    const auto n0 = metric(center[0]);
    size_t k = 1;
    while (k + 1 < center.size() && cv::norm(metric(center[k]) - n0) < .25)
      ++k;
    const auto n1 = metric(center[k]);
    r.near_lateral_m = n0.y;
    r.near_m = n0.x;
    r.near_heading = std::atan2(n1.y - n0.y, n1.x - n0.x);
  } else if (!center.empty()) {
    r.notes.push_back("중앙 경로가 너무 짧음");
  }
  for (const auto *line : lines)
    if (!line->note.empty())
      r.notes.push_back(std::string(line == &r.left ? "왼쪽: " : "오른쪽: ") + line->note);

  if (views) {
    r.view = cv::Mat(h, w, CV_8UC3, cv::Scalar(30, 30, 30));
    r.view.setTo(cv::Scalar(55, 55, 55), bev_valid_);
    r.view.setTo(cv::Scalar(0, 150, 200), r.bev_classes == 1);
    r.view.setTo(cv::Scalar(170, 170, 170), r.bev_classes == 2);
    for (const auto &q : pts)
      cv::circle(r.view, q.p, 2, cv::Scalar(90, 90, 90), -1);
    for (const auto &ch : r.chains)
      drawPath(r.view, ch.path, cv::Scalar(140, 0, 140), 1);
    const cv::Scalar colours[2] = {cv::Scalar(0, 255, 0), cv::Scalar(255, 200, 0)};
    for (int side = 0; side < 2; ++side)
      drawPath(r.view, lines[side]->path, colours[side], 3, lines[side]->state == LineState::Held);
    drawPath(r.view, center, cv::Scalar(0, 0, 255), 2, r.state == "held");
    if (r.valid)
      cv::circle(r.view, {cvRound(r.target_x), cvRound(r.target_y)}, 7, cv::Scalar(0, 0, 255), 2);
    cv::line(r.view, vehicle, vehicle - cv::Point2f(0, 25), cv::Scalar(255, 0, 255), 2);
    char text[96];
    std::snprintf(text, sizeof text, "%s  lat %+.3f  k %+.2f", r.state.c_str(), r.lateral_m,
                  r.curvature);
    cv::putText(r.view, text, {8, 22}, cv::FONT_HERSHEY_SIMPLEX, .6, cv::Scalar(0, 255, 255), 2);
  }
  r.processing_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_clock).count();
  return r;
}
} // namespace deep_lite

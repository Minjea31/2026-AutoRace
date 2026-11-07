#include "autorace_lane_fusion/lane.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace lane_lite {
const std::vector<const char *> &groups() {
  static const std::vector<const char *> g = {
      "입력", "색상", "ROI·BEV", "Sliding Window", "차로·추적", "거리 보정", "딥 퓨전"};
  return g;
}
const std::vector<Spec> &specs() {
  static const std::vector<Spec> s = {
      {"width", "입력", "처리 영상 너비(px). BEV도 같은 크기", 640, 160, 1280, 1},
      {"height", "입력", "처리 영상 높이(px)", 480, 120, 960, 1},
      {"yellow_enabled", "색상", "노란 차선 사용: 0=끔, 1=켬", 1, 0, 1, 1},
      {"yellow_h_min", "색상", "노랑 Hue 하한(OpenCV 0-179)", 15, 0, 179, 1},
      {"yellow_h_max", "색상", "노랑 Hue 상한", 40, 0, 179, 1},
      {"yellow_s_min", "색상", "노랑 최소 채도(S)", 80, 0, 255, 1},
      {"yellow_v_min", "색상", "노랑 최소 밝기(V)", 80, 0, 255, 1},
      {"white_enabled", "색상", "흰 차선 사용: 0=끔, 1=켬", 1, 0, 1, 1},
      {"white_s_max", "색상", "흰색 최대 채도(S)", 70, 0, 255, 1},
      {"white_v_min", "색상", "흰색 최소 밝기(V), 절대 하한", 120, 0, 255, 1},
      {"white_tophat", "색상",
       "흰색이 주변 노면보다 밝아야 하는 정도(Top-hat). 넓은 반사광·조명 변화 제거",
       30, 0, 200, 1},
      {"roi_top", "ROI·BEV", "검출 영역 위쪽(0=영상 위)", .45, 0, 1, .01},
      {"roi_bottom", "ROI·BEV", "검출 영역 아래쪽", 1, 0, 1, .01},
      {"roi_left", "ROI·BEV", "검출 영역 왼쪽", 0, 0, 1, .01},
      {"roi_right", "ROI·BEV", "검출 영역 오른쪽", 1, 0, 1, .01},
      {"bev_top_y", "ROI·BEV", "BEV 사다리꼴 위쪽 높이", .495, 0, 1, .005},
      {"bev_bottom_y", "ROI·BEV", "BEV 사다리꼴 아래쪽 높이", 1, 0, 1, .005},
      {"bev_top_left", "ROI·BEV", "BEV 위 왼쪽 x", .29, 0, 1, .001},
      {"bev_top_right", "ROI·BEV", "BEV 위 오른쪽 x", .65, 0, 1, .001},
      {"bev_bottom_left", "ROI·BEV", "BEV 아래 왼쪽 x(화면 밖 가능)", -.64, -2, 1, .001},
      {"bev_bottom_right", "ROI·BEV", "BEV 아래 오른쪽 x(화면 밖 가능)", 1.62, 0, 3, .001},
      {"white_max_width", "색상",
       "이보다 두꺼운 흰 덩어리(BEV px) 제거: 횡단보도 블록·노면 글자. 0=끔", 0, 0, 120, 1},
      {"white_max_length", "색상",
       "이보다 긴 흰 선(BEV px, 덩어리의 긴 변)은 실선으로 보고 제거해 흰 점선만 남김. 0=끔",
       0, 0, 2000, 1},
      {"line_width_max", "Sliding Window",
       "BEV에서 차선 띠의 최대 두께(px). Top-hat 크기와 윈도우 안 띠 폭", 40, 4, 120, 1},
      {"min_area", "Sliding Window", "BEV 특징 연결요소 최소 면적(px²)", 60, 0, 3000, 1},
      {"windows", "Sliding Window", "아래→위 윈도우 개수", 12, 4, 30, 1},
      {"margin", "Sliding Window", "윈도우 반폭(px)", 50, 10, 200, 1},
      {"min_pixels", "Sliding Window", "윈도우를 관측으로 인정할 최소 픽셀 수", 30, 3, 3000, 1},
      {"max_misses", "Sliding Window", "연속 미관측 윈도우 수가 이 값이면 탐색 종료", 3, 1, 15, 1},
      {"min_windows", "Sliding Window", "차선으로 인정할 최소 관측 윈도우 수", 3, 2, 20, 1},
      {"max_turn_deg", "Sliding Window",
       "연속한 윈도우 중심 사이 최대 꺾임(도). 넘으면 그 윈도우는 미관측: 차선 끝에서 노면 글자·정지선으로 꺾여 들어가지 않게. 급커브 바깥 경계는 윈도우당 15~25°",
       40, 10, 90, 1},
      {"max_residual", "Sliding Window", "윈도우 중심 곡선 맞춤 RMS 상한(px)", 10, 1, 60, .5},
      {"histogram_start", "Sliding Window",
       "시작점 히스토그램을 계산할 BEV 하단 영역의 시작(0=맨 위)", .5, 0, .95, .01},
      {"lane_width_px", "차로·추적", "BEV에서 좌우 경계 사이 차로 폭(px)", 320, 40, 1200, 1},
      {"width_tolerance", "차로·추적", "차로 폭 허용 오차 비율", .35, .05, .9, .01},
      {"max_angle_deg", "차로·추적",
       "좌우 경계 방향 차이 허용치(도). 넘으면 평행하지 않은 쪽(횡단보도·정지선 조각) 제외. 노랑+노랑 쌍에는 적용 안 함(급커브)",
       30, 5, 90, 1},
      {"role_by_color", "차로·추적",
       "1=노랑은 왼쪽·흰색은 오른쪽 경계로 고정, 0=위치·폭으로 판단", 0, 0, 1, 1},
      {"vehicle_center_u", "차로·추적", "BEV에서 차량 중심선 가로 위치(0-1)", .5, 0, 1, .001},
      {"lookahead_row", "차로·추적", "목표점 BEV 세로 위치(0=먼 곳, 1=차량 앞)", .55, .05, .98, .01},
      {"smoothing", "차로·추적", "20 Hz 기준 새 관측 반영 비율(1=필터 없음)", .6, .05, 1, .05},
      {"track_jump_px", "차로·추적",
       "한 프레임 경계 이동 허용치(px). 넘으면 3프레임 연속일 때만 교체", 60, 5, 400, 1},
      {"track_memory", "차로·추적", "관측이 끊겨도 마지막 경계를 유지하는 시간(s)", .5, 0, 3, .05},
      {"lane_width_m", "거리 보정", "실제 차로 폭(m). 가로 px→m 환산", .55, .1, 3, .01},
      {"bev_near_m", "거리 보정", "BEV 맨 아래 행의 기준점(base_link) 전방 거리(m), 실측",
       .25, 0, 5, .01},
      {"bev_far_m", "거리 보정", "BEV 맨 위 행의 전방 거리(m), 실측", 1.2, .05, 10, .01},
      {"deep_mode", "딥 퓨전",
       "딥 mask 사용(mask가 있을 때만): 0=HSV만, 1=gated(HSV∧딥), 2=딥만, "
       "3=딥 우선(딥이 놓친 쪽 경계만 예상 위치 주변에서 HSV로 보완)", 0, 0, 3, 1},
      {"deep_dilate", "딥 퓨전", "gated에서 딥 mask를 넓히는 반경(BEV px)", 6, 0, 40, 1},
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
    if (!std::isfinite(v) || v < s.min || v > s.max ||
        (s.step == 1 && v != std::floor(v)))
      throw std::invalid_argument(std::string("Invalid parameter: ") + s.name);
  }
  const auto &c = *this;
  if (c["roi_top"] + .05 >= c["roi_bottom"] ||
      c["roi_left"] + .05 >= c["roi_right"] ||
      c["bev_top_y"] + .05 >= c["bev_bottom_y"] ||
      c["bev_top_left"] + .05 >= c["bev_top_right"] ||
      c["bev_bottom_left"] + .05 >= c["bev_bottom_right"] ||
      c["yellow_h_min"] > c["yellow_h_max"] ||
      c["bev_near_m"] + .05 >= c["bev_far_m"])
    throw std::invalid_argument("Inverted ROI/BEV/colour/distance limits");
}
Config Config::load(const std::string &path) {
  Config c;
  const auto root = YAML::LoadFile(path);
  if (const auto n = root["lane_lite"]) {
    if (!n.IsMap())
      throw std::invalid_argument("lane_lite must be a map");
    for (const auto &kv : n) {
      const auto key = kv.first.as<std::string>();
      if (!c.values.count(key))
        throw std::invalid_argument("Unknown lane_lite parameter: " + key);
      c.values[key] = kv.second.as<double>();
    }
  } else {
    // Reuse a calibration saved by the old lane GUI.
    const bool sliding = root["algorithm"] &&
                         root["algorithm"].as<std::string>() == "sliding_window";
    auto legacy = root[sliding && root["sliding_window"] ? "sliding_window" : "lane"];
    if (!legacy || !legacy.IsMap())
      throw std::invalid_argument("YAML needs a lane_lite (or legacy lane) map");
    const bool hsv = !legacy["color_space"] || legacy["color_space"].as<double>() == 0;
    const std::map<std::string, std::string> rename = {
        {"yellow_sat_min", "yellow_s_min"},
        {"yellow_light_min", "yellow_v_min"},
        {"white_sat_max", "white_s_max"},
        {"white_light_min", "white_v_min"}};
    for (const auto &kv : legacy) {
      auto key = kv.first.as<std::string>();
      const bool colour = key.rfind("yellow_h", 0) == 0 || rename.count(key);
      if (rename.count(key))
        key = rename.at(key);
      if (c.values.count(key) && (hsv || !colour) && key != "white_tophat")
        c.values[key] = kv.second.as<double>();
    }
  }
  c.validate();
  return c;
}
void Config::save(const std::string &path) const {
  validate();
  YAML::Emitter e;
  e << YAML::BeginMap << YAML::Key << "lane_lite" << YAML::Value
    << YAML::BeginMap;
  for (const auto &s : specs())
    e << YAML::Key << s.name << YAML::Value << values.at(s.name);
  e << YAML::EndMap << YAML::EndMap;
  std::ofstream f(path);
  f << e.c_str() << '\n';
  if (!f)
    throw std::runtime_error("Cannot save YAML: " + path);
}
const char *name(LineState state) {
  return state == LineState::Detected    ? "DETECTED"
         : state == LineState::Predicted ? "HELD"
                                         : "LOST";
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
// Nearest vertex. An end vertex means "no perpendicular overlap".
size_t nearest(const Path &p, cv::Point2f q, float *distance = nullptr) {
  size_t best = 0;
  float d = 1e9f;
  for (size_t i = 0; i < p.size(); ++i)
    if (length(p[i] - q) < d) {
      d = length(p[i] - q);
      best = i;
    }
  if (distance)
    *distance = d;
  return best;
}
// Signed distance of q from the path, positive when q lies to the RIGHT of the
// near -> far travel direction. Uses the local tangent, so it holds in a bend
// where screen x alone says nothing about left and right.
float sideOf(const Path &p, cv::Point2f q) {
  const size_t i = nearest(p, q);
  return (q - p[i]).dot(rightOf(tangent(p, i)));
}
cv::Point2f nearDirection(const Path &p) {
  return unit(p[std::max<size_t>(1, p.size() / 3)] - p.front());
}
// Near third of the path runs more than 60 deg from the driving direction.
bool transverse(const Path &p) {
  return std::abs(nearDirection(p).y) < .5f;
}
bool interior(const Path &p, size_t i) { return i > 0 && i + 1 < p.size(); }
Path offset(const Path &p, float distance) {
  Path out;
  for (size_t i = 0; i < p.size(); ++i)
    out.push_back(p[i] + rightOf(tangent(p, i)) * distance);
  return out;
}
// Overlap distances from `a` to `b`, signed positive when b lies to the right.
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
// Median direction difference (deg) between two near -> far paths where they
// overlap; near-third directions when they do not. Lane boundaries are
// parallel even in a bend; a stop line or crosswalk stripe edge is not.
float angleBetween(const Path &a, const Path &b) {
  std::vector<float> out;
  for (size_t i = 0; i < a.size(); ++i) {
    const size_t j = nearest(b, a[i]);
    if (interior(b, j))
      out.push_back(std::acos(std::clamp(tangent(a, i).dot(tangent(b, j)), -1.f, 1.f)));
  }
  if (out.size() < 3)
    out = {std::acos(std::clamp(nearDirection(a).dot(nearDirection(b)), -1.f, 1.f))};
  return median(out) * float(180 / CV_PI);
}
// Move `fresh` toward `old` by (1 - alpha) where the two overlap.
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
// Nearest point of polyline `p` to q (on a segment, not only a vertex).
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
// left), one point per base vertex:
//  1. where the base normal crosses the other boundary, the midpoint: the two
//     sides correspond across the lane also in a bend (nearest vertices lag
//     on the inside of a curve);
//  2. else the midpoint to the closest point of the other boundary, if that
//     is a plausible lane width away (hairpin entry: the boundaries are not
//     parallel and the normal misses the short inner one);
//  3. else (other side shorter, or absent) the normal offset by half the
//     nearest measured width, else the learnt width. Offsetting the fitted
//     curve keeps the curvature of a single boundary; toward the inside of a
//     bend the offset is capped at 80 % of the local radius so it never folds
//     (BEV perspective makes a tight bend look tighter than the lane is wide).
// A 5-point mean removes the steps between the three cases.
Path centreLine(const Path &base, const Path &other, float sign, float width, float tol) {
  const size_t n = base.size();
  std::vector<float> across(n, NAN);
  std::vector<cv::Point2f> mid(n);
  std::vector<int> kind(n, 3);
  for (size_t i = 0; i < n; ++i) {
    const cv::Point2f p = base[i], d = rightOf(tangent(base, i)) * sign;
    for (size_t j = 1; j < other.size(); ++j) {
      // p + t d = a + u (b - a)
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
      if (w > width * (1 - tol) && w < width * (1 + tol) && (q - p).dot(d) > 0) {
        kind[i] = 2, mid[i] = (p + q) * .5f;
        across[i] = w;
      }
    }
  }
  for (size_t i = 0; i < n; ++i) {
    if (kind[i] != 3)
      continue;
    float w = width; // nearest measured width
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

struct Peak {
  int x;
  bool yellow;
};
// Column histogram peaks of the lower BEV, strongest first.
std::vector<Peak> findPeaks(const cv::Mat &mask, const cv::Mat &yellow, int y0,
                            int minimum, int separation_px) {
  cv::Mat hist, yhist;
  cv::reduce(mask.rowRange(y0, mask.rows) / 255, hist, 0, cv::REDUCE_SUM, CV_32F);
  cv::reduce(yellow.rowRange(y0, mask.rows) / 255, yhist, 0, cv::REDUCE_SUM, CV_32F);
  cv::blur(hist, hist, {9, 1});
  cv::blur(yhist, yhist, {9, 1});
  std::vector<Peak> out;
  for (int n = 0; n < 6; ++n) {
    double best;
    cv::Point at;
    cv::minMaxLoc(hist, nullptr, &best, nullptr, &at);
    if (best < minimum)
      break;
    out.push_back({at.x, yhist.at<float>(at.x) * 2 > hist.at<float>(at.x)});
    hist.colRange(std::max(0, at.x - separation_px),
                  std::min(hist.cols, at.x + separation_px + 1))
        .setTo(0);
  }
  return out;
}
// Keep pixels on a thin run along a row OR a column. Painted lines of any
// direction are thin across; broad glare and floor patches are thin in neither.
cv::Mat thinBands(const cv::Mat &mask, int max_width) {
  auto rows = [max_width](const cv::Mat &src) {
    cv::Mat dst = cv::Mat::zeros(src.size(), CV_8U);
    for (int y = 0; y < src.rows; ++y) {
      const uchar *s = src.ptr<uchar>(y);
      uchar *d = dst.ptr<uchar>(y);
      for (int x = 0; x < src.cols;) {
        int end = x;
        while (end < src.cols && s[end])
          ++end;
        if (end > x && end - x <= max_width)
          std::fill(d + x, d + end, 255);
        x = std::max(end, x + 1);
      }
    }
    return dst;
  };
  cv::Mat transposed, columns;
  cv::transpose(mask, transposed);
  cv::transpose(rows(transposed), columns);
  return rows(mask) | columns;
}
// HSV colour candidates of the colour BEV. White must be a thin band brighter
// than its own surroundings (top-hat); crosswalk blocks / road text (too thick)
// and solid white lines (too long) are removed. white_raw: before the filters.
void hsvColours(const cv::Mat &bev, const Config &c, cv::Mat &yellow, cv::Mat &white,
                cv::Mat &white_raw) {
  cv::Mat hsv, blurred, tophat, value;
  cv::GaussianBlur(bev, blurred, {5, 5}, 0);
  cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, cv::Scalar(c["yellow_h_min"], c["yellow_s_min"], c["yellow_v_min"]),
              cv::Scalar(c["yellow_h_max"], 255, 255), yellow);
  cv::inRange(hsv, cv::Scalar(0, 0, c["white_v_min"]), cv::Scalar(179, c["white_s_max"], 255),
              white_raw);
  cv::extractChannel(hsv, value, 2);
  const int k = 2 * int(c["line_width_max"]) + 1;
  cv::morphologyEx(value, tophat, cv::MORPH_TOPHAT,
                   cv::getStructuringElement(cv::MORPH_RECT, {k, k}));
  white = white_raw & (tophat >= c["white_tophat"]);
  if (const int t = c["white_max_width"]; t > 0) {
    cv::Mat thick;
    cv::morphologyEx(white, thick, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_RECT, {t, t}));
    cv::dilate(thick, thick, cv::getStructuringElement(cv::MORPH_RECT, {7, 7}));
    white &= ~thick;
  }
  if (const double limit = c["white_max_length"]; limit > 0) {
    std::vector<std::vector<cv::Point>> parts;
    cv::findContours(white.clone(), parts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (size_t i = 0; i < parts.size(); ++i)
      if (parts[i].size() >= 2) {
        const auto box = cv::minAreaRect(parts[i]).size;
        if (std::max(box.width, box.height) > limit)
          cv::drawContours(white, parts, int(i), 0, cv::FILLED);
      }
  }
}
// Final marking mask from colour candidates (yellow/white are modified):
// colour switches, BEV validity, thin bands per colour (glare touching a
// yellow line does not widen it) and the minimum component area. `colours`
// collects the candidates for display.
cv::Mat features(cv::Mat &yellow, cv::Mat &white, cv::Mat white_raw, const cv::Mat &valid,
                 const Config &c, cv::Mat *colours) {
  if (!c["yellow_enabled"])
    yellow.setTo(0);
  if (!c["white_enabled"])
    white.setTo(0), white_raw.setTo(0);
  yellow &= valid;
  white &= valid;
  if (colours)
    *colours |= (yellow | white_raw) & valid;
  const auto open3 = cv::getStructuringElement(cv::MORPH_RECT, {3, 3});
  cv::morphologyEx(yellow, yellow, cv::MORPH_OPEN, open3);
  cv::morphologyEx(white, white, cv::MORPH_OPEN, open3);
  // A line of thickness t has min(row, column) run <= t*sqrt(2) at any angle.
  const int run = cvRound(c["line_width_max"] * 1.45);
  cv::Mat mask = thinBands(yellow, run) | thinBands(white, run);
  if (c["min_area"] > 0) {
    cv::Mat labels, stats, centroids;
    const int count = cv::connectedComponentsWithStats(mask, labels, stats, centroids);
    std::vector<uchar> keep(count, 0);
    for (int i = 1; i < count; ++i)
      keep[i] = stats.at<int>(i, cv::CC_STAT_AREA) >= c["min_area"];
    for (int y = 0; y < mask.rows; ++y) {
      const int *label = labels.ptr<int>(y);
      uchar *m = mask.ptr<uchar>(y);
      for (int x = 0; x < mask.cols; ++x)
        m[x] = keep[label[x]] ? m[x] : 0;
    }
  }
  yellow &= mask;
  return mask;
}

struct Search {
  Path centers;
  std::vector<double> weights;
  std::vector<cv::Point> pixels;
  std::vector<cv::RotatedRect> boxes;
  std::vector<bool> hits;
};
// Windows that step along the marking's own direction, so a boundary that
// bends from vertical to transverse is followed without a second model.
// Only observed window centres are kept; missed windows are never fitted.
Search slide(const cv::Mat &mask, cv::Point2f start, cv::Point2f dir,
             const Path &prior, const Config &c) {
  const int w = mask.cols, h = mask.rows;
  const float step = std::max(4, h / int(c["windows"])), margin = c["margin"];
  const float band = c["line_width_max"];
  const size_t min_pixels = c["min_pixels"];
  const float max_turn = float(std::cos(c["max_turn_deg"] * CV_PI / 180));
  Search s;
  cv::Point2f p = start - dir * (step * .5f);
  int misses = 0;
  std::vector<cv::Point> found, inside;
  std::vector<float> offsets;
  for (int i = 0; i < 2 * int(c["windows"]); ++i) {
    if (!prior.empty())
      dir = unit(dir + tangent(prior, nearest(prior, p)), dir);
    const cv::Point2f q = p + dir * step, across = rightOf(dir);
    if (q.x < -margin || q.x > w - 1 + margin || q.y < -step || q.y > h - 1 + step)
      break; // the boundary left the BEV
    const cv::RotatedRect box(q, {2 * margin, step},
                              float(std::atan2(across.y, across.x) * 180 / CV_PI));
    s.boxes.push_back(box);
    const cv::Rect bounds = box.boundingRect() & cv::Rect(0, 0, w, h);
    found.clear(), inside.clear(), offsets.clear();
    if (!bounds.empty())
      cv::findNonZero(mask(bounds), found);
    cv::Mat hist = cv::Mat::zeros(1, 2 * int(margin) + 1, CV_32F), smooth;
    for (auto pt : found) {
      pt += bounds.tl();
      const cv::Point2f v = cv::Point2f(pt) - q;
      const float o = v.dot(across);
      if (std::abs(v.dot(dir)) <= step * .5f && std::abs(o) <= margin) {
        inside.push_back(pt);
        offsets.push_back(o);
        hist.at<float>(cvRound(o + margin)) += 1;
      }
    }
    // The cross-track peak closest to the prediction picks one marking; a
    // stop line or a second marking in the window is not averaged in.
    cv::blur(hist, smooth, {5, 1});
    double largest;
    cv::minMaxLoc(smooth, nullptr, &largest);
    int peak = -1;
    for (int b = 0; b < smooth.cols && largest > 0; ++b)
      if (smooth.at<float>(b) * 3 >= largest &&
          (peak < 0 || std::abs(b - margin) < std::abs(peak - margin)))
        peak = b;
    bool hit = false;
    if (peak >= 0) {
      cv::Point2d sum;
      size_t chosen = 0, first = s.pixels.size();
      for (size_t k = 0; k < inside.size(); ++k)
        if (std::abs(offsets[k] - (peak - margin)) <= band) {
          s.pixels.push_back(inside[k]);
          sum += cv::Point2d(inside[k]);
          ++chosen;
        }
      const cv::Point2f center(chosen ? sum * (1. / chosen) : cv::Point2d());
      const cv::Point2f turn = s.centers.empty() ? dir : unit(center - s.centers.back(), dir);
      // The marking may bend by at most max_turn_deg per window, measured on
      // the observed centres: a painted line bends gradually, while the end of
      // a line running into road text or a stop line turns sharply.
      const size_t n = s.centers.size();
      const cv::Point2f before = n >= 2 ? unit(s.centers[n - 1] - s.centers[n - 2], dir) : dir;
      if (chosen >= min_pixels && turn.dot(dir) > .5f && turn.dot(before) >= max_turn) {
        dir = unit(dir + turn, dir);
        s.centers.push_back(center);
        s.weights.push_back(double(chosen));
        p = center;
        hit = true;
      } else
        s.pixels.resize(first);
    }
    s.hits.push_back(hit);
    if (!hit) {
      p = q;
      // With a track, keep looking for a boundary that starts later (inner
      // side of a bend, dashed line); otherwise stop after a gap.
      if (++misses >= c["max_misses"] && !(!prior.empty() && s.centers.empty()))
        break;
    } else
      misses = 0;
  }
  return s;
}
// Parametric polynomial x(τ), y(τ) over the normalised arc length of the
// observed window centres. It follows any orientation, also a >90° hairpin,
// without the straight chord frame of a lateral-offset fit, so a real bend is
// neither straightened nor trimmed away. The degree grows with the evidence:
// quadratic up to 5 centres (3 centres are kept exactly), cubic, and a quintic
// only for a long S / hairpin the cubic misses. Noise between windows is
// averaged, not followed. If it still does not fit, one outlier window is
// dropped, then the far end is trimmed (the near part matters most for
// control). Never extrapolates.
Line fitLine(const Search &s, const Config &c) {
  Line line;
  line.boxes = s.boxes;
  line.hits = s.hits;
  line.centers = s.centers;
  line.observed = int(s.centers.size());
  // Full quality at twice the minimum: a short, real bend is not a weak one.
  line.quality = std::min(1., line.observed / (2 * c["min_windows"]));
  if (line.observed < c["min_windows"]) {
    line.note = "관측 윈도우 " + std::to_string(line.observed) + "개 < min_windows";
    return line;
  }
  Path pts = s.centers;
  auto weights = s.weights;
  cv::Mat cx, cy;
  double rms = 1e9, arc = 0;
  auto power = [](double t, int degree, double *row) {
    for (int k = 0; k <= degree; ++k)
      row[k] = k ? row[k - 1] * t : 1;
  };
  int degree = 0;
  for (int attempt = 0; pts.size() >= size_t(c["min_windows"]); ++attempt) {
    const int n = int(pts.size());
    std::vector<double> tau(n, 0);
    for (int i = 1; i < n; ++i)
      tau[i] = tau[i - 1] + length(pts[i] - pts[i - 1]);
    arc = tau.back();
    if (arc < 8)
      break;
    for (auto &t : tau)
      t = 2 * t / arc - 1; // [-1, 1]: well conditioned powers
    std::vector<int> degrees = {std::min(n - 1, n <= 5 ? 2 : 3)};
    rms = 1e9;
    size_t worst = 0;
    for (int d : degrees) {
      cv::Mat a(n, d + 1, CV_64F), bx(n, 1, CV_64F), by(n, 1, CV_64F);
      for (int i = 0; i < n; ++i) {
        const double wgt = std::sqrt(weights[i]);
        double *row = a.ptr<double>(i);
        power(tau[i], d, row);
        for (int k = 0; k <= d; ++k)
          row[k] *= wgt;
        bx.at<double>(i) = wgt * pts[i].x;
        by.at<double>(i) = wgt * pts[i].y;
      }
      if (!cv::solve(a, bx, cx, cv::DECOMP_QR) || !cv::solve(a, by, cy, cv::DECOMP_QR) ||
          !cv::checkRange(cx) || !cv::checkRange(cy))
        continue;
      degree = d;
      double sum = 0, worst_error = -1, row[6];
      for (int i = 0; i < n; ++i) {
        power(tau[i], d, row);
        double x = 0, y = 0;
        for (int k = 0; k <= d; ++k)
          x += cx.at<double>(k) * row[k], y += cy.at<double>(k) * row[k];
        const double e = std::hypot(pts[i].x - x, pts[i].y - y);
        sum += e * e;
        if (e > worst_error)
          worst_error = e, worst = i;
      }
      rms = std::sqrt(sum / n);
      if (rms <= c["max_residual"])
        break;
    }
    if (rms <= c["max_residual"])
      break;
    // Drop the worst window once, then trim the far end.
    const size_t drop = attempt == 0 ? worst : pts.size() - 1;
    pts.erase(pts.begin() + drop);
    weights.erase(weights.begin() + drop);
  }
  line.residual = rms;
  if (rms > c["max_residual"]) {
    line.note = arc < 8 ? "관측 길이 부족" : "윈도우 중심이 곡선에 맞지 않음";
    return line;
  }
  // About one vertex per 1/32 of the BEV height of arc, at least 8.
  const int samples = std::max(8, std::min(64, int(arc / 15) + 1));
  double row[6];
  for (int k = 0; k <= samples; ++k) {
    power(2. * k / samples - 1, degree, row);
    double x = 0, y = 0;
    for (int j = 0; j <= degree; ++j)
      x += cx.at<double>(j) * row[j], y += cy.at<double>(j) * row[j];
    line.path.emplace_back(float(x), float(y));
  }
  line.observed = int(pts.size());
  line.state = LineState::Detected;
  return line;
}
void drawPath(cv::Mat &image, const Path &p, cv::Scalar color, int thickness, bool dashed) {
  for (size_t i = 1; i < p.size(); ++i)
    if (!dashed || (i / 2) % 2 == 0)
      cv::line(image, p[i - 1], p[i], color, thickness, cv::LINE_AA);
}
} // namespace

Pipeline::Pipeline(const Config &config) { configure(config); }
void Pipeline::configure(const Config &config) {
  config.validate();
  config_ = config;
  // Geometry: ROI ∩ trapezoid in the source image, then one homography.
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
  Hinv_ = H_.inv();
  cv::Mat valid = cv::Mat::zeros(h, w, CV_8U), roi = valid.clone();
  std::vector<cv::Point> poly(src.begin(), src.end());
  cv::fillConvexPoly(valid, poly, 255);
  cv::rectangle(roi, {cvRound(c["roi_left"] * (w - 1)), cvRound(c["roi_top"] * (h - 1))},
                {cvRound(c["roi_right"] * (w - 1)), cvRound(c["roi_bottom"] * (h - 1))},
                255, cv::FILLED);
  valid &= roi;
  cv::warpPerspective(valid, bev_valid_, H_, {w, h}, cv::INTER_NEAREST);
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

Result Pipeline::process(const cv::Mat &bgr, double time, bool views, const cv::Mat &deep) {
  const auto start_clock = std::chrono::steady_clock::now();
  if (bgr.empty() || bgr.type() != CV_8UC3 || !std::isfinite(time))
    throw std::invalid_argument("Expected a BGR8 image and finite time");
  if (!deep.empty() && deep.type() != CV_8U)
    throw std::invalid_argument("Deep mask must be CV_8U class ids");
  const auto &c = config_;
  if (last_time_ >= 0 && (time <= last_time_ ||
                          time - last_time_ > std::max(.5, c["track_memory"])))
    reset();
  const double dt = last_time_ < 0 ? .05 : std::clamp(time - last_time_, .001, .2);
  last_time_ = time;
  const int w = c["width"], h = c["height"];
  // 0 HSV, 1 gated (HSV ∧ deep), 2 deep only, 3 deep first + HSV for a missed side.
  const int mode = deep.empty() ? 0 : int(c["deep_mode"]);
  Result r;
  static const char *source_names[] = {"hsv", "gated", "deep", "deep"};
  r.source = source_names[mode];
  r.deep_used = mode > 0;
  cv::resize(bgr, r.original, {w, h});
  // The colour BEV feeds only HSV and the display views (17 ms on the Jetson),
  // so it is warped on first use. Replicated borders avoid artificial edges.
  auto colourBev = [&]() -> const cv::Mat & {
    if (r.bev.empty())
      cv::warpPerspective(r.original, r.bev, H_, {w, h}, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    return r.bev;
  };
  cv::Mat colours = views ? cv::Mat::zeros(h, w, CV_8U) : cv::Mat();
  cv::Mat *show = views ? &colours : nullptr;

  // 1. Features in BEV. Deep: class mask (0 bg, 1 yellow/orange, 2 white
  // dashed) in source coordinates, warped into the same BEV. gated keeps only
  // the HSV pixels the network also calls lane paint (drops text, arrows,
  // crosswalks); 2/3 use the network mask alone (no HSV work).
  cv::Mat yellow, white, white_raw;
  if (mode <= 1)
    hsvColours(colourBev(), c, yellow, white, white_raw);
  if (mode > 0) {
    cv::Mat d, bev_deep;
    cv::resize(deep, d, {w, h}, 0, 0, cv::INTER_NEAREST);
    cv::warpPerspective(d, bev_deep, H_, {w, h}, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
    cv::Mat deep_yellow = bev_deep == 1, deep_white = bev_deep == 2;
    if (mode >= 2) {
      yellow = deep_yellow;
      white = white_raw = deep_white;
    } else {
      if (const int k = c["deep_dilate"]; k > 0) {
        const auto disk = cv::getStructuringElement(cv::MORPH_ELLIPSE, {2 * k + 1, 2 * k + 1});
        cv::dilate(deep_yellow, deep_yellow, disk);
        cv::dilate(deep_white, deep_white, disk);
      }
      yellow &= deep_yellow;
      white &= deep_white;
    }
  }
  cv::Mat mask = features(yellow, white, white_raw, bev_valid_, c, show);

  // 2. Seeds: a recent track starts from its own near end; otherwise the
  // histogram peaks that best form one corridor of the expected width, or
  // the peak nearest the expected position of a single side.
  const double cu = c["vehicle_center_u"] * (w - 1), tol = c["width_tolerance"];
  const float band = c["line_width_max"];
  const cv::Point2f vehicle(float(cu), float(h - 1));
  Path prior[2];
  for (int side = 0; side < 2; ++side)
    if (!tracks_[side].path.empty() && time - tracks_[side].seen <= c["track_memory"])
      prior[side] = tracks_[side].path;
  const int min_peak = std::max(5, int(c["min_pixels"]) / 2);
  auto colourOk = [&](const Peak &p, int side) {
    return !c["role_by_color"] || p.yellow == (side == 0);
  };
  Line *lines[2] = {&r.left, &r.right};
  std::vector<cv::Point> used[2];
  // Seeds the sides in `wanted` from `m` (peaks of `peak_mask`); `known` x of
  // a side already found anchors the other.
  auto seeds = [&](const cv::Mat &peak_mask, const cv::Mat &peak_yellow, const bool wanted[2],
                   double seed[2]) {
    auto peaks = findPeaks(peak_mask, peak_yellow, int(c["histogram_start"] * (h - 1)), min_peak,
                           int(c["margin"]));
    if (peaks.empty())
      peaks = findPeaks(peak_mask, peak_yellow, 0, min_peak, int(c["margin"]));
    auto nearestPeak = [&](int side, double x, double radius) {
      double best = NAN, distance = radius;
      for (const auto &p : peaks)
        if (colourOk(p, side) && std::abs(p.x - x) < distance && p.x != seed[1 - side]) {
          distance = std::abs(p.x - x);
          best = p.x;
        }
      return best;
    };
    for (int side = 0; side < 2; ++side)
      if (wanted[side] && !prior[side].empty())
        seed[side] = prior[side].front().x;
    if (wanted[0] && wanted[1] && std::isnan(seed[0]) && std::isnan(seed[1])) {
      double best = 1e9;
      for (const auto &a : peaks)
        for (const auto &b : peaks) {
          const double sep = b.x - a.x;
          if (sep <= 0 || std::abs(sep - width_) > tol * width_ || !colourOk(a, 0) ||
              !colourOk(b, 1))
            continue;
          const double score = std::abs(sep - width_) / width_ +
                               std::abs((a.x + b.x) / 2. - cu) / width_;
          if (score < best) {
            best = score;
            seed[0] = a.x;
            seed[1] = b.x;
          }
        }
    }
    for (int side = 0; side < 2; ++side)
      if (wanted[side] && std::isnan(seed[side])) {
        const double sign = side == 0 ? -1 : 1;
        // Anchor: the other boundary found this frame, else its track.
        double other = seed[1 - side];
        if (std::isnan(other) && lines[1 - side]->state == LineState::Detected)
          other = lines[1 - side]->path.front().x;
        else if (std::isnan(other) && !prior[1 - side].empty())
          other = prior[1 - side].front().x;
        const double expected = std::isnan(other) ? cu + sign * width_ / 2 : other + sign * width_;
        seed[side] = nearestPeak(side, expected, std::isnan(other) ? width_ * .75 : tol * width_ * 1.5);
      }
  };
  // Direction-following sliding windows both ways from the seed and the fit.
  auto detect = [&](int side, double seed_x, const cv::Mat &m, const cv::Mat &m_yellow) {
    auto &line = *lines[side];
    if (std::isnan(seed_x)) {
      line = {};
      line.note = "시작점 없음";
      return;
    }
    cv::Point2f start, dir(0, -1);
    if (!prior[side].empty()) {
      // Start a little nearer than the old near end: the car moved forward.
      dir = tangent(prior[side], 0);
      start = prior[side].front() - dir * float(h / c["windows"]);
    } else {
      // Lowest marking pixel under the seed: a bend entry often begins
      // mid-BEV and must not end the search by bottom misses.
      const int x0 = std::clamp(cvRound(seed_x - band), 0, w - 1);
      const int x1 = std::clamp(cvRound(seed_x + band), x0 + 1, w);
      cv::Mat column;
      cv::reduce(m.colRange(x0, x1), column, 1, cv::REDUCE_MAX);
      int y = h - 1;
      while (y > 0 && !column.at<uchar>(y))
        --y;
      start = {float(seed_x), float(y)};
    }
    // A seed in the middle of a bend must still collect the part nearer the
    // vehicle. Near -> far order is kept.
    auto search = slide(m, start, dir, prior[side], c);
    const auto back = slide(m, start, -dir, {}, c);
    search.centers.insert(search.centers.begin(), back.centers.rbegin(), back.centers.rend());
    search.weights.insert(search.weights.begin(), back.weights.rbegin(), back.weights.rend());
    search.pixels.insert(search.pixels.end(), back.pixels.begin(), back.pixels.end());
    search.boxes.insert(search.boxes.end(), back.boxes.begin(), back.boxes.end());
    search.hits.insert(search.hits.end(), back.hits.begin(), back.hits.end());
    line = fitLine(search, c);
    // Paths run near -> far: an inverted path flips heading and the side of
    // the centre offset. In a hairpin both ends can be about equally far from
    // the vehicle, so there a tracked boundary keeps the orientation of its
    // track (continuity) instead of re-deciding by distance every frame. A
    // clearly inverted path is turned round in any case, so one bad frame
    // cannot leave the track backwards.
    if (auto &path = line.path; path.size() > 1) {
      const float lead = length(path.back() - vehicle) - length(path.front() - vehicle);
      const bool flip =
          lead < -.25f * h ||
          (lead < .25f * h &&
           (!prior[side].empty() ? length(path.back() - prior[side].front()) <
                                       length(path.front() - prior[side].front())
                                 : lead < -.05f * h));
      if (flip)
        std::reverse(path.begin(), path.end());
    }
    line.seed = start;
    used[side] = line.state == LineState::Detected ? search.pixels : std::vector<cv::Point>();
    size_t yellow_pixels = 0;
    for (const auto &p : search.pixels)
      yellow_pixels += m_yellow.at<uchar>(p) != 0;
    line.yellow = 2 * yellow_pixels > search.pixels.size();
  };
  auto reject = [&](int side, const std::string &why) {
    auto &line = *lines[side];
    line.rejected = line.path;
    line.path.clear();
    line.state = LineState::Lost;
    line.note = why;
    used[side].clear();
  };
  // Left/right of a FRESH boundary (no track) from geometry along the line,
  // not from screen x: the other boundary (found now, else tracked) must lie
  // on its inner side; with nothing else known, the vehicle must. A tracked
  // boundary keeps its role by continuity.
  auto checkRole = [&](int side) {
    auto &line = *lines[side];
    if (c["role_by_color"] || line.state != LineState::Detected || !prior[side].empty())
      return;
    const float sign = side == 0 ? 1.f : -1.f; // inner side of this boundary
    const Path *ref = lines[1 - side]->state == LineState::Detected ? &lines[1 - side]->path
                      : !prior[1 - side].empty()                   ? &prior[1 - side]
                                                                   : nullptr;
    if (ref) {
      if (sign * sideOf(line.path, ref->front()) <= 0)
        reject(side, std::string("반대편 경계의 ") + (side ? "왼쪽" : "오른쪽") + "에 있어 제외");
    } else if (sign * sideOf(line.path, vehicle) < 0) {
      *lines[1 - side] = line;
      lines[1 - side]->note = "진행 방향 기준 차량 " + std::string(side ? "왼쪽" : "오른쪽") +
                              "의 경계라 역할 교정";
      used[1 - side] = used[side];
      line = {};
      line.note = "역할 교정으로 비움";
      used[side].clear();
    }
  };

  {
    const bool both[2] = {true, true};
    double seed[2] = {NAN, NAN};
    seeds(mask, yellow, both, seed);
    for (int side = 0; side < 2; ++side)
      detect(side, seed[side], mask, yellow);
  }
  // 2b. Deep first (3): a side the network missed is searched again in HSV,
  // only around where that boundary is expected (its track, or the other
  // boundary offset by the lane width), so HSV noise elsewhere (text, other
  // roads) stays out. With nothing to anchor it, the whole BEV is used.
  if (mode == 3 && (r.left.state != LineState::Detected || r.right.state != LineState::Detected)) {
    cv::Mat hy, hw, hraw;
    hsvColours(colourBev(), c, hy, hw, hraw);
    cv::Mat hmask = features(hy, hw, hraw, bev_valid_, c, show);
    cv::Mat corridor = cv::Mat::zeros(h, w, CV_8U);
    bool anchored = true, wanted[2] = {false, false};
    const int reach = cvRound(std::max<double>(c["margin"], .5 * tol * width_));
    for (int side = 0; side < 2; ++side) {
      if (lines[side]->state == LineState::Detected)
        continue;
      wanted[side] = true;
      const float sign = side == 0 ? -1.f : 1.f;
      const Path expected = !prior[side].empty() ? prior[side]
                            : lines[1 - side]->state == LineState::Detected
                                ? offset(lines[1 - side]->path, sign * float(width_))
                            : !prior[1 - side].empty() ? offset(prior[1 - side], sign * float(width_))
                                                       : Path();
      if (expected.size() < 2)
        anchored = false;
      else
        cv::polylines(corridor, std::vector<cv::Point>(expected.begin(), expected.end()), false,
                      255, 2 * reach);
    }
    if (anchored)
      hmask &= corridor, hy &= corridor;
    if (cv::countNonZero(hmask)) {
      double seed[2] = {NAN, NAN};
      seeds(hmask, hy, wanted, seed);
      mask |= hmask;
      yellow |= hy;
      for (int side = 0; side < 2; ++side)
        if (wanted[side]) {
          detect(side, seed[side], mask, yellow);
          if (lines[side]->state == LineState::Detected)
            r.source = "deep+hsv";
        }
    }
  }
  r.bev_mask = mask;
  for (int side = 0; side < 2; ++side)
    checkRole(side);

  // A white boundary whose near part lies across the road is a stop line or
  // crosswalk edge, also when a track crept onto it gradually. Only when the
  // other boundary (detected now, or still tracked) turns the same way is it
  // a bend. The track is forgotten so the next frame re-seeds from the
  // histogram instead of searching around the stop line for track_memory.
  // Colour of the near third only: a search that starts on a yellow bend
  // and runs onto a stop line is mostly yellow but its path is the line.
  auto crossing = [&](int side) {
    const auto &line = *lines[side];
    if (line.state != LineState::Detected || !transverse(line.path))
      return false;
    int yellow_count = 0, all = 0;
    for (size_t i = 0; i <= line.path.size() / 3; ++i) {
      const cv::Rect box = cv::Rect(cvRound(line.path[i].x) - 4, cvRound(line.path[i].y) - 4, 9, 9) &
                           cv::Rect(0, 0, w, h);
      yellow_count += cv::countNonZero(yellow(box));
      all += cv::countNonZero(mask(box));
    }
    return 2 * yellow_count <= all;
  };
  for (int side = 0; side < 2; ++side) {
    const auto &other = lines[1 - side]->state == LineState::Detected ? lines[1 - side]->path
                                                                       : prior[1 - side];
    if (crossing(side) && !(other.size() > 1 && transverse(other))) {
      reject(side, "가로 흰 선(정지선·횡단보도)으로 보고 제외");
      tracks_[side] = {};
      prior[side].clear();
    }
  }

  // 3. Corridor check: two detections must form one plausible lane.
  if (r.left.state == LineState::Detected && r.right.state == LineState::Detected) {
    auto ws = separation(r.left.path, r.right.path);
    if (ws.empty()) // no overlap: compare the two near ends
      ws.push_back((r.right.path.front() - r.left.path.front())
                       .dot(rightOf(tangent(r.left.path, 0))));
    const auto [mn, mx] = std::minmax_element(ws.begin(), ws.end());
    // The angle test targets white crosswalk / stop-line fragments. Two
    // yellow boundaries in a tight bend legitimately differ by >30 deg in
    // BEV (the inner line turns faster), so a yellow pair is width-checked only.
    const float angle = angleBetween(r.left.path, r.right.path);
    const bool yellow_pair = r.left.yellow && r.right.yellow;
    const bool bad_width = *mn < width_ * (1 - tol) || *mx > width_ * (1 + tol);
    if (bad_width || (!yellow_pair && angle > c["max_angle_deg"])) {
      // Drop the side that disagrees more with its own track; without
      // tracks, the one with fewer observed windows. A white line lying
      // across the road always goes first; then the shorter of a crossing
      // pair (a crosswalk stripe fragment is short).
      auto disagreement = [&](int side) {
        if (crossing(side) && !crossing(1 - side))
          return 1e9;
        // An angle-only failure of a yellow/white pair: the white one is the
        // stop-line / crosswalk fragment this test exists for.
        if (!bad_width && lines[side]->yellow != lines[1 - side]->yellow)
          return lines[side]->yellow ? 0. : 1e8;
        if (!bad_width)
          return 1e6 - lines[side]->observed;
        const double d = prior[side].empty()
                             ? NAN
                             : std::abs(median(separation(lines[side]->path, prior[side])));
        return std::isnan(d) ? 1e6 - lines[side]->observed : d;
      };
      const int drop = disagreement(0) > disagreement(1) ? 0 : 1;
      reject(drop, bad_width ? "차로 폭 " + std::to_string(int(*mn)) + "~" +
                                   std::to_string(int(*mx)) + " px 불일치로 제외"
                             : "좌우 방향 차이 " + std::to_string(int(angle)) +
                                   "° > max_angle_deg로 제외");
    } else {
      // Learn only from pairs close to the calibrated width and only within
      // +-10 %: road text or arrows paired with a boundary must not shrink
      // the offset used whenever a single boundary is seen.
      const double base = c["lane_width_px"], m = median(ws);
      if (std::abs(m - base) <= .5 * tol * base)
        width_ = std::clamp(width_ + .1 * (m - width_), .9 * base, 1.1 * base);
    }
  }

  // 4. Tracking: accept, smooth, confirm jumps or colour changes, or hold.
  const double alpha = 1 - std::pow(1 - c["smoothing"], dt / .05);
  for (int side = 0; side < 2; ++side) {
    auto &line = *lines[side];
    auto &track = tracks_[side];
    bool accept = line.state == LineState::Detected, relock = false;
    if (accept && !prior[side].empty()) {
      double jump = std::abs(median(separation(line.path, prior[side])));
      if (std::isnan(jump)) {
        float d;
        nearest(prior[side], line.path.front(), &d);
        jump = d;
      }
      // A colour change (white glare beside a yellow line) counts as a jump.
      if (track.colour >= 0 && bool(track.colour) != line.yellow)
        jump = std::max(jump, c["track_jump_px"] + 1);
      if (jump > c["track_jump_px"]) {
        const bool crossing = !line.yellow && transverse(line.path);
        relock = ++track.jumps >= 3 && !crossing;
        if (!relock) {
          accept = false;
          line.rejected = line.path;
          line.note = crossing ? std::string("가로 흰 선으로 이동하지 않음")
                               : "급격한 이동 " + std::to_string(int(jump)) + " px, 확인 " +
                                     std::to_string(std::min(track.jumps, 3)) + "/3";
          used[side].clear();
        }
      } else
        track.jumps = 0;
    }
    if (accept) {
      track.path = !prior[side].empty() && !relock
                       ? blend(line.path, prior[side], alpha, c["track_jump_px"])
                       : line.path;
      track.seen = time;
      track.quality = line.quality;
      track.colour = line.yellow;
      if (relock)
        track.jumps = 0;
      line.path = track.path;
      line.state = LineState::Detected;
    } else if (!prior[side].empty()) {
      line.path = prior[side];
      line.state = LineState::Predicted;
      line.age = time - track.seen;
      line.quality = .5 * track.quality * (1 - line.age / std::max(.01, c["track_memory"]));
    } else {
      track = {};
      line.path.clear();
      line.state = LineState::Lost;
    }
  }

  // 5. Centre path along the longer detection, matched across the lane to
  // the other one (or offset by the lane width). Held boundaries never mix in.
  const bool dl = r.left.state == LineState::Detected, dr = r.right.state == LineState::Detected;
  Path center;
  double confidence = 0;
  if (dl || dr) {
    const bool left_base = dl && (!dr || r.left.path.size() >= r.right.path.size());
    const Line &base = left_base ? r.left : r.right, &other = left_base ? r.right : r.left;
    center = centreLine(base.path, dl && dr ? other.path : Path(), left_base ? 1.f : -1.f,
                        float(width_), float(tol));
    confidence = dl && dr ? (r.left.quality + r.right.quality) / 2 : .7 * base.quality;
    r.state = dl && dr ? "both" : dl ? "left only" : "right only";
    if (!center_.empty() && time - center_seen_ <= c["track_memory"])
      center = blend(center, center_, alpha, c["track_jump_px"]);
    center_ = center;
    center_seen_ = time;
    center_confidence_ = confidence;
  } else if (!center_.empty() && time - center_seen_ <= c["track_memory"]) {
    center = center_;
    confidence = center_confidence_ *
                 (1 - (time - center_seen_) / std::max(.01, c["track_memory"]));
    r.state = "held";
  } else {
    center_.clear();
    r.state = "lost";
  }
  r.lane_width_px = width_;

  // 6. Lookahead target: the first centre point at the straight-line
  // distance of lookahead_row from the vehicle, within the observed support.
  const double m_px = c["lane_width_m"] / c["lane_width_px"];
  const double m_row = (c["bev_far_m"] - c["bev_near_m"]) / (h - 1);
  auto metric = [&](cv::Point2f p) { // forward, left (m) from base_link
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
    r.lateral_px = cu - r.target_x;
    r.lateral_m = target.y;
    r.lookahead_m = target.x;
    r.heading = std::atan2(b.y - a.y, b.x - a.x);
    const double ab = cv::norm(b - a), at = cv::norm(target - a), tb = cv::norm(b - target);
    const double cross = (target - a).x * (b - a).y - (target - a).y * (b - a).x;
    r.curvature = ab * at * tb > 1e-9 ? 2 * cross / (ab * at * tb) : 0; // left positive
    // Near point: first centre point, heading over the first ~0.25 m of path.
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

  // 7. Views: BEV search trace, BEV path, and the path projected back.
  if (!views) {
    r.processing_ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - start_clock).count();
    return r;
  }
  r.search_view = colourBev() * .45;
  r.search_view.setTo(cv::Scalar(230, 230, 230), mask & ~yellow);
  r.search_view.setTo(cv::Scalar(0, 210, 255), yellow);
  for (int side = 0; side < 2; ++side) {
    const auto &line = *lines[side];
    const cv::Scalar color = side == 0 ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 200, 0);
    for (size_t i = 0; i < line.boxes.size(); ++i) {
      cv::Point2f corners[4];
      line.boxes[i].points(corners);
      for (int k = 0; k < 4; ++k)
        cv::line(r.search_view, corners[k], corners[(k + 1) % 4],
                 line.hits[i] ? color : cv::Scalar(110, 110, 110), 1);
    }
    for (const auto &p : line.centers)
      cv::circle(r.search_view, p, 3, color, -1);
    if (line.seed.x >= 0)
      cv::circle(r.search_view, line.seed, 8, color, 2);
    drawPath(r.search_view, prior[side], {200, 80, 200}, 1, true);
  }
  cv::Mat drawing = cv::Mat::zeros(h, w, CV_8UC3);
  for (const auto *line : lines) {
    drawPath(drawing, line->rejected, {0, 140, 255}, 1, true);
    drawPath(drawing, line->path,
             line->state == LineState::Detected ? cv::Scalar(0, 0, 255) : cv::Scalar(200, 80, 200),
             3, line->state != LineState::Detected);
  }
  if (dl != dr) // opposite boundary implied by the lane width
    drawPath(drawing, offset((dl ? r.left : r.right).path, float(dl ? width_ : -width_)),
             {0, 165, 255}, 2, true);
  if (r.valid) {
    drawPath(drawing, center, {255, 80, 0}, 3, r.state != "both");
    const cv::Point target(cvRound(r.target_x), cvRound(r.target_y));
    cv::line(drawing, {cvRound(cu), h - 1}, target, {255, 0, 255}, 1);
    cv::circle(drawing, target, 7, {0, 255, 0}, -1);
  }
  cv::line(drawing, {cvRound(cu), h - 1}, {cvRound(cu), h - 40}, {255, 0, 255}, 2);
  cv::Mat drawn;
  cv::cvtColor(drawing, drawn, cv::COLOR_BGR2GRAY);
  r.path_view = r.bev.clone();
  drawing.copyTo(r.path_view, drawn);
  drawing.copyTo(r.search_view, drawn);
  cv::putText(r.path_view, r.state, {10, 24}, cv::FONT_HERSHEY_SIMPLEX, .7, {0, 255, 255}, 2);
  cv::Mat projected, projected_mask;
  cv::warpPerspective(drawing, projected, Hinv_, {w, h});
  cv::cvtColor(projected, projected_mask, cv::COLOR_BGR2GRAY);
  r.overlay = r.original.clone();
  projected.copyTo(r.overlay, projected_mask);

  cv::Mat line_pixels = cv::Mat::zeros(h, w, CV_8U);
  for (const auto &pixels : used)
    for (const auto &p : pixels)
      line_pixels.at<uchar>(p) = 255;
  cv::warpPerspective(colours, r.color_mask, Hinv_, {w, h}, cv::INTER_NEAREST);
  cv::warpPerspective(mask, r.feature_mask, Hinv_, {w, h}, cv::INTER_NEAREST);
  cv::warpPerspective(line_pixels, r.line_mask, Hinv_, {w, h}, cv::INTER_NEAREST);
  cv::warpPerspective(yellow, r.yellow_mask, Hinv_, {w, h}, cv::INTER_NEAREST);
  r.processing_ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - start_clock).count();
  return r;
}
} // namespace lane_lite

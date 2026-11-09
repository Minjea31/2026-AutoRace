#include "autorace_lane_fusion/lane.hpp"
#include "autorace_lane_fusion/candidates.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <set>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace lane_lite {
using namespace geo;

const std::vector<const char *> &groups() {
  static const std::vector<const char *> g = {"입력", "ROI·BEV", "색상 (HSV)", "후보 (체인·윈도우)",
                                              "퓨전", "차로·추적", "출력 검증", "거리 보정"};
  return g;
}
const std::vector<Spec> &specs() {
  static const std::vector<Spec> s = {
      {"width", "입력", "처리 영상 너비(px). BEV도 같은 크기", 640, 160, 1280, 1},
      {"height", "입력", "처리 영상 높이(px)", 480, 120, 960, 1},
      {"roi_top", "ROI·BEV", "검출 영역 위쪽(0=영상 위)", .45, 0, 1, .01},
      {"roi_bottom", "ROI·BEV", "검출 영역 아래쪽", 1, 0, 1, .01},
      {"roi_left", "ROI·BEV", "검출 영역 왼쪽", 0, 0, 1, .01},
      {"roi_right", "ROI·BEV", "검출 영역 오른쪽", 1, 0, 1, .01},
      {"bev_top_y", "ROI·BEV", "BEV 사다리꼴 위쪽 높이", .495, 0, 1, .001},
      {"bev_bottom_y", "ROI·BEV", "BEV 사다리꼴 아래쪽 높이", 1, 0, 1, .001},
      {"bev_top_left", "ROI·BEV", "BEV 위 왼쪽 x", .29, -2, 1, .001},
      {"bev_top_right", "ROI·BEV", "BEV 위 오른쪽 x", .65, 0, 3, .001},
      {"bev_bottom_left", "ROI·BEV", "BEV 아래 왼쪽 x(화면 밖 가능)", -.64, -3, 1, .001},
      {"bev_bottom_right", "ROI·BEV", "BEV 아래 오른쪽 x(화면 밖 가능)", 1.62, 0, 4, .001},
      {"yellow_enabled", "색상 (HSV)", "HSV 노란 차선 사용: 0=끔, 1=켬", 1, 0, 1, 1},
      {"yellow_h_min", "색상 (HSV)", "노랑 Hue 하한(OpenCV 0-179)", 15, 0, 179, 1},
      {"yellow_h_max", "색상 (HSV)", "노랑 Hue 상한", 40, 0, 179, 1},
      {"yellow_s_min", "색상 (HSV)", "노랑 최소 채도(S)", 80, 0, 255, 1},
      {"yellow_v_min", "색상 (HSV)", "노랑 최소 밝기(V)", 80, 0, 255, 1},
      {"white_enabled", "색상 (HSV)", "HSV 흰 차선 사용: 0=끔, 1=켬", 1, 0, 1, 1},
      {"white_s_max", "색상 (HSV)", "흰색 최대 채도(S)", 70, 0, 255, 1},
      {"white_v_min", "색상 (HSV)", "흰색 최소 밝기(V), 절대 하한", 120, 0, 255, 1},
      {"white_tophat", "색상 (HSV)", "흰색이 주변 노면보다 밝아야 하는 정도(Top-hat)", 30, 0, 200, 1},
      {"white_max_width", "색상 (HSV)", "이보다 두꺼운 흰 덩어리(BEV px) 제거: 횡단보도·노면 글자. 0=끔", 0, 0,
       120, 1},
      {"white_max_length", "색상 (HSV)", "이보다 긴 흰 선(BEV px)은 실선으로 보고 제거(흰 점선만 사용). 0=끔", 0,
       0, 2000, 1},
      {"line_width_max", "색상 (HSV)", "BEV 차선 띠 최대 두께(px): Top-hat 크기, 두께 필터", 40, 4, 120, 1},
      {"min_area", "색상 (HSV)", "HSV 특징 연결요소 최소 면적(px²)", 60, 0, 3000, 1},
      {"cell", "후보 (체인·윈도우)", "중심점 격자 크기(BEV px). 선 두께보다 크게", 16, 4, 64, 1},
      {"min_cell_pixels", "후보 (체인·윈도우)", "격자 한 칸을 차선으로 인정할 최소 픽셀 수", 6, 1, 500, 1},
      {"max_gap", "후보 (체인·윈도우)", "이어 붙일 최대 간격(BEV px). 흰 점선 빈칸보다 크게, 차로 폭보다 작게", 110,
       10, 400, 1},
      {"max_turn_deg", "후보 (체인·윈도우)", "체인이 한 번에 꺾일 수 있는 최대 각도(도)", 35, 5, 90, 1},
      {"min_length", "후보 (체인·윈도우)", "후보로 인정할 최소 길이(BEV px)", 60, 10, 1000, 1},
      {"windows", "후보 (체인·윈도우)", "BEV 높이를 나눈 윈도우 수(윈도우 길이 = 높이/값)", 12, 4, 30, 1},
      {"margin", "후보 (체인·윈도우)", "윈도우 반폭(px). 지그재그 폭보다 넓게", 50, 10, 200, 1},
      {"min_pixels", "후보 (체인·윈도우)", "윈도우를 관측으로 인정할 최소 픽셀 수", 30, 3, 3000, 1},
      {"max_misses", "후보 (체인·윈도우)", "연속 미관측 윈도우 수가 이 값이면 탐색 종료", 3, 1, 15, 1},
      {"min_windows", "후보 (체인·윈도우)", "윈도우로 정제할 최소 관측 윈도우 수(모자라면 체인 피팅)", 3, 2, 20, 1},
      {"max_residual", "후보 (체인·윈도우)", "체인 곡선 맞춤 RMS 상한(px)", 6, 1, 60, .5},
      {"deep_max_age", "퓨전", "딥 마스크를 쓸 수 있는 최대 나이(s, 프레임 촬영 시각 기준). 넘으면 HSV만",
       .12, 0, 1, .01},
      {"hsv_min_quality", "퓨전",
       "딥으로 고른 경계의 품질이 이 값보다 낮거나 한쪽이 없으면 HSV 후보를 추가로 계산", .6, 0, 1, .05},
      {"hsv_penalty", "퓨전", "딥이 있을 때 HSV 단독 후보에 더하는 선택 비용(클수록 딥 우선)", .35, 0, 2, .05},
      {"merge_px", "퓨전", "딥·HSV 후보를 같은 선으로 볼 거리(BEV px)", 24, 2, 200, 1},
      {"lane_width_px", "차로·추적", "BEV에서 좌우 경계(선 중심) 사이 차로 폭(px)", 320, 40, 1200, 1},
      {"width_tolerance", "차로·추적", "차로 폭 허용 오차 비율", .35, .05, .9, .01},
      {"max_angle_deg", "차로·추적", "좌우 경계 방향 차이 허용치(도). 노랑+노랑 쌍은 제외(급커브)", 30, 5, 90,
       1},
      {"vehicle_center_u", "차로·추적", "BEV에서 차량 중심선 가로 위치(0-1)", .5, 0, 1, .001},
      {"lookahead_row", "차로·추적", "목표점 BEV 세로 위치(0=먼 곳, 1=차량 앞)", .55, .05, .98, .01},
      {"smoothing", "차로·추적", "20 Hz 기준 새 관측 반영 비율(1=필터 없음)", .6, .05, 1, .05},
      {"track_jump_px", "차로·추적", "한 프레임 경계 이동 허용치(px). 넘으면 3프레임 연속일 때만 교체", 60, 5,
       400, 1},
      {"track_memory", "차로·추적", "관측이 끊겨도 마지막 경계·출력을 유지하는 시간(s)", .5, 0, 3, .05},
      {"role_frames", "차로·추적",
       "한쪽만 보일 때 그 선을 이만큼 연속 추적했어야 중앙 경로를 만듦(역할 확인)", 3, 1, 30, 1},
      {"max_target_jump_m", "출력 검증", "20 Hz 기준 목표점 횡오차가 한 번에 바뀔 수 있는 양(m). 넘으면 3프레임 확인",
       .12, .01, 1, .01},
      {"max_curvature", "출력 검증", "발행 곡률 상한(1/m)", 3, .2, 20, .1},
      {"curvature_span_m", "출력 검증", "곡률을 원 맞춤할 중앙 경로 길이(m, 가까운 쪽부터)", .6, .15, 3, .05},
      {"lane_width_m", "거리 보정", "실제 차로 폭(m, 선 중심 사이). 가로 px→m 환산", .55, .1, 3, .01},
      {"bev_near_m", "거리 보정", "BEV 맨 아래 행의 base_link 전방 거리(m), 실측", .25, 0, 5, .01},
      {"bev_far_m", "거리 보정", "BEV 맨 위 행의 전방 거리(m), 실측", 1.2, .05, 10, .01},
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
  if (c["roi_top"] + .05 >= c["roi_bottom"] || c["roi_left"] + .05 >= c["roi_right"] ||
      c["bev_top_y"] + .05 >= c["bev_bottom_y"] || c["bev_top_left"] + .05 >= c["bev_top_right"] ||
      c["bev_bottom_left"] + .05 >= c["bev_bottom_right"] || c["yellow_h_min"] > c["yellow_h_max"] ||
      c["bev_near_m"] + .05 >= c["bev_far_m"])
    throw std::invalid_argument("Inverted ROI/BEV/colour/distance limits");
}
Config Config::load(const std::string &path) {
  // Settings of the removed multi-mode / sliding-window pipeline.
  static const std::set<std::string> retired = {
      "deep_mode", "deep_dilate", "histogram_start", "role_by_color"};
  Config c;
  const auto root = YAML::LoadFile(path);
  const auto n = root["lane_lite"];
  if (!n || !n.IsMap())
    throw std::invalid_argument("YAML needs a lane_lite map: " + path);
  for (const auto &kv : n) {
    const auto key = kv.first.as<std::string>();
    if (c.values.count(key))
      c.values[key] = kv.second.as<double>();
    else if (!retired.count(key))
      throw std::invalid_argument("Unknown lane_lite parameter: " + key);
  }
  c.validate();
  return c;
}
void Config::save(const std::string &path) const {
  validate();
  YAML::Emitter e;
  e << YAML::BeginMap << YAML::Key << "lane_lite" << YAML::Value << YAML::BeginMap;
  for (const auto &s : specs())
    e << YAML::Key << s.name << YAML::Value << values.at(s.name);
  e << YAML::EndMap << YAML::EndMap;
  std::ofstream f(path);
  f << e.c_str() << '\n';
  if (!f)
    throw std::runtime_error("Cannot save YAML: " + path);
}
const char *name(LineState state) {
  return state == LineState::Detected ? "DETECTED" : state == LineState::Predicted ? "HELD" : "LOST";
}
const char *sourceName(int source) {
  return source == (kDeep | kHsv) ? "deep+hsv" : source == kDeep ? "deep" : source == kHsv ? "hsv" : "none";
}

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
  const std::vector<cv::Point2f> dst = {{0, 0}, {float(w - 1), 0}, {float(w - 1), float(h - 1)}, {0, float(h - 1)}};
  H_ = cv::getPerspectiveTransform(src, dst);
  Hinv_ = H_.inv();
  // Usable BEV area: inside the camera image and the ROI.
  cv::Mat roi = cv::Mat::zeros(h, w, CV_8U);
  cv::rectangle(roi, {cvRound(c["roi_left"] * (w - 1)), cvRound(c["roi_top"] * (h - 1))},
                {cvRound(c["roi_right"] * (w - 1)), cvRound(c["roi_bottom"] * (h - 1))}, 255, cv::FILLED);
  cv::warpPerspective(roi, bev_valid_, H_, {w, h}, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
  cv::erode(bev_valid_, bev_valid_, cv::Mat(), {-1, -1}, 2);
  reset();
}
void Pipeline::reset() {
  tracks_[0] = tracks_[1] = {};
  center_.clear();
  center_seen_ = last_time_ = held_time_ = -1;
  center_confidence_ = 0;
  out_jumps_ = 0;
  base_side_ = -1;
  held_ = {};
  motion_[0] = motion_[1] = motion_[2] = 0;
  width_ = config_["lane_width_px"];
}
void Pipeline::move(double forward_m, double yaw_rad) {
  if (!std::isfinite(forward_m) || !std::isfinite(yaw_rad))
    return;
  const double mid = motion_[2] + yaw_rad / 2; // arc: chord along the mid heading
  motion_[0] += forward_m * std::cos(mid);
  motion_[1] += forward_m * std::sin(mid);
  motion_[2] += yaw_rad;
}

Result Pipeline::process(const Image &image, double time, bool views, const cv::Mat &deep) {
  const auto start_clock = std::chrono::steady_clock::now();
  if (!std::isfinite(time))
    throw std::invalid_argument("Expected a finite time");
  if (!deep.empty() && deep.type() != CV_8U)
    throw std::invalid_argument("Deep mask must be CV_8U class ids");
  const auto &c = config_;
  if (last_time_ >= 0 && (time <= last_time_ || time - last_time_ > std::max(.5, c["track_memory"])))
    reset();
  const double dt = last_time_ < 0 ? .05 : std::clamp(time - last_time_, .001, .2);
  last_time_ = time;
  const int w = c["width"], h = c["height"];
  const double cu = c["vehicle_center_u"] * (w - 1), tol = c["width_tolerance"];
  const float gate = float(c["track_jump_px"]);
  const cv::Point2f vehicle(float(cu), float(h - 1));
  // 0. Odometry since the previous frame moves every remembered path (both
  // boundary tracks and the centre) into the current vehicle frame, so the
  // left/right continuity gates and a held centre stay on the road while the
  // car moves or turns. Points now behind the BEV bottom row are dropped.
  if (motion_[0] != 0 || motion_[1] != 0 || motion_[2] != 0) {
    const double mpx = c["lane_width_m"] / c["lane_width_px"];
    const double mrow = (c["bev_far_m"] - c["bev_near_m"]) / (h - 1);
    const double cs = std::cos(motion_[2]), sn = std::sin(motion_[2]);
    auto shift = [&](Path &path) {
      Path moved;
      for (const auto &p : path) {
        const double f = c["bev_near_m"] + (h - 1 - p.y) * mrow - motion_[0];
        const double l = (cu - p.x) * mpx - motion_[1];
        const double y = h - 1 - (cs * f + sn * l - c["bev_near_m"]) / mrow;
        if (y <= h - 1)
          moved.emplace_back(float(cu - (-sn * f + cs * l) / mpx), float(y));
      }
      path = moved.size() >= 2 ? moved : Path();
    };
    for (auto &track : tracks_)
      shift(track.path);
    shift(center_);
    // The output gate's last good target too (else a real turn looks like a jump).
    if (held_time_ >= 0) {
      auto point = [&](double &fwd, double &left) {
        const double f = fwd - motion_[0], l = left - motion_[1];
        fwd = cs * f + sn * l;
        left = -sn * f + cs * l;
      };
      point(held_.lookahead_m, held_.lateral_m);
      point(held_.near_m, held_.near_lateral_m);
      held_.heading -= motion_[2];
      held_.near_heading -= motion_[2];
      shift(held_.center_path);
      held_.target_x = cu - held_.lateral_m / mpx;
      held_.target_y = h - 1 - (held_.lookahead_m - c["bev_near_m"]) / mrow;
      held_.lateral_px = cu - held_.target_x;
    }
    motion_[0] = motion_[1] = motion_[2] = 0;
  }
  constexpr double kInf = std::numeric_limits<double>::infinity();
  Result r;

  // The picture is decoded/resized only when HSV or the views need it.
  bool asked = false;
  auto original = [&]() -> const cv::Mat & {
    if (!asked) {
      asked = true;
      const cv::Mat bgr = image ? image() : cv::Mat();
      if (!bgr.empty()) {
        if (bgr.type() != CV_8UC3)
          throw std::invalid_argument("Expected a BGR8 image");
        cv::resize(bgr, r.original, {w, h});
        r.image_used = true;
      }
    }
    return r.original;
  };
  auto colourBev = [&]() -> const cv::Mat & {
    if (r.bev.empty() && !original().empty())
      cv::warpPerspective(r.original, r.bev, H_, {w, h}, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    return r.bev;
  };
  Path prior[2];
  for (int side = 0; side < 2; ++side)
    if (!tracks_[side].path.empty() && time - tracks_[side].seen <= c["track_memory"])
      prior[side] = tracks_[side].path;

  // 1. Deep candidates.
  cv::Mat deep_bev, hsv_bev, colours;
  if (!deep.empty()) {
    cv::Mat sized = deep;
    if (deep.cols != w || deep.rows != h)
      cv::resize(deep, sized, {w, h}, 0, 0, cv::INTER_NEAREST);
    cv::warpPerspective(sized, deep_bev, H_, {w, h}, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
    deep_bev.setTo(0, bev_valid_ == 0);
    r.candidates = buildCandidates(deep_bev, kDeep, vehicle, c);
    r.deep_used = true;
    if (views)
      r.deep_mask = sized;
  }

  // 3. Left/right: one cost over all (left, right) choices incl. "none".
  int pick[2] = {-1, -1};
  std::string why[2];
  auto select = [&]() {
    const size_t n = r.candidates.size();
    std::vector<float> inner(n);
    std::vector<float> cont[2] = {std::vector<float>(n, NAN), std::vector<float>(n, NAN)};
    for (size_t i = 0; i < n; ++i) {
      inner[i] = sideOf(r.candidates[i].path, vehicle);
      for (int s = 0; s < 2; ++s)
        if (!prior[s].empty())
          cont[s][i] = distanceTo(r.candidates[i].path, prior[s]);
    }
    // Cost of candidate i as side s (inf: not allowed).
    auto unary = [&](size_t i, int s) -> double {
      const auto &k = r.candidates[i];
      const bool track = !std::isnan(cont[s][i]) && cont[s][i] < 1.5f * gate;
      // The vehicle lies right of a left boundary; a tracked line keeps its
      // role while the vehicle straddles it (no flip on the line itself).
      const bool side_ok = s == 0 ? inner[i] > 0 : inner[i] < 0;
      if (!side_ok && !(track && std::abs(inner[i]) < .5 * width_))
        return kInf;
      if (std::abs(inner[i]) > 1.5 * width_)
        return kInf; // beyond the neighbouring lane
      if (k.cls == 2 && transverse(k.path) && !track)
        return kInf; // white line across the road: stop line / crosswalk edge
      double u = track ? .5 * cont[s][i] / gate : prior[s].empty() ? .6 : 1.2;
      u += .6 * std::abs(std::abs(inner[i]) - .5 * width_) / width_;
      if (r.deep_used)
        u += k.source == kHsv ? c["hsv_penalty"] : k.source == (kDeep | kHsv) ? -.1 : 0;
      // Yellow paint is a solid boundary; a white piece is more often a
      // fragment (stop line, crosswalk, the other lane's dash): tie-break.
      return u + .4 * (1 - k.quality) + (k.cls == 2 ? .05 : 0);
    };
    auto pairCost = [&](size_t a, size_t b) -> double { // a left, b right
      const auto &A = r.candidates[a].path, &B = r.candidates[b].path;
      const auto ws = separation(A, B);
      const float m = ws.empty() ? (B.front() - A.front()).dot(rightOf(tangent(A, 0))) : median(ws);
      if (!(m > width_ * (1 - tol) && m < width_ * (1 + tol)))
        return kInf;
      // Two yellow boundaries of a tight bend legitimately differ in BEV.
      const float angle = angleBetween(A, B);
      if (!(r.candidates[a].cls == 1 && r.candidates[b].cls == 1) && angle > c["max_angle_deg"])
        return kInf;
      return .8 * std::abs(m - width_) / (tol * width_) + .3 * std::min(1., angle / c["max_angle_deg"]);
    };
    constexpr double kNone = 1.6;
    std::vector<double> u[2] = {std::vector<double>(n), std::vector<double>(n)};
    for (size_t i = 0; i < n; ++i)
      u[0][i] = unary(i, 0), u[1][i] = unary(i, 1);
    double best = 2 * kNone;
    pick[0] = pick[1] = -1;
    for (size_t i = 0; i < n; ++i)
      for (int s = 0; s < 2; ++s)
        if (u[s][i] + kNone < best) {
          best = u[s][i] + kNone;
          pick[s] = int(i), pick[1 - s] = -1;
        }
    for (size_t a = 0; a < n; ++a)
      for (size_t b = 0; b < n; ++b)
        if (a != b && std::isfinite(u[0][a]) && std::isfinite(u[1][b]) && u[0][a] + u[1][b] < best) {
          const double cost = u[0][a] + u[1][b] + pairCost(a, b);
          if (cost < best)
            best = cost, pick[0] = int(a), pick[1] = int(b);
        }
    for (int s = 0; s < 2; ++s) {
      why[s].clear();
      if (pick[s] < 0)
        why[s] = n ? "조건에 맞는 후보 없음" : "후보 없음";
    }
  };
  select();

  // 2. HSV only when the deep result needs it: no mask, a side missing, or a
  // weak deep side. Candidates are compared, never mask-merged.
  const bool hsv_on = c["yellow_enabled"] > 0 || c["white_enabled"] > 0;
  auto weak = [&](int s) { return pick[s] < 0 || r.candidates[pick[s]].quality < c["hsv_min_quality"]; };
  if (hsv_on && (deep.empty() || weak(0) || weak(1)) && !colourBev().empty()) {
    hsv_bev = hsvClasses(colourBev(), bev_valid_, c, views ? &colours : nullptr);
    const size_t deep_count = r.candidates.size();
    for (auto &k : buildCandidates(hsv_bev, kHsv, vehicle, c)) {
      int same = -1;
      for (size_t d = 0; d < deep_count && same < 0; ++d)
        if (distanceTo(k.path, r.candidates[d].path) < c["merge_px"])
          same = int(d);
      if (same >= 0) { // HSV confirms a deep boundary
        r.candidates[same].source |= kHsv;
        r.candidates[same].quality = std::max(r.candidates[same].quality, k.quality);
      } else {
        r.candidates.push_back(k);
      }
    }
    r.hsv_used = true;
    select();
  }

  Line *lines[2] = {&r.left, &r.right};
  for (int s = 0; s < 2; ++s) {
    auto &line = *lines[s];
    if (pick[s] < 0) {
      line.note = why[s];
      continue;
    }
    const auto &k = r.candidates[pick[s]];
    line.path = k.path;
    line.state = LineState::Detected;
    line.quality = k.quality;
    line.residual = k.residual;
    line.observed = int(k.points.size());
    line.source = k.source;
    line.yellow = k.cls == 1;
  }
  if (r.left.state == LineState::Detected && r.right.state == LineState::Detected) {
    // Learn the width only near the calibration (+-10 %).
    const double base = c["lane_width_px"], m = median(separation(r.left.path, r.right.path));
    if (std::isfinite(m) && std::abs(m - base) <= .5 * tol * base)
      width_ = std::clamp(width_ + .1 * (m - width_), .9 * base, 1.1 * base);
  }

  // 4. Tracking: smooth, confirm jumps / class changes over 3 frames, or hold.
  const double alpha = 1 - std::pow(1 - c["smoothing"], dt / .05);
  for (int s = 0; s < 2; ++s) {
    auto &line = *lines[s];
    auto &track = tracks_[s];
    bool accept = line.state == LineState::Detected, relock = false;
    if (accept && !prior[s].empty()) {
      double jump = distanceTo(line.path, prior[s]);
      if (track.cls && track.cls != (line.yellow ? 1 : 2))
        jump = std::max(jump, c["track_jump_px"] + 1);
      if (jump > c["track_jump_px"]) {
        relock = ++track.jumps >= 3;
        if (!relock) {
          accept = false;
          line.rejected = line.path;
          line.note = "급격한 이동 " + std::to_string(int(jump)) + " px, 확인 " + std::to_string(track.jumps) + "/3";
        }
      } else {
        track.jumps = 0;
      }
    }
    if (accept) {
      const bool continued = !prior[s].empty() && !relock;
      track.path = continued ? blend(line.path, prior[s], alpha, gate) : line.path;
      track.frames = continued ? track.frames + 1 : 1;
      track.seen = time;
      track.quality = line.quality;
      track.cls = line.yellow ? 1 : 2;
      if (relock)
        track.jumps = 0;
      line.path = track.path;
      line.frames = track.frames;
    } else if (!prior[s].empty()) {
      line.path = prior[s];
      line.state = LineState::Predicted;
      line.age = time - track.seen;
      line.frames = track.frames;
      line.quality = .5 * track.quality * (1 - line.age / std::max(.01, c["track_memory"]));
    } else {
      track = {};
      line.path.clear();
      line.state = LineState::Lost;
    }
  }

  // 5. Centre path. Both sides: corresponding points across the lane. One
  // side: only when its role is certain (tracked for role_frames, or the
  // vehicle sits clearly inside it), else nothing new goes to the controller.
  bool dl = r.left.state == LineState::Detected, dr = r.right.state == LineState::Detected;
  if (dl != dr) {
    const Line &one = dl ? r.left : r.right;
    const float inside = std::abs(sideOf(one.path, vehicle));
    if (one.frames < c["role_frames"] && !(inside > .25 * width_ && inside < .75 * width_)) {
      r.notes.push_back(std::string(dl ? "왼쪽" : "오른쪽") + "만 보이고 역할이 불확실해 중앙 경로 보류");
      dl = dr = false;
    }
  }
  Path center;
  double confidence = 0;
  if (dl || dr) {
    // One base boundary carries the centre; the other only corrects the
    // width where it is seen. The base keeps its side while it is detected,
    // so both <-> one-side transitions do not move the centre.
    if (dl && dr) {
      if (base_side_ < 0 || lines[base_side_]->state != LineState::Detected)
        base_side_ = r.left.frames != r.right.frames ? (r.left.frames > r.right.frames ? 0 : 1)
                     : r.left.path.size() >= r.right.path.size() ? 0 : 1;
    } else {
      base_side_ = dl ? 0 : 1;
    }
    const bool left_base = base_side_ == 0;
    const Line &base = left_base ? r.left : r.right, &other = left_base ? r.right : r.left;
    center = centreLine(base.path, dl && dr ? other.path : Path(), left_base ? 1.f : -1.f, float(width_),
                        float(tol));
    if (center.size() >= 4) { // the offset of a ragged boundary zigzags
      Path smooth;
      double rms;
      if (fitPath(center, std::vector<float>(center.size(), 1.f), 1e9, smooth, rms))
        center = smooth;
    }
    confidence = dl && dr ? (r.left.quality + r.right.quality) / 2 : .7 * base.quality;
    r.state = dl && dr ? "both" : dl ? "left only" : "right only";
  }
  // A fresh centre too short to give a target (stop line, one short dash)
  // does not replace a recent path: it is treated like a dropout.
  const bool usable = center.size() >= 3 && pathLength(center) >= .1 * h;
  const bool recent = !center_.empty() && time - center_seen_ <= c["track_memory"];
  if ((dl || dr) && (usable || !recent)) {
    // Reacquisition blends with the (motion-compensated) held path: no jump.
    if (recent)
      center = blend(center, center_, alpha, gate);
    center_ = center;
    center_seen_ = time;
    center_confidence_ = confidence;
  } else if (recent) {
    if (dl || dr)
      r.notes.push_back("중앙 경로가 너무 짧아 이전 경로 유지");
    base_side_ = -1;
    center = center_; // already moved with the car (step 0)
    confidence = center_confidence_ * (1 - (time - center_seen_) / std::max(.01, c["track_memory"]));
    r.state = "held";
  } else {
    center.clear();
    center_.clear();
    r.state = "lost";
  }
  r.lane_width_px = width_;
  int used_source = 0;
  for (const auto *line : lines)
    if (line->state == LineState::Detected)
      used_source |= line->source;
  r.source = sourceName(used_source);

  // 6. Lookahead target (straight-line distance lookahead_row ahead) and metrics.
  const double m_px = c["lane_width_m"] / c["lane_width_px"];
  const double m_row = (c["bev_far_m"] - c["bev_near_m"]) / (h - 1);
  auto metric = [&](cv::Point2f p) { // forward, left (m) from base_link
    return cv::Point2d(c["bev_near_m"] + (h - 1 - p.y) * m_row, (cu - p.x) * m_px);
  };
  if (center.size() >= 3 && pathLength(center) >= .1 * h) {
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

  // 7. Output gate: a centre path off the lane, or a target that jumps more
  // than a car can move (unconfirmed for 3 frames), never reaches the
  // controller; the last good output is held for track_memory instead.
  auto copyOutput = [](const Result &from, Result &to) {
    to.valid = from.valid, to.center_path = from.center_path, to.confidence = from.confidence;
    to.target_x = from.target_x, to.target_y = from.target_y, to.lateral_px = from.lateral_px;
    to.lateral_m = from.lateral_m, to.lookahead_m = from.lookahead_m, to.heading = from.heading;
    to.curvature = from.curvature, to.near_lateral_m = from.near_lateral_m, to.near_m = from.near_m;
    to.near_heading = from.near_heading;
  };
  const bool fresh = r.valid && r.state != "held";
  if (fresh) {
    std::string reject;
    if (std::abs(r.near_lateral_m) > .9 * c["lane_width_m"])
      reject = "중앙 경로의 가까운 점이 차로 밖 (" + std::to_string(r.near_lateral_m).substr(0, 5) + " m)";
    const bool recent = held_time_ >= 0 && time - held_time_ <= c["track_memory"];
    // Last good target extended along its heading to this lookahead distance
    // (odometry may have moved it closer to the car).
    const double expected = held_.lateral_m + std::tan(std::clamp(held_.heading, -1.2, 1.2)) *
                                                  std::clamp(r.lookahead_m - held_.lookahead_m, -.3, .3);
    if (reject.empty() && recent &&
        std::abs(r.lateral_m - expected) > c["max_target_jump_m"] * std::max(1., dt / .05)) {
      if (++out_jumps_ < 3)
        reject = "목표점 급변 " + std::to_string(int(std::abs(r.lateral_m - expected) * 100)) +
                 " cm, 확인 " + std::to_string(out_jumps_) + "/3";
    }
    if (reject.empty()) {
      out_jumps_ = 0;
      held_ = {};
      copyOutput(r, held_);
      held_time_ = time;
    } else {
      r.notes.push_back("출력 보류: " + reject);
      if (recent) {
        copyOutput(held_, r);
        r.confidence = held_.confidence * (1 - (time - held_time_) / std::max(.01, c["track_memory"]));
        r.state = "held";
      } else {
        r.valid = false;
        r.state = "rejected";
      }
    }
  }
  for (const auto *line : lines)
    if (!line->note.empty())
      r.notes.push_back(std::string(line == &r.left ? "왼쪽: " : "오른쪽: ") + line->note);

  // 8. Views: candidates (deep / HSV), boundaries, centre and target.
  if (views) {
    const cv::Mat &bev = colourBev();
    const cv::Mat base = bev.empty() ? cv::Mat(h, w, CV_8UC3, cv::Scalar(30, 30, 30)) : bev;
    r.bev_mask = cv::Mat::zeros(h, w, CV_8U);
    if (!deep_bev.empty())
      r.bev_mask.setTo(255, deep_bev > 0);
    if (!hsv_bev.empty())
      r.bev_mask.setTo(255, hsv_bev > 0);
    r.search_view = base * .45;
    if (!hsv_bev.empty()) {
      r.search_view.setTo(cv::Scalar(0, 110, 140), hsv_bev == 1);
      r.search_view.setTo(cv::Scalar(130, 130, 130), hsv_bev == 2);
    }
    if (!deep_bev.empty()) {
      r.search_view.setTo(cv::Scalar(0, 210, 255), deep_bev == 1);
      r.search_view.setTo(cv::Scalar(235, 235, 235), deep_bev == 2);
    }
    for (const auto &k : r.candidates) {
      const cv::Scalar colour = k.source == kHsv ? cv::Scalar(0, 140, 255) : cv::Scalar(200, 60, 200);
      drawPath(r.search_view, k.path, colour, 1, k.source == kHsv);
      for (const auto &p : k.points)
        cv::circle(r.search_view, p, 2, colour, -1);
    }
    for (int s = 0; s < 2; ++s)
      drawPath(r.search_view, prior[s], {150, 150, 150}, 1, true);
    cv::Mat drawing = cv::Mat::zeros(h, w, CV_8UC3), used = cv::Mat::zeros(h, w, CV_8U);
    const cv::Scalar side_colour[2] = {{0, 255, 0}, {255, 200, 0}};
    for (int s = 0; s < 2; ++s) {
      const auto &line = *lines[s];
      drawPath(drawing, line.rejected, {0, 140, 255}, 1, true);
      drawPath(drawing, line.path, line.state == LineState::Detected ? side_colour[s] : cv::Scalar(200, 80, 200), 3,
               line.state != LineState::Detected);
      if (line.state == LineState::Detected)
        drawPath(used, line.path, 255, int(c["cell"]) / 2);
    }
    if (dl != dr)
      drawPath(drawing, offset((dl ? r.left : r.right).path, float(dl ? width_ : -width_)), {0, 165, 255}, 2, true);
    if (r.valid) {
      drawPath(drawing, r.center_path, {0, 0, 255}, 3, r.state != "both");
      const cv::Point target(cvRound(r.target_x), cvRound(r.target_y));
      cv::line(drawing, {cvRound(cu), h - 1}, target, {255, 0, 255}, 1);
      cv::circle(drawing, target, 7, {0, 255, 0}, -1);
    }
    cv::line(drawing, {cvRound(cu), h - 1}, {cvRound(cu), h - 40}, {255, 0, 255}, 2);
    cv::Mat drawn;
    cv::cvtColor(drawing, drawn, cv::COLOR_BGR2GRAY);
    r.path_view = base.clone();
    drawing.copyTo(r.path_view, drawn);
    drawing.copyTo(r.search_view, drawn);
    const std::string label = r.state + " | " + r.source + (r.hsv_used ? " | HSV on" : "");
    cv::putText(r.path_view, label, {10, 24}, cv::FONT_HERSHEY_SIMPLEX, .7, {0, 255, 255}, 2);
    cv::putText(r.search_view, label, {10, 24}, cv::FONT_HERSHEY_SIMPLEX, .7, {0, 255, 255}, 2);
    const cv::Mat &orig = original();
    r.overlay = orig.empty() ? cv::Mat(h, w, CV_8UC3, cv::Scalar(30, 30, 30)) : orig.clone();
    cv::Mat projected, projected_mask;
    cv::warpPerspective(drawing, projected, Hinv_, {w, h});
    cv::cvtColor(projected, projected_mask, cv::COLOR_BGR2GRAY);
    projected.copyTo(r.overlay, projected_mask);
    cv::Mat yellow_bev = cv::Mat::zeros(h, w, CV_8U);
    if (!deep_bev.empty())
      yellow_bev.setTo(255, deep_bev == 1);
    if (!hsv_bev.empty())
      yellow_bev.setTo(255, hsv_bev == 1);
    if (colours.empty()) {
      r.color_mask = cv::Mat::zeros(h, w, CV_8U);
      cv::putText(r.color_mask, "HSV not needed (deep)", {12, 60}, cv::FONT_HERSHEY_SIMPLEX, .8, 120, 2);
    } else {
      cv::warpPerspective(colours, r.color_mask, Hinv_, {w, h}, cv::INTER_NEAREST);
    }
    cv::warpPerspective(r.bev_mask, r.feature_mask, Hinv_, {w, h}, cv::INTER_NEAREST);
    cv::warpPerspective(used, r.line_mask, Hinv_, {w, h}, cv::INTER_NEAREST);
    cv::warpPerspective(yellow_bev, r.yellow_mask, Hinv_, {w, h}, cv::INTER_NEAREST);
    if (r.bev.empty())
      r.bev = base;
  }
  r.processing_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_clock).count();
  return r;
}
} // namespace lane_lite

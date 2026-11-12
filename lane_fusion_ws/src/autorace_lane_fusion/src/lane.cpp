#include "autorace_lane_fusion/lane.hpp"
#include "autorace_lane_fusion/candidates.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <set>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace lane_lite {
using namespace geo;

const std::vector<const char *> &groups() {
  static const std::vector<const char *> g = {"입력", "ROI·BEV", "색상 (HSV)", "Sliding Window",
                                              "퓨전", "차로·추적", "출력", "갈림길", "거리 보정"};
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
      {"line_width_max", "색상 (HSV)", "BEV 차선 띠 최대 두께(px): Top-hat 크기, 두께 필터, 윈도우 안 띠 폭", 40, 4,
       120, 1},
      {"min_area", "색상 (HSV)", "HSV 특징 연결요소 최소 면적(px²)", 60, 0, 3000, 1},
      {"windows", "Sliding Window", "아래→위 윈도우 개수(윈도우 길이 = BEV 높이/값)", 12, 4, 30, 1},
      {"margin", "Sliding Window", "윈도우 반폭(px)", 50, 10, 200, 1},
      {"min_pixels", "Sliding Window", "윈도우를 관측으로 인정할 최소 픽셀 수", 30, 3, 3000, 1},
      {"max_misses", "Sliding Window", "연속 미관측 윈도우 수가 이 값이면 탐색 종료", 3, 1, 15, 1},
      {"min_windows", "Sliding Window", "차선으로 인정할 최소 관측 윈도우 수", 3, 2, 20, 1},
      {"max_residual", "Sliding Window", "윈도우 중심 곡선 맞춤 RMS 상한(px)", 10, 1, 60, .5},
      {"histogram_start", "Sliding Window", "시작점 히스토그램을 계산할 BEV 하단 영역의 시작(0=맨 위)", .5, 0, .95,
       .01},
      {"deep_max_age", "퓨전",
       "딥 마스크를 쓸 수 있는 최대 나이(s, 촬영 시각 차). 차량 이동량(odom)으로 현재 프레임에 정합. odom이 없으면 "
       "0.04 s까지만",
       .2, 0, 1, .01},
      {"merge_px", "퓨전", "딥·HSV 후보를 같은 선으로 볼 거리(BEV px). 같으면 두 검출기가 서로 확인", 24, 2, 200, 1},
      {"lane_width_px", "차로·추적", "BEV에서 좌우 경계(선 중심) 사이 차로 폭(px)", 320, 40, 1200, 1},
      {"width_tolerance", "차로·추적", "차로 폭 허용 오차 비율", .35, .05, .9, .01},
      {"max_angle_deg", "차로·추적", "좌우 경계 방향 차이 허용치(도). 노랑+노랑 쌍은 제외(급커브)", 30, 5, 90,
       1},
      {"vehicle_center_u", "차로·추적", "BEV에서 차량 중심선 가로 위치(0-1)", .5, 0, 1, .001},
      {"smoothing", "차로·추적", "20 Hz 기준 새 관측 반영 비율(1=필터 없음). 경계·중앙 경로에 적용, track_jump_px 넘는 변화는 그대로", .6, .05, 1,
       .05},
      {"track_jump_px", "차로·추적", "한 프레임 경계 이동 허용치(px). 넘으면 3프레임 연속일 때만 교체", 60, 5,
       400, 1},
      {"track_memory", "차로·추적", "관측이 끊겨도 마지막 경계·중앙 경로를 유지하는 시간(s). 지나면 무효(정지)", .5,
       0, 3, .05},
      {"curve_lane_width_m", "차로·추적",
       "우회전 커브의 차로 폭(m, 최대). 왼쪽 선만 보이고 그 선이 오른쪽으로 꺾일수록 중앙 경로를 만드는 폭을 "
       "lane_width_m에서 이 값까지 늘림(바깥 선에 붙지 않게). 0 = 끔",
       0, 0, 2, .01},
      {"curve_angle_deg", "차로·추적",
       "왼쪽 선 방향(차 기준, 오른쪽 +)이 이 각도면 curve_lane_width_m 전부 적용. 이 값의 1/3 이하는 직선 폭", 45, 5, 90,
       1},
      {"curve_memory", "차로·추적",
       "커브(왼쪽 선만, 오른쪽으로 꺾임) 중과 그 뒤 이 시간(s) 동안 왼쪽 선을 odom으로 옮기며 기억하고, 그보다 "
       "반 차로폭 넘게 바깥(왼쪽)에 있는 선(이미 지나온 차로)은 경계로 쓰지 않음. 0 = 끔",
       0, 0, 10, .1},
      {"role_frames", "차로·추적",
       "한쪽만 보일 때 그 선을 이만큼 연속 추적했어야 중앙 경로를 만듦(역할 확인)", 3, 1, 30, 1},
      {"lookahead_row", "출력", "목표점 BEV 세로 위치(0=먼 곳, 1=차량 앞)", .55, .05, .98, .01},
      {"max_curvature", "출력", "발행 곡률 상한(1/m)", 3, .2, 20, .1},
      {"curvature_span_m", "출력", "곡률을 원 맞춤할 중앙 경로 길이(m, 가까운 쪽부터)", .6, .15, 3, .05},
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
  // Settings of earlier fusion versions (multi-mode, cell chains, deep-first
  // HSV switching, output gate): ignored so older YAML files still load.
  static const std::set<std::string> retired = {
      "deep_mode", "deep_dilate", "role_by_color", "cell", "min_cell_pixels", "max_gap", "max_turn_deg",
      "min_length", "hsv_min_quality", "hsv_penalty", "max_target_jump_m",
      // 2026-11-12: fork (A/B) logic removed from the lane node, to be redone
      "fork_length", "fork_timeout", "fork_default", "fork_shift", "fork_shift_rate", "fork_steer_max",
      "fork_steer_time", "fork_default_wait", "fork_island_px"};
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
  center_seen_ = last_time_ = -1;
  center_confidence_ = 0;
  curve_k_ = 0;
  curve_until_ = odom_seen_ = -1;
  curve_left_.clear();
  base_side_ = -1;
  motion_[0] = motion_[1] = motion_[2] = 0;
  width_ = config_["lane_width_px"];
}
void Pipeline::move(double dx, double dy, double dyaw) {
  if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dyaw))
    return;
  const double cs = std::cos(motion_[2]), sn = std::sin(motion_[2]);
  motion_[0] += cs * dx - sn * dy;
  motion_[1] += sn * dx + cs * dy;
  motion_[2] += dyaw;
}
void Pipeline::move(double forward_m, double yaw_rad) { // chord of the arc, along the mid heading
  move(forward_m * std::cos(yaw_rad / 2), forward_m * std::sin(yaw_rad / 2), yaw_rad);
}

Result Pipeline::process(const cv::Mat &bgr, double time, bool views, const DeepInput &deep) {
  const auto start_clock = std::chrono::steady_clock::now();
  if (bgr.empty() || bgr.type() != CV_8UC3 || !std::isfinite(time))
    throw std::invalid_argument("Expected a BGR8 image and finite time");
  if (!deep.mask.empty() && deep.mask.type() != CV_8U)
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
  const double mpx = c["lane_width_m"] / c["lane_width_px"];
  const double mrow = (c["bev_far_m"] - c["bev_near_m"]) / (h - 1);
  // BEV pixel <-> metric (forward, left from base_link) as one affine map.
  const cv::Matx33d to_metric(0, -mrow, c["bev_near_m"] + (h - 1) * mrow, -mpx, 0, cu * mpx, 0, 0, 1);
  // Car motion (dx fwd, dy left, dyaw) in the old frame: old metric -> new metric.
  auto motionMatrix = [](double dx, double dy, double dyaw) {
    const double cs = std::cos(dyaw), sn = std::sin(dyaw);
    return cv::Matx33d(cs, sn, -(cs * dx + sn * dy), -sn, cs, sn * dx - cs * dy, 0, 0, 1);
  };

  // 0. Odometry since the previous frame moves the remembered centre path into
  // the current vehicle frame (blend target, and the path held through a
  // dropout). The boundary tracks are NOT moved: on the car bags that cost
  // detections in bends (lane_ws behaviour kept). Points now behind the BEV
  // bottom row are dropped.
  if (motion_[0] != 0 || motion_[1] != 0 || motion_[2] != 0) {
    const cv::Matx33d m = to_metric.inv() * motionMatrix(motion_[0], motion_[1], motion_[2]) * to_metric;
    auto shift = [&](Path &path) {
      Path moved;
      for (const auto &p : path) {
        const cv::Vec3d q = m * cv::Vec3d(p.x, p.y, 1);
        if (q[1] <= h - 1)
          moved.emplace_back(float(q[0]), float(q[1]));
      }
      path = moved.size() >= 2 ? moved : Path();
    };
    shift(center_);
    shift(curve_left_);
    odom_seen_ = time;
    motion_[0] = motion_[1] = motion_[2] = 0;
  }
  constexpr double kInf = std::numeric_limits<double>::infinity();
  Result r;
  if (bgr.cols == w && bgr.rows == h)
    r.original = bgr; // no copy: views clone before drawing
  else
    cv::resize(bgr, r.original, {w, h});
  r.image_used = true;
  // Replicated borders keep the warp free of artificial black/bright edges.
  cv::warpPerspective(r.original, r.bev, H_, {w, h}, cv::INTER_LINEAR, cv::BORDER_REPLICATE);

  Path prior[2];
  for (int side = 0; side < 2; ++side)
    if (!tracks_[side].path.empty() && time - tracks_[side].seen <= c["track_memory"])
      prior[side] = tracks_[side].path;
  std::vector<Detection> found;

  // 1. HSV detection, every frame (the main detector).
  cv::Mat hsv_bev, deep_bev, colours;
  if (c["yellow_enabled"] > 0 || c["white_enabled"] > 0) {
    hsv_bev = hsvClasses(r.bev, bev_valid_, c, views ? &colours : nullptr);
    found.push_back(detect(hsv_bev, kHsv, prior, width_, c));
    r.hsv_used = true;
  }
  // 2. Deep detection on a recent mask, warped into THIS frame's BEV: mask
  // image -> BEV at the mask's capture -> car motion since then. A mask that
  // is too old, or older than one frame with unknown motion, is not used.
  if (!deep.mask.empty()) {
    const int age_ms = int(std::lround(deep.age * 1000));
    if (!(deep.age >= 0 && deep.age <= c["deep_max_age"])) {
      r.deep_note = "딥 마스크 " + std::to_string(age_ms) + " ms: deep_max_age 초과로 사용 안 함";
    } else if (!deep.moved && deep.age > kDeepStillAge) {
      r.deep_note = "딥 마스크 " + std::to_string(age_ms) + " ms: 차량 이동량(odom)이 없어 정합 불가, 사용 안 함";
    } else {
      cv::Matx33d warp = cv::Matx33d(H_) * cv::Matx33d(double(w) / deep.mask.cols, 0, 0, 0,
                                                        double(h) / deep.mask.rows, 0, 0, 0, 1);
      if (deep.moved)
        warp = to_metric.inv() * motionMatrix(deep.dx, deep.dy, deep.dyaw) * to_metric * warp;
      cv::warpPerspective(deep.mask, deep_bev, cv::Mat(warp), {w, h}, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
      deep_bev &= bev_valid_; // class ids 1/2 survive an AND with 255
      found.push_back(detect(deep_bev, kDeep, prior, width_, c));
      // An older measurement is a weaker one (alignment error grows with the
      // motion since its capture): its quality fades with the age.
      const double fade = std::clamp(1 - deep.age / std::max(.01, c["deep_max_age"]), 0., 1.);
      for (auto &k : found.back().side)
        k.quality *= fade;
      r.deep_used = true;
      r.deep_age = deep.age;
      r.deep_note = "딥 마스크 " + std::to_string(age_ms) + " ms" + (deep.moved ? " (odom 정합)" : "");
      if (views) {
        cv::Mat image_mask;
        cv::warpPerspective(deep_bev, image_mask, Hinv_, {w, h}, cv::INTER_NEAREST);
        r.deep_mask = image_mask;
      }
    }
  }

  // 3. Candidates of both sources. One line found by both (closer than
  // merge_px) becomes one candidate confirmed by two independent detectors.
  // A mask of (about) this frame is averaged with HSV where they overlap (the
  // longer gives the extent); an older, odometry-aligned mask only confirms
  // (on the car bags averaging it in added jitter) and fills a side HSV missed.
  bool forget[2] = {false, false};
  std::string why[2];
  for (const auto &d : found)
    for (int s = 0; s < 2; ++s) {
      forget[s] |= d.forget[s];
      if (!d.note[s].empty())
        why[s] += std::string(why[s].empty() ? "" : " / ") + sourceName(d.source) + ": " + d.note[s];
      if (d.side[s].path.empty())
        continue;
      bool merged = false;
      for (auto &k : r.candidates)
        if (!(k.source & d.side[s].source) &&
            distanceTo(d.side[s].path, k.path) < c["merge_px"]) {
          const auto &o = d.side[s];
          Candidate both = o.length > k.length ? o : k;
          if (r.deep_age > kDeepStillAge) // an aligned older mask confirms; this frame's HSV gives the path
            both.path = (k.source & kHsv ? k : o).path;
          else
            both.path = blend(both.path, o.length > k.length ? k.path : o.path, .5, float(c["merge_px"]));
          both.source = k.source | o.source;
          both.quality = std::max(k.quality, o.quality);
          both.observed = std::max(k.observed, o.observed);
          both.length = pathLength(both.path);
          k = both;
          merged = true;
          break;
        }
      if (!merged)
        r.candidates.push_back(d.side[s]);
    }

  // 4. Left/right: one cost over all (left, right) choices incl. "none". The
  // geometry decides (track continuity, side of the vehicle, lane width,
  // direction, fit quality, two-source agreement); the source does not.
  const size_t n = r.candidates.size();
  int pick[2] = {-1, -1};
  {
    std::vector<float> inner(n);
    std::vector<float> cont[2] = {std::vector<float>(n, NAN), std::vector<float>(n, NAN)};
    for (size_t i = 0; i < n; ++i) {
      inner[i] = sideOf(r.candidates[i].path, vehicle);
      for (int s = 0; s < 2; ++s)
        if (!prior[s].empty())
          cont[s][i] = distanceTo(r.candidates[i].path, prior[s]);
    }
    // After a right-hand bend the lane just driven runs parallel outside
    // (left of) the current left line (234303 bag, 46.5 s on). While in or
    // shortly after a bend, a line more than half a lane outside the
    // remembered left line is no boundary of this lane.
    std::vector<char> outside(n, 0);
    // Only with odometry: a remembered line that does not move with the car
    // would exclude the right lines.
    if (c["curve_memory"] > 0 && time <= curve_until_ && !curve_left_.empty() && odom_seen_ >= 0 &&
        time - odom_seen_ <= .2)
      for (size_t i = 0; i < n; ++i) {
        const auto gap = separation(r.candidates[i].path, curve_left_); // > 0: the remembered line lies right of it
        if (!gap.empty() && median(gap) > .5 * width_)
          outside[i] = 1;
      }
    auto unary = [&](size_t i, int s) -> double {
      const auto &k = r.candidates[i];
      if (outside[i])
        return kInf;
      const bool track = !std::isnan(cont[s][i]) && cont[s][i] < 1.5f * gate;
      // The vehicle lies right of a left boundary; a tracked line keeps its
      // role while the vehicle straddles it (no flip on the line itself).
      const bool side_ok = s == 0 ? inner[i] > 0 : inner[i] < 0;
      if (!side_ok && !(track && std::abs(inner[i]) < .5 * width_))
        return kInf;
      if (std::abs(inner[i]) > 1.5 * width_)
        return kInf; // beyond the neighbouring lane
      double u = track ? .5 * cont[s][i] / gate : prior[s].empty() ? .6 : 1.2;
      u += .6 * std::abs(std::abs(inner[i]) - .5 * width_) / width_;
      u += .4 * (1 - k.quality);
      if (k.source == (kDeep | kHsv))
        u -= .15;
      // Yellow paint is a solid boundary; a white piece is more often a
      // fragment (stop line, crosswalk, the other lane's dash): tie-break.
      return u + (k.cls == 2 ? .05 : 0);
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
    for (size_t i = 0; i < n; ++i)
      for (int s = 0; s < 2; ++s)
        if (u[s][i] + kNone < best)
          best = u[s][i] + kNone, pick[s] = int(i), pick[1 - s] = -1;
    for (size_t a = 0; a < n; ++a)
      for (size_t b = 0; b < n; ++b)
        if (a != b && u[0][a] + u[1][b] < best) {
          const double cost = u[0][a] + u[1][b] + pairCost(a, b);
          if (cost < best)
            best = cost, pick[0] = int(a), pick[1] = int(b);
        }
  }
  Line *lines[2] = {&r.left, &r.right};
  for (int s = 0; s < 2; ++s) {
    auto &line = *lines[s];
    if (pick[s] < 0) {
      line.note = why[s].empty() ? (n ? "조건에 맞는 후보 없음" : "후보 없음") : why[s];
      if (forget[s]) { // re-seed from the histogram, not around a stop line
        tracks_[s] = {};
        prior[s].clear();
      }
      continue;
    }
    const auto &k = r.candidates[pick[s]];
    line.path = k.path;
    line.state = LineState::Detected;
    line.quality = k.quality;
    line.residual = k.residual;
    line.observed = k.observed;
    line.source = k.source;
    line.yellow = k.cls == 1;
    line.near_white = k.near_cls == 2;
  }
  if (r.left.state == LineState::Detected && r.right.state == LineState::Detected) {
    // Learn the width only near the calibration (+-10 %): road text or arrows
    // paired with a boundary must not shrink the single-side offset.
    const double base = c["lane_width_px"], m = median(separation(r.left.path, r.right.path));
    if (std::isfinite(m) && std::abs(m - base) <= .5 * tol * base)
      width_ = std::clamp(width_ + .1 * (m - width_), .9 * base, 1.1 * base);
  }

  // 5. Tracking: smooth, confirm jumps / class changes over 3 frames, or hold.
  const double alpha = 1 - std::pow(1 - c["smoothing"], dt / .05);
  for (int s = 0; s < 2; ++s) {
    auto &line = *lines[s];
    auto &track = tracks_[s];
    bool accept = line.state == LineState::Detected, relock = false;
    if (accept && !prior[s].empty()) {
      double jump = distanceTo(line.path, prior[s]);
      if (track.cls && track.cls != (line.near_white ? 2 : 1))
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
      // The near half decides the boundary's colour (a dash continued by a
      // far yellow edge stays white).
      track.cls = line.near_white ? 2 : 1;
      if (relock)
        track.jumps = 0;
      line.path = track.path;
      line.frames = track.frames;
      if (s == 0)
        curve_left_ = track.path; // remembered (and moved with the car) for the bend
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

  // 6. Centre path. Both sides: corresponding points across the lane. One
  // side: only when its role is certain (tracked for role_frames, or the
  // vehicle sits clearly inside it). Held boundaries never mix in.
  bool dl = r.left.state == LineState::Detected, dr = r.right.state == LineState::Detected;
  if (dl != dr) {
    const Line &one = dl ? r.left : r.right;
    const float inside = std::abs(sideOf(one.path, vehicle));
    if (one.frames < c["role_frames"] && !(inside > .25 * width_ && inside < .75 * width_)) {
      r.notes.push_back(std::string(dl ? "왼쪽" : "오른쪽") + "만 보이고 역할이 불확실해 중앙 경로 보류");
      dl = dr = false;
    }
  }
  auto metric = [&](cv::Point2f p) { // forward, left (m) from base_link
    return cv::Point2d(c["bev_near_m"] + (h - 1 - p.y) * mrow, (cu - p.x) * mpx);
  };
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
    // No refit here: the boundaries are already fitted and
    // smoothed once, a further fit would straighten the bend.
    // The track's bends all turn right and are wider than the straights:
    // there only the outer (left) line is in view, and half the straight
    // width from it puts the centre against the outer line. The wider the
    // left line turns to the right (direction relative to the car), the
    // closer the width used gets to curve_lane_width_m.
    double width = width_;
    if (dl && !dr && c["curve_lane_width_m"] > c["lane_width_m"] && r.left.path.size() >= 2) {
      const cv::Point2f d = r.left.path.back() - r.left.path.front(); // near -> far
      const double angle = std::atan2(d.x, -d.y) * 180 / CV_PI;       // > 0: bends right
      const double full = c["curve_angle_deg"], start = full / 3;
      // Low-passed (time constant 0.3 s): the end-to-end direction of a
      // fitted line wobbles from frame to frame.
      const double k = std::clamp((angle - start) / (full - start), 0., 1.);
      curve_k_ += (1 - std::exp(-dt / .3)) * (k - curve_k_);
      if (curve_k_ > .3)
        curve_until_ = time + c["curve_memory"];
      width = width_ * (1 + curve_k_ * (c["curve_lane_width_m"] / c["lane_width_m"] - 1));
      if (curve_k_ > .01)
        r.notes.push_back("커브 차로 폭 " + std::to_string(int(std::lround(width))) + " px (왼쪽 선 " +
                          std::to_string(int(std::lround(angle))) + "도)");
    }
    center = centreLine(base.path, dl && dr ? other.path : Path(), left_base ? 1.f : -1.f, float(width),
                        float(tol));
    confidence = dl && dr ? (r.left.quality + r.right.quality) / 2 : .7 * base.quality;
    r.state = dl && dr ? "both" : dl ? "left only" : "right only";
  }
  // A fresh centre too short to give a target (stop line, one short dash)
  // does not replace a recent path: it is treated like a dropout.
  const bool usable = center.size() >= 3 && pathLength(center) >= .1 * h;
  const bool recent = !center_.empty() && time - center_seen_ <= c["track_memory"];
  if ((dl || dr) && (usable || !recent)) {
    // Blend with the previous, motion-compensated path (lane_ws): steadies
    // both <-> one-side switches and fit jitter; points further than
    // track_jump_px from it are not blended, so a real change passes at once.
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

  // 7. Lookahead target (straight-line distance lookahead_row ahead) and metrics.
  if (center.size() >= 3 && pathLength(center) >= .1 * h) {
    const double want = c["bev_near_m"] + (1 - c["lookahead_row"]) * (h - 1) * mrow;
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
  if (!r.deep_note.empty())
    r.notes.insert(r.notes.begin(), r.deep_note);
  for (const auto *line : lines)
    if (!line->note.empty())
      r.notes.push_back(std::string(line == &r.left ? "왼쪽: " : "오른쪽: ") + line->note);

  // 8. Views (only on request): candidates (deep / HSV), boundaries, centre, target.
  if (views) {
    r.bev_mask = cv::Mat::zeros(h, w, CV_8U);
    if (!deep_bev.empty())
      r.bev_mask.setTo(255, deep_bev > 0);
    if (!hsv_bev.empty())
      r.bev_mask.setTo(255, hsv_bev > 0);
    r.search_view = r.bev * .45;
    if (!hsv_bev.empty()) {
      r.search_view.setTo(cv::Scalar(0, 110, 140), hsv_bev == 1);
      r.search_view.setTo(cv::Scalar(130, 130, 130), hsv_bev == 2);
    }
    if (!deep_bev.empty()) {
      r.search_view.setTo(cv::Scalar(0, 210, 255), deep_bev == 1);
      r.search_view.setTo(cv::Scalar(235, 235, 235), deep_bev == 2);
    }
    for (const auto &k : r.candidates) {
      const cv::Scalar colour = k.source == kHsv    ? cv::Scalar(0, 140, 255)
                                : k.source == kDeep ? cv::Scalar(200, 60, 200)
                                                    : cv::Scalar(80, 220, 80);
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
        drawPath(used, line.path, 255, std::max(1, int(c["line_width_max"]) / 2));
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
    r.path_view = r.bev.clone();
    drawing.copyTo(r.path_view, drawn);
    drawing.copyTo(r.search_view, drawn);
    const std::string label = r.state + " | " + r.source + (r.deep_used ? " | deep " : " | no deep");
    cv::putText(r.path_view, label, {10, 24}, cv::FONT_HERSHEY_SIMPLEX, .7, {0, 255, 255}, 2);
    cv::putText(r.search_view, label, {10, 24}, cv::FONT_HERSHEY_SIMPLEX, .7, {0, 255, 255}, 2);
    r.overlay = r.original.clone();
    cv::Mat projected, projected_mask;
    cv::warpPerspective(drawing, projected, Hinv_, {w, h});
    cv::cvtColor(projected, projected_mask, cv::COLOR_BGR2GRAY);
    projected.copyTo(r.overlay, projected_mask);
    cv::Mat yellow_bev = cv::Mat::zeros(h, w, CV_8U);
    if (!deep_bev.empty())
      yellow_bev.setTo(255, deep_bev == 1);
    if (!hsv_bev.empty())
      yellow_bev.setTo(255, hsv_bev == 1);
    if (colours.empty())
      r.color_mask = cv::Mat::zeros(h, w, CV_8U);
    else
      cv::warpPerspective(colours, r.color_mask, Hinv_, {w, h}, cv::INTER_NEAREST);
    cv::warpPerspective(r.bev_mask, r.feature_mask, Hinv_, {w, h}, cv::INTER_NEAREST);
    cv::warpPerspective(used, r.line_mask, Hinv_, {w, h}, cv::INTER_NEAREST);
    cv::warpPerspective(yellow_bev, r.yellow_mask, Hinv_, {w, h}, cv::INTER_NEAREST);
  }
  r.processing_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_clock).count();
  return r;
}
} // namespace lane_lite

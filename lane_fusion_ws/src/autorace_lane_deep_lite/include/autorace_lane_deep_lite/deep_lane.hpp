#pragma once
// Deep-only lane keeping with the lane_ws LOGIC (2026-11-12).
//
// Detection: the lanedeep U-Net class mask (0 background, 1 yellow/orange
// lane, 2 white dashed lane) instead of the HSV colour features. Everything
// after it is the lane_ws code (autorace_lane_lite, copied as lane_ws.hpp /
// lane_ws.cpp): histogram/track seeds, direction-following sliding windows,
// corridor check, tracking (smoothing, 3-frame jump confirmation, hold),
// centre path, lookahead target, held path moved with odometry.
// Output layout is the same as /lane_lite/target, so the lane_ws PID
// (autorace_lane_pid, target_topic:=/lane_deep/target) drives it.
#include "autorace_lane_deep_lite/lane_ws.hpp"
#include <map>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace deep_lite {
using Spec = lane_lite::Spec;
// The lane_ws parameters without the HSV-only ones (the copy has none).
const std::vector<Spec> &specs();

struct Config {
  std::map<std::string, double> values; // = specs() keys
  Config();
  double operator[](const std::string &key) const { return values.at(key); }
  void validate() const;
  // Reads `lane_deep_lite:` or a lane_ws YAML (`lane_lite:`). Keys of the old
  // cell-chain deep_lite (cell, max_gap, ...) and other unknown keys are ignored.
  static Config load(const std::string &path);
  void save(const std::string &path) const; // `lane_deep_lite:` with specs() keys
  lane_lite::Config lane() const;
};

enum class LineState { Lost = 0, Held = 1, Detected = 2 };
const char *name(LineState state);
using Path = lane_lite::Path;

struct Line {
  Path path;
  LineState state = LineState::Lost;
  int cls = 0; // 1 yellow, 2 white
  double quality = 0, age = 0;
  std::string note;
};

struct Result {
  cv::Mat bev_classes;      // BEV lane mask used by the search (0/255)
  cv::Mat view;             // BEV debug picture (views only)
  std::vector<Path> chains; // unused (kept for the tuner's count): always empty
  Line left, right;
  Path center_path;
  bool valid = false;
  double target_x = 0, target_y = 0;
  // Control-facing values, metres/radians, left positive (same as lane_ws).
  double lateral_m = 0, heading = 0, curvature = 0, lookahead_m = 0, confidence = 0;
  double near_lateral_m = 0, near_heading = 0, near_m = 0;
  double lane_width_px = 0, processing_ms = 0;
  std::string state = "lost";
  std::vector<std::string> notes;
};

class Pipeline {
public:
  explicit Pipeline(const Config &config = {});
  void configure(const Config &config);
  void reset();
  // classes: CV_8U class ids of one camera frame, any size. time: frame stamp (s).
  Result process(const cv::Mat &classes, double time, bool views = false);
  // Odometry since the previous call (lane_ws: moves a HELD centre path).
  void move(double forward_m, double yaw_rad) { lane_.move(forward_m, yaw_rad); }
  const Config &config() const { return config_; }

private:
  Config config_;
  lane_lite::Pipeline lane_;
  cv::Mat blank_; // lane_ws wants a BGR frame; the deep input needs no picture
};
} // namespace deep_lite

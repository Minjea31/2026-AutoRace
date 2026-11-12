#pragma once
// lane_ws autorace_lane_lite (include/autorace_lane_lite/lane.hpp, src/lane.cpp,
// main 00e18a1) copied so the deep-only mode runs the lane_ws logic without
// touching lane_ws. Differences: process() needs the deep class mask (it
// replaces the HSV colour features), no HSV parameters, no YAML I/O
// (deep_lite::Config). Keep the rest in sync with lane_ws by hand.
// Compact single-corridor lane perception. Everything after the colour masks
// runs in BEV pixels (x right, y down); boundaries are smooth polylines, so
// steep and transverse (hairpin) markings share one representation.
#include <map>
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <vector>

namespace lane_lite {
struct Spec {
  const char *name, *group, *help;
  double value, min, max, step;
};
const std::vector<Spec> &specs();
const std::vector<const char *> &groups();

struct Config {
  std::map<std::string, double> values;
  Config();
  double operator[](const std::string &key) const { return values.at(key); }
  void validate() const;
};

enum class LineState { Lost = 0, Predicted = 1, Detected = 2 };
const char *name(LineState state);
using Path = std::vector<cv::Point2f>; // BEV px, ordered near -> far

struct Line {
  Path path;     // used boundary (smoothed or held)
  Path rejected; // this frame's fit that was not used
  LineState state = LineState::Lost;
  double quality = 0, residual = 0, age = 0;
  int observed = 0;
  bool yellow = false; // majority colour of the observed pixels
  std::string note;
  // Actual search trace, for display only.
  cv::Point seed{-1, -1};
  std::vector<cv::RotatedRect> boxes;
  std::vector<bool> hits;
  Path centers;
};

struct Result {
  cv::Mat original, overlay;                     // source image
  cv::Mat color_mask, feature_mask, line_mask;   // source coordinates
  cv::Mat bev, bev_mask, search_view, path_view; // BEV
  Line left, right;
  Path center_path;
  bool valid = false;
  // Control-facing measurement at the lookahead point. Left is positive.
  double target_x = 0, target_y = 0, lateral_px = 0, lateral_m = 0;
  double heading = 0, curvature = 0, lookahead_m = 0, confidence = 0;
  // Same quantities at the NEAREST centre-path point (BEV bottom side):
  // lateral offset (m, left +), path heading over the first ~0.25 m, distance.
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
  // views=false skips every display image (search/path/overlay/masks); the
  // driving node uses it when nobody needs the debug picture.
  // classes: deep lane mask of this frame (CV_8U class ids in source image
  // coordinates, any size: 0 background, 1 yellow, 2 white dashed), required.
  // bgr is only drawn on (any BGR picture of the same frame, or black).
  Result process(const cv::Mat &bgr, double time, bool views, const cv::Mat &classes);
  // Vehicle motion (odometry) since the previous call: forward distance (m)
  // and yaw change (rad, left +). Accumulated and applied only to a HELD
  // centre path, so a short dropout keeps the path on the road.
  void move(double forward_m, double yaw_rad);

private:
  struct Track {
    Path path;
    double seen = -1, quality = 0;
    int jumps = 0, colour = -1; // -1 unknown, 0 white, 1 yellow
  };
  Config config_;
  Track tracks_[2];
  Path center_;
  double center_seen_ = -1, center_confidence_ = 0, last_time_ = -1, width_ = 0;
  double motion_[3] = {0, 0, 0}; // x fwd (m), y left (m), yaw since center_ was last placed
};
} // namespace lane_lite

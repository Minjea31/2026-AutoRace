#pragma once
// Lane perception from the lanedeep U-Net class mask ONLY (no camera image, no
// HSV). The network already removes solid white lines, crosswalks, stop lines
// and glare (classes: 0 background, 1 orange/yellow lane, 2 white dashed
// lane), so instead of the HSV noise filters this pipeline just:
//   1. warps the class mask into BEV (nearest neighbour),
//   2. takes one centroid per grid cell and class,
//   3. chains same-class centroids along their direction (bridges dash gaps),
//   4. fits each chain, picks the left/right pair that forms one lane,
//   5. tracks both sides, builds the centre path and the lookahead target.
// Output layout is the same as /lane_fusion/target, so the fusion PID drives it.
#include <map>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace deep_lite {
struct Spec {
  const char *name, *group, *help;
  double value, min, max, step;
};
const std::vector<Spec> &specs();

struct Config {
  std::map<std::string, double> values;
  Config();
  double operator[](const std::string &key) const { return values.at(key); }
  void validate() const;
  // Reads `lane_deep_lite:`; a lane_lite / lane_fusion YAML (`lane_lite:`) is
  // accepted too and only the keys that exist here are taken (BEV, widths).
  static Config load(const std::string &path);
  void save(const std::string &path) const;
};

enum class LineState { Lost = 0, Held = 1, Detected = 2 };
const char *name(LineState state);
using Path = std::vector<cv::Point2f>; // BEV px, ordered near -> far

struct Chain {
  Path points;       // cell centroids, near -> far
  Path path;         // fitted, near -> far (empty: fit failed)
  int cls = 0;       // 1 yellow, 2 white dashed
  double length = 0; // arc length of the centroids (px)
  double residual = 0;
};

struct Line {
  Path path;
  LineState state = LineState::Lost;
  int cls = 0;
  double quality = 0, age = 0;
  std::string note;
};

struct Result {
  cv::Mat bev_classes;          // BEV class mask (0/1/2)
  cv::Mat view;                 // BEV debug picture (views only)
  std::vector<Chain> chains;    // all fitted candidates of this frame
  Line left, right;
  Path center_path;
  bool valid = false;
  double target_x = 0, target_y = 0;
  // Control-facing values, metres/radians, left positive (same as lane_fusion).
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
  // classes: CV_8U class ids of one camera frame, any size (resized to
  // width x height). time: frame stamp in seconds.
  Result process(const cv::Mat &classes, double time, bool views = false);
  const Config &config() const { return config_; }

private:
  struct Track {
    Path path;
    double seen = -1, quality = 0;
    int jumps = 0, cls = 0;
  };
  Config config_;
  cv::Mat H_, bev_valid_;
  Track tracks_[2];
  Path center_;
  double center_seen_ = -1, center_confidence_ = 0, last_time_ = -1, width_ = 0;
};
} // namespace deep_lite

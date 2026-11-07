#pragma once
// Lane fusion perception (perception only, no control).
//
// One pipeline, deep first:
//  1. Deep (U-Net class mask: 0 bg, 1 yellow, 2 white dashed) -> BEV ->
//     candidate boundaries (cell centroids -> direction-keeping chains -> fit).
//  2. HSV only when needed (no mask, a side missed, or a weak deep side): the
//     HSV features (top-hat / thickness / length filters) give candidates the
//     same way. An HSV candidate on a deep one confirms it; the others enter
//     the selection with a penalty. Masks are never OR/AND-merged.
//  3. Left/right by one cost over all candidate pairs: track continuity,
//     vehicle side, lane width, direction, source and quality.
//  4. Tracking, centre path from corresponding boundary points (one side only
//     when its role is certain), and a validation gate before the output.
// Everything after the masks runs in BEV pixels (x right, y down).
#include "autorace_lane_fusion/geometry.hpp"
#include <functional>
#include <map>
#include <opencv2/core.hpp>
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
  // Reads `lane_lite:`. Keys of the old multi-mode pipeline (deep_mode,
  // sliding-window settings) are ignored, so older YAML files still load.
  static Config load(const std::string &path);
  void save(const std::string &path) const;
};

enum class LineState { Lost = 0, Predicted = 1, Detected = 2 }; // Predicted = HELD
const char *name(LineState state);

enum Source { kDeep = 1, kHsv = 2 };
const char *sourceName(int source);

struct Candidate {
  Path path;   // fitted, near -> far
  Path points; // chained cell centroids
  int cls = 0; // 1 yellow, 2 white
  int source = 0;
  double length = 0, residual = 0, quality = 0;
};

struct Line {
  Path path;     // used boundary (smoothed or held)
  Path rejected; // this frame's choice that was not used
  LineState state = LineState::Lost;
  double quality = 0, residual = 0, age = 0;
  int observed = 0, source = 0, frames = 0; // centroids, kDeep|kHsv, frames tracked
  bool yellow = false;
  std::string note;
};

struct Result {
  cv::Mat original, overlay;                   // source image (views only)
  cv::Mat color_mask, feature_mask, line_mask; // source coordinates (views only)
  cv::Mat yellow_mask, deep_mask;              // source coordinates (views only)
  cv::Mat bev, bev_mask, search_view, path_view;
  std::vector<Candidate> candidates;
  Line left, right;
  Path center_path;
  bool valid = false;
  bool deep_used = false, hsv_used = false, image_used = false;
  std::string source = "none"; // of the chosen boundaries: deep | deep+hsv | hsv | none
  // Control-facing measurement at the lookahead point. Left is positive.
  double target_x = 0, target_y = 0, lateral_px = 0, lateral_m = 0;
  double heading = 0, curvature = 0, lookahead_m = 0, confidence = 0;
  // Same at the NEAREST centre-path point: lateral (m), heading over ~0.25 m, distance.
  double near_lateral_m = 0, near_heading = 0, near_m = 0;
  double lane_width_px = 0, processing_ms = 0;
  std::string state = "lost";
  std::vector<std::string> notes;
};

class Pipeline {
public:
  using Image = std::function<cv::Mat()>;
  explicit Pipeline(const Config &config = {});
  void configure(const Config &config);
  void reset();
  // image: returns the BGR frame; called only when HSV or the views need it
  // (a JPEG is then decoded only on those frames). deep: CV_8U class mask of
  // this frame or of a recent one (the caller checks its age), or empty.
  Result process(const Image &image, double time, bool views, const cv::Mat &deep = cv::Mat());
  Result process(const cv::Mat &bgr, double time, bool views = true, const cv::Mat &deep = cv::Mat()) {
    return process(Image([&bgr] { return bgr; }), time, views, deep);
  }

private:
  struct Track {
    Path path;
    double seen = -1, quality = 0;
    int jumps = 0, cls = 0, frames = 0;
  };
  Config config_;
  cv::Mat H_, Hinv_, bev_valid_; // BEV geometry, rebuilt only by configure()
  Track tracks_[2];
  Path center_;
  double center_seen_ = -1, center_confidence_ = 0, last_time_ = -1, width_ = 0;
  // Output gate: last published target and pending jump confirmations.
  Result held_;
  double held_time_ = -1;
  int out_jumps_ = 0;
  int base_side_ = -1; // boundary the centre path is built on (0 left, 1 right)
};
} // namespace lane_lite

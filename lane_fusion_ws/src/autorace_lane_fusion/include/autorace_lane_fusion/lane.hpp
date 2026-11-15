#pragma once
// Lane fusion perception (perception only, no control).
//
// OpenCV (HSV) and the U-Net run independently and are fused per boundary:
//  1. HSV features (top-hat / thickness / solid-white length filters) of every
//     frame -> lane_ws boundary detection (track seeds or histogram corridor,
//     direction-following sliding windows, cubic fit, stop-line / role checks).
//  2. Deep class mask (0 bg, 1 yellow, 2 white dashed), when one of an earlier
//     frame is recent enough: warped into THIS frame's BEV with the vehicle
//     motion between the two capture times, then the same detection.
//  3. Candidates of both sources are compared by geometry only (track
//     continuity, side of the vehicle, lane width, direction, fit quality,
//     agreement of the two sources); the source itself carries no weight.
//  4. One left/right tracker (smoothing, 3-frame jump confirmation, hold),
//     centre path from corresponding boundary points (one side only when its
//     role is certain), moved with odometry and held through a short dropout.
// Everything after the masks runs in BEV pixels (x right, y down).
#include "autorace_lane_fusion/geometry.hpp"
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
  // Reads `lane_lite:`. Keys of earlier fusion versions (deep_mode, cell
  // chains, hsv_penalty, output gate) are ignored, so older YAML files load.
  static Config load(const std::string &path);
  void save(const std::string &path) const;
};

enum class LineState { Lost = 0, Predicted = 1, Detected = 2 }; // Predicted = HELD
const char *name(LineState state);

enum Source { kDeep = 1, kHsv = 2 };
const char *sourceName(int source);

struct Candidate {
  Path path;   // fitted, near -> far
  Path points; // observed sliding-window centres
  int cls = 0;      // 1 yellow, 2 white (majority of the line's pixels)
  int near_cls = 0; // the same over the near half of the BEV only (a dash continued by a far yellow edge stays white)
  int source = 0, observed = 0;
  double length = 0, residual = 0, quality = 0;
};

// A deep mask for process(). The mask is usually of an EARLIER frame (the
// network runs asynchronously); it is used only if its age is within
// deep_max_age and the car motion between the two capture times is known
// (moved), or the age is at most kDeepStillAge.
constexpr double kDeepStillAge = .04; // s: usable without odometry (about one camera frame)
struct DeepInput {
  cv::Mat mask;     // CV_8U class ids, any size (scaled to the processing size)
  double age = 0;   // s: frame capture time - mask capture time
  bool moved = false;
  double dx = 0, dy = 0, dyaw = 0; // car motion mask -> frame (m fwd, m left, rad left +), mask-time frame
  DeepInput() = default;
  DeepInput(const cv::Mat &m) : mask(m) {} // NOLINT: this frame's own mask
};

struct Line {
  Path path;     // used boundary (smoothed or held)
  Path rejected; // this frame's choice that was not used
  LineState state = LineState::Lost;
  double quality = 0, residual = 0, age = 0;
  int observed = 0, source = 0, frames = 0; // centroids, kDeep|kHsv, frames tracked
  bool yellow = false;
  bool near_white = false; // near half of the boundary is white paint
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
  double deep_age = -1;        // s, age of the deep mask used (-1: none)
  std::string deep_note;       // why a given deep mask was or was not used
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
  explicit Pipeline(const Config &config = {});
  void configure(const Config &config);
  void reset();
  // bgr: this frame (any size, resized to width x height). views=false skips
  // every display image (the driving node when nobody watches).
  // cone_exit_side: transient recovery prior (-1 none, 0 left, 1 right).
  Result process(const cv::Mat &bgr, double time, bool views = true, const DeepInput &deep = {},
                 int cone_exit_side = -1);
  // Vehicle motion (odometry) since the previous call. The next process()
  // moves the remembered centre path into the new vehicle frame.
  void move(double forward_m, double yaw_rad); // along an arc
  void move(double dx, double dy, double dyaw); // m fwd, m left, rad, in the previous frame

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
  double curve_k_ = 0; // 0 straight .. 1 curve_lane_width_m (left-only right-hand bend), low-passed
  double curve_until_ = -1, odom_seen_ = -1; // bend memory active until; last frame with odometry motion
  Path curve_left_;         // last accepted left boundary, moved with the car (bend memory)
  double motion_[3] = {0, 0, 0}; // x fwd (m), y left (m), yaw since the previous process()
  int base_side_ = -1;           // boundary the centre path is built on (0 left, 1 right)
};
} // namespace lane_lite

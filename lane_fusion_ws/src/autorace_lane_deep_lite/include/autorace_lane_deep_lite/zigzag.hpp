#pragma once
// Zigzag lane marking (slow zone after the curve, 2026-11-20) on the deep class mask.
//
// The yellow class is warped to the same BEV as lane_ws (bev_* / roi_* / width /
// height of the deep_lite config). Each yellow component is checked for concave
// notches (cv::convexityDefects deeper than notch_min_px): a straight or curved
// line has 0-1, a single kink 1-2, a zigzag line one per tooth. A component with
// >= notches notches is a zigzag line.
// level: 0 none, 1 zigzag line(s) on one side only, 2 zigzag lines on both sides
// (>= 2 zigzag components whose mean x differ by >= side_sep_px).
// Bag check (car 06, 11-19 venue bags): notch_min_px 10, notches 3, side_sep_px 150
// -> level 2 from 115.02 s at the curve exit, no level 2 outside zigzag zones
// (also not on the wrinkled cloth of the obstacle area).
#include "autorace_lane_deep_lite/deep_lane.hpp"
#include <opencv2/core.hpp>
#include <vector>

namespace deep_lite {
struct ZigzagConfig {
  double notch_min_px = 10, notch_max_px = 60, min_area_px = 120, side_sep_px = 150;
  int notches = 3;
};

struct Zigzag {
  int level = 0;                   // 0 none, 1 one side, 2 both sides
  std::vector<double> line_x;      // BEV mean x of each zigzag line, left to right
  std::vector<cv::Point> notch_px; // BEV notch points of zigzag lines (debug)
};

// classes: CV_8U class ids of one camera frame (any size), config: deep_lite config.
Zigzag detectZigzag(const cv::Mat &classes, const Config &config, const ZigzagConfig &zig = {});
} // namespace deep_lite

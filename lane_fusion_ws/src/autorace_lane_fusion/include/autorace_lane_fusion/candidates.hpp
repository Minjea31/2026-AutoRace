#pragma once
// Boundary detection on one BEV class mask (deep or HSV), and the HSV feature
// stage that turns the colour BEV into such a class mask.
#include "autorace_lane_fusion/lane.hpp"

namespace lane_lite {
// HSV colour BEV -> class mask (0 bg, 1 yellow, 2 white) after the noise
// filters: top-hat white, crosswalk/text thickness, solid-white length, thin
// bands per colour and minimum area. `colours` (optional) gets the raw colour
// candidates for display.
cv::Mat hsvClasses(const cv::Mat &bev, const cv::Mat &valid, const Config &c, cv::Mat *colours);

// One source's left/right boundaries, found the lane_ws way: a recent track
// starts from its own near end, otherwise the histogram peaks that best form
// one corridor of `width`; direction-following sliding windows; cubic fit.
// A white line across the road (stop line / crosswalk edge) is rejected and a
// boundary well past the vehicle centre changes side. The two sides are NOT
// checked against each other here: pairing is the fusion's job.
struct Detection {
  int source = 0;
  Candidate side[2];               // path empty: nothing on that side
  bool forget[2] = {false, false}; // stop line / role swap: drop that track unless another source fills it
  std::string note[2];
};
Detection detect(const cv::Mat &classes, int source, const Path prior[2], double width, const Config &c);
} // namespace lane_lite

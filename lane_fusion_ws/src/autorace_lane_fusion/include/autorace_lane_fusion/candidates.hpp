#pragma once
// Candidate boundaries from a BEV class mask (deep or HSV), and the HSV
// feature stage that turns the colour BEV into such a class mask.
#include "autorace_lane_fusion/lane.hpp"

namespace lane_lite {
// HSV colour BEV -> class mask (0 bg, 1 yellow, 2 white) after the noise
// filters: top-hat white, crosswalk/text thickness, solid-white length, thin
// bands per colour and minimum area. `colours` (optional) gets the raw colour
// candidates for display.
cv::Mat hsvClasses(const cv::Mat &bev, const cv::Mat &valid, const Config &c, cv::Mat *colours);

// Class mask -> fitted candidates: one centroid per grid cell and class, chained
// along the marking direction (dash gaps bridged, sharp turns refused), split
// chains joined end to end, duplicates along one thick band removed.
std::vector<Candidate> buildCandidates(const cv::Mat &classes, int source, cv::Point2f vehicle,
                                       const Config &c);
} // namespace lane_lite

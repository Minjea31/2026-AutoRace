// Zigzag lane marking detector, see zigzag.hpp.
#include "autorace_lane_deep_lite/zigzag.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace deep_lite {
Zigzag detectZigzag(const cv::Mat &classes, const Config &c, const ZigzagConfig &z) {
  if (classes.empty() || classes.type() != CV_8U)
    throw std::invalid_argument("Expected a CV_8U class mask");
  const int w = int(c["width"]), h = int(c["height"]);
  // Same geometry as lane_ws.cpp step 1 (trapezoid -> full BEV, ROI, eroded valid area).
  const std::vector<cv::Point2f> src = {
      {float(c["bev_top_left"] * (w - 1)), float(c["bev_top_y"] * (h - 1))},
      {float(c["bev_top_right"] * (w - 1)), float(c["bev_top_y"] * (h - 1))},
      {float(c["bev_bottom_right"] * (w - 1)), float(c["bev_bottom_y"] * (h - 1))},
      {float(c["bev_bottom_left"] * (w - 1)), float(c["bev_bottom_y"] * (h - 1))}};
  const std::vector<cv::Point2f> dst = {
      {0, 0}, {float(w - 1), 0}, {float(w - 1), float(h - 1)}, {0, float(h - 1)}};
  const cv::Mat H = cv::getPerspectiveTransform(src, dst);
  cv::Mat valid = cv::Mat::zeros(h, w, CV_8U), roi = valid.clone();
  std::vector<cv::Point> poly(src.begin(), src.end());
  cv::fillConvexPoly(valid, poly, 255);
  cv::rectangle(roi, {cvRound(c["roi_left"] * (w - 1)), cvRound(c["roi_top"] * (h - 1))},
                {cvRound(c["roi_right"] * (w - 1)), cvRound(c["roi_bottom"] * (h - 1))}, 255,
                cv::FILLED);
  valid &= roi;
  cv::Mat bev_valid, scaled, bev;
  cv::warpPerspective(valid, bev_valid, H, {w, h}, cv::INTER_NEAREST);
  cv::erode(bev_valid, bev_valid, cv::Mat(), {-1, -1}, 2);
  cv::resize(classes, scaled, {w, h}, 0, 0, cv::INTER_NEAREST);
  cv::warpPerspective(scaled, bev, H, {w, h}, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
  cv::Mat yellow = (bev == 1) & bev_valid;

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(yellow, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  Zigzag out;
  for (const auto &contour : contours) {
    if (contour.size() < 5 || cv::contourArea(contour) < z.min_area_px)
      continue;
    std::vector<int> hull;
    std::vector<cv::Vec4i> defects;
    try {
      cv::convexHull(contour, hull, false, false);
      if (hull.size() < 4)
        continue;
      cv::convexityDefects(contour, hull, defects);
    } catch (const cv::Exception &) {
      continue; // self-intersecting hull of a tiny or degenerate contour
    }
    std::vector<cv::Point> notches;
    for (const auto &d : defects) {
      const double depth = d[3] / 256.;
      if (depth >= z.notch_min_px && depth <= z.notch_max_px)
        notches.push_back(contour[d[2]]);
    }
    if (int(notches.size()) < z.notches)
      continue;
    double sx = 0;
    for (const auto &p : contour)
      sx += p.x;
    out.line_x.push_back(sx / contour.size());
    out.notch_px.insert(out.notch_px.end(), notches.begin(), notches.end());
  }
  std::sort(out.line_x.begin(), out.line_x.end());
  if (!out.line_x.empty())
    out.level = out.line_x.size() >= 2 && out.line_x.back() - out.line_x.front() >= z.side_sep_px ? 2 : 1;
  return out;
}
} // namespace deep_lite

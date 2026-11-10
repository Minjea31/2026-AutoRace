#pragma once
// BEV polyline helpers shared by candidate building, selection, tracking and
// the centre path. Paths are ordered near -> far (BEV px, x right, y down).
#include <opencv2/core.hpp>
#include <vector>

namespace lane_lite {
using Path = std::vector<cv::Point2f>;
namespace geo {
float length(cv::Point2f v);
cv::Point2f unit(cv::Point2f v, cv::Point2f fallback = {0, -1});
// Right-hand normal of a travel direction (up in BEV -> +x).
inline cv::Point2f rightOf(cv::Point2f d) { return {-d.y, d.x}; }
cv::Point2f tangent(const Path &p, size_t i);
size_t nearest(const Path &p, cv::Point2f q, float *distance = nullptr);
inline bool interior(const Path &p, size_t i) { return i > 0 && i + 1 < p.size(); }
// Signed distance of q from the path, positive when q is RIGHT of the near ->
// far direction (local tangent, so it also holds in a bend).
float sideOf(const Path &p, cv::Point2f q);
float pathLength(const Path &p);
Path offset(const Path &p, float distance);
// Overlap distances from a to b, positive when b lies to the right of a.
std::vector<float> separation(const Path &a, const Path &b);
float median(std::vector<float> v);
// How far a lies from b: median overlap distance, else the near-end gap.
float distanceTo(const Path &a, const Path &b);
// Median direction difference (deg) where the two overlap; near thirds otherwise.
float angleBetween(const Path &a, const Path &b);
// Move `fresh` toward `old` by (1 - alpha) where the two overlap.
Path blend(const Path &fresh, const Path &old, double alpha, float gate);
cv::Point2f closestPoint(const Path &p, cv::Point2f q);
// Near third runs more than 60 deg from the driving direction.
bool transverse(const Path &p);
// Centre between `base` and the boundary on its `sign` side (+1 right, -1 left).
Path centreLine(const Path &base, const Path &other, float sign, float width, float tol);
// Signed curvature (1/m, left +) of metric points (x forward, y left).
double circleCurvature(const std::vector<cv::Point2d> &p);
void drawPath(cv::Mat &image, const Path &p, cv::Scalar color, int thickness, bool dashed = false);
} // namespace geo
} // namespace lane_lite

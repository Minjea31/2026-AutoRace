#pragma once
// Vehicle pose history integrated from the odometry twist (forward speed,
// yaw rate), for the car motion between two capture times: deep mask ->
// frame, and frame -> frame. Times in seconds on ONE clock (the node uses the
// receive time of every message). Not thread-safe. Shared by node and eval.
#include <algorithm>
#include <cmath>
#include <deque>

namespace lane_lite {
class Odometry {
public:
  void add(double t, double v, double w) {
    if (!std::isfinite(t) || !std::isfinite(v) || !std::isfinite(w))
      return;
    if (!s_.empty() && (t <= s_.back().t || t - s_.back().t > kGap))
      s_.clear(); // clock went back (bag loop) or odometry stalled
    Sample n{t, 0, 0, 0, v, w};
    if (!s_.empty()) {
      n = advance(s_.back(), t);
      n.v = v, n.w = w;
    }
    s_.push_back(n);
    while (s_.size() > 2 && t - s_.front().t > kKeep)
      s_.pop_front();
  }
  // Car motion from t0 to t1 in the t0 vehicle frame (m forward, m left, rad
  // left +). False when the history does not cover both times, or when the
  // motion is beyond what the car can do (a broken odometry source, e.g. a
  // diverged EKF, must not drag the lane tracks around).
  bool between(double t0, double t1, double &dx, double &dy, double &dyaw) const {
    Sample a, b;
    if (!at(t0, a) || !at(t1, b))
      return false;
    const double c = std::cos(a.yaw), s = std::sin(a.yaw), ex = b.x - a.x, ey = b.y - a.y;
    dx = c * ex + s * ey;
    dy = -s * ex + c * ey;
    dyaw = b.yaw - a.yaw;
    const double dt = std::abs(t1 - t0);
    return std::hypot(dx, dy) <= kMaxSpeed * dt + .01 && dx >= -kMaxReverse * dt - .01 &&
           std::abs(dyaw) <= kMaxYawRate * dt + .02;
  }

private:
  struct Sample {
    double t, x, y, yaw, v, w;
  };
  // A gap longer than kGap restarts the history; up to kAhead past the last
  // sample the last twist is extrapolated (odometry runs at ~10-50 Hz).
  static constexpr double kGap = .5, kKeep = 3, kAhead = .15;
  static constexpr double kMaxSpeed = 1.5, kMaxReverse = .5, kMaxYawRate = 4; // m/s, rad/s: LIMO limits + margin
  static Sample advance(const Sample &p, double t) { // p's twist held until t
    const double dt = t - p.t, mid = p.yaw + p.w * dt / 2;
    return {t, p.x + p.v * dt * std::cos(mid), p.y + p.v * dt * std::sin(mid), p.yaw + p.w * dt, p.v, p.w};
  }
  bool at(double t, Sample &out) const {
    if (s_.empty() || t < s_.front().t || t > s_.back().t + kAhead)
      return false;
    const auto it = std::upper_bound(s_.begin(), s_.end(), t, [](double v, const Sample &x) { return v < x.t; });
    out = advance(*(it - 1), t);
    return true;
  }
  std::deque<Sample> s_;
};
} // namespace lane_lite

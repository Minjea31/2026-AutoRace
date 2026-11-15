#pragma once
#include <algorithm>
#include <cmath>
#include <deque>

namespace lane_lite {
// A short, forward-motion turn history. No global heading or exit-map assumption.
// Side: -1 none, 0 left (recent right turn), 1 right (recent left turn).
class ConeExitPrior {
public:
  void add(double t, double v, double w) {
    if (!std::isfinite(t) || !std::isfinite(v) || !std::isfinite(w) ||
        std::abs(v) > 1.5 || std::abs(w) > 4.) {
      clear();
      return;
    }
    if (!history_.empty() && (t <= history_.back().t || t - history_.back().t > .25))
      clear();
    history_.push_back({t, v, w});
    while (history_.size() > 2 && history_[1].t < t - 2.)
      history_.pop_front();
  }
  void signal(bool active, double t) {
    if (!active || !std::isfinite(t)) {
      active_ = false;
      side_ = -1;
      return;
    }
    if (!active_ || t < signal_time_ || t - signal_time_ > .5)
      side_ = recentSide(t);
    active_ = true;
    signal_time_ = t;
  }
  int side(double t) const {
    if (!active_ || t < signal_time_ || t - signal_time_ > .5 || history_.empty() ||
        t < history_.back().t || t - history_.back().t > .25)
      return -1;
    return side_;
  }

private:
  struct Sample { double t, v, w; };
  std::deque<Sample> history_;
  bool active_ = false;
  double signal_time_ = -1;
  int side_ = -1;
  void clear() { history_.clear(); active_ = false; side_ = -1; }
  int recentSide(double t) const {
    if (history_.empty() || t < history_.back().t || t - history_.back().t > .25)
      return -1;
    double angle = 0, absolute = 0, distance = 0;
    for (size_t i = 1; i < history_.size(); ++i) {
      const auto &a = history_[i - 1], &b = history_[i];
      const double dt = b.t - std::max(a.t, t - 2.);
      if (dt <= 0 || a.v < .03 || b.v < .03)
        continue;
      const double turn = .5 * (a.w + b.w) * dt;
      angle += turn;
      absolute += std::abs(turn);
      distance += .5 * (a.v + b.v) * dt;
    }
    if (distance < .08 || std::abs(angle) < .1745329252 || std::abs(angle) < .7 * absolute)
      return -1;
    return angle < 0 ? 0 : 1;
  }
};
} // namespace lane_lite

#pragma once
#include <cmath>
namespace lane_pid {
class ParkingStop {
 public:
  void receive(bool start, double now) {
    if (start && std::isfinite(now)) { waiting_ = true; started_ = now; }
  }
  bool ready(double stamp, double now) {
    if (!waiting_ || !std::isfinite(stamp) || !std::isfinite(now) || stamp < started_ ||
        now-stamp < -.05 || now-stamp > .3) return false;
    waiting_ = false;
    return true;
  }
  bool waiting() const { return waiting_; }
 private:
  bool waiting_ = false;
  double started_ = 0.;
};
}

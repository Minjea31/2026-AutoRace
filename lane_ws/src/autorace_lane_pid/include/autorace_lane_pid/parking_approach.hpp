#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace lane_pid {
// Fixed slowdown windows; a new confirmation is required after each cooldown.
// Times are monotonic receipt times, independent of ROS clock changes.
class ParkingApproach {
public:
  ParkingApproach(int confirmations = 2, double speed = .3, double duration = 10., double gap = .5,
                  double cooldown = 5., bool calibration_stop = false)
      : confirmations_(confirmations), speed_(speed), duration_(duration), gap_(gap),
        cooldown_(cooldown), calibration_stop_(calibration_stop) {
    if (confirmations < 1 || !std::isfinite(speed) || speed < 0 || speed > 1 ||
        !std::isfinite(duration) || duration <= 0 || !std::isfinite(gap) || gap <= 0 ||
        !std::isfinite(cooldown) || cooldown < 0)
      throw std::invalid_argument("Invalid parking approach slowdown parameters");
  }
  bool receive(bool detected, double now) {
    if (waiting_ || session_active_)
      return false;
    // A persistent sign is still the same approach, even after cooldown.
    if (calibration_stop_ && !calibration_armed_) {
      if (detected) {
        clear_since_ = -std::numeric_limits<double>::infinity();
      } else {
        if (!std::isfinite(clear_since_)) clear_since_ = now;
        if (now - clear_since_ >= gap_) calibration_armed_ = true;
      }
      return false;
    }
    if (now < rearm_at_) {
      count_ = 0;
      last_ = -std::numeric_limits<double>::infinity();
      return false;
    }
    if (!detected || now - last_ > gap_)
      count_ = 0;
    last_ = detected ? now : -std::numeric_limits<double>::infinity();
    if (!detected)
      return false;
    if (++count_ < confirmations_)
      return false;
    count_ = 0;
    until_ = now + duration_;
    waiting_ = calibration_stop_;
    session_active_ = calibration_stop_;
    if (calibration_stop_) calibration_armed_ = false;
    rearm_at_ = until_ + cooldown_;
    return true;
  }
  bool ready(double now) {
    if (!waiting_)
      return false;
    waiting_ = false;
    until_ = now + duration_;
    rearm_at_ = until_ + cooldown_;
    return true;
  }
  void release() { waiting_=false; session_active_=false; until_=0.; }
  bool waiting() const { return waiting_; }
  bool active(double now) const { return waiting_ || session_active_ || now < until_; }
  double limit(double now, double nominal) const {
    if (waiting_)
      return 0.;
    return active(now) ? std::min(nominal, speed_) : nominal;
  }

private:
  int confirmations_, count_ = 0;
  double speed_, duration_, gap_, cooldown_;
  bool calibration_stop_ = false, waiting_ = false, calibration_armed_ = true, session_active_ = false;
  double clear_since_ = -std::numeric_limits<double>::infinity();
  double last_ = -std::numeric_limits<double>::infinity(), until_ = 0, rearm_at_ = 0;
};
} // namespace lane_pid

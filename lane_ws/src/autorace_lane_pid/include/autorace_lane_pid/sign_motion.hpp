// Parking-sign departure gate: C++ copy of parking_ws autorace_parking/sign_motion.py (SignMotionGate).
// The parking node ignores /yolo/parking_sign until the car has been moving (/odom |v| >= 0.02 for
// 3 samples) for ignore_sec. lane_pid must use the same rule, otherwise it publishes
// /parking/approach_start for a sign the parking node rejected and the mux parking guard holds
// the car at STOP forever (2026-11-19, car 06: sign seen 1.8 s after departure).
// Keep this in sync with sign_motion.py (thresholds 0.01 / 0.02 m/s, 3 samples, 0.3 s timeout).
#pragma once
#include <cmath>
#include <limits>
#include <stdexcept>

namespace lane_pid {

class SignMotionGate {
public:
  explicit SignMotionGate(double ignore_sec = 5., double timeout = .3)
      : ignore_sec_(ignore_sec), timeout_(timeout) {
    if (!std::isfinite(ignore_sec) || ignore_sec < 0)
      throw std::invalid_argument("Parking departure ignore time must be finite and nonnegative");
  }

  // stamp: message time [s], speed: measured /odom linear.x [m/s], wall: steady clock [s]
  void update(double stamp, double speed, double wall) {
    if (has_sample_) {
      if (stamp <= stamp_)
        return;
      if (wall - wall_ > timeout_) {
        moving_ = false;
        run_ = stops_ = 0;
        departed_ = kNone;
        ignore_until_ = kInf;
      }
    }
    has_sample_ = true;
    stamp_ = stamp;
    speed_ = speed;
    wall_ = wall;
    if (std::abs(speed) < .01) {
      ++stops_;
      run_ = 0;
      departed_ = kNone;
      if (stops_ >= 3) {
        moving_ = false;
        ignore_until_ = kInf;
      }
    } else if (std::abs(speed) >= .02) {
      stops_ = 0;
      if (!moving_) {
        if (!run_)
          departed_ = wall;
        if (++run_ >= 3) {
          moving_ = true;
          ignore_until_ = departed_ + ignore_sec_;
        }
      }
    } else {
      run_ = stops_ = 0;
      departed_ = kNone;
    }
  }

  bool allows(double wall) const {
    return has_sample_ && moving_ && std::abs(speed_) >= .02 && wall - wall_ >= 0 &&
           wall - wall_ <= timeout_ && wall >= ignore_until_;
  }

private:
  static constexpr double kInf = std::numeric_limits<double>::infinity();
  static constexpr double kNone = -std::numeric_limits<double>::infinity();
  double ignore_sec_, timeout_;
  bool has_sample_ = false, moving_ = false;
  double stamp_ = 0, speed_ = 0, wall_ = 0;
  int run_ = 0, stops_ = 0;
  double departed_ = kNone, ignore_until_ = kInf;
};

} // namespace lane_pid

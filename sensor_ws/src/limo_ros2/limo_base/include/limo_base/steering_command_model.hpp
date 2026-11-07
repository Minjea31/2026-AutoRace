#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace AgileX {
// Measured physical inner-wheel angle = gain * wire angle - deadband.
// The calibration changes command encoding only. Wheel/IMU localization and
// the existing steering feedback decoding remain unchanged.
struct SteeringCommandModel {
  bool enabled = false;
  double gain = 1.92;
  double deadband = 0.23;
  double wire_limit = 0.35;
  void validate() const {
    if (!std::isfinite(gain) || !std::isfinite(deadband) || !std::isfinite(wire_limit)
        || gain <= 0.0 || deadband < 0.0 || wire_limit <= 0.0 || wire_limit > 0.7
        || gain * wire_limit <= deadband || gain * wire_limit - deadband >= 1.4)
      throw std::invalid_argument("Invalid calibrated Ackermann steering command model");
  }
  double encode(double physical_inner_angle, double legacy_scale) const {
    if (!enabled) return physical_inner_angle / legacy_scale;
    if (std::abs(physical_inner_angle) < 1e-6) return 0.0;
    return std::copysign(std::min(wire_limit, (std::abs(physical_inner_angle) + deadband) / gain), physical_inner_angle);
  }
};
}

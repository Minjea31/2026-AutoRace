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
  // Vehicle-specific neutral trim in wire units, independent of gain calibration.
  double wire_offset = 0.0;
  void validate() const {
    if (!std::isfinite(gain) || !std::isfinite(deadband) || !std::isfinite(wire_limit)
        || !std::isfinite(wire_offset) || std::abs(wire_offset) > wire_limit
        || gain <= 0.0 || deadband < 0.0 || wire_limit <= 0.0 || wire_limit > 0.7
        || gain * wire_limit <= deadband || gain * wire_limit - deadband >= 1.4)
      throw std::invalid_argument("Invalid calibrated Ackermann steering command model");
  }
  double encode(double physical_inner_angle, double legacy_scale) const {
    double command = physical_inner_angle / legacy_scale;
    if (enabled) {
      command = std::abs(physical_inner_angle) < 1e-6 ? 0.0 :
          std::copysign(std::min(wire_limit, (std::abs(physical_inner_angle) + deadband) / gain), physical_inner_angle);
    }
    // Preserve the exact legacy encoding and limits for the default vehicle 06.
    if (wire_offset == 0.0) return command;
    return std::max(-wire_limit, std::min(wire_limit, command + wire_offset));
  }
};
}

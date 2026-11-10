"""Encode parking steering through the LIMO driver's legacy command mapping.

The planner and follower keep physical bicycle angles. Only the outgoing
Ackermann message is encoded, so the shared driver can leave lane commands
unchanged. Geometry and legacy scale match limo_base's Ackermann converter.
"""
import math


LEGACY_INNER_ANGLE_SCALE = 2.47


def parking_steering_command(physical_angle, cfg):
    """Return a central angle that makes the legacy driver send calibrated wire values."""
    if not cfg.require_steering_calibration:
        return physical_angle
    if not math.isfinite(physical_angle) or abs(physical_angle) >= math.pi / 2:
        raise ValueError('Parking steering must be finite and between -pi/2 and pi/2')
    angle = abs(physical_angle)
    inner = math.atan2(cfg.wheelbase * math.sin(angle),
                       cfg.wheelbase * math.cos(angle) - cfg.steering_track / 2 * math.sin(angle))
    if inner < 1e-6:
        return 0.0
    wire = min(cfg.steering_command_limit_expected,
               (inner + cfg.steering_command_deadband_expected) / cfg.steering_command_gain_expected)
    legacy_inner = wire * LEGACY_INNER_ANGLE_SCALE
    if not 0 < legacy_inner < math.pi / 2:
        raise ValueError('Parking calibration exceeds the legacy driver steering range')
    # Invert central -> inner-wheel angle; the mux will convert this central
    # angle to yaw rate, and the uncalibrated driver will divide inner by 2.47.
    command = math.atan2(cfg.wheelbase * math.sin(legacy_inner),
                         cfg.wheelbase * math.cos(legacy_inner) + cfg.steering_track / 2 * math.sin(legacy_inner))
    return math.copysign(command, physical_angle)

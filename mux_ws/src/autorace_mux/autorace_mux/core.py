"""Deterministic arbitration and conversion, independent of ROS."""

from dataclasses import dataclass
import math
from typing import Dict, Optional, Tuple


# Order is the priority contract in docs/2026_AutoRace.drawio. Lane is last.
INPUTS = (
    ('crosswalk', '/mission/crosswalk/command_01'),
    ('signal', '/mission/signal/command_02'),
    ('parking', '/mission/parking/command_03'),
    ('cone', '/mission/cone/command_04'),
    ('objavoid', '/mission/objavoid/command_05'),
    ('slow', '/mission/slow/command_08'),
    ('lane', '/mission/lane'),
)


@dataclass(frozen=True)
class Config:
    publish_rate_hz: float = 30.0
    input_timeout_sec: float = 0.25
    future_tolerance_sec: float = 0.05
    require_stamp: bool = True
    wheelbase_m: float = 0.2
    max_forward_speed_mps: float = 0.8
    max_reverse_speed_mps: float = 0.3
    max_steering_angle_rad: float = 0.65
    max_yaw_rate_rps: float = 2.5
    max_acceleration_mps2: float = 1.0
    max_deceleration_mps2: float = 2.0
    steering_limit_enabled: bool = False
    yaw_rate_limit_enabled: bool = False

    def __post_init__(self):
        for name, value in vars(self).items():
            if name in ('require_stamp', 'steering_limit_enabled', 'yaw_rate_limit_enabled'):
                if not isinstance(value, bool):
                    raise ValueError(f'{name} must be a bool')
                continue
            if isinstance(value, bool) or not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
            if value < 0 or (name != 'future_tolerance_sec' and value == 0):
                raise ValueError(f'{name} must be positive (tolerance may be zero)')
        if self.max_steering_angle_rad >= math.pi / 2:
            raise ValueError('max_steering_angle_rad must be less than pi/2')


@dataclass(frozen=True)
class Candidate:
    source: str
    speed: float
    steering_angle: float
    stamp_ns: int
    received_at: float


class Arbitrator:
    def __init__(self, config: Config):
        self.config = config
        self._candidates: Dict[str, Candidate] = {}
        self._last_ros_ns: Optional[int] = None

    def discard(self, source: str):
        self._candidates.pop(source, None)

    def _observe_clock(self, ros_now_ns: int):
        # A bag/simulation reset must not resurrect candidates from the old clock.
        if self._last_ros_ns is not None and ros_now_ns < self._last_ros_ns:
            self._candidates.clear()
        self._last_ros_ns = ros_now_ns

    def _stamp_valid(self, stamp_ns: int, ros_now_ns: int) -> bool:
        if stamp_ns == 0:
            return not self.config.require_stamp
        age = (ros_now_ns - stamp_ns) / 1e9
        return -self.config.future_tolerance_sec <= age < self.config.input_timeout_sec

    def receive(
        self, source: str, speed: float, steering_angle: float,
        stamp_ns: int, received_at: float, ros_now_ns: int,
    ) -> Tuple[bool, str]:
        if source not in dict(INPUTS):
            raise ValueError(f'Unknown input: {source}')
        self._observe_clock(ros_now_ns)
        reason = ''
        if not all(math.isfinite(x) for x in (speed, steering_angle, received_at)):
            reason = 'non-finite command'
        elif abs(steering_angle) >= math.pi / 2:
            reason = 'steering must be strictly between -pi/2 and pi/2'
        elif speed < 0 and source != 'parking':
            reason = 'reverse is allowed only for parking'
        if reason:
            self.discard(source)
            return False, reason
        previous = self._candidates.get(source)
        # Delayed commands must not erase a newer candidate, even if already stale.
        if previous and 0 < stamp_ns < previous.stamp_ns:
            return False, 'out-of-order stamp'
        if stamp_ns < 0 or not self._stamp_valid(stamp_ns, ros_now_ns):
            self.discard(source)
            return False, 'missing, stale, or future header stamp'
        self._candidates[source] = Candidate(
            source, speed, steering_angle, stamp_ns, received_at)
        return True, ''

    def select(self, steady_now: float, ros_now_ns: int) -> Optional[Candidate]:
        self._observe_clock(ros_now_ns)
        for source, _ in INPUTS:
            candidate = self._candidates.get(source)
            if candidate is None:
                continue
            age = steady_now - candidate.received_at
            if (0 <= age < self.config.input_timeout_sec
                    and self._stamp_valid(candidate.stamp_ns, ros_now_ns)):
                # A zero-speed mission is an active stop command, not an inactive input.
                return candidate
            self.discard(source)
        return None


class TwistConverter:
    def __init__(self, config: Config):
        self.config = config
        self._speed = 0.0
        self._previous_tick: Optional[float] = None

    def convert(self, candidate: Optional[Candidate], steady_now: float) -> Tuple[float, float]:
        cfg = self.config
        dt = 0.0 if self._previous_tick is None else max(
            0.0, min(steady_now - self._previous_tick, cfg.input_timeout_sec))
        self._previous_tick = steady_now
        # Stops bypass the ramp so a crosswalk or watchdog stop is immediate.
        if candidate is None or candidate.speed == 0:
            self._speed = 0.0
            return 0.0, 0.0
        target = max(-cfg.max_reverse_speed_mps,
                     min(candidate.speed, cfg.max_forward_speed_mps))
        if self._speed * target < 0:
            target = 0.0  # Decelerate to zero before changing direction.
        increasing = abs(target) > abs(self._speed)
        step = (cfg.max_acceleration_mps2 if increasing else cfg.max_deceleration_mps2) * dt
        self._speed += max(-step, min(target - self._speed, step))
        angle = candidate.steering_angle
        if cfg.steering_limit_enabled:
            angle = max(-cfg.max_steering_angle_rad, min(angle, cfg.max_steering_angle_rad))
        yaw_rate = self._speed * math.tan(angle) / cfg.wheelbase_m
        if cfg.yaw_rate_limit_enabled:
            yaw_rate = max(-cfg.max_yaw_rate_rps, min(yaw_rate, cfg.max_yaw_rate_rps))
        return self._speed, yaw_rate

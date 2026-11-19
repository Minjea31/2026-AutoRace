"""Ignore signs after the parking stop-line stop, not after ordinary stops."""
import math


class SignMotionGate:
    def __init__(self, ignore_sec=5., timeout=.3):
        if not math.isfinite(ignore_sec) or ignore_sec < 0:
            raise ValueError('Parking stop-line ignore time must be finite and nonnegative')
        self.ignore_sec, self.timeout = ignore_sec, timeout
        self.sample = None
        self.moving = False
        self.run = self.stops = 0
        self.ignore_until = -math.inf
        self.await_stop = False
        self.retry_departure_required = False

    def on_parking_stop(self, wall):
        # Only the accepted white stop-line event starts the five-second window.
        self.ignore_until = max(self.ignore_until, wall+self.ignore_sec)

    def require_departure(self):
        # Preserve failed-attempt recovery: brake before retrying on new signs.
        # This latch does not create or extend the stop-line cooldown.
        self.await_stop = self.stops < 3
        self.retry_departure_required = True
        self.moving = False
        self.run = 0

    def update(self, stamp, speed, wall):
        if self.sample is not None:
            if stamp <= self.sample[0]:
                return
            if wall-self.sample[2] > self.timeout:
                self.moving = False
                self.run = self.stops = 0
        self.sample = stamp, speed, wall
        if self.await_stop:
            self.stops = self.stops+1 if abs(speed) < .01 else 0
            if self.stops >= 3:
                self.await_stop = False
            return
        if abs(speed) < .01:
            self.stops += 1
            self.run = 0
            if self.stops >= 3:
                self.moving = False
        elif abs(speed) >= .02:
            self.stops = 0
            if not self.moving:
                self.run += 1
                if self.run >= 3:
                    self.moving = True
                    self.retry_departure_required = False
        else:
            self.run = self.stops = 0

    def allows(self, wall):
        return wall >= self.ignore_until and not self.retry_departure_required

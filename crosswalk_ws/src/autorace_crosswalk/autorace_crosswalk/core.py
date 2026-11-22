"""Crosswalk delay, stop hold and cooldown, independent of ROS."""


class CrosswalkState:
    def __init__(self, stop_delay_sec=0., stop_hold_sec=3., cooldown_sec=10.):
        self.stop_delay = stop_delay_sec
        self.stop_hold = stop_hold_sec
        self.cooldown = cooldown_sec
        self.ready_at = None
        self.confirmations = 0
        self.phase = 'waiting'
        self.started_at = None
        self.completed_at = None

    def receive(self, detected):
        if self.phase == 'waiting':
            self.confirmations = self.confirmations + 1 if detected else 0
            if self.confirmations >= 2:
                self.phase = 'waiting_white'

    def receive_white(self, detected, now=None):
        if self.phase == 'waiting_white' and detected:
            if now is None and self.stop_delay:
                raise ValueError('A detection time is required for a delayed stop')
            self.phase = 'ready'
            self.ready_at = None if now is None else now + self.stop_delay
            return True
        return False

    def tick(self, now, lane_available):
        # Start the full three seconds only when a command can be published.
        if (self.phase == 'ready' and lane_available
                and (self.ready_at is None or now >= self.ready_at)):
            self.phase = 'stopping'
            self.started_at = now
        if self.phase == 'stopping' and now - self.started_at >= self.stop_hold:
            self.phase = 'cooldown'
            self.completed_at = now
            self.confirmations = 0
        if self.phase == 'cooldown' and now - self.completed_at >= self.cooldown:
            self.phase = 'waiting'
            self.ready_at = None
            self.started_at = None
            self.completed_at = None
        return self.phase == 'stopping'

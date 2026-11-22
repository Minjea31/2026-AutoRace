"""Confirmed-red episodes held until confirmed green, independent of ROS."""


class SignalState:
    def __init__(self, confirm_frames, signal_gap_timeout_sec,
                 require_white_mark=False, stop_delay_sec=0.):
        self.required = confirm_frames
        self.gap = signal_gap_timeout_sec
        self.require_white_mark = require_white_mark
        self.stop_delay = stop_delay_sec
        self.armed = False
        self.stop_at = None
        self.stopping = False
        self.stop_generation = 0
        self.started_at = None
        self.release_reason = ''
        self.reset_confirmation()

    def reset_confirmation(self):
        self.candidate = None
        self.count = 0
        self.last_received = None

    def release(self, reason):
        self.armed = False
        self.stop_at = None
        self.stopping = False
        self.started_at = None
        self.release_reason = reason
        self.reset_confirmation()

    def begin_stop(self, now):
        self.armed = False
        self.stop_at = None
        self.stopping, self.started_at = True, now
        self.stop_generation += 1
        self.release_reason = ''

    def receive_white(self, detected, now):
        self.tick(now)
        if not detected or not self.armed or self.stop_at is not None or self.stopping:
            return False
        self.armed = False
        self.stop_at = now + self.stop_delay
        self.tick(now)
        return True

    def tick(self, now):
        if self.stop_at is not None and now >= self.stop_at:
            self.begin_stop(now)
        if self.last_received is not None and now - self.last_received > self.gap:
            self.reset_confirmation()

    def receive(self, green, now):
        self.tick(now)
        if self.candidate != green:
            self.candidate, self.count = green, 0
        self.last_received = now
        self.count += 1
        if self.count < self.required:
            return
        self.count = 0
        if green:
            self.release('green confirmed')
        elif not self.stopping and not self.armed and self.stop_at is None:
            self.release_reason = ''
            if self.require_white_mark:
                self.armed = True
            elif self.stop_delay:
                self.stop_at = now + self.stop_delay
            else:
                self.begin_stop(now)

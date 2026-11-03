"""Traffic light confirmation and stop duration, independent of ROS."""


class SignalState:
    def __init__(self, confirm_frames, max_red_wait_sec, signal_gap_timeout_sec):
        self.required = confirm_frames
        self.max_wait = max_red_wait_sec
        self.gap = signal_gap_timeout_sec
        self.stopping = False
        self.started_at = None
        self.release_reason = ''
        self.reset_confirmation()

    def reset_confirmation(self):
        self.candidate = None
        self.count = 0
        self.last_received = None

    def release(self, reason):
        self.stopping = False
        self.started_at = None
        self.release_reason = reason
        self.reset_confirmation()

    def tick(self, now):
        if self.stopping and now-self.started_at >= self.max_wait:
            self.release('red wait timeout')
        if self.last_received is not None and now-self.last_received > self.gap:
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
        elif not self.stopping:
            self.stopping, self.started_at = True, now
            self.release_reason = ''

"""Three-second crosswalk stops with a ten-second cooldown, independent of ROS."""


class CrosswalkState:
    def __init__(self):
        self.confirmations = 0
        self.phase = 'waiting'
        self.started_at = None
        self.completed_at = None

    def receive(self, detected):
        if self.phase == 'waiting':
            self.confirmations = self.confirmations + 1 if detected else 0
            if self.confirmations >= 2:
                self.phase = 'waiting_white'

    def receive_white(self, detected):
        if self.phase == 'waiting_white' and detected:
            self.phase = 'ready'

    def tick(self, now, lane_available):
        # Start the full three seconds only when a command can be published.
        if self.phase == 'ready' and lane_available:
            self.phase = 'stopping'
            self.started_at = now
        if self.phase == 'stopping' and now - self.started_at >= 3.0:
            self.phase = 'cooldown'
            self.completed_at = now
            self.confirmations = 0
        if self.phase == 'cooldown' and now - self.completed_at >= 10.0:
            self.phase = 'waiting'
            self.started_at = None
            self.completed_at = None
        return self.phase == 'stopping'

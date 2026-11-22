"""Only observations received after reverse departure may release parking."""
import math


class ExitStopGate:
    """One bounded brake attempt before an early reverse-to-lane handoff."""
    def __init__(self,timeout=.8):
        self.timeout=timeout
        self.reset()

    def reset(self):
        self.started=None
        self.attempted=False

    def update(self,eligible,stationary,wall):
        if self.started is not None and wall-self.started>=self.timeout:
            self.started=None
            return 'follow'
        if not eligible:
            return 'hold' if self.started is not None else 'follow'
        if stationary:
            self.started=None
            return 'release'
        if self.started is not None:
            return 'hold'
        if self.attempted:
            return 'follow'
        self.started=wall
        self.attempted=True
        return 'hold'


class ExitLaneGate:
    def __init__(self, timeout=.25, confidence=.1):
        if not math.isfinite(timeout) or timeout <= 0 or not math.isfinite(confidence) or not 0 <= confidence < 1:
            raise ValueError('Invalid exit lane handoff timeout or confidence')
        self.timeout, self.confidence = timeout, confidence
        self.reset()

    def reset(self):
        self.started_ros_ns = self.started_wall = None
        self.target_received = None
        self.command = None

    def begin(self, ros_ns, wall):
        if self.started_ros_ns is None:
            self.reset()
            self.started_ros_ns, self.started_wall = ros_ns, wall

    def observe_target(self, data, wall):
        self.target_received = None
        if (self.started_wall is not None and wall >= self.started_wall and len(data) >= 8
                and all(math.isfinite(v) for v in data[:8]) and data[0] > .5
                and data[5] > self.confidence and data[6] in (0., 1., 2.)
                and data[7] in (0., 1., 2.) and 2. in data[6:8]):
            # HELD alone is tracking memory, not a newly detected boundary.
            self.target_received = wall

    def observe_command(self, speed, steering, stamp_ns, wall):
        self.command = None
        if (self.started_wall is not None and wall >= self.started_wall
                and stamp_ns > self.started_ros_ns and math.isfinite(speed) and speed > 0
                and math.isfinite(steering) and abs(steering) < math.pi / 2):
            self.command = (stamp_ns, wall)

    def ready(self, ros_ns, wall):
        if self.target_received is None or self.command is None:
            return False
        stamp, received = self.command
        return (ros_ns >= self.started_ros_ns
                and 0 <= wall - self.target_received < self.timeout
                and 0 <= wall - received < self.timeout
                and -.05 <= (ros_ns - stamp) / 1e9 < self.timeout)

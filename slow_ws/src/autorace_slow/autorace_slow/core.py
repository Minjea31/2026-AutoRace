"""Independent confirmation and renewable steady-time holds for two signs."""
from dataclasses import dataclass


@dataclass
class Mission:
    count: int = 0
    last_received: float = None
    deadline: float = None
    armed: bool = False
    start_at: float = None


class SlowState:
    def __init__(self, confirm_frames=2, hold_sec=10., gap_timeout_sec=.5, require_white_mark=True, slow_hold_sec=15., start_delay_sec=0.):
        self.required = confirm_frames
        self.hold = hold_sec
        self.slow_hold = slow_hold_sec
        self.gap = gap_timeout_sec
        self.require_white_mark = require_white_mark
        self.start_delay = start_delay_sec
        self.missions = {kind: Mission() for kind in ('slow', 'bump')}

    def tick(self, now):
        for mission in self.missions.values():
            if mission.deadline is not None and now >= mission.deadline:
                mission.deadline = None
                mission.start_at = None
                mission.armed = False
                mission.count = 0
                mission.last_received = None
            if mission.last_received is not None and now-mission.last_received > self.gap:
                mission.count = 0

    def receive(self, kind, detected, now):
        """Return True only on a new activation, never on a timer extension."""
        self.tick(now)
        mission = self.missions[kind]
        mission.last_received = now
        if not detected:
            mission.count = 0
            return False
        if mission.start_at is not None and now < mission.start_at:
            return False  # Repeated signs cannot move a scheduled start.
        if mission.deadline is not None and kind != 'slow':
            mission.deadline = now+self.hold
            return False
        if mission.armed:
            return False
        mission.count += 1
        if mission.count < self.required:
            return False
        mission.count = 0
        if self.require_white_mark:
            mission.armed = True
            return False
        mission.deadline = now + (self.slow_hold if kind == 'slow' else self.hold)
        return True

    def receive_white(self, detected, now, eligible_kinds=None):
        """A new positive floor observation starts only previously armed missions."""
        self.tick(now)
        activated = []
        if detected:
            for kind, mission in self.missions.items():
                if (mission.armed and (mission.deadline is None or kind == 'slow')
                        and (eligible_kinds is None or kind in eligible_kinds)):
                    mission.armed = False
                    # A renewal keeps an already active mission running.
                    if mission.deadline is None:
                        mission.start_at = now + self.start_delay
                    mission.deadline = now + self.start_delay + (self.slow_hold if kind == 'slow' else self.hold)
                    activated.append(kind)
        return tuple(activated)

    def stop_slow(self):
        self.missions['slow'] = Mission()

    def selected(self, now):
        self.tick(now)
        return next((kind for kind in ('bump', 'slow')
                     if self.missions[kind].deadline is not None
                     and (self.missions[kind].start_at is None or now >= self.missions[kind].start_at)), None)

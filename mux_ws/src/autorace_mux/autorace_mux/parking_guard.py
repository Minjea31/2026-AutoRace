"""An armed parking session never falls back to unbounded lane motion."""
import math
from dataclasses import replace


class ParkingGuard:
    PHASES = {'IDLE','RESET','HOLD','APPROACH','DRIVE','FAILED','RELEASE'}

    def __init__(self, timeout=.3):
        self.timeout = timeout
        self.armed = False
        self.limit = .3
        self.phase = 'IDLE'
        self.ready = False
        self.received = None
        self.stamp = -math.inf
        self.event_stamp = -math.inf

    def arm(self, stamp, now, ros_now):
        if (math.isfinite(stamp) and -.05 <= ros_now-stamp <= self.timeout
                and stamp > self.event_stamp):
            self.armed,self.phase,self.ready = True,'HOLD',False
            self.event_stamp = stamp
            self.received = now

    def receive(self, state, now, ros_now):
        try:
            stamp,phase,ready = state['stamp'],state['phase'],state['ready']
            limit=state.get('limit',.3)
            if isinstance(limit,bool) or not isinstance(limit,(float,int)) or not math.isfinite(limit) or not 0<=limit<=.3:return False
            if (not isinstance(ready,bool) or (isinstance(stamp,bool) or not isinstance(stamp,(int,float)))
                    or not math.isfinite(stamp) or phase not in self.PHASES
                    or not -.05 <= ros_now-stamp <= self.timeout or stamp <= self.stamp
                    or stamp < self.event_stamp):
                return False
        except (KeyError,TypeError):
            return False
        self.stamp,self.received,self.limit = stamp,now,limit
        if phase in ('RESET','RELEASE') and ready:
            self.armed = False
        elif phase != 'IDLE':
            self.armed = True
        # An idle heartbeat cannot release an already armed session.
        self.phase,self.ready = phase,ready
        return True

    def apply(self, candidate, now, ros_now):
        if not self.armed:
            return candidate
        if (self.received is None or not 0 <= now-self.received <= self.timeout
                or not -.05 <= ros_now-self.stamp <= self.timeout or not self.ready):
            return None
        if self.phase == 'APPROACH':
            if candidate is None:return None
            # A higher priority mission may stop, but cannot accelerate through approach.
            return replace(candidate,speed=min(self.limit,max(0.,candidate.speed)))
        if self.phase == 'DRIVE':
            # Higher priority stop commands retain their priority; other missions cannot
            # seize motion when a parking command expires.
            if candidate and (candidate.source == 'parking' or candidate.speed == 0.):
                return candidate
        return None

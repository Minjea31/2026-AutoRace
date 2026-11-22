"""Zigzag slow zone (2026-11-20): the slow sign after the curve is never in the camera view.

Input: /lane_deep/zigzag level from lane_deep_lite (0 none, 1 one side, 2 both sides).
  ARMED    -> ACTIVE    level 2 on confirm_frames consecutive messages
  ACTIVE   -> ended     /start_crosswalk (crosswalk mission starts): disabled for good
  ACTIVE   -> ended     no zigzag line (level 0 or no message) for exit_sec while the
                        crosswalk sign is not seen yet: zone left without a crosswalk
                        (fallback, e.g. the sign was hidden by a car ahead)
  any      -> disabled  /yolo/crosswalk_sign: never triggers again; an ACTIVE slow keeps
                        running until /start_crosswalk (no exit_sec fallback any more, so
                        it does not speed up just before the crosswalk), at most
                        max_after_sign_sec after the sign
Times are steady seconds.
"""


class ZigzagSlow:
    def __init__(self, confirm_frames=2, exit_sec=2.0, gap_sec=0.5, max_after_sign_sec=15.0):
        if (int(confirm_frames) < 1 or not exit_sec > 0 or not gap_sec > 0
                or not max_after_sign_sec > 0):
            raise ValueError('confirm_frames >= 1 and positive times are required')
        self.max_after_sign = float(max_after_sign_sec)
        self.sign_time = None
        self.confirm = int(confirm_frames)
        self.exit_sec = float(exit_sec)
        self.gap = float(gap_sec)
        self.active = False
        self.disabled = False
        self.run = 0
        self.last_message = None
        self.last_zigzag = None

    def receive(self, level, now):
        """Return 'start' when the zigzag slow starts on this message, else None."""
        if self.last_message is not None and now - self.last_message > self.gap:
            self.run = 0
        self.last_message = now
        if level >= 1:
            self.last_zigzag = now
        self.run = self.run + 1 if level >= 2 else 0
        if not self.active and not self.disabled and self.run >= self.confirm:
            self.active = True
            self.run = 0
            return 'start'
        return None

    def tick(self, now):
        """Return 'exit' (zone left, no sign yet) or 'timeout' (sign seen long ago) when a fallback ends the slow."""
        if not self.active:
            return None
        if self.sign_time is not None:
            if now - self.sign_time >= self.max_after_sign:
                self.active = False
                return 'timeout'
            return None
        if self.last_zigzag is None or now - self.last_zigzag >= self.exit_sec:
            self.active = False
            self.run = 0
            return 'exit'
        return None

    def disable(self, now=None):
        """Crosswalk sign seen: zigzag never starts a slow again; an active one runs to /start_crosswalk."""
        was = self.disabled
        self.disabled = True
        if not was and self.active:
            self.sign_time = now
        return not was

    def stop(self, disable=False):
        """End an active slow now; returns True if one was active."""
        was = self.active
        self.active = False
        self.run = 0
        if disable:
            self.disabled = True
        return was

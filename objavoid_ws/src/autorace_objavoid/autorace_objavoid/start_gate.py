"""Latched mission entry; this gate does not alter avoidance control."""
from dataclasses import dataclass
import math


@dataclass
class StartGate:
    x_min: float = 0.0
    x_max: float = 2.5
    y_min: float = -0.75
    y_max: float = 0.75
    minimum_objects: int = 2
    image_timeout: float = 0.3
    active: bool = False
    white_stamp: float = -math.inf
    white_received: float = -math.inf
    white_hit: bool = False

    def __post_init__(self):
        if (not all(math.isfinite(v) for v in (self.x_min, self.x_max, self.y_min,
                                                self.y_max, self.image_timeout))
                or self.x_min < 0 or self.x_min >= self.x_max or self.y_min >= self.y_max
                or type(self.minimum_objects) is not int or self.minimum_objects < 2
                or self.image_timeout <= 0):
            raise ValueError('Invalid mission entry ROI/count/timeout')

    def reset(self):
        self.active = self.white_hit = False
        self.white_stamp = self.white_received = -math.inf

    def observe_white(self, hit, stamp, received):
        if stamp > self.white_stamp:
            self.white_hit, self.white_stamp, self.white_received = hit, stamp, received

    def count(self, groups, c):
        # Clusters are in laser axes; ROI is in base_link axes, as read_scan uses.
        yaw = math.radians(c.lidar_yaw_deg)
        co, si = math.cos(yaw), math.sin(yaw)
        count = 0
        for group in groups:
            inside = 0
            for x, y in group:
                bx, by = c.lidar_x + co*x - si*y, c.lidar_y + si*x + co*y
                inside += self.x_min <= bx <= self.x_max and self.y_min <= by <= self.y_max
            count += inside >= c.cluster_min_points
        return count

    def update(self, count, scan_fresh, ros_now, now, future_tolerance):
        if (not self.active and scan_fresh and count >= self.minimum_objects
                and self.white_hit
                and -future_tolerance <= ros_now - self.white_stamp < self.image_timeout
                and 0 <= now - self.white_received < self.image_timeout):
            self.active = True
        return self.active

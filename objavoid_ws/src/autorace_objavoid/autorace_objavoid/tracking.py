"""Fixed lateral avoidance goals, independent of the collision preview length."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class AvoidanceTarget:
    x: float
    y: float
    yaw: float
    lateral: float
    frame: str

    @classmethod
    def start(cls, pose, side, c, road=None):
        if road is None:
            return cls(pose.x, pose.y, pose.yaw, side * c.direction_offset, pose.frame)
        yaw = pose.yaw + math.atan2(road.nx, road.ny)
        nx, ny = -math.sin(yaw), math.cos(yaw)
        lane = max(1, min(3, road.lane - side))
        return cls(pose.x - nx * road.offset, pose.y - ny * road.offset,
                   yaw, (2 - lane) * c.lane_width, pose.frame)

    def errors(self, pose):
        lateral = (-math.sin(self.yaw) * (pose.x - self.x)
                   + math.cos(self.yaw) * (pose.y - self.y))
        heading = math.atan2(math.sin(pose.yaw - self.yaw), math.cos(pose.yaw - self.yaw))
        return self.lateral - lateral, heading

    def command(self, pose, c):
        error, heading = self.errors(pose)
        lookahead = max(c.preview_distance, c.lane_width)
        desired_heading = math.atan2(error, lookahead)
        angle = c.lane_gain * (desired_heading - heading)
        return max(-c.near_steering, min(c.near_steering, angle))

    def reached(self, pose, c):
        error, heading = self.errors(pose)
        return abs(error) <= max(.02, c.lane_change_hysteresis) and abs(heading) <= math.radians(5)

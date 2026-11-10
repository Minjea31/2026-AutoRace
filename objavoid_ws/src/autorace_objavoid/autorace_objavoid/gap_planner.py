"""Forward-only gap fallback, armed solely by five distinct blocked scans."""
from dataclasses import dataclass, replace
import math
import time


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def advance(x, y, yaw, angle, length, wheelbase):
    curvature = math.tan(angle) / wheelbase
    turn = curvature * length
    if abs(curvature) < 1e-8:
        dx, dy = length, 0.
    else:
        dx, dy = math.sin(turn)/curvature, (1-math.cos(turn))/curvature
    co, si = math.cos(yaw), math.sin(yaw)
    return x+co*dx-si*dy, y+si*dx+co*dy, yaw+turn


def geometry_at(scan, x, y, yaw):
    co, si = math.cos(yaw), math.sin(yaw)
    def point(px, py):
        return co*(px-x)+si*(py-y), -si*(px-x)+co*(py-y)
    road = scan.road
    if road is not None:
        road = replace(road, nx=co*road.nx+si*road.ny, ny=-si*road.nx+co*road.ny,
                       offset=road.offset+road.nx*x+road.ny*y)
    return replace(scan, points=tuple(point(px, py) for px, py in scan.points),
                   boxes=tuple((*point(bx, by), hx, hy, a-yaw) for bx, by, hx, hy, a in scan.boxes),
                   road=road)


@dataclass(frozen=True)
class Leg:
    x: float
    y: float
    yaw: float
    steering: float
    length: float


class GapFallback:
    """No normal-control decision is changed before the trigger threshold."""
    def __init__(self, path_clear, enabled=True, extra_width=.1, on_search=None, on_failure=None):
        if type(enabled) is not bool or not math.isfinite(extra_width) or extra_width < 0:
            raise ValueError('invalid gap fallback settings')
        self.path_clear = path_clear
        self.enabled, self.extra_width = enabled, extra_width
        self.on_search = on_search
        self.on_failure = on_failure
        self.reset()

    @property
    def active(self):
        return bool(self.route)

    def reset(self):
        self.blocked_frames = 0
        self.last_stamp = None
        self.route, self.samples = (), ()
        self.index, self.frame = 0, ''
        self.checked_stamp = None
        self.last_failure = None

    def _config(self, c, length):
        return replace(c, preview_distance=length,
                       min_preview_distance=min(c.min_preview_distance, length),
                       clearance_margin=max(c.clearance_margin, self.extra_width/2))

    def update(self, primary, checked, scan, c, pose):
        state = primary[0]
        if not self.enabled:
            self.reset()
            return primary, checked
        if state.startswith(('WAIT_', 'STOP_')) and state != 'STOP_PATH_BLOCKED':
            self.reset()
            return primary, checked
        if not scan.front.known(c) or scan.front.distance <= c.stop_distance:
            self.reset()
            safety = 'STOP_FRONT_UNKNOWN' if not scan.front.known(c) else 'STOP_TOO_CLOSE'
            return (safety, 0., 0., 0), c
        if scan.road is not None and scan.road.body_gap(
                0., 0., 0., c.vehicle_length/2+c.clearance_margin,
                c.vehicle_width/2+c.clearance_margin) <= 0:
            self.reset()
            return ('STOP_ROAD_BOUNDARY', 0., 0., 0), c
        if self.active:
            return self.follow(scan, c, pose)
        if state != 'STOP_PATH_BLOCKED':
            self.blocked_frames = 0
            self.last_stamp = scan.stamp
            return primary, checked
        if self.last_stamp == scan.stamp:
            return primary, checked
        if self.last_stamp is not None and scan.stamp < self.last_stamp:
            self.blocked_frames = 0
        self.last_stamp = scan.stamp
        self.blocked_frames += 1
        if self.blocked_frames < 5 or self.blocked_frames % 5:
            return primary, checked
        if pose is None:
            return ('STOP_GAP_ODOM', 0., 0., 0), c
        if self.on_search is not None:
            self.on_search()
        if self.plan(scan, c, pose):
            return self.follow(scan, c, pose)
        if self.on_failure is not None:
            self.on_failure(self.last_failure or 'GAP_NO_PATH')
        return primary, checked

    def gaps(self, scan, c):
        """Free lateral intervals at observed obstacle stations, in road axes."""
        heading = math.atan2(-scan.road.nx, scan.road.ny) if scan.road is not None else 0.
        co, si = math.cos(heading), math.sin(heading)
        boxes = []
        for x, y, hx, hy, yaw in scan.boxes:
            a = yaw-heading
            boxes.append((co*x+si*y, -si*x+co*y,
                          abs(math.cos(a))*hx+abs(math.sin(a))*hy,
                          abs(math.sin(a))*hx+abs(math.cos(a))*hy))
        required = c.vehicle_width + self.extra_width
        candidates = []
        for station in sorted(set(x for x, y, hx, hy in boxes if x+hx > c.vehicle_length/2)):
            occupied = sorted((y-hy, y+hy, x+hx) for x, y, hx, hy in boxes
                              if abs(x-station) <= hx+c.vehicle_length/2+c.clearance_margin)
            if not occupied:
                continue
            goal_x = max(v[2] for v in occupied)+c.vehicle_length/2+c.clearance_margin
            merged = []
            for low, high, _ in occupied:
                if merged and low <= merged[-1][1]:
                    merged[-1] = (merged[-1][0], max(high, merged[-1][1]))
                else:
                    merged.append((low, high))
            intervals = []
            if scan.road is not None:
                low = -scan.road.half_width-scan.road.offset
                high = scan.road.half_width-scan.road.offset
                for a, b in merged:
                    if a > low:
                        intervals.append((low, min(a, high)))
                    low = max(low, b)
                intervals.append((low, high))
            else:
                intervals = [(a[1], b[0]) for a, b in zip(merged, merged[1:])]
            for low, high in intervals:
                if high-low >= required:
                    lateral = (low+high)/2
                    candidates.append((abs(lateral)+.15*goal_x, goal_x, lateral, heading))
        return sorted(set(candidates))[:8]

    def plan(self, scan, c, pose):
        """Shoot two forward arcs toward a gap, then cross its obstacle station."""
        self.last_failure = None
        try:
            self._config(c, c.preview_distance)
        except ValueError:
            self.last_failure = 'GAP_INVALID_CONFIG'
            return False
        started = time.perf_counter()
        candidates = []
        gaps = self.gaps(scan, c)
        if not gaps:
            self.last_failure = 'GAP_NO_WIDE_GAP'
            return False
        for _, goal_x, lateral, heading in gaps:
            co, si = math.cos(heading), math.sin(heading)
            for sign in (-1, 1):
                if not (scan.left if sign > 0 else scan.right).known(c):
                    continue
                for magnitude in (c.near_steering, c.near_steering*.75, c.near_steering*.5):
                    angle = sign*magnitude
                    maximum = min(max(goal_x+abs(lateral), .3), math.pi/2*c.wheelbase/math.tan(magnitude))
                    for correction_magnitude in (c.near_steering, c.near_steering*.65):
                        def shoot(length):
                            x, y, yaw = advance(0., 0., 0., angle, length, c.wheelbase)
                            delta = wrap(heading-yaw)
                            correction = math.copysign(correction_magnitude, delta)
                            recovery = abs(delta)*c.wheelbase/math.tan(correction_magnitude)
                            ex, ey, final_yaw = advance(x, y, yaw, correction, recovery, c.wheelbase)
                            return -si*ex+co*ey-lateral, (x, y, yaw, correction, recovery, ex, ey, final_yaw)
                        previous = .01
                        value, _ = shoot(previous)
                        for step in range(1, 9):
                            current = .01+(maximum-.01)*step/8
                            other, _ = shoot(current)
                            if value*other <= 0:
                                low, high = previous, current
                                low_value = value
                                for _ in range(12):
                                    middle = (low+high)/2
                                    mid_value, _ = shoot(middle)
                                    if low_value*mid_value <= 0:
                                        high = middle
                                    else:
                                        low, low_value = middle, mid_value
                                length = (low+high)/2
                                _, data = shoot(length)
                                x, y, yaw, correction, recovery, ex, ey, final_yaw = data
                                remaining = max(0., goal_x-(co*ex+si*ey))
                                legs = [Leg(0., 0., 0., angle, length)]
                                if recovery > 1e-4:
                                    legs.append(Leg(x, y, yaw, correction, recovery))
                                if remaining > 1e-4:
                                    legs.append(Leg(ex, ey, final_yaw, 0., remaining))
                                candidates.append((sum(l.length for l in legs), legs))
                                break
                            previous, value = current, other
        if not candidates:
            self.last_failure = ('GAP_NO_ENTRY_CURVE' if scan.left.known(c) or scan.right.known(c)
                                 else 'GAP_SIDE_UNKNOWN')
            return False
        blocked_entry = blocked_route = 0
        for _, legs in sorted(candidates, key=lambda item: item[0]):
            if time.perf_counter()-started > .1:
                self.last_failure = 'GAP_SEARCH_TIMEOUT'
                break
            clear = True
            for index, leg in enumerate(legs):
                if not self.path_clear(geometry_at(scan, leg.x, leg.y, leg.yaw), leg.steering,
                                       self._config(c, leg.length)):
                    blocked_entry += index == 0
                    blocked_route += index != 0
                    clear = False
                    break
            if clear:
                co, si = math.cos(pose.yaw), math.sin(pose.yaw)
                self.route = tuple(Leg(pose.x+co*l.x-si*l.y, pose.y+si*l.x+co*l.y,
                                       pose.yaw+l.yaw, l.steering, l.length) for l in legs)
                samples = []
                for index, leg in enumerate(self.route):
                    count = max(1, math.ceil(leg.length/.025))
                    for i in range(count+1):
                        progress = leg.length*i/count
                        samples.append((*advance(leg.x, leg.y, leg.yaw, leg.steering, progress, c.wheelbase),
                                        index, progress))
                self.samples, self.index, self.frame = tuple(samples), 0, pose.frame
                self.checked_stamp = None
                return True
        if self.last_failure is None:
            self.last_failure = ('GAP_ENTRY_BLOCKED' if blocked_entry and not blocked_route
                                 else 'GAP_ROUTE_BLOCKED')
        return False

    def follow(self, scan, c, pose):
        def stop(state, reason=None):
            self.reset()
            self.last_stamp = scan.stamp
            self.last_failure = reason
            if reason is not None and self.on_failure is not None:
                self.on_failure(reason)
            return (state, 0., 0., 0), c
        if pose is None or pose.frame != self.frame:
            return stop('STOP_GAP_ODOM')
        if not scan.front.known(c):
            return stop('STOP_FRONT_UNKNOWN')
        if scan.front.distance <= c.stop_distance:
            return stop('STOP_TOO_CLOSE')
        nearest = min(range(self.index, len(self.samples)),
                      key=lambda i: math.hypot(self.samples[i][0]-pose.x, self.samples[i][1]-pose.y))
        self.index = nearest
        point = self.samples[nearest]
        if math.hypot(point[0]-pose.x, point[1]-pose.y) > .15:
            return stop('STOP_GAP_DEVIATED')
        goal = self.samples[-1]
        if (nearest >= len(self.samples)-3 and math.hypot(goal[0]-pose.x, goal[1]-pose.y) <= .06
                and abs(wrap(goal[2]-pose.yaw)) <= math.radians(5)):
            return stop('GAP_DONE')
        co, si = math.cos(pose.yaw), math.sin(pose.yaw)
        def local(x, y):
            return co*(x-pose.x)+si*(y-pose.y), -si*(x-pose.x)+co*(y-pose.y)
        if scan.stamp != self.checked_stamp:
            for index in range(point[3], len(self.route)):
                leg = self.route[index]
                if index == point[3]:
                    x, y, yaw, _, progress = point
                    length = leg.length-progress
                else:
                    x, y, yaw, length = leg.x, leg.y, leg.yaw, leg.length
                if length <= 1e-4:
                    continue
                lx, ly = local(x, y)
                if not self.path_clear(geometry_at(scan, lx, ly, yaw-pose.yaw), leg.steering,
                                       self._config(c, length)):
                    return stop('STOP_GAP_BLOCKED', 'GAP_ROUTE_BLOCKED')
            self.checked_stamp = scan.stamp
        target = goal
        for sample in self.samples[nearest:]:
            if math.hypot(sample[0]-pose.x, sample[1]-pose.y) >= .18:
                target = sample
                break
        tx, ty = local(target[0], target[1])
        distance2 = tx*tx+ty*ty
        angle = math.atan2(2*c.wheelbase*ty, max(distance2, 1e-6))
        angle = max(-c.near_steering, min(c.near_steering, angle))
        minimum = max(c.min_preview_distance, c.near_speed*(c.scan_timeout+1/c.publish_rate))
        for steering in dict.fromkeys((angle, angle*.75)):
            if steering and not (scan.left if steering > 0 else scan.right).known(c):
                continue
            low, high = minimum, c.preview_distance
            if not self.path_clear(scan, steering, self._config(c, low)):
                continue
            if self.path_clear(scan, steering, self._config(c, high)):
                low = high
            else:
                for _ in range(8):
                    middle = (low+high)/2
                    if self.path_clear(scan, steering, self._config(c, middle)):
                        low = middle
                    else:
                        high = middle
            checked = self._config(c, low)
            return ('GAP_FOLLOW', min(c.near_speed, .1), steering, 0), checked
        return stop('STOP_GAP_BLOCKED', 'GAP_COMMAND_BLOCKED')

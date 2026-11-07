"""Ordered path memory. Coordinates supplied to remaining() are vehicle-relative."""
import math


def projection(point, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    norm = dx*dx+dy*dy
    if norm < 1e-12:
        return None
    t = max(0., min(1., ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/norm))
    q = tuple(a[k]+t*(b[k]-a[k]) for k in range(3))
    return math.dist(point[:2], q[:2]), t, q


class PathMemory:
    def __init__(self):
        self.clear()

    def clear(self):
        self.points = []
        self.index = 0
        self.finished = False

    def store(self, points):
        """Keep a compatible previously observed tail beyond the new observation."""
        points = list(points)
        if len(points) < 2 or not all(math.isfinite(v) for q in points for v in q):
            return False
        old = self.points[self.index:] if not self.finished else []
        candidates = []
        dx, dy = points[-1][0]-points[-2][0], points[-1][1]-points[-2][1]
        for i, (a, b) in enumerate(zip(old, old[1:])):
            hit = projection(points[-1], a, b)
            vx, vy = b[0]-a[0], b[1]-a[1]
            norm = math.hypot(dx, dy)*math.hypot(vx, vy)
            if hit and hit[0] <= .10 and norm > 1e-12 and (dx*vx+dy*vy)/norm >= math.cos(math.pi/4):
                candidates.append((hit[0], i))
        if candidates:
            _, i = min(candidates)
            tail = old[i+1:]
            while tail and math.dist(points[-1][:2], tail[0][:2]) < .02:
                tail = tail[1:]
            points.extend(tail)
        self.points, self.index, self.finished = points[:1000], 0, False
        return True

    def remaining(self, local_points, goal_tolerance, max_distance):
        if self.finished:
            return [], 'memory_goal_reached'
        if len(local_points) != len(self.points) or len(local_points) < 2:
            return [], 'memory_path_unavailable'
        # Restrict nearest-point search along the ordered path to avoid jumping
        # to a distant branch of a hairpin merely because it is nearby in XY.
        choices, arc = [], 0.
        for i in range(self.index, len(local_points)-1):
            a, b = local_points[i:i+2]
            hit = projection((0., 0., 0.), a, b)
            if hit:
                choices.append((hit[0], i, hit[1], hit[2]))
            arc += math.dist(a[:2], b[:2])
            if arc >= max_distance:
                break
        if not choices:
            return [], 'memory_path_unusable'
        distance, i, t, q = min(choices)
        if distance > max_distance:
            return [], 'memory_path_too_far'
        end = local_points[-1]
        if (i == len(local_points)-2
                and (math.hypot(end[0], end[1]) <= goal_tolerance or t >= 1.-1e-8)):
            self.finished = True
            return [], 'memory_goal_reached'
        self.index = max(self.index, i)
        return [q]+local_points[i+1:], None

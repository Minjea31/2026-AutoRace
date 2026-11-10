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
        self.update_reason = 'empty'

    def refresh(self, points, validate):
        """Latest observation owns the prefix; preserve only an aligned old tail."""
        points = list(points)
        if not validate(points):
            return False
        old = self.points[self.index:] if not self.finished else []
        candidates = []
        for i, (a, b) in enumerate(zip(old, old[1:])):
            hit = projection(points[-1], a, b)
            dx, dy = points[-1][0]-points[-2][0], points[-1][1]-points[-2][1]
            vx, vy = b[0]-a[0], b[1]-a[1]
            if hit and hit[0] <= .10 and dx*vx+dy*vy > 0:
                tail = old[i+1:]
                while tail and math.dist(points[-1], tail[0]) < 1e-8:
                    tail = tail[1:]
                if tail and validate(points+tail):
                    candidates.append((hit[0], points+tail))
        self.points = min(candidates, key=lambda item: item[0])[1] if candidates else points
        self.index, self.finished = 0, False
        self.update_reason = 'refreshed_with_tail' if candidates else 'refreshed'
        return True

    def store(self, points):
        """Initialize once; subsequently append only an aligned extension of the goal.

        The old prefix is never replaced by a shorter or laterally shifted detection.
        Extension is measured along the new polyline past the old goal, not by
        Euclidean distance from the vehicle (which is misleading in a hairpin).
        """
        points = list(points)
        if len(points) < 2 or not all(math.isfinite(v) for q in points for v in q):
            self.update_reason = 'invalid_observation'
            return False
        if not self.points:
            self.points, self.index, self.finished = points[:], 0, False
            self.update_reason = 'stored'
            return True
        old = self.points[self.index:]
        dx, dy = old[-1][0]-old[-2][0], old[-1][1]-old[-2][1]
        candidates = []
        for i, (a, b) in enumerate(zip(points, points[1:])):
            hit = projection(old[-1], a, b)
            vx, vy = b[0]-a[0], b[1]-a[1]
            norm = math.hypot(dx, dy)*math.hypot(vx, vy)
            if hit and hit[0] <= .10 and norm > 1e-12 and (dx*vx+dy*vy)/norm >= -1e-12:
                candidates.append((hit[0], i, hit[1], hit[2]))
        if not candidates:
            self.update_reason = 'kept_no_aligned_extension'
            return False
        _, i, t, q = min(candidates)
        # Require the old end to lie on the new observation, rather than beyond it.
        # A clamped endpoint alone must never be mistaken for forward progress.
        a, b = points[i:i+2]
        vx, vy = b[0]-a[0], b[1]-a[1]
        raw_t = ((old[-1][0]-a[0])*vx+(old[-1][1]-a[1])*vy)/(vx*vx+vy*vy)
        if not -1e-8 <= raw_t <= 1.+1e-8:
            self.update_reason = 'kept_shorter_path'
            return False
        tail = points[i+1:]
        extension = math.dist(q[:2], tail[0][:2])+sum(
            math.dist(a[:2], b[:2]) for a, b in zip(tail, tail[1:]))
        if extension <= 1e-8:
            self.update_reason = 'kept_shorter_path'
            return False
        while tail and math.dist(old[-1][:2], tail[0][:2]) <= 1e-8:
            tail = tail[1:]
        if not tail:
            self.update_reason = 'kept_capacity_limit'
            return False
        tx, ty = tail[0][0]-old[-1][0], tail[0][1]-old[-1][1]
        norm = math.hypot(dx, dy)*math.hypot(tx, ty)
        if norm < 1e-12 or (dx*tx+dy*ty)/norm < -1e-12:
            self.update_reason = 'kept_discontinuous_join'
            return False
        self.points, self.index, self.finished = old+tail, 0, False
        self.update_reason = 'extended'
        return True

    def replace_from_vehicle(self, points, local_points, max_distance):
        """Replace an unusable memory only with a nearby forward continuation."""
        if len(points) != len(local_points) or len(points) < 2 or not all(
                math.isfinite(v) for q in points+local_points for v in q):
            self.update_reason = 'recovery_invalid_path'
            return False
        candidates = []
        for i, (a, b) in enumerate(zip(local_points, local_points[1:])):
            hit = projection((0., 0., 0.), a, b)
            length = math.dist(a[:2], b[:2])
            if hit and length > 1e-9 and (b[0]-a[0])/length >= math.cos(math.pi/4):
                candidates.append((hit[0], i, hit[1]))
        if not candidates:
            self.update_reason = 'recovery_direction_mismatch'
            return False
        distance, i, t = min(candidates)
        if distance > max_distance:
            self.update_reason = 'recovery_too_far_from_vehicle'
            return False
        a, b = points[i:i+2]
        q = tuple(a[k]+t*(b[k]-a[k]) for k in range(3))
        tail = list(points[i+1:])
        while tail and math.dist(q[:2], tail[0][:2]) < 1e-8:
            tail.pop(0)
        if not tail:
            self.update_reason = 'recovery_insufficient_forward_path'
            return False
        self.points, self.index, self.finished = [q]+tail, 0, False
        self.update_reason = 'replaced_unusable_memory'
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

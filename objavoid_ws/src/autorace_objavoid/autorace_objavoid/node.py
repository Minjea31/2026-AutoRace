"""Two-obstacle LiDAR avoidance with deep lane corridor guidance."""
from dataclasses import dataclass, replace
import math
import time
import numpy as np
from .start_gate import StartGate
from .latest_scan import LatestScan

@dataclass(frozen=True)
class Config:
    cruise_speed: float = 0.5
    avoid_speed: float = 0.4
    near_speed: float = 0.2
    near_distance: float = 0.5
    far_distance: float = 2.5
    roi_half_width: float = 0.75
    priority_half_angle_deg: float = 35.0
    max_avoid_steering: float = 0.6981317008
    max_lane_steering: float = 0.35
    lane_gain: float = 0.8
    lane_width: float = 0.52
    scan_timeout: float = 0.0  # Keep the last valid scan until a newer one arrives.
    lane_timeout: float = 0.5
    future_tolerance: float = 0.05
    publish_rate: float = 20.0
    front_half_angle_deg: float = 25.0
    side_limit_angle_deg: float = 90.0
    lidar_yaw_deg: float = 0.0
    lidar_x: float = 0.103
    lidar_y: float = 0.0
    inf_is_clear: bool = False
    wheelbase: float = 0.2
    preview_distance: float = 0.5
    max_preview_distance: float = 1.0
    cluster_jump_distance: float = 0.12
    cluster_min_points: int = 3
    cluster_max_distance: float = 3.5
    object_max_extent: float = 1.0
    vehicle_length: float = 0.322
    vehicle_width: float = 0.220
    clearance_margin: float = 0.05
    visual_road_distance: float = 2.5
    turn_distance: float = -1.0  # Negative: legacy single turn preview.
    follow_steering: float = 0.0

    def __post_init__(self):
        if not all(math.isfinite(v) for v in vars(self).values()):
            raise ValueError('settings must be finite')
        if not 0.2 <= self.near_speed <= self.avoid_speed <= self.cruise_speed <= 0.5:
            raise ValueError('speed must be within 0.2..0.5 m/s')
        if not 0 < self.max_lane_steering <= self.max_avoid_steering <= math.radians(40) + 1e-9:
            raise ValueError('invalid steering limits')
        if (min(self.lane_timeout, self.publish_rate, self.wheelbase,
                self.preview_distance, self.cluster_jump_distance, self.cluster_max_distance,
                self.object_max_extent, self.vehicle_length, self.vehicle_width,
                self.near_distance, self.far_distance, self.roi_half_width, self.max_preview_distance) <= 0
                or self.scan_timeout < 0 or self.clearance_margin < 0 or self.future_tolerance < 0 or self.lane_gain < 0
                or self.turn_distance < -1 or abs(self.follow_steering) > self.max_avoid_steering
                or 3*self.lane_width <= self.vehicle_width + 2*self.clearance_margin
                or type(self.cluster_min_points) is not int or self.cluster_min_points < 1
                or not 0 < self.priority_half_angle_deg < 90
                or not 0 < self.front_half_angle_deg < self.side_limit_angle_deg <= 110):
            raise ValueError('invalid geometry/timing settings')


@dataclass(frozen=True)
class Sector:
    distance: float
    valid_beams: int


@dataclass(frozen=True)
class RoadBand:
    nx: float
    ny: float
    offset: float
    half_width: float
    lane: int

    def body_gap(self, x, y, yaw, half_length, half_width):
        co, si = math.cos(yaw), math.sin(yaw)
        radius = (half_length * abs(self.nx * co + self.ny * si)
                  + half_width * abs(-self.nx * si + self.ny * co))
        return self.half_width - abs(self.nx * x + self.ny * y + self.offset) - radius

    def point(self, distance, lateral):
        return (self.ny * distance + self.nx * (lateral - self.offset),
                -self.nx * distance + self.ny * (lateral - self.offset))

@dataclass(frozen=True)
class MeasuredRoad(RoadBand):
    # Each unit normal points into the half-plane containing the vehicle.
    # A single edge imposes exactly one constraint, never a fabricated opposite.
    edges: tuple = ()

    def body_gap(self, x, y, yaw, half_length, half_width):
        co,si=math.cos(yaw),math.sin(yaw)
        return min(nx*x+ny*y+offset-half_length*abs(nx*co+ny*si)
                   -half_width*abs(-nx*si+ny*co) for nx,ny,offset in self.edges)


@dataclass(frozen=True)
class Sectors:
    left: Sector
    front: Sector
    right: Sector
    stamp: float
    received: float
    frame_id: str = 'laser_link'
    points: tuple = ()
    boxes: tuple = ()  # (centre x/y, half length/width, yaw), matching displayed boxes.
    road: RoadBand | None = None


def read_scan(msg, received, c):
    if (not all(math.isfinite(v) for v in (
            msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max))
            or msg.angle_increment == 0 or not 0 <= msg.range_min < msg.range_max
            or len(msg.ranges) < 2
            or (len(msg.ranges) - 1) * abs(msg.angle_increment) > 2 * math.pi + .01):
        raise ValueError('invalid LaserScan geometry')
    values = {'left': [], 'front': [], 'right': []}
    points = []
    for i, value in enumerate(msg.ranges):
        angle = math.degrees(msg.angle_min + i * msg.angle_increment) + c.lidar_yaw_deg
        angle = (angle + 180) % 360 - 180
        a = math.radians(angle)
        co, si = math.cos(a), math.sin(a)
        valid = math.isfinite(value) and max(msg.range_min, 1e-6) <= value <= msg.range_max
        if valid:
            points.append((c.lidar_x + value * co, c.lidar_y + value * si))
        if abs(angle) <= c.front_half_angle_deg:
            name = 'front'
        elif c.front_half_angle_deg < angle <= c.side_limit_angle_deg:
            name = 'left'
        elif -c.side_limit_angle_deg <= angle < -c.front_half_angle_deg:
            name = 'right'
        else:
            continue
        if valid:
            values[name].append(float(value))
        elif c.inf_is_clear and value == math.inf:
            values[name].append(float(msg.range_max))
    sectors = {name: Sector(min(points) if points else math.inf, len(points))
               for name, points in values.items()}
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if not math.isfinite(stamp) or not math.isfinite(received):
        raise ValueError('invalid scan time')
    return Sectors(sectors['left'], sectors['front'], sectors['right'], stamp, received,
                   getattr(msg.header, 'frame_id', '') or 'laser_link', tuple(points))


def cluster_scan(msg, c):
    """Group consecutive finite laser returns by Euclidean distance, in scan frame."""
    groups, current = [], []
    first_valid = last_valid = False
    limit = min(msg.range_max, c.cluster_max_distance)
    for index, value in enumerate(msg.ranges):
        valid = math.isfinite(value) and max(msg.range_min, 1e-6) <= value <= limit
        if index == 0:
            first_valid = valid
        last_valid = valid
        if not valid:
            if current:
                groups.append(current)
                current = []
            continue
        angle = msg.angle_min + index * msg.angle_increment
        point = (value * math.cos(angle), value * math.sin(angle))
        if current and math.dist(current[-1], point) > c.cluster_jump_distance:
            groups.append(current)
            current = []
        current.append(point)
    if current:
        groups.append(current)
    full_circle = (len(msg.ranges) - 1) * abs(msg.angle_increment) >= (
        2 * math.pi - 2 * abs(msg.angle_increment))
    if (full_circle and first_valid and last_valid and len(groups) > 1
            and math.dist(groups[-1][-1], groups[0][0]) <= c.cluster_jump_distance):
        tail = groups[-1]
        if math.dist(tail[-1], groups[0][0]) < 1e-8:
            tail = tail[:-1]  # A 360-degree scan can repeat the same endpoint.
        groups = [tail + groups[0]] + groups[1:-1]
    return tuple(tuple(points) for points in groups if len(points) >= c.cluster_min_points)


def scan_fresh(scan, c, ros_now, steady_now):
    """Zero timeout keeps the latest valid input; reception health is separate."""
    return (scan is not None and scan.stamp > 0
            and ros_now-scan.stamp >= -c.future_tolerance
            and steady_now >= scan.received
            and (c.scan_timeout == 0 or
                 (ros_now-scan.stamp < c.scan_timeout and steady_now-scan.received < c.scan_timeout)))


def compact_clusters(groups, c):
    """Long wall-like groups are separate from compact obstacle-side preference."""
    return tuple(p for p in groups if math.hypot(
        max(x for x, y in p) - min(x for x, y in p),
        max(y for x, y in p) - min(y for x, y in p)) <= c.object_max_extent)


def cluster_bounds(points):
    xs, ys = zip(*points)
    return min(xs) - .02, max(xs) + .02, min(ys) - .02, max(ys) + .02


def object_sectors(scan, groups, c):
    distances = {1: math.inf, -1: math.inf}
    counts = {1: scan.left.valid_beams, -1: scan.right.valid_beams}
    a = math.radians(c.lidar_yaw_deg)
    co, si = math.cos(a), math.sin(a)
    boxes = []
    for group in groups:
        xmin, xmax, ymin, ymax = cluster_bounds(group)
        x, y = (xmin + xmax) / 2, (ymin + ymax) / 2
        boxes.append((c.lidar_x + co * x - si * y, c.lidar_y + si * x + co * y,
                      (xmax - xmin) / 2, (ymax - ymin) / 2, a))
        for x, y in group:
            bx, by = c.lidar_x + co * x - si * y, c.lidar_y + si * x + co * y
            # Include adjacent/rear corners outside the old +/-90 degree sectors.
            if (-c.vehicle_length / 2 - .1 <= bx <= c.far_distance + c.vehicle_length / 2
                    and abs(by) > c.vehicle_width / 2 + c.clearance_margin):
                side = 1 if by > 0 else -1
                distances[side] = min(distances[side], math.hypot(x, y))
                counts[side] += 1
    return Sectors(Sector(distances[1], counts[1]), scan.front,
                   Sector(distances[-1], counts[-1]), scan.stamp, scan.received,
                   scan.frame_id, scan.points, tuple(boxes), scan.road)


def path_clearances(scan, routes, gap_cap=None):
    """Check all candidate swept bodies in bulk, keeping every raw return and box."""
    if not routes:
        return []
    poses = np.asarray([preview_poses(angle,c) for angle,c in routes])
    x,y,yaw = poses[:,:,0],poses[:,:,1],poses[:,:,2]
    co,si = np.cos(yaw),np.sin(yaw)
    lengths,widths,caps = [],[],[]
    for angle,c in routes:
        hl,hw = c.vehicle_length/2+c.clearance_margin,c.vehicle_width/2+c.clearance_margin
        step = c.preview_distance/24
        k = max(abs(math.tan(angle)/c.wheelbase),abs(math.tan(c.follow_steering)/c.wheelbase))
        padding = step/2+2*math.hypot(hl,hw)*math.sin(min(math.pi/2,k*step/4))
        lengths.append(hl+padding);widths.append(hw+padding);caps.append(c.far_distance if gap_cap is None else min(c.far_distance,gap_cap))
    hl,hw = np.asarray(lengths)[:,None],np.asarray(widths)[:,None]
    gaps = np.asarray(caps,dtype=float)
    if scan.road is not None:
        r=scan.road
        if isinstance(r,MeasuredRoad):
            for nx,ny,offset in r.edges:
                radius=hl*np.abs(nx*co+ny*si)+hw*np.abs(-nx*si+ny*co)
                gaps=np.minimum(gaps,np.min(nx*x+ny*y+offset-radius,axis=1))
        else:
            radius=hl*np.abs(r.nx*co+r.ny*si)+hw*np.abs(-r.nx*si+r.ny*co)
            gaps=np.minimum(gaps,np.min(r.half_width-np.abs(r.nx*x+r.ny*y+r.offset)-radius,axis=1))
    cloud=np.asarray(scan.points,dtype=float).reshape(-1,2)
    if len(cloud):
        # Exact broad phase: outside this union a point cannot reduce the
        # capped score for any swept rectangle. No obstacle is subsampled.
        rx,ry=hl*np.abs(co)+hw*np.abs(si),hl*np.abs(si)+hw*np.abs(co)
        extra=max(caps)
        keep=((cloud[:,0]>=np.min(x-rx)-extra)&(cloud[:,0]<=np.max(x+rx)+extra)
              &(cloud[:,1]>=np.min(y-ry)-extra)&(cloud[:,1]<=np.max(y+ry)+extra))
        cloud=cloud[keep]
    co,si,x,y,hl,hw = (v[:,:,None] for v in (co,si,x,y,hl,hw))
    if len(cloud):
        dx,dy=cloud[:,0]-x,cloud[:,1]-y
        gx,gy=np.abs(co*dx+si*dy)-hl,np.abs(-si*dx+co*dy)-hw
        hit=np.any((gx<=0)&(gy<=0),axis=(1,2))
        distance=np.min(np.hypot(np.maximum(gx,0),np.maximum(gy,0)),axis=(1,2))
        gaps=np.minimum(gaps,np.where(hit,0.,distance))
    if scan.boxes:
        ox,oy,bx,by,a=np.asarray(scan.boxes,dtype=float).T
        oc,os=np.cos(a),np.sin(a)
        dx,dy=ox-x,oy-y
        along,across=np.abs(co*oc+si*os),np.abs(si*oc-co*os)
        gap=np.maximum.reduce([
            np.abs(co*dx+si*dy)-hl-bx*along-by*across,
            np.abs(-si*dx+co*dy)-hw-bx*across-by*along,
            np.abs(oc*dx+os*dy)-bx-hl*along-hw*across,
            np.abs(-os*dx+oc*dy)-by-hl*across-hw*along])
        gaps=np.minimum(gaps,np.min(gap,axis=(1,2)))
    return np.maximum(gaps,0.).tolist()


def path_clearance(scan, steering, c):
    return path_clearances(scan,[(steering,c)])[0]


def path_clear(scan, steering, c):
    return path_clearance(scan, steering, c) > 0.


def lane_hint(data, c):
    if len(data) != 11 or not all(math.isfinite(v) for v in data) or data[0] != 1:
        return None
    if data[3] <= 0:
        return None
    angle = c.lane_gain * (math.atan2(data[1], data[3]) + data[2])
    return max(-c.max_lane_steering, min(c.max_lane_steering, angle))


def lane_reference(data):
    """Connect the two reported centre points; these are not measured road edges."""
    if len(data) != 11 or data[0] != 1 or not all(math.isfinite(v) for v in data):
        return ()
    if not 0 < data[10] <= data[3]+1e-6:
        return ()
    return ((float(data[10]), float(data[8])), (float(data[3]), float(data[1])))


def preview_poses(steering, c):
    """The checked and displayed initial turn plus the planned onward turn."""
    def advance(x, y, yaw, angle, distance):
        k = math.tan(angle)/c.wheelbase
        arc = min(distance, math.radians(60)/max(abs(k),1e-9))
        dx,dy = (arc,0.) if abs(k)<1e-6 else (math.sin(k*arc)/k,(1-math.cos(k*arc))/k)
        co,si = math.cos(yaw),math.sin(yaw)
        x,y = x+co*dx-si*dy,y+si*dx+co*dy
        yaw += k*arc
        tail = distance-arc
        return x+tail*math.cos(yaw),y+tail*math.sin(yaw),yaw
    split = c.preview_distance if c.turn_distance < 0 else min(c.preview_distance,c.turn_distance)
    junction = advance(0.,0.,0.,steering,split)
    return [advance(0.,0.,0.,steering,d) if d <= split else
            advance(*junction,c.follow_steering,d-split)
            for d in (c.preview_distance*i/24 for i in range(25))]


def preview_points(steering, c):
    return [(x,y) for x,y,_ in preview_poses(steering,c)]


def default_config_args(argv, config_path):
    """Load the installed YAML unless the caller supplied a parameter file.

    Defaults precede caller arguments so explicit -p overrides retain priority.
    """
    if any(arg == '--params-file' or arg.startswith('--params-file=') for arg in argv[1:]):
        return list(argv)
    return [argv[0], '--ros-args', '--params-file', str(config_path), '--', *argv[1:]]


def lane_boundary(data, c):
    if lane_hint(data, c) is None or data[6] != 2 or data[7] != 2 or not lane_reference(data):
        return None
    heading = data[9]
    nx, ny = -math.sin(heading), math.cos(heading)
    return RoadBand(nx, ny, -nx*data[10]-ny*data[8], c.lane_width/2, 2)


def current_road(data, band, mask_fresh, c):
    """Only camera outer edges constrain avoidance; lane targets are guidance only."""
    return band if mask_fresh and band is not None else None


def obstacle_max_x(box):
    x,_,hx,hy,yaw = box
    return x+abs(math.cos(yaw))*hx+abs(math.sin(yaw))*hy


def nearest_objects(scan, c=None, primary=None):
    """Rank threats to our lane; keep all other boxes in collision checks."""
    c = c or Config()
    def radius(b):
        return abs(math.sin(b[4]))*b[2]+abs(math.cos(b[4]))*b[3]
    def on_road(b):
        r = scan.road
        if isinstance(r,MeasuredRoad):
            return all(nx*b[0]+ny*b[1]+offset >= -math.hypot(b[2],b[3]) for nx,ny,offset in r.edges)
        return r is None or abs(r.nx*b[0]+r.ny*b[1]+r.offset) <= r.half_width+radius(b)
    candidates = [b for b in scan.boxes if
        b[0] > 0. and abs(math.atan2(b[1],b[0])) <= math.radians(c.priority_half_angle_deg)+1e-12
        and b[0]-b[2] <= c.far_distance
        and abs(b[1]) <= c.roi_half_width and on_road(b)]
    threats = [b for b in candidates if abs(b[1]) <= c.lane_width/2+c.clearance_margin+radius(b)]
    threats.sort(key=lambda b:max(0.,b[0]-b[2]))
    first = None
    if primary is not None:
        matches = [b for b in candidates if math.dist(b[:2],primary[:2]) < .30
                   and abs(b[1]-primary[1]) < .15]
        if matches:
            first = min(matches,key=lambda b:math.dist(b[:2],primary[:2]))
    if first is None:
        first = threats[0] if threats else None
    if first is None:
        return []
    following = [b for b in candidates if b != first and b[0] > max(0.,first[0])+.10
                 and abs(b[1]) <= 1.5*c.lane_width+radius(b)]
    return [first]+sorted(following,key=lambda b:b[0])[:1]


def plan_avoidance(scan, c, lane_angle=0., lane_ok=False, primary=None, previous_angle=None, lane_data=None):
    """Nearest obstacle sets passing side; second obstacle resolves the onward route.

    Check finite steering samples against the swept vehicle. If every path is
    blocked, retain minimum forward speed as requested and report NO_CLEAR_PATH.
    """
    objects = nearest_objects(scan, c, primary)
    def pass_angle(box, side_hint=0):
        x,y,hx,hy,yaw = box
        radius = abs(math.sin(yaw))*hx+abs(math.cos(yaw))*hy
        side = side_hint if side_hint and abs(y) < .03 else (-1 if y >= 0 else 1)
        target_y = y+side*(radius+c.vehicle_width/2+c.clearance_margin)
        # Once the box is already clear to the side, straight is sufficient;
        # retaining its identity must not force another turn away from it.
        target_y = side*max(0.,side*target_y)
        # Aim before the obstacle's front face, not its centre: turn earlier.
        ahead = max(.30,x-hx-c.vehicle_length/2)
        return math.atan2(2*c.wheelbase*target_y,ahead*ahead+target_y*target_y)
    desired = lane_angle
    preferred = []
    if objects:
        first = pass_angle(objects[0],(-1 if objects[1][1] >= 0 else 1) if len(objects)>1 else 0)
        preferred.append(first)
        desired = first
        if len(objects) > 1:
            second = pass_angle(objects[1])
            preferred.append(second)
            remaining = obstacle_max_x(objects[0])-c.vehicle_length/2
            blend = max(.15,min(.75,1.-remaining/.6))
            # Only a primary within the forward priority cone remains ranked.
            # Passed boxes still participate in every swept-body check.
            if first*second < 0:
                blend = min(blend,.25)
            desired = (1.-blend)*first+blend*second
    clamp = lambda a: max(-c.max_avoid_steering,min(c.max_avoid_steering,a))
    desired = clamp(desired)
    angles = sorted(set(map(clamp,[desired,lane_angle,0.]+([previous_angle] if previous_angle is not None else [])+preferred+
                        [c.max_avoid_steering*i/8 for i in range(-8,9)])),
                    key=lambda a: (abs(a-desired),abs(a)))
    # Missing dashed/yellow detections reduce speed, not obstacle avoidance priority.
    # Inspect progressively shorter arcs before declaring every direction blocked.
    # Every chosen arc still uses the complete vehicle body and clearance margin.
    horizon = min(c.preview_distance,c.max_preview_distance)
    if len(objects) > 1:
        horizon = min(c.far_distance,c.max_preview_distance,max(horizon,math.hypot(*objects[1][:2])+c.vehicle_length/2+c.clearance_margin+c.lane_width))
    split = max(.25,obstacle_max_x(objects[0])+c.vehicle_length/2+.15) if objects else horizon
    for fraction in (1., .75, .5, .25):
        checked = replace(c, preview_distance=horizon*fraction,turn_distance=split,follow_steering=0.)
        routes = []
        for initial in angles:
            follows = [0.]
            if len(objects) > 1 and checked.preview_distance > split:
                junction = preview_poses(initial,replace(checked,preview_distance=split))[-1]
                x,y,hx,hy,yaw = objects[1]
                dx,dy = x-junction[0],y-junction[1]
                co,si = math.cos(junction[2]),math.sin(junction[2])
                local = (co*dx+si*dy,-si*dx+co*dy,hx,hy,yaw-junction[2])
                follow = clamp(pass_angle(local))
                follows = sorted(set([follow,0.,-c.max_lane_steering,c.max_lane_steering]),key=lambda a:abs(a-follow))
            for follow in follows:
                route = replace(checked,follow_steering=follow)
                routes.append((initial,route))
        def accepts(initial, route):
            return (len(objects)<2 or fraction<1. or
                route.preview_distance < obstacle_max_x(objects[1])+c.vehicle_length/2+c.clearance_margin or
                preview_poses(initial,route)[-1][0] > obstacle_max_x(objects[1])+c.vehicle_length/2+c.clearance_margin)
        clear = [(min(gap,.10),initial,route)
                 for (initial,route),gap in zip(routes,path_clearances(scan,routes,gap_cap=.10))
                 if gap > 0 and accepts(initial,route)]
        if not clear:
            continue
        best_gap = max(gap for gap,_,_ in clear)
        def route_cost(item):
            _,initial,route=item
            x,y,yaw=preview_poses(initial,route)[-1]
            continuity=abs(initial-previous_angle) if previous_angle is not None else 0.
            # Compare only collision-checked, near-best-clearance routes.
            # Penalise both terminal heading and sideways runaway paths.
            target_heading=lane_data[9] if lane_data is not None else 0.
            target_y=(lane_data[8]+math.tan(target_heading)*(x-lane_data[10])) if lane_data is not None else 0.
            recovery=abs(yaw-target_heading)+abs(y-target_y)/max(route.preview_distance,.1)
            # A weak quadratic cost has zero slope at the previous angle:
            # it discourages large changes without the L1 "stickiness" that
            # could outweigh the desired steering and prevent unwinding.
            return (abs(initial-desired)+.2*continuity*continuity,recovery)
        _,angle,checked = min((item for item in clear if item[0] >= best_gap-.02),key=route_cost)
        obstacle_gap = path_clearance(replace(scan,road=None),angle,checked)
        speed = c.cruise_speed if lane_ok else c.near_speed
        if objects and obstacle_gap < c.clearance_margin:
            speed = min(speed,c.avoid_speed)
        if fraction < 1.:
            speed = min(speed,c.near_speed+(c.avoid_speed-c.near_speed)*fraction)
        state = 'AVOID' if objects else 'LANE'
        if fraction < 1.:
            state += '/SHORT_PREVIEW'
        return state,max(c.near_speed,speed),angle,checked
    # No direction is certified clear. Honour continuous motion, retain the
    # obstacle-side preference rather than replacing it with lane recovery.
    return 'NO_CLEAR_PATH', c.near_speed, desired, c


def plan_in_lane(scan,c,lane_data,primary=None,previous_angle=None):
    """Use lane_deep guidance without treating white dividers as road limits."""
    hint=lane_hint(lane_data,c) if lane_data is not None else None
    if hint is None:
        return 'LANE_UNAVAILABLE',.2,0.,c
    # scan.road may carry an actual external boundary, but never fabricate one
    # from the current lane centre and its 0.52m width.
    for margin in dict.fromkeys((c.clearance_margin,0.)):
        checked=replace(c,clearance_margin=margin)
        result=plan_avoidance(scan,checked,hint,True,primary,previous_angle,lane_data)
        state,speed,angle,route=result
        if path_is_valid(state) and path_clearance(scan,angle,route)>0:
            if margin < c.clearance_margin:
                return state+'/BODY_WIDTH',.2,angle,route
            return result
    return 'NO_CLEAR_PATH',.2,0.,c


def plan_with_boundaries(scan,c,lane_angle=0.,lane_ok=False,primary=None,previous_angle=None):
    if scan.road is None:
        return 'ROAD_UNAVAILABLE',c.near_speed,lane_angle,c
    return plan_avoidance(scan,c,lane_angle,lane_ok,primary,previous_angle)


def avoidance_command(scan, c, lane_angle=0., lane_ok=False):
    """Public three-value command API; plan_avoidance also returns checked geometry."""
    return plan_avoidance(scan, c, lane_angle, lane_ok)[:3]


def path_is_valid(state):
    return state.split('/')[0] in ('AVOID','LANE')


def main(args=None):
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy
    from rcl_interfaces.msg import ParameterDescriptor
    from sensor_msgs.msg import LaserScan, CompressedImage, Image
    from std_msgs.msg import Float32MultiArray
    from ackermann_msgs.msg import AckermannDriveStamped

    class ObjavoidNode(Node):
        def __init__(self):
            super().__init__('objavoid_node')
            ro = ParameterDescriptor(read_only=True)
            self.c = Config(**{k: self.declare_parameter(k, v, ro).value for k, v in vars(Config()).items()})
            self.declare_parameter('manual_active', True)
            self.start_gate = StartGate(**{k: self.declare_parameter('start_'+k, v, ro).value
                for k, v in dict(x_min=0., x_max=2.5, y_min=-.75, y_max=.75,
                                 minimum_objects=2, image_timeout=.3).items()})
            from .white_roi import WhiteROI, load_lane_white, validate_white
            self.white_roi = WhiteROI(**{k: self.declare_parameter('white_roi_'+k, v, ro).value
                                        for k, v in vars(WhiteROI()).items()})
            white_config = self.declare_parameter('lane_white_config', '', ro).value
            self.white_thresholds = (load_lane_white(white_config) if white_config else validate_white(
                self.declare_parameter('white_s_max', 70, ro).value,
                self.declare_parameter('white_v_min', 120, ro).value))
            camera = self.declare_parameter('white_image_topic', '/camera/color/image_raw/compressed', ro).value
            self.white_sub = self.create_subscription(CompressedImage, camera, self.on_white_image, qos_profile_sensor_data)
            self.white_debug_pub = self.create_publisher(CompressedImage, '/objavoid/white_roi/debug/compressed', qos_profile_sensor_data)
            self.scan = None
            self.scan_input = LatestScan()
            self.scan_sequence = 0
            self.scan_skipped = 0
            self.scan_group = MutuallyExclusiveCallbackGroup()
            self.object_scan = None
            self.plan_key = None
            self.plan_result = None
            self.plan_ms = 0.
            self.scan_period = None
            self.primary_box = None
            self.clusters = ()
            self.lane = None
            self.path_config = self.c
            self.last_angle = 0.
            self.last_ros = None
            self.last_state = None
            scan_topic = self.declare_parameter('scan_topic', '/scan', ro).value
            lane_topic = self.declare_parameter('lane_target_topic', '/lane_deep/target', ro).value
            mask_topic = self.declare_parameter('lane_mask_topic', '/lane_fusion/deep_mask', ro).value
            command = self.declare_parameter('command_topic', '/mission/objavoid/command_05', ro).value
            if not all((scan_topic, lane_topic, mask_topic, command)) or command == '/cmd_vel':
                raise ValueError('invalid topics')
            self.pub = self.create_publisher(AckermannDriveStamped, command, QoSProfile(depth=1))
            self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.on_scan,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                callback_group=self.scan_group)
            self.lane_sub = self.create_subscription(Float32MultiArray, lane_topic, self.on_lane, QoSProfile(depth=1))
            self.mask_sub = self.create_subscription(Image, mask_topic, self.on_mask, qos_profile_sensor_data)
            from .visualization import Visualizer
            self.visualizer = Visualizer(self, preview_points, cluster_bounds)
            self.web = None
            self.web_last = -math.inf
            enabled = self.declare_parameter('web_enabled', True, ro).value
            host = self.declare_parameter('web_host', '127.0.0.1', ro).value
            port = self.declare_parameter('web_port', 8085, ro).value
            projection_config = self.declare_parameter('web_lane_config', '', ro).value
            if type(enabled) is not bool or type(port) is not int or not 1 <= port <= 65535:
                raise ValueError('invalid web settings')
            from .web_view import WebViewer, GroundProjection
            projection = None
            try:
                projection = GroundProjection.load(projection_config or config_path.with_name('camera_projection.yaml'))
            except (OSError,ValueError,KeyError,TypeError) as error:
                self.get_logger().warning(f'Camera projection unavailable: {error}')
            self.projection = projection
            if enabled:
                try:
                    self.web = WebViewer(host, port, projection,
                        (self.white_roi.left, self.white_roi.right, self.white_roi.top, self.white_roi.bottom))
                    self.get_logger().info(f'Objavoid viewer: http://{host}:{port}')
                except OSError as error:
                    self.get_logger().warning(f'Web viewer unavailable: {error}')
            self.timer = self.create_timer(1/self.c.publish_rate, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

        def publish_web(self, state, speed, angle, now, road=None, lane_ok=None):
            if self.web is None or now-self.web_last < .1:
                return
            from .web_view import snapshot
            ros_now = self.get_clock().now().nanoseconds*1e-9
            fresh = scan_fresh(self.scan, self.c, ros_now, now)
            groups = compact_clusters(self.clusters, self.c) if fresh else ()
            scan = replace(self.object_scan,road=road) if fresh else None
            lane_fresh = self.lane is not None and 0 <= now-self.lane[1] < self.c.lane_timeout
            points = lane_reference(self.lane[0]) if lane_fresh else ()
            if lane_ok is None:
                road = None
                lane_ok = lane_fresh
            white_ok = (self.start_gate.white_hit
                and -self.c.future_tolerance <= ros_now-self.start_gate.white_stamp < self.start_gate.image_timeout
                and 0 <= now-self.start_gate.white_received < self.start_gate.image_timeout)
            if scan is not None:
                scan = replace(scan,road=road)
            data = snapshot(state, speed, angle, self.path_config, self.start_gate,
                scan, road, points, nearest_objects(scan,self.c,self.primary_box) if scan else (), fresh, lane_ok,
                white_ok, self.start_gate.count(groups,self.c),
                preview_points(angle,self.path_config) if speed > 0 and path_is_valid(state) else ())
            data['path_valid'] = speed > 0 and path_is_valid(state)
            data['diagnostics'] = dict(plan_ms=self.plan_ms,
                scan_age_ms=(ros_now-self.scan.stamp)*1000 if self.scan else None,
                receive_age_ms=(now-self.scan.received)*1000 if self.scan else None,
                scan_period_ms=self.scan_period*1000 if self.scan_period is not None else None,
                checked_distance=self.path_config.preview_distance,
                path_held=state.startswith('SCAN_UNAVAILABLE'))
            latest = self.scan_input.snapshot()
            data['diagnostics'].update(
                latest_receive_age_ms=(time.monotonic()-latest.received)*1000 if latest else None,
                received_scans=latest.sequence if latest else 0,
                skipped_scans=self.scan_skipped,
                pending_scans=max(0,latest.sequence-self.scan_sequence) if latest else 0)
            data['scan_receiving'] = self.scan is not None and 0 <= time.monotonic()-self.scan.received < .25
            self.web.publish(data)
            self.web_last = now

        def on_scan(self, msg):
            # Separate callback group/thread: no clustering, JPEG or planning.
            self.scan_input.put(msg, time.monotonic())

        def consume_scan(self):
            latest = self.scan_input.snapshot()
            if latest is None or latest.sequence == self.scan_sequence:
                return
            self.scan_skipped += max(0,latest.sequence-self.scan_sequence-1)
            self.scan_sequence = latest.sequence
            msg = latest.message
            try:
                scan = read_scan(msg, latest.received, self.c)
                if scan.stamp <= 0 or scan.stamp > self.get_clock().now().nanoseconds*1e-9+self.c.future_tolerance:
                    return
                if self.scan and scan.stamp <= self.scan.stamp:
                    return
                clusters = cluster_scan(msg, self.c)
                object_scan = object_sectors(scan,clusters,self.c)
                self.scan_period = latest.period
                self.scan,self.clusters,self.object_scan = scan,clusters,object_scan
            except (ValueError, TypeError, OverflowError) as error:
                # A malformed frame must not erase the last valid geometry.
                self.get_logger().warning(f'Ignoring invalid scan: {error}', throttle_duration_sec=5.)

        def on_white_image(self, msg):
            import cv2
            now = time.monotonic()
            ros_now = self.get_clock().now().nanoseconds * 1e-9
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if (stamp <= 0 or stamp <= self.start_gate.white_stamp
                    or not -self.c.future_tolerance <= ros_now - stamp < self.start_gate.image_timeout):
                return
            try:
                image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError('Cannot decode camera JPEG')
                if self.web is not None:
                    self.web.set_camera(image, now)
                ratio, bounds, mask = self.white_roi.measure(image, *self.white_thresholds)
                self.start_gate.observe_white(ratio >= self.white_roi.minimum_ratio, stamp, now)
                if self.white_debug_pub.get_subscription_count():
                    preview = self.white_roi.preview(image, bounds, ratio, mask, ['objavoid'])
                    ok, encoded = cv2.imencode('.jpg', preview)
                    if ok:
                        self.white_debug_pub.publish(CompressedImage(
                            header=msg.header, format='jpeg', data=encoded.tobytes()))
            except (ValueError, cv2.error) as error:
                self.start_gate.observe_white(False, stamp, now)
                self.get_logger().warning(str(error), throttle_duration_sec=5.)


        def on_lane(self, msg):
            data = tuple(msg.data)
            self.lane = (data, time.monotonic()) if lane_hint(data, self.c) is not None else None

        def on_mask(self, msg):
            if (msg.encoding != 'mono8' or min(msg.width, msg.height) <= 0
                    or msg.step < msg.width or len(msg.data) < msg.step*msg.height):
                return
            mask = np.frombuffer(msg.data, dtype=np.uint8, count=msg.height*msg.step).reshape(msg.height, msg.step)[:, :msg.width]
            if self.web is not None:
                self.web.set_lane_mask(mask, time.monotonic())
            # The mask is display-only. Lane perception belongs to lane_deep.

        def tick(self):
            now = time.monotonic()
            ros_now = self.get_clock().now().nanoseconds*1e-9
            if self.last_ros is not None and ros_now < self.last_ros:
                self.scan = self.lane = None
                self.clusters = ()
                self.start_gate.reset()
                self.plan_key = None
                self.primary_box = None
                self.last_angle = 0.
            self.last_ros = ros_now
            # Atomically take just the newest input. The scan used below stays
            # unchanged for this whole plan, even as reception continues.
            self.consume_scan()
            if not self.get_parameter('manual_active').value:
                self.start_gate.reset()
                self.plan_key = None
                self.primary_box = None
                self.last_angle = 0.
                self.path_config = self.c
                self.publish_web('IDLE', 0., 0., now)
                return
            fresh = scan_fresh(self.scan, self.c, ros_now, now)
            groups = compact_clusters(self.clusters, self.c) if fresh else ()
            count = self.start_gate.count(groups, self.c)
            if not self.start_gate.update(count, fresh, ros_now, now, self.c.future_tolerance):
                self.get_logger().info(f'WAIT_START objects={count} white={self.start_gate.white_hit}',
                                       throttle_duration_sec=2.)
                self.publish_web('WAIT_START', 0., 0., now)
                return
            lane_fresh = self.lane is not None and 0 <= now-self.lane[1] < self.c.lane_timeout
            lane_data = self.lane[0] if lane_fresh else None
            road = None
            lane_ok = lane_fresh
            angle = lane_hint(lane_data,self.c) if lane_fresh else 0.
            previous_config = self.path_config
            if fresh:
                # Replan for new scans or meaningful lane geometry changes, not
                # every 25Hz mask timestamp while the 10Hz scan is unchanged.
                reference_key=tuple(round(v,3) for v in lane_data) if lane_data is not None else None
                key = (self.scan.stamp,reference_key,lane_ok)
                if key != self.plan_key:
                    started = time.perf_counter()
                    scan = replace(self.object_scan,road=road)
                    targets = nearest_objects(scan,self.c,self.primary_box)
                    self.primary_box = targets[0] if targets else None
                    self.plan_result = plan_in_lane(scan,self.c,lane_data,self.primary_box,
                        self.last_angle if self.plan_result is not None and path_is_valid(self.plan_result[0]) else None)
                    self.plan_ms = (time.perf_counter()-started)*1000
                    self.plan_key = key
                state,speed,angle,self.path_config = self.plan_result
            else:
                # Keep the previous command curve visible, explicitly stale.
                # Do not pretend it has been rechecked against missing scans.
                angle = self.last_angle
                state,speed,angle = 'SCAN_UNAVAILABLE',.2,0.
            finished = time.monotonic()
            if not scan_fresh(self.scan,self.c,self.get_clock().now().nanoseconds*1e-9,finished):
                state,speed,angle = 'SCAN_UNAVAILABLE',.2,0.
                self.path_config = previous_config
            if self.lane is None or not 0 <= finished-self.lane[1] < self.c.lane_timeout:
                state,speed,angle = 'LANE_UNAVAILABLE',.2,0.
                lane_ok=False
                road=None
                self.path_config=previous_config
            if not lane_ok:
                state += '/LANE_UNCONFIRMED'
            self.last_angle = angle
            if state != self.last_state:
                self.get_logger().info(f'{state} speed={speed:.2f} steering={angle:.3f}')
                self.last_state = state
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.drive.speed, msg.drive.steering_angle = float(speed), float(angle)
            self.pub.publish(msg)
            self.publish_web(state, speed, angle, now, road, lane_ok)
            if self.visualizer.due(state, now):
                self.visualizer.publish(state, speed, angle, now, self.path_config,
                    lane_reference(self.lane[0]) if lane_fresh else (), road,
                    self.clusters if fresh else (), self.scan.frame_id if self.scan else 'laser_link')

    import sys
    from pathlib import Path
    from ament_index_python.packages import get_package_share_directory
    config_path = Path(get_package_share_directory('autorace_objavoid'))/'config'/'objavoid.yaml'
    rclpy.init(args=default_config_args(sys.argv if args is None else args, config_path))
    node = None
    executor = None
    try:
        node = ObjavoidNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            if node.web is not None:
                node.web.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

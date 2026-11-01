"""Deterministic planar Ackermann model, collision geometry and laser scan.

No ROS or graphics dependencies. Pose follows sensor_ws/limo_driver.cpp's
base_link convention: integrate v in the heading direction and w=v*tan(delta)/L.
"""
from dataclasses import dataclass
import math

import numpy as np
from .world import boundary_walls, cones, signs


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def intrinsics(camera):
    if camera['intrinsics'] is not None:
        return tuple(camera['intrinsics'])
    w, h = camera['width'], camera['height']
    return (w/(2*math.tan(math.radians(camera['hfov_deg'])/2)),
            h/(2*math.tan(math.radians(camera['vfov_deg'])/2)), (w-1)/2, (h-1)/2)


def camera_mount(cfg, stream):
    """Position and rotation in base_link, before ROS optical-axis conversion."""
    c = cfg['camera']
    tilt = rotation((0, c['pitch'], 0))
    r = tilt @ rotation(c[stream]['rpy'])
    p = np.array(c['mount_xyz']) + tilt @ (np.array(c['link_xyz']) + np.array(c[stream]['xyz']))
    return p, r


@dataclass
class Box:
    name: str
    x: float
    y: float
    yaw: float
    length: float
    width: float
    zmin: float
    zmax: float

    def corners(self):
        p = np.array([[1, 1], [-1, 1], [-1, -1], [1, -1]]) * [self.length/2, self.width/2]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return p @ np.array([[c, s], [-s, c]]) + [self.x, self.y]


@dataclass
class SignPanel(Box):
    shape: str='circle'

    def width_at(self,z):
        if not self.zmin<=z<=self.zmax:
            return 0.
        t=(z-self.zmin)/(self.zmax-self.zmin)
        if self.shape=='circle':
            return self.width*math.sqrt(max(0.,1-(2*t-1)**2))
        return self.width*(1-t)


def overlaps(a, b):
    """Separating-axis test; touching counts as collision."""
    if a.zmin > b.zmax or b.zmin > a.zmax:
        return False
    pa, pb = a.corners(), b.corners()
    for yaw in (a.yaw, b.yaw):
        for axis in (np.array([math.cos(yaw), math.sin(yaw)]),
                     np.array([-math.sin(yaw), math.cos(yaw)])):
            aa, bb = pa @ axis, pb @ axis
            if aa.max() < bb.min()-1e-10 or bb.max() < aa.min()-1e-10:
                return False
    return True


def make_obstacles(cfg):
    v, world = cfg['vehicle'], cfg['world']
    boxes = [Box(o['name'], *o['pose'], v['length'], v['width'],
                 v['ground_clearance'], v['height']) for o in world['obstacles']]
    for wall in boundary_walls(world):
        x, y, z = wall['center']
        length, width, height = wall['size']
        boxes.append(Box(wall['name'], x, y, 0, length, width, z-height/2, z+height/2))
    for sign in signs(world):
        x,y,yaw=sign['pose']
        top=sign['panel_center_height']+sign['panel_height']/2
        boxes += [Box(sign['name']+'_post',x,y,yaw,sign['post_width'],sign['post_width'],0,top-sign['panel_height']),
                  SignPanel(sign['name']+'_panel',x,y,yaw,sign['thickness'],sign['panel_width'],
                            top-sign['panel_height'],top,sign['shape'])]
    for key,thickness in (('traffic_light',.035),('advertisement',.02)):
        item=world.get(key)
        if item:
            x,y,yaw=item['pose']
            bottom=item['center_height']-item['height']/2
            boxes += [Box(key+'_post',x,y,yaw,.018,.018,0,bottom),
                      Box(key+'_panel',x,y,yaw,thickness,item['width'],bottom,bottom+item['height'])]
    bump=world.get('speed_bump')
    if bump:
        boxes.append(Box('speed_bump',*bump['pose'],bump['length'],bump['width'],0,bump['height']))
    return boxes


class Simulation:
    def __init__(self, cfg):
        self.cfg = cfg
        self.obstacles = make_obstacles(cfg)
        self.cones = cones(cfg['world'])
        self.rng = np.random.default_rng(cfg['simulation']['seed'])
        v = cfg['vehicle']
        inner_radius = v['wheelbase']/math.tan(v['max_inner_steering']) + v['driver_track']/2
        self.min_radius = max(v['min_turn_radius'], inner_radius)
        self.max_steering = math.atan(v['wheelbase']/self.min_radius)
        l = cfg['lidar']
        span = math.radians(l['angle_max_deg']-l['angle_min_deg'])
        n = max(2, round(l['sample_rate']/l['frequency'] * span/(2*math.pi))+1)
        self.angles = np.linspace(math.radians(l['angle_min_deg']), math.radians(l['angle_max_deg']), n)
        self.time = 0.0
        self.reset()

    def reset(self):
        # Keep /clock monotonic across episode resets.
        self.x, self.y, self.yaw = map(float, self.cfg['simulation']['spawn'])
        self.speed = self.steering = self.yaw_rate = self.acceleration = self.yaw_acceleration = 0.0
        self.cmd_speed = self.cmd_yaw_rate = 0.0
        self.last_command = -math.inf
        self.collision = False
        self.collision_names = []
        self.collisions = 0
        self._contact = False
        self.distance = 0.0
        self.wheel_travel = 0.0

    def body(self, x=None, y=None, yaw=None):
        v = self.cfg['vehicle']
        return Box('ego', self.x if x is None else x, self.y if y is None else y,
                   self.yaw if yaw is None else yaw, v['length'], v['width'],
                   v['ground_clearance'], v['height'])

    def collisions_at(self, x, y, yaw):
        body = self.body(x, y, yaw)
        names = [b.name for b in self.obstacles if overlaps(body, b)]
        for cone in self.cones:
            radius=cone.radius_at(body.zmin)
            if radius<=0 or body.zmin>cone.height or body.zmax<0:
                continue
            dx,dy=cone.x-body.x,cone.y-body.y
            c,s=math.cos(body.yaw),math.sin(body.yaw)
            p=np.array([c*dx+s*dy,-s*dx+c*dy])
            nearest=np.clip(p,[-body.length/2,-body.width/2],[body.length/2,body.width/2])
            if np.linalg.norm(p-nearest)<=radius:
                names.append(cone.name)
        corners = body.corners()
        if np.any(corners < 0) or np.any(corners > np.array(self.cfg['world']['size'])):
            names.append('map_boundary')
        return names

    def command(self, speed, yaw_rate):
        if not math.isfinite(speed) or not math.isfinite(yaw_rate):
            self.cmd_speed = self.cmd_yaw_rate = 0.0
            self.last_command = -math.inf
            return False
        vmax = self.cfg['vehicle']['max_speed']
        self.cmd_speed = float(np.clip(speed, -vmax, vmax))
        self.cmd_yaw_rate = yaw_rate
        self.last_command = self.time
        return True

    @staticmethod
    def approach(current, target, amount):
        return current + float(np.clip(target-current, -amount, amount))

    def step(self, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be positive and finite')
        v = self.cfg['vehicle']
        expired = self.time-self.last_command >= self.cfg['simulation']['command_timeout']
        target_speed = 0.0 if expired else self.cmd_speed
        if abs(target_speed) > 1e-8:
            target_steering = np.clip(math.atan(v['wheelbase']*self.cmd_yaw_rate/target_speed),
                                      -self.max_steering, self.max_steering)
        else:
            target_steering = self.steering
        old_speed, old_w = self.speed, self.yaw_rate
        self.steering = self.approach(self.steering, target_steering, v['max_steering_rate']*dt)
        # A lost command stops immediately. Normal velocity changes have a
        # configurable acceleration limit. No rotation is possible at speed=0.
        self.speed = 0.0 if expired else self.approach(self.speed, target_speed, v['max_acceleration']*dt)
        self.yaw_rate = self.speed*math.tan(self.steering)/v['wheelbase']
        # Swept motion subdivided to <= 2 mm or 0.5 degree prevents tunnelling.
        substeps = max(1, math.ceil(max(abs(self.speed)*dt/0.002, abs(self.yaw_rate)*dt/0.008)))
        h = dt/substeps
        self.collision_names = []
        for _ in range(substeps):
            angle = self.yaw_rate*h
            mid = self.yaw + angle/2
            scale = math.sin(angle/2)/(angle/2) if abs(angle) > 1e-9 else 1.0
            nx, ny = self.x+self.speed*h*scale*math.cos(mid), self.y+self.speed*h*scale*math.sin(mid)
            yaw = wrap(self.yaw+angle)
            contact = self.collisions_at(nx, ny, yaw)
            if contact:
                self.collision_names = contact
                self.speed = self.yaw_rate = 0.0
                break
            self.x, self.y, self.yaw = nx, ny, yaw
            self.distance += abs(self.speed)*h
            self.wheel_travel += self.speed*h
        self.collision = bool(self.collision_names)
        if self.collision and not self._contact:
            self.collisions += 1
        self._contact = self.collision
        self.acceleration = (self.speed-old_speed)/dt
        self.yaw_acceleration = (self.yaw_rate-old_w)/dt
        self.time += dt

    def status(self):
        return {'time': self.time, 'pose': [self.x, self.y, self.yaw],
                'speed': self.speed, 'steering_angle': self.steering,
                'collision': self.collision,
                'collision_names': self.collision_names, 'collisions': self.collisions,
                'command_stale': self.time-self.last_command >= self.cfg['simulation']['command_timeout']}

    def scan(self):
        """Horizontal first-return ray/OBB intersection, respecting obstacle height.

        The stationary scene is sampled instantaneously, with time_increment=0.
        Painted lines have no reflective geometry; configured walls do.
        """
        cfg, v = self.cfg['lidar'], self.cfg['vehicle']
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy, dz = cfg['xyz']
        origin = np.array([self.x+c*dx-s*dy, self.y+s*dx+c*dy])
        z = v['base_link_height']+dz
        theta = self.angles + self.yaw + cfg['yaw']
        directions = np.stack((np.cos(theta), np.sin(theta)), axis=1)
        ranges = np.full(len(theta), np.inf)
        for box in self.obstacles:
            if not box.zmin <= z <= box.zmax:
                continue
            width=box.width_at(z) if isinstance(box,SignPanel) else box.width
            if width<=0:
                continue
            cb, sb = math.cos(box.yaw), math.sin(box.yaw)
            transform = np.array([[cb, -sb], [sb, cb]])
            o = (origin-[box.x, box.y]) @ transform
            d = directions @ transform
            half = np.array([box.length, width])/2
            parallel = np.abs(d) < 1e-12
            safe = np.where(parallel, 1.0, d)
            a, b = (-half-o)/safe, (half-o)/safe
            lo, hi = np.minimum(a, b), np.maximum(a, b)
            lo = np.where(parallel, -np.inf, lo)
            hi = np.where(parallel, np.inf, hi)
            invalid_parallel = np.any(parallel & (np.abs(o) > half), axis=1)
            entry, leave = lo.max(axis=1), hi.min(axis=1)
            hit = np.where(entry >= 0, entry, leave)
            valid = (leave >= np.maximum(entry, 0)) & ~invalid_parallel
            ranges = np.minimum(ranges, np.where(valid, hit, np.inf))
        for cone in self.cones:
            radius=cone.radius_at(z)
            if radius<=0:
                continue
            delta=origin-[cone.x,cone.y]
            b=directions@delta
            discriminant=b*b-(delta@delta-radius*radius)
            root=np.sqrt(np.maximum(discriminant,0))
            hit=np.where(-b-root>=0,-b-root,-b+root)
            ranges=np.minimum(ranges,np.where((discriminant>=0)&(hit>=0),hit,np.inf))
        if cfg['noise_stddev']:
            ranges += self.rng.normal(0, cfg['noise_stddev'], ranges.shape)
        valid = ((ranges >= max(cfg['range_min'], cfg['physical_range_min'])) &
                 (ranges <= min(cfg['range_max'], cfg['physical_range_max'])))
        if cfg['dropout_probability']:
            valid &= self.rng.random(len(ranges)) >= cfg['dropout_probability']
        invalid = np.inf if cfg['invalid_range_is_inf'] else 0.0
        return np.where(valid, ranges, invalid).astype(np.float32)

    def imu_acceleration(self):
        # Specific force in the body-aligned IMU frame, including sensor offset.
        x, y, _ = self.cfg['imu']['xyz']
        w, alpha = self.yaw_rate, self.yaw_acceleration
        return (self.acceleration-alpha*y-w*w*x,
                self.speed*w+alpha*x-w*w*y, 9.80665)

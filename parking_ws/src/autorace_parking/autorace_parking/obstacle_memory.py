"""Mission-frame obstacle voxels and static-position Kalman tracks; no ROS imports."""
from dataclasses import dataclass
from copy import copy
import math
import numpy as np

from .core import Scan, Cluster, cluster_points
from .tracking import compose, inverse


class MemoryScan(Scan):
    """Already filtered point snapshot; raw visibility rays remain untouched."""
    def filtered_obstacles(self, cfg):
        return self

    def candidate_clusters(self, cfg):
        # A remembered object remains one candidate object even if its measured
        # surface fragments are disconnected. Unowned noise is collision-only.
        groups = {}
        for point in self.points:
            owner = self.point_owners.get(tuple(point))
            if owner is not None:
                groups.setdefault(owner,[]).append(point)
        return sorted((Cluster(points,owner) for owner,points in groups.items()
                       if len(points)>=cfg.cluster_min_points),key=lambda c:c.center)


@dataclass
class Track:
    id: int
    center: tuple
    variance: float = .1**2
    observed: bool = True


class ObstacleMemory:
    def __init__(self, resolution=.02, radius=6., limit=30000):
        self.resolution, self.radius, self.limit = resolution, radius, limit
        self.reset()

    def reset(self):
        self.cells = {}
        self.tracks = {}
        self.next_id = 0
        self.stamp = None

    @staticmethod
    def measured_clear(scan, point):
        sx,sy,yaw = scan.sensor_pose
        dx,dy = point[0]-sx,point[1]-sy
        distance = math.hypot(dx,dy)
        angle = math.atan2(dy,dx)-yaw
        angle = math.atan2(math.sin(angle),math.cos(angle))
        if not scan.ranges or not scan.angle_increment:
            return False
        index = (angle-scan.angle_min)/scan.angle_increment
        if not 0 <= index <= len(scan.ranges)-1:
            return False
        for i in {math.floor(index),math.ceil(index)}:
            r = scan.ranges[i]
            if math.isnan(r) or r < max(.001,scan.range_min):
                return False
            if distance+.03 >= min(r,scan.range_max,scan.max_range):
                return False
        return True

    @staticmethod
    def measured_clear_many(scan, points):
        """Same two-beam visibility rule, evaluated in one bounded array pass."""
        points=np.asarray(points,dtype=float).reshape(-1,2)
        clear=np.zeros(len(points),dtype=bool)
        if not len(points) or not scan.ranges or not scan.angle_increment:return clear
        sx,sy,yaw=scan.sensor_pose
        dx,dy=points[:,0]-sx,points[:,1]-sy
        distance=np.hypot(dx,dy)
        angle=np.arctan2(dy,dx)-yaw
        angle=np.arctan2(np.sin(angle),np.cos(angle))
        index=(angle-scan.angle_min)/scan.angle_increment
        valid=np.flatnonzero(np.isfinite(index)&(index>=0)&(index<=len(scan.ranges)-1))
        if not len(valid):return clear
        ranges=np.asarray(scan.ranges,dtype=float)
        lower,upper=np.floor(index[valid]).astype(int),np.ceil(index[valid]).astype(int)
        a,b=ranges[lower],ranges[upper]
        boundary=np.minimum(np.minimum(a,b),min(scan.range_max,scan.max_range))
        clear[valid]=(~np.isnan(a)&~np.isnan(b)&(a>=max(.001,scan.range_min))
                      &(b>=max(.001,scan.range_min))&(distance[valid]+.03<boundary))
        return clear

    def update(self, raw_scan, filtered, pose, stamp, cfg, groups=None):
        if self.stamp is not None and stamp <= self.stamp:
            return
        dt = max(0.,stamp-self.stamp) if self.stamp is not None else 0.
        self.stamp = stamp
        transform = inverse(pose)
        old=list(self.cells.items())
        c,s=math.cos(transform[2]),math.sin(transform[2])
        local=np.asarray([(transform[0]+c*v[0]-s*v[1],transform[1]+s*v[0]+c*v[1])
                          for _,v in old],dtype=float).reshape(-1,2)
        observed_clear=self.measured_clear_many(raw_scan,local)
        for i,(key,cell) in enumerate(old):
            if math.hypot(*local[i]) > self.radius:
                del self.cells[key]
            elif observed_clear[i]:
                cell[3] += 1
                if cell[3] >= 3:
                    del self.cells[key]
            else:
                cell[3] = 0
        for track in self.tracks.values():
            track.observed = False
            track.variance += .005**2*dt
        if groups is None:
            groups = cluster_points(filtered.points,cfg.cluster_distance,cfg.cluster_min_points)
        centers = [compose(pose,(*g.center,0.))[:2] for g in groups]
        pairs = sorted((math.dist(center,t.center),i,t.id)
                       for i,center in enumerate(centers) for t in self.tracks.values()
                       if math.dist(center,t.center) <= .25)
        assigned,used = {},set()
        for _,i,tid in pairs:
            if i not in assigned and tid not in used:
                assigned[i] = tid;used.add(tid)
        owners = {}
        for i,group in enumerate(groups):
            center = centers[i]
            if i in assigned:
                track = self.tracks[assigned[i]]
                gain = track.variance/(track.variance+.03**2)
                track.center = tuple(a+gain*(b-a) for a,b in zip(track.center,center))
                track.variance *= 1-gain
                track.observed = True
            else:
                tid = self.next_id;self.next_id += 1
                track = Track(tid,center)
                self.tracks[tid] = track
            for point in group.points:
                owners[tuple(point)] = track.id
        c,s=math.cos(pose[2]),math.sin(pose[2])
        for point in filtered.points:
            x,y=pose[0]+c*point[0]-s*point[1],pose[1]+s*point[0]+c*point[1]
            key = math.floor(x/self.resolution),math.floor(y/self.resolution)
            previous = self.cells.get(key)
            owner = owners.get(tuple(point))
            if owner is None and previous is not None:
                owner = previous[2]
            self.cells[key] = [x,y,owner,0]
        if len(self.cells) > self.limit:
            keys = sorted(self.cells,key=lambda k:math.hypot(self.cells[k][0]-pose[0],self.cells[k][1]-pose[1]))
            for key in keys[self.limit:]:
                del self.cells[key]
        alive = {v[2] for v in self.cells.values()}
        self.tracks = {tid:t for tid,t in self.tracks.items() if tid in alive}

    def local_points(self, pose, radius=None):
        transform = inverse(pose)
        c,s=math.cos(transform[2]),math.sin(transform[2])
        points = [(transform[0]+c*x-s*y,transform[1]+s*x+c*y) for x,y,_,_ in self.cells.values()]
        return points if radius is None else [p for p in points if math.hypot(*p)<=radius]

    def snapshot(self, scan, pose, cfg):
        result = MemoryScan.__new__(MemoryScan)
        result.__dict__ = copy(scan.__dict__)
        result.points = []
        result.point_owners = {}
        transform = inverse(pose)
        c,s=math.cos(transform[2]),math.sin(transform[2])
        for x,y,owner,_ in self.cells.values():
            point = (transform[0]+c*x-s*y,transform[1]+s*x+c*y)
            if math.hypot(*point) <= cfg.max_range+.5:
                result.points.append(point)
                result.point_owners[point] = owner
        return result

    def boxes(self, pose):
        # Draw the same per-object measured footprint used by gap detection.
        # KF centers stabilize association; shifting a bounding box to that
        # center would detach it from the measured object and mislead the user.
        grouped = {}
        for x,y,owner,_ in self.cells.values():
            if owner in self.tracks:
                grouped.setdefault(owner,[]).append((x,y))
        transform = inverse(pose)
        result = []
        for tid,points in grouped.items():
            xs,ys = zip(*points);xmin,xmax,ymin,ymax = min(xs),max(xs),min(ys),max(ys)
            cx,cy = (xmin+xmax)/2,(ymin+ymax)/2
            hx,hy = (xmax-xmin)/2,(ymax-ymin)/2
            corners = [(cx-hx,cy-hy),(cx+hx,cy-hy),(cx+hx,cy+hy),(cx-hx,cy+hy),(cx-hx,cy-hy)]
            result.append((tid,self.tracks[tid].observed,[compose(transform,(*p,0.))[:2] for p in corners]))
        return result

    def diagnostics(self):
        return {'cells':len(self.cells),'tracks':len(self.tracks),
                'observed_tracks':sum(t.observed for t in self.tracks.values())}

"""Build one reverse-only Ackermann route from the combined entry history."""
from copy import copy
from dataclasses import replace
import math
import time
from .core import CollisionChecker,Gap,plan_path,wrap
from .tracking import compose,inverse
from .regions import outside_grid


def lane_targets(trace,pose,cfg,grid,heading_tolerance):
    if not trace:return []
    origin=trace[0];c,s=math.cos(origin[2]),math.sin(origin[2])
    padding=replace(cfg,clearance=cfg.clearance+cfg.exit_grid_margin)
    targets=[]
    for point in reversed(trace):
        if abs(wrap(point[2]-origin[2]))>heading_tolerance:continue
        if grid is not None and (not outside_grid(point,grid,padding) or
                abs(-s*(point[0]-origin[0])+c*(point[1]-origin[1]))>cfg.exit_lane_tolerance):continue
        if not targets or math.dist(point[:2],targets[-1][:2])>=.15:targets.append(point)
    return targets


def transformed_scan(scan,transform):
    result=copy(scan)
    result.points=[compose(transform,(*p,0.))[:2] for p in scan.points]
    result.sensor_pose=compose(transform,scan.sensor_pose)
    result.point_owners={}
    result.occupied_slots=[dict(r,pose=compose(transform,r['pose'])) for r in getattr(scan,'occupied_slots',())]
    return result


def single_reverse_route(trace,pose,targets,scan,cfg):
    """Plan in a frame facing backwards, then restore body headings.

    Forward-only search in this frame guarantees negative body velocity along
    the entire output. Recovery detours and odometry jitter are observations,
    not gear changes or waypoints that the departure must replay literally.
    """
    if not targets:return [],'missing_lane_history'
    backwards=(pose[0],pose[1],wrap(pose[2]+math.pi))
    transform=compose(inverse(backwards),pose)
    source=transformed_scan(scan,transform)
    known=[compose(inverse(backwards),(p[0],p[1],wrap(p[2]+math.pi))) for p in trace]
    deadline=time.monotonic()+cfg.planning_timeout
    # Try the closest lane return, then earlier points if a wider turn is needed.
    choices=[targets[i] for i in sorted({0,len(targets)//2,len(targets)-1})]
    for index,target in enumerate(choices):
        remaining=deadline-time.monotonic()
        if remaining<=.005:break
        x,y,yaw=compose(inverse(backwards),(target[0],target[1],wrap(target[2]+math.pi)))
        settings=replace(cfg,allow_reverse=False,require_goal_heading=True,goal_tolerance=.02,
            goal_yaw_tolerance=.04,preferred_turn_radius=cfg.turn_radius,
            max_steering_paths_only=False,planning_timeout=remaining/(len(choices)-index))
        path=plan_path(source,Gap(x,y,yaw,.6,0,1),settings,known)
        if not path:continue
        route=[compose(backwards,p) for p in path]
        route=[(x,y,wrap(a-math.pi)) for x,y,a in route]
        # A long detour is not a useful reconstruction of the entry corridor.
        length=sum(math.dist(a[:2],b[:2]) for a,b in zip(route,route[1:]))
        if length>max(3.,2*sum(math.dist(a[:2],b[:2]) for a,b in zip(trace,trace[1:]))):continue
        return route,''
    return [],'no_reverse_route'

"""Build one reverse-only Ackermann route from the combined entry history."""
from copy import copy
from dataclasses import replace
import math
import time
from .core import CollisionChecker,Gap,plan_path,wrap,advance
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
    # A short continuation of the observed lane gives enough distance to
    # finish heading alignment after a delayed steering response. It is only
    # available after an actual outside-grid lane pose has been recorded;
    # the complete generated route still passes current map collision checks.
    if targets:
        extra=(origin[0]-cfg.recovery_reverse_distance*c,origin[1]-cfg.recovery_reverse_distance*s,origin[2])
        if grid is None or outside_grid(extra,grid,padding):targets.append(extra)
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
    deadline=time.monotonic()+(cfg.exit_planning_timeout or cfg.planning_timeout)
    # Try the closest lane return, then earlier points if a wider turn is needed.
    choices=[targets[i] for i in sorted({0,len(targets)//2,len(targets)-1})]
    origin=trace[0]
    angle=wrap(pose[2]-origin[2])
    # Near the returned lane, allow a short collision-checked reverse arc to
    # finish heading alignment at another point inside the lane corridor.
    # Demanding the exact old centerline can make a 5cm endpoint error require
    # a large S turn even though the vehicle is already outside the bay.
    if .03<abs(angle)<math.pi/2 and min(math.dist(pose[:2],p[:2]) for p in targets)<.6:
        radius=cfg.wheelbase/(math.tan(cfg.steering_limit)*cfg.steering_response_scale*cfg.reverse_steering_response_scale)
        aligned=advance(pose,-radius*abs(angle),math.copysign(1/radius,angle))
        lateral=-math.sin(origin[2])*(aligned[0]-origin[0])+math.cos(origin[2])*(aligned[1]-origin[1])
        # A short alignment arc must also reach the validated lane return.
        # Stopping ahead of it can leave the enlarged body inside the grid,
        # repeatedly accepting tiny arcs that cannot satisfy exit arrival.
        beyond_return=(aligned[0]-targets[0][0])*math.cos(origin[2])+(aligned[1]-targets[0][1])*math.sin(origin[2])
        if abs(lateral)<=cfg.exit_lane_tolerance and beyond_return<=0.:choices.insert(0,aligned)
    for index,target in enumerate(choices):
        remaining=deadline-time.monotonic()
        if remaining<=.005:break
        x,y,yaw=compose(inverse(backwards),(target[0],target[1],wrap(target[2]+math.pi)))
        actual_limit=cfg.wheelbase/(math.tan(cfg.steering_limit)*cfg.steering_response_scale)
        reverse_radius=(actual_limit if cfg.require_steering_calibration else cfg.turn_radius)/cfg.reverse_steering_response_scale
        settings=replace(cfg,allow_reverse=False,require_goal_heading=True,goal_tolerance=.02,
            goal_yaw_tolerance=.04,preferred_turn_radius=reverse_radius,min_turn_radius=reverse_radius,
            steering_response_scale=cfg.steering_response_scale*cfg.reverse_steering_response_scale,
            max_planning_turn_radius=0.,entry_outward_penalty=0.,
            max_steering_paths_only=False,planning_timeout=remaining/(len(choices)-index))
        freedom=min(.20,cfg.exit_arrival_yaw_tolerance*.6) if cfg.exit_arrival_yaw_tolerance>.20 else 0.
        headings=(yaw,wrap(yaw-freedom),wrap(yaw+freedom)) if freedom else ()
        path=plan_path(source,Gap(x,y,yaw,.6,0,1,heading_options=headings),settings,known)
        if not path:continue
        route=[compose(backwards,p) for p in path]
        route=[(x,y,wrap(a-math.pi)) for x,y,a in route]
        # A long detour is not a useful reconstruction of the entry corridor.
        length=sum(math.dist(a[:2],b[:2]) for a,b in zip(route,route[1:]))
        direct=math.dist(pose[:2],target[:2])
        if length>min(max(3.,2*sum(math.dist(a[:2],b[:2]) for a,b in zip(trace,trace[1:]))),max(.5,1.75*direct+.2)):continue
        return route,''
    return [],'no_reverse_route'

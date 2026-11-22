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
    result.yellow_segments=[(compose(transform,(*a,0.))[:2],compose(transform,(*b,0.))[:2],side)
        for a,b,side in getattr(scan,'yellow_segments',())]
    result.occupied_slots=[dict(r,pose=compose(transform,r['pose'])) for r in getattr(scan,'occupied_slots',())]
    # The short-edge rule selects entry; departure retains its own route policy.
    result.entry_slot=None
    return result


def single_reverse_route(trace,pose,targets,scan,cfg,grid=None):
    """Plan in a frame facing backwards, then restore body headings.

    Forward-only search in this frame guarantees negative body velocity along
    the entire output. Recovery detours and odometry jitter are observations,
    not gear changes or waypoints that the departure must replay literally.
    """
    if not cfg.exit_collision_check_enabled:
        return arc_straight_departure(trace,pose,scan,cfg)
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
        if (abs(lateral)<=cfg.exit_lane_tolerance and beyond_return<=0.
                and (grid is None or outside_grid(aligned,grid,cfg))):choices.insert(0,aligned)
    for index,target in enumerate(choices):
        remaining=deadline-time.monotonic()
        if remaining<=.005:break
        x,y,yaw=compose(inverse(backwards),(target[0],target[1],wrap(target[2]+math.pi)))
        actual_limit=cfg.wheelbase/(math.tan(cfg.steering_limit)*cfg.steering_response_scale)
        reverse_radius=(actual_limit if cfg.require_steering_calibration else cfg.turn_radius)/cfg.reverse_steering_response_scale
        settings=replace(cfg,allow_reverse=False,require_goal_heading=True,goal_tolerance=.02,
            clearance=cfg.clearance+cfg.exit_path_tracking_margin,
            goal_yaw_tolerance=.02 if cfg.exit_curve_handoff else .04,
            preferred_turn_radius=reverse_radius,min_turn_radius=reverse_radius,
            steering_response_scale=cfg.steering_response_scale*cfg.reverse_steering_response_scale,
            max_planning_turn_radius=0.,entry_outward_penalty=0.,
            max_steering_paths_only=False,planning_timeout=remaining/(len(choices)-index))
        # Curve-only handoff needs an aligned lane return. Permitting +/-0.2
        # rad here used to manufacture an endpoint that immediately needed a
        # second corrective route, despite tracking its first route accurately.
        freedom=(min(.20,cfg.exit_arrival_yaw_tolerance*.6)
                 if not cfg.exit_curve_handoff and cfg.exit_arrival_yaw_tolerance>.20 else 0.)
        headings=(yaw,wrap(yaw-freedom),wrap(yaw+freedom)) if freedom else ()
        path=plan_path(source,Gap(x,y,yaw,.6,0,1,heading_options=headings,kind='departure'),settings,known)
        if not path:continue
        route=[compose(backwards,p) for p in path]
        route=[(x,y,wrap(a-math.pi)) for x,y,a in route]
        if grid is not None and not outside_grid(route[-1],grid,cfg):continue
        if cfg.exit_curve_handoff and abs(wrap(route[-1][2]-origin[2]))<=.03:
            # Leave room for the measured steering delay to settle after the
            # last arc. The controller may hand off on this checked straight
            # tail instead of reaching an arc endpoint while still misaligned.
            end=route[-1]
            tail=[advance(end,-.02*i,0.) for i in range(1,11)]
            checker=CollisionChecker(scan,replace(cfg,clearance=cfg.clearance+cfg.exit_path_tracking_margin))
            if all(checker.free(compose(inverse(pose),p)) for p in tail):route+=tail
        # A long detour is not a useful reconstruction of the entry corridor.
        length=sum(math.dist(a[:2],b[:2]) for a,b in zip(route,route[1:]))
        direct=math.dist(pose[:2],target[:2])
        if length>min(max(3.,2*sum(math.dist(a[:2],b[:2]) for a,b in zip(trace,trace[1:]))),max(.5,1.75*direct+.2)):continue
        return route,''
    return [],'no_reverse_route'


def reverse_entry_route(entry, pose, scan, cfg):
    """Reuse a nominal smooth entry only when the actual join is feasible now."""
    from .regions import first_path_collision
    if len(entry)<2:return []
    route=list(reversed(entry))
    if math.dist(pose[:2],route[0][:2])>.05 or abs(wrap(pose[2]-route[0][2]))>.12:return []
    route[0]=pose
    radius=cfg.turn_radius/cfg.reverse_steering_response_scale
    for a,b in zip(route,route[1:]):
        distance=math.dist(a[:2],b[:2])
        if distance < 1e-5:
            if abs(wrap(b[2]-a[2]))>.005:return []
            continue
        if abs(wrap(b[2]-a[2]))/distance>1/radius+.02:return []
        mid=a[2]+wrap(b[2]-a[2])/2
        if (b[0]-a[0])*math.cos(mid)+(b[1]-a[1])*math.sin(mid)>=0:return []
    if cfg.exit_collision_check_enabled:
        tr=inverse(pose)
        checker=CollisionChecker(scan,replace(cfg,clearance=cfg.clearance+cfg.exit_path_tracking_margin))
        if first_path_collision(checker,[compose(tr,p) for p in route],cfg.collision_step):return []
    return route


def arc_straight_departure(trace, pose, scan, cfg):
    """A single reverse circular arc followed by one aligned reverse straight.

    Fit the circle to the recorded lane, never to a faulty current EKF pose.
    Refuse impossible radii rather than manufacturing an S-turn.
    """
    if not trace:
        return [], 'missing_lane_history'
    origin = trace[0]
    local = compose(inverse(origin), pose)
    x, y, angle = local
    radius_min = cfg.wheelbase/(math.tan(cfg.steering_limit)*cfg.steering_response_scale*cfg.reverse_steering_response_scale)
    if abs(angle) < .02:
        if abs(y) > cfg.exit_lane_tolerance:
            return [], 'single_arc_unavailable'
        arc_length, curvature = 0., 0.
    else:
        if abs(y) < 1e-4 or abs(angle) > math.pi*.75:
            return [], 'single_arc_unavailable'
        curvature = (1-math.cos(angle))/y
        if angle*curvature <= 0 or abs(curvature)>1/radius_min+1e-8:
            return [], 'single_arc_unavailable'
        arc_length = angle/curvature
    aligned = advance(local, -arc_length, curvature)
    straight_length = aligned[0]
    if straight_length < -.02 or arc_length+straight_length > cfg.max_range*3:
        return [], 'single_arc_unavailable'
    route = [pose]
    anchor = local
    for length, k in ((arc_length,curvature),(max(0.,straight_length),0.)):
        if length <= 1e-6: continue
        count = max(1,math.ceil(length/cfg.collision_step))
        route.extend(compose(origin,advance(anchor,-length*i/count,k)) for i in range(1,count+1))
        anchor = advance(anchor,-length,k)
    if cfg.exit_collision_check_enabled:
        checker = CollisionChecker(scan,replace(cfg,clearance=cfg.clearance+cfg.exit_path_tracking_margin),
                                   known_free_poses=[compose(inverse(pose),p) for p in trace])
        for point in route:
            if not checker.free(compose(inverse(pose),point)):
                return [], 'path_obstacle' if checker.last_block != 'unknown_space' else 'unobserved_departure'
    return route, ''


def arc_then_straight(route):
    """A reverse circular arc followed by one straight (either may be empty)."""
    curved=[];straight_started=False
    for a,b in zip(route,route[1:]):
        distance=math.dist(a[:2],b[:2])
        if distance<1e-5:continue
        mid=a[2]+wrap(b[2]-a[2])/2
        if (b[0]-a[0])*math.cos(mid)+(b[1]-a[1])*math.sin(mid)>=0:return False
        curvature=wrap(b[2]-a[2])/distance
        if abs(curvature)<.02:
            straight_started=True
        else:
            if straight_started:return False
            if curved and curvature*curved[0]<0:return False
            curved.append(curvature)
    if curved and max(curved)-min(curved) > .05+.10*abs(sum(curved)/len(curved)):
        return False
    return len(route)>1


def anchored_reverse_entry(entry,pose,scan,cfg):
    """Retain the accepted curve when the stopped pose differs by arrival error.

    Anchor the entire recorded route, rather than bending its first tiny segment.
    Endpoint lane tolerance and every swept obstacle check remain mandatory.
    """
    if len(entry)<2:return []
    join_limit=max(.05,min(.10,2*cfg.goal_tolerance))
    if (math.dist(pose[:2],entry[-1][:2])>join_limit
            or abs(wrap(pose[2]-entry[-1][2]))>.12):return []
    correction=compose(pose,inverse(entry[-1]))
    adjusted=[compose(correction,p) for p in entry]
    lane=entry[0]
    end=adjusted[0]
    lateral=-math.sin(lane[2])*(end[0]-lane[0])+math.cos(lane[2])*(end[1]-lane[1])
    if (abs(lateral)>cfg.exit_lane_tolerance
            or abs(wrap(end[2]-lane[2]))>cfg.exit_arrival_yaw_tolerance):return []
    return reverse_entry_route(adjusted,pose,scan,cfg)


def curve_only_route(route):
    """End at the last arc sample; never execute the aligned straight tail."""
    route=list(route)
    if len(route)<2:return route
    if not arc_then_straight(route):return []
    last_curve=None
    for i,(a,b) in enumerate(zip(route,route[1:])):
        distance=math.dist(a[:2],b[:2])
        if distance>1e-6 and abs(wrap(b[2]-a[2]))/distance>=.02:
            last_curve=i+1
    return route[:last_curve+1] if last_curve is not None else route[:1]

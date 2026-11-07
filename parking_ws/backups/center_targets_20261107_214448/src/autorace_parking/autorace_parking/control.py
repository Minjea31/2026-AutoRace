"""Local parking plans and a bounded pure-pursuit Ackermann controller."""
from dataclasses import replace
from collections import Counter
import math
import time

from .core import (CollisionChecker, Gap, cluster_points, find_gaps, plan_path,
                   advance, sample_candidate, straight_turn_candidates, parking_candidates, front_clearance, wrap)
from .tracking import compose, inverse
from .enclosed import enclosed_goals


def local_goal(goal, pose):
    x, y, yaw = compose(inverse(pose), (goal.x, goal.y, goal.yaw))
    return replace(goal, x=x, y=y, yaw=yaw)


def same_gap(goal, candidate, row_yaw):
    # Preserve anchored depth and relaxed heading, matching the bay along its row.
    delta = ((goal.x-candidate.x)*math.cos(row_yaw)
             + (goal.y-candidate.y)*math.sin(row_yaw))
    return abs(delta) <= max(.15, .25*min(goal.width, candidate.width))


def forward_turn_target(goal, cfg):
    """Extend a front-face target along the bay heading to fit a forward turn.

    A body-clear target can still be shallower than the Ackermann turn. Solve
    straight -> minimum-radius arc -> straight, and remove a negative final
    straight by moving the target into the bay. This is only a goal hypothesis;
    sample_candidate still rejects every unobserved or obstructed driven pose.
    """
    theta = wrap(goal.yaw)
    if abs(math.sin(theta)) < 1e-6 or abs(theta) >= math.pi-.05:
        return goal
    direction = 1 if theta > 0 else -1
    turn = advance((0., 0., 0.), cfg.turn_radius*abs(theta), direction/cfg.turn_radius)
    suffix = (goal.y-turn[1])/math.sin(theta)
    prefix = goal.x-turn[0]-suffix*math.cos(theta)
    if suffix < 0 and prefix >= -1e-6:
        return replace(goal, x=goal.x-suffix*math.cos(theta),
                       y=goal.y-suffix*math.sin(theta))
    return goal


def same_enclosure(goal,candidate):
    ids=set(goal.support_ids)
    required=min(len(ids),max(3 if goal.confirmed else 2,math.ceil(.6*len(ids))))
    return (not ids or len(ids & set(candidate.support_ids))>=required) and math.hypot(candidate.x-goal.x,candidate.y-goal.y)<=.6


def needs_reverse_for_goal(goal,cfg):
    return bool(goal is not None and math.hypot(goal.x,goal.y)>cfg.goal_tolerance and
                (goal.x<=.25 or goal.x**2+goal.y**2 < 2*cfg.turn_radius*abs(goal.y)))


def refine_goal(goal, candidates, cfg, checker):
    """Keep the selected bay identity, while refining its measured empty center."""
    if goal is None or not candidates:
        return goal, False
    ids = set(goal.support_ids)
    old_free=checker.free((goal.x,goal.y,goal.yaw),allow_unknown=True)
    old_clearance=min((math.hypot(x-goal.x,y-goal.y) for x,y in checker.scan.points),default=math.inf)
    # Candidates are already ranked by usable clearance. Choosing the closest
    # grid alternative to the old center would perpetuate the original bias.
    for candidate in candidates:
        if not same_enclosure(goal,candidate):
            continue
        distance = math.hypot(candidate.x-goal.x, candidate.y-goal.y)
        if distance > .6:
            continue
        if goal.confirmed and old_free and math.isfinite(old_clearance):
            candidate_clearance=min((math.hypot(x-candidate.x,y-candidate.y) for x,y in checker.scan.points),default=math.inf)
            if candidate_clearance<=old_clearance+.003:
                # Vehicle-distance ranking alone must not pull a validated,
                # reachable center back and forth between two empty pockets.
                continue
        gain = min(cfg.goal_update_gain, cfg.goal_update_step/max(distance, 1e-9))
        maximum = min(1.,cfg.goal_update_step/max(distance,1e-9))
        # A newly observed object can invalidate the old center. The default
        # low-pass step may remain inside it; try larger, still bounded steps
        # toward this validated center rather than freezing the estimate.
        for fraction in range(5):
            blend = gain+(maximum-gain)*fraction/4
            refined = replace(candidate, x=goal.x+blend*(candidate.x-goal.x),
                              y=goal.y+blend*(candidate.y-goal.y),
                              confirmed=goal.confirmed or candidate.confirmed,
                              support_ids=tuple(sorted(ids)) if goal.confirmed else candidate.support_ids)
            if checker.free((refined.x, refined.y, refined.yaw), allow_unknown=True):
                return refined, distance > 1e-6 or refined.confirmed != goal.confirmed
    return goal, False


def build_plan(scan, cfg, goal=None, origin=(0., 0., 0.), diagnostics=None, known_free_poses=(),
               recovery_allowed=True, blind_reverse_remaining=None, validate_goal=False, front_scan=None,
               failed_plans=0,stalled_route=False):
    """Return (final goal, path, final approach).

    Prefer driving straight along the row before turning into the bay. If
    occluded, advance only through the observed prefix and then observe again.
    New bays must lie to the right of the mission-start pose, not necessarily
    to the right of the current vehicle after it has turned.
    """
    scan = scan.filtered_obstacles(cfg)
    front_scan = scan if front_scan is None else front_scan
    blind_budget = cfg.blind_reverse_distance if blind_reverse_remaining is None else max(0., min(cfg.blind_reverse_distance, blind_reverse_remaining))
    started = time.monotonic()
    deadline = started + cfg.planning_timeout
    details = diagnostics if diagnostics is not None else {}
    clusters = scan.candidate_clusters(cfg)
    geometric_goals = find_gaps(clusters, cfg, origin, diagnostics=details)
    visible_goals = []
    candidate_checks = []
    space_checker = CollisionChecker(scan,cfg)
    c,s = math.cos(cfg.row_yaw),math.sin(cfg.row_yaw)
    for candidate in geometric_goals:
        a,b = clusters[candidate.left],clusters[candidate.right]
        pa = [(c*x+s*y,-s*x+c*y) for x,y in a.points]
        pb = [(c*x+s*y,-s*x+c*y) for x,y in b.points]
        left,right = max(p[0] for p in pa),min(p[0] for p in pb)
        bottom,top = min(p[1] for p in pa+pb),max(p[1] for p in pa+pb)
        blocked = any(left+1e-9 < c*x+s*y < right-1e-9
                      and bottom-1e-9 <= -s*x+c*y <= top+1e-9 for x,y in scan.points)
        reason = 'points_between_objects' if cfg.validate_parking_space and blocked else ''
        if cfg.validate_parking_space and not reason and not space_checker.free((candidate.x,candidate.y,candidate.yaw),allow_unknown=True):
            reason = 'goal_footprint_obstacle'
        candidate_checks.append({'track_ids':[a.track_id,b.track_id],'width_m':candidate.width,
                                 'required_width_m':cfg.vehicle_width+2*cfg.clearance,
                                 'accepted':not bool(reason),'reason':reason})
        if not reason:
            visible_goals.append(candidate)
    if cfg.enclosed_space_detection:
        details['enclosed_space_detection'] = True
        details['pair_filter_counts'] = {}
        enclosed = enclosed_goals(scan,clusters,cfg,origin,details)
        visible_goals = enclosed
    details['candidate_checks'] = candidate_checks
    terminal_goal=bool(goal is not None and goal.confirmed and math.hypot(goal.x,goal.y)<=.15
                       and space_checker.free((goal.x,goal.y,goal.yaw),allow_unknown=True))
    reachable_goal,reachable_path=None,[]
    if cfg.enclosed_space_detection and cfg.continuous_replanning and len(visible_goals)>1 and not terminal_goal:
        # Clearance alone can favor an empty pocket that the measured turning
        # radius cannot reach. Try other centers in this same bay before
        # committing to an approach that will end in front of an obstacle.
        alternatives=([goal] if goal is not None else [])+[g for g in visible_goals if goal is None or same_enclosure(goal,g)]
        checked=[]
        for i,candidate in enumerate(alternatives):
            if any(math.hypot(candidate.x-g.x,candidate.y-g.y)<.02 for g in checked):continue
            checked.append(candidate)
            remaining=deadline-time.monotonic()
            if remaining<=.005:break
            budget=min(.12,remaining/max(1,len(alternatives)-i))
            candidate_path=plan_path(scan,candidate,replace(cfg,planning_timeout=budget,
                allow_reverse=cfg.allow_reverse and not cfg.recovery_on_front_obstacle_only),
                known_free_poses,inverse(origin)[:2] if cfg.allow_reverse else (0.,0.))
            if candidate_path:
                reachable_goal,reachable_path=candidate,candidate_path
                break
        if reachable_goal is not None:
            if goal is None:
                visible_goals=[reachable_goal]+[g for g in visible_goals if g is not reachable_goal]
            elif math.hypot(reachable_goal.x-goal.x,reachable_goal.y-goal.y)>cfg.goal_update_step:
                goal=replace(reachable_goal,confirmed=goal.confirmed or reachable_goal.confirmed,
                             support_ids=goal.support_ids if goal.confirmed else reachable_goal.support_ids)
                details['goal_relocated_for_reachability']=True
            elif reachable_goal is not goal:
                visible_goals=[reachable_goal]+[g for g in visible_goals if g is not reachable_goal]
    if cfg.continuous_replanning and not details.get('goal_relocated_for_reachability') and not terminal_goal:
        goal, details['goal_updated'] = refine_goal(goal, visible_goals, cfg, space_checker)
    elif details.get('goal_relocated_for_reachability'):
        details['goal_updated']=True
    goal_reselected = bool(validate_goal and not cfg.continuous_replanning and goal is not None and visible_goals
                           and not any(same_gap(goal, candidate, cfg.row_yaw) for candidate in visible_goals))
    if goal_reselected:
        goal = None
    # Missing/occluded pairs do not erase a target; a clearly different visible
    # pair can replace a stale target whose foreground sign has been filtered.
    goals = [goal] if goal is not None else visible_goals
    details.update(input_points=len(scan.points), clusters=len(clusters), candidates=len(goals),
                   goal_reselected=goal_reselected,
                   ignored_small_clusters=scan.ignored_small_clusters, ignored_small_points=scan.ignored_small_points,
                   assume_unobserved_free=cfg.assume_unobserved_free,
                   continuous_replanning=cfg.continuous_replanning,
                   recovery_on_path_failure=cfg.recovery_on_path_failure,
                   failed_plans=failed_plans, recovery_failure_count=cfg.recovery_failure_count,
                   max_observed_prefix_m=0., blocked_by='', min_turn_radius_m=cfg.turn_radius,
                   preferred_turn_radius_m=cfg.preferred_turn_radius,
                   blind_reverse_remaining_m=blind_budget, blind_reverse_distance_m=0.,
                   target_depth_adjusted=False, known_corridor_poses=len(known_free_poses),
                   scan_fov_deg=math.degrees(abs(scan.angle_increment)*max(0, len(scan.ranges)-1)))

    def record_pair(target):
        if target is not None and target.support_ids and cfg.enclosed_space_detection:
            indices=[i for i,c in enumerate(clusters) if c.track_id in target.support_ids]
        else:
            matches = [g for g in visible_goals if target is not None and
                       ((target.support_ids and len(set(target.support_ids)&set(g.support_ids))>=2)
                        or same_gap(target,g,cfg.row_yaw))]
            if not matches:return
            pair = min(matches,key=lambda g:math.hypot(g.x-target.x,g.y-target.y))
            indices=pair.support_indices or (pair.left,pair.right)
        selected = []
        for index in indices:
            cluster = clusters[index]
            owners = Counter(getattr(scan,'point_owners',{}).get(tuple(p)) for p in cluster.points)
            owners.pop(None,None)
            selected.append({'cluster_index':index,'center':cluster.center,'bounds':cluster.bounds,
                             'track_id':owners.most_common(1)[0][0] if owners else None})
        details['selected_pair'] = selected

    def finish(target, path, final, code):
        record_pair(target)
        details.update(reason_code=code, duration_s=time.monotonic()-started, path_poses=len(path))
        if (final and path and target is not None and not cfg.require_goal_heading
                and not (cfg.enclosed_space_detection and target.support_ids)):
            target = replace(target, yaw=path[-1][2])
        if target is not None:
            details['target'] = {'x': target.x, 'y': target.y, 'yaw': target.yaw}
            details['goal_confirmed'] = target.confirmed
            details['support_ids'] = target.support_ids
        return target, path, final

    if goal is None and not cfg.enclosed_space_detection and cfg.goal_from_observed_front and cfg.require_goal_heading:
        # Anchor this depth once, when selecting a new bay. Replanning retains
        # the accepted mission-frame goal rather than pushing it away each scan.
        adjusted = [forward_turn_target(target, cfg) for target in goals]
        details['target_depth_adjusted'] = any(math.hypot(a.x-b.x, a.y-b.y) > 1e-6
                                               for a, b in zip(goals, adjusted))
        goals = adjusted
    # Keep the assumed startup bubble fixed in the mission frame. Otherwise
    # successive reverse replans could expand it into an unseen rear corridor.
    clear_center = inverse(origin)[:2] if cfg.allow_reverse else (0., 0.)
    checker = CollisionChecker(scan, cfg, known_free_poses, clear_center)

    def recover(code):
        target = goals[0] if goals else goal
        clearance = front_clearance(front_scan, cfg)
        details['front_clearance_m'] = clearance
        path_failure = bool(cfg.recovery_on_path_failure and failed_plans >= cfg.recovery_failure_count
                            and (needs_reverse_for_goal(target,cfg) or (stalled_route and target is not None and target.confirmed))
                            and math.hypot(target.x,target.y) > cfg.goal_tolerance
                            and code in ('budget_exceeded','no_observed_path'))
        details['recovery_trigger'] = 'path_failure' if path_failure else 'front_obstacle'
        if cfg.recovery_on_front_obstacle_only and not path_failure:
            if target is None:
                details['recovery_no_target'] = True
                return finish(None, [], False, code)
            if (clearance is None or clearance > cfg.recovery_front_clearance
                    or math.hypot(target.x,target.y) <= cfg.goal_tolerance):
                details['recovery_waiting_for_front_obstacle'] = True
                return finish(None, [], False, code)
        if cfg.allow_reverse and recovery_allowed:
            rear_checker = (CollisionChecker(scan, replace(cfg,assume_unobserved_free=False),
                                             known_free_poses, clear_center) if path_failure else checker)
            path = [(0., 0., 0.)]
            distance, blind_distance, accepted_distance = 0., 0., 0.
            target = goals[0] if goals else goal
            turn_left = bool(cfg.recovery_left_when_goal_right and target is not None and target.y < 0)
            straight = min(cfg.recovery_straight_distance, cfg.recovery_reverse_distance)
            details['recovery_mode'] = 'straight_then_left_reverse' if turn_left else 'straight_reverse'
            while distance+cfg.collision_step <= cfg.recovery_reverse_distance+1e-8:
                distance += cfg.collision_step
                if turn_left and distance > straight:
                    arc_length = distance-straight
                    if cfg.max_steering_paths_only and arc_length > cfg.max_turn_arc_length+1e-8:
                        break
                    # Negative travel with positive curvature moves the rear left
                    # and turns the vehicle nose right toward a right-side bay.
                    pose = advance((-straight, 0., 0.), -arc_length, 1/cfg.turn_radius)
                else:
                    pose = (-distance, 0., 0.)
                if not rear_checker.free(pose):
                    blocked = rear_checker.last_block
                    if (blocked == 'unknown_space' and distance <= blind_budget+1e-8
                            and checker.free(pose, allow_unknown=True)):
                        blind_distance = distance
                    else:
                        rear_checker.last_block = blocked
                        break
                path.append(pose)
                accepted_distance = distance
            # Shorter prefixes cannot clear the controller's endpoint margin.
            if len(path) >= 4:
                details['recovery_distance_m'] = accepted_distance
                details['blind_reverse_distance_m'] = blind_distance
                details['recovery_straight_m'] = min(straight, accepted_distance) if turn_left else accepted_distance
                return finish(goals[0] if goals else goal, path, False, 'recovery_ready')
            details['recovery_blocked_by'] = rear_checker.last_block
        if cfg.allow_reverse and not recovery_allowed:
            details['recovery_limit_reached'] = True
        return finish(None, [], False, code)

    target = goals[0] if goals else goal
    record_pair(target)
    if target is not None:
        details['target'] = {'x': target.x, 'y': target.y, 'yaw': target.yaw}
        details['goal_confirmed'] = target.confirmed
        details['support_ids'] = target.support_ids
    clearance = front_clearance(front_scan,cfg)
    if (cfg.recovery_on_front_obstacle_only and target is not None
            and math.hypot(target.x,target.y) > cfg.goal_tolerance
            and clearance is not None and clearance <= cfg.recovery_front_clearance):
        return recover('front_obstacle')
    if not goals:
        if (checker.free((0., 0., 0.)) or (cfg.allow_reverse and blind_budget > 0
                and checker.free((0., 0., 0.), allow_unknown=True))):
            return recover('no_gap')
        details['blocked_by'] = checker.last_block
        return finish(None, [], False, 'start_blocked')
    if not checker.free((0., 0., 0.)):
        details['blocked_by'] = checker.last_block
        if (checker.last_block == 'unknown_space' and cfg.allow_reverse and blind_budget > 0
                and checker.free((0., 0., 0.), allow_unknown=True)):
            return recover('start_blocked')
        return finish(None, [], False, 'start_blocked')
    for target in goals:
        if (reachable_goal is not None and reachable_path
                and math.hypot(target.x-reachable_goal.x,target.y-reachable_goal.y)<1e-6):
            return finish(target,reachable_path,True,'path_ready')
        partial, progress = [], 0.
        for _, parts in parking_candidates(target, cfg):
            path, complete = sample_candidate(checker, parts, cfg)
            if complete:
                return finish(target, path, True, 'path_ready')
            length = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(path, path[1:]))
            details['max_observed_prefix_m'] = max(details['max_observed_prefix_m'], length)
            details['blocked_by'] = checker.last_block
            if length > max(.15, progress) and math.hypot(*path[-1][:2]) > .13:
                partial, progress = path, length
        if partial:
            if not cfg.require_complete_entry_path:
                return finish(target, partial, False, 'approach_ready')
            details['partial_path_rejected']=True
    # Try cheap, observed approaches to all candidates before spending the
    # search budget. One timeout per bay can otherwise outlive the ROS adapter's
    # result age limit, so even a useful later result would always be discarded.
    for target in goals:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        path = plan_path(scan, target, replace(cfg, planning_timeout=remaining,
                         allow_reverse=cfg.allow_reverse and not cfg.recovery_on_front_obstacle_only),
                         known_free_poses, clear_center)
        if path:
            return finish(target, path, True, 'path_ready')
    code = 'budget_exceeded' if time.monotonic() >= deadline else 'no_observed_path'
    return recover(code)


def build_plan_report(scan, cfg, goal=None, origin=(0., 0., 0.), known_free_poses=(), recovery_allowed=True,
                      blind_reverse_remaining=None, validate_goal=False, front_scan=None, failed_plans=0,stalled_route=False):
    """Process-worker interface with operator diagnostics and the normal plan."""
    details = {}
    target, path, final = build_plan(scan, cfg, goal, origin, diagnostics=details,
                                   known_free_poses=known_free_poses, recovery_allowed=recovery_allowed,
                                   blind_reverse_remaining=blind_reverse_remaining, validate_goal=validate_goal, front_scan=front_scan,
                                   failed_plans=failed_plans,stalled_route=stalled_route)
    return {'goal': target, 'path': path, 'final': final, 'diagnostics': details}


class Follower:
    def __init__(self, cfg, speed=1.0, lookahead=0.18, hold_seconds=3.0, deceleration=1.0,
                 max_yaw_rate=1.0, exit_heading_tolerance=0.05,
                 require_parking_heading=True, parking_position_tolerance=None, max_steering_rate=0.,
                 min_turn_speed=0.):
        self.cfg, self.speed, self.lookahead = cfg, speed, lookahead
        self.lookahead_point = None
        self.lookahead_distance = lookahead
        self.hold_seconds, self.deceleration = hold_seconds, deceleration
        self.max_yaw_rate = max_yaw_rate
        self.require_parking_heading = require_parking_heading
        self.parking_position_tolerance = parking_position_tolerance or cfg.goal_tolerance
        self.max_steering_rate = max_steering_rate
        if not math.isfinite(min_turn_speed) or not 0 <= min_turn_speed <= speed:
            raise ValueError("최소 회전 속도는 0 이상이며 주차 속도 이하여야 합니다.")
        self.min_turn_speed = min_turn_speed
        self.last_steering = 0.
        self.last_command_time = None
        if not math.isfinite(exit_heading_tolerance) or not 0 < exit_heading_tolerance < math.pi/2:
            raise ValueError('exit_heading_tolerance must be between zero and pi/2')
        self.exit_heading_tolerance = exit_heading_tolerance
        self.exit_path_length = 0.
        self.goal = None
        self.path_goal = None
        self.path = []
        self.entry_path = []
        self.entry_locked = False
        self.index = 0
        self.final_approach = False
        self.arrival_count = 0
        self.parked = False
        self.parked_since = None
        self.exiting = self.exited = False
        self.entry_trace = []
        self.previous_sample = None
        self.measured_speed = 0.0
        self.measured_curvature = 0.0
        self.recovering = False
        self.recovery_start_pose = None
        self.recovery_finished = False
        self.recovery_distance_limit = 0.
        self.motion_direction = self.last_drive_direction = 1
        self.segment_end = 0
        self.gear_confirmations = 0

    def path_direction(self, index):
        for i in range(index, len(self.path)-1):
            a, b = self.path[i:i+2]
            dx, dy = b[0]-a[0], b[1]-a[1]
            if math.hypot(dx, dy) < 1e-8:
                continue
            yaw = a[2]+wrap(b[2]-a[2])/2
            return 1 if dx*math.cos(yaw)+dy*math.sin(yaw) >= 0 else -1
        return self.last_drive_direction

    def set_segment(self, start):
        self.index = start
        self.motion_direction = self.path_direction(start)
        self.segment_end = max(0, len(self.path)-1)
        for i in range(start+1, len(self.path)-1):
            if self.path_direction(i) != self.motion_direction:
                self.segment_end = i
                break
        self.gear_confirmations = 0

    def accept(self, goal, path, final_approach, origin, recovery=False):
        if not path or self.parked or (self.entry_locked and not recovery):
            return
        if goal is not None:
            gx, gy, yaw = compose(origin, (goal.x, goal.y, goal.yaw))
            self.goal = replace(goal, x=gx, y=gy, yaw=yaw)
        else:
            self.goal = None
        self.path_goal = self.goal
        self.path = [compose(origin, point) for point in path]
        if recovery:
            self.entry_path = []
            self.entry_locked = False
        elif self.cfg.hold_entry_path or self.cfg.continuous_replanning:
            self.entry_path = self.path.copy()
            self.entry_locked = self.cfg.hold_entry_path
        self.final_approach, self.index = final_approach, 0
        self.recovering = recovery
        self.recovery_finished = False
        self.recovery_start_pose = None
        self.recovery_distance_limit = min(self.cfg.recovery_reverse_distance,
            sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(self.path,self.path[1:]))) if recovery else 0.
        self.set_segment(0)

    def local_path(self, pose):
        transform = inverse(pose)
        points = (self.entry_path if (self.entry_locked or self.cfg.continuous_replanning) and self.entry_path and not self.recovering and not self.exiting
                  else self.path[self.index:])
        return [compose(transform, p) for p in points]

    def stop(self, state, now):
        self.lookahead_point = None
        self.last_steering, self.last_command_time = 0., now
        return 0., 0., state

    def observe_pose(self, pose, now, new_scan):
        if new_scan:
            if self.previous_sample is not None:
                previous, stamp = self.previous_sample
                if now - stamp > .001:
                    dx,dy=pose[0]-previous[0],pose[1]-previous[1]
                    distance=math.hypot(dx,dy)
                    self.measured_speed = distance / (now - stamp)
                    if distance>.003:
                        sign=1 if dx*math.cos(previous[2])+dy*math.sin(previous[2])>=0 else -1
                        self.measured_curvature=max(-2/self.cfg.turn_radius,min(2/self.cfg.turn_radius,
                            wrap(pose[2]-previous[2])/(sign*distance)))
            self.previous_sample = pose, now
        if self.parked:
            return
        if not self.entry_trace:
            self.entry_trace.append(pose)
            return
        a = self.entry_trace[-1]
        distance = math.hypot(pose[0] - a[0], pose[1] - a[1])
        if distance < .005:
            return
        count = max(1, math.ceil(distance / self.cfg.collision_step))
        for i in range(1, count + 1):
            t = i / count
            self.entry_trace.append((a[0] + t * (pose[0] - a[0]), a[1] + t * (pose[1] - a[1]),
                                     wrap(a[2] + t * wrap(pose[2] - a[2]))))

    def prepare_exit(self, pose, now):
        # Keep vehicle headings unchanged: reverse waypoint order and velocity,
        # rather than turning the vehicle by pi. Stop at the last recorded
        # lane-aligned pose: reverse the entry turn, not the earlier straight.
        self.parked, self.parked_since = True, now
        self.path = [pose]
        lane_yaw = self.entry_trace[0][2] if self.entry_trace else pose[2]
        if abs(wrap(pose[2]-lane_yaw)) > self.exit_heading_tolerance:
            for point in reversed(self.entry_trace):
                previous = self.path[-1]
                if math.hypot(point[0]-previous[0], point[1]-previous[1]) <= 1e-9:
                    continue
                # Keep an actual recorded footprint, including at the endpoint.
                self.path.append(point)
                if abs(wrap(point[2]-lane_yaw)) <= self.exit_heading_tolerance:
                    break
        self.exit_path_length = sum(math.hypot(b[0]-a[0], b[1]-a[1])
                                    for a,b in zip(self.path, self.path[1:]))
        self.index = self.arrival_count = 0
        x, y, yaw = self.path[-1]
        self.goal = replace(self.goal, x=x, y=y, yaw=yaw)
        self.recovering = False
        self.recovery_finished = False
        self.set_segment(0)
        # Arrival was confirmed at rest; the hold period is also stationary.
        self.last_drive_direction = self.motion_direction

    def command(self, scan, pose, new_scan=False, now=None, obstacle_scan=None):
        now = time.monotonic() if now is None else now
        self.observe_pose(pose, now, new_scan)
        self.lookahead_point = None
        self.lookahead_distance = self.lookahead + min(.15, self.measured_speed*.2)
        if self.recovering and self.path:
            if self.recovery_start_pose is None:
                self.recovery_start_pose = pose
            travelled = math.hypot(pose[0]-self.recovery_start_pose[0],pose[1]-self.recovery_start_pose[1])
            if travelled >= self.recovery_distance_limit-1e-6:
                self.recovering = False
                if self.cfg.continuous_replanning:
                    self.recovery_finished = True
                    return self.stop('PATH_HELD',now)
                self.path = []
                return self.stop('REOBSERVE',now)
        if self.recovering and self.path and new_scan:
            # This short recovery corridor was checked before acceptance. Keep
            # its remembered coverage, but let fresh obstacle returns veto it.
            checker = CollisionChecker(scan if obstacle_scan is None else obstacle_scan, replace(self.cfg, initial_clear_radius=self.cfg.max_range+1.))
            transform = inverse(pose)
            if any(not checker.free(compose(transform, p)) for p in self.path[self.index:]):
                self.recovering = False
                if self.cfg.continuous_replanning:
                    self.recovery_finished = True
                else:
                    self.path = []
                return self.stop('RECOVERY_BLOCKED', now)
        if (self.cfg.continuous_replanning and self.path and new_scan
                and not self.recovering and not self.exiting and not self.parked):
            checker = CollisionChecker(scan if obstacle_scan is None else obstacle_scan,
                                       replace(self.cfg,initial_clear_radius=self.cfg.max_range+1.))
            transform = inverse(pose)
            horizon = max(.25,self.measured_speed*.3+self.measured_speed**2/(2*self.deceleration))
            distance = 0.
            previous = (0.,0.,0.)
            for point in self.path[self.index:]:
                local = compose(transform,point)
                length = math.hypot(local[0]-previous[0],local[1]-previous[1])
                if length > 1e-9:
                    covered = min(length,max(0.,horizon-distance))
                    count = max(1,math.ceil(covered/self.cfg.collision_step))
                    for j in range(1,count+1):
                        fraction = covered*j/count/length
                        sample = (previous[0]+fraction*(local[0]-previous[0]),
                                  previous[1]+fraction*(local[1]-previous[1]),
                                  wrap(previous[2]+fraction*wrap(local[2]-previous[2])))
                        if not checker.free(sample):
                            return self.stop('ROUTE_BLOCKED',now)
                elif not checker.free(local):
                    return self.stop('ROUTE_BLOCKED',now)
                distance += length
                if distance >= horizon:break
                previous = local
        if self.exited:
            return self.stop('EXITED', now)
        if self.parked and not self.exiting:
            if now - self.parked_since < self.hold_seconds:
                return self.stop('PARKED', now)
            self.exiting = True
        if self.goal is None and not self.recovering:
            return self.stop('EXIT_PATH_ERROR' if self.exiting else 'PLANNING', now)
        goal = local_goal(self.goal, pose) if self.goal is not None else None
        # The fallback planner may legally end within its configured goal
        # tolerances. A stricter controller used to reject that endpoint and
        # discard the path, leaving a forward-only replan with no solution.
        position_tolerance = .07 if self.exiting else self.parking_position_tolerance
        yaw_tolerance = .20 if self.exiting else self.cfg.goal_yaw_tolerance
        heading_ok = goal is not None and (abs(goal.yaw) <= yaw_tolerance if self.exiting or self.require_parking_heading else True)
        last_segment = not self.path or self.segment_end == len(self.path)-1
        arrived = (not self.recovering and goal is not None and last_segment
                   and (self.exiting or self.final_approach or not self.require_parking_heading)
                   and math.hypot(goal.x, goal.y) <= position_tolerance and heading_ok)
        if new_scan:
            self.arrival_count = self.arrival_count + 1 if arrived and self.measured_speed < .03 else 0
        if arrived:
            if self.arrival_count >= 3:
                if self.exiting:
                    self.exited = True
                    self.path = []
                    return self.stop('EXITED', now)
                self.prepare_exit(pose, now)
                return self.stop('PARKED', now)
            return self.stop('EXIT_CONFIRMING' if self.exiting else 'CONFIRMING', now)
        if self.recovery_finished and self.path:
            return self.stop('PATH_HELD',now)
        if (self.cfg.recovery_on_front_obstacle_only and not self.recovering
                and not self.exiting and goal is not None):
            clearance = front_clearance(scan,self.cfg)
            if clearance is not None and clearance <= self.cfg.recovery_front_clearance:
                if not (self.entry_locked or self.cfg.continuous_replanning):
                    self.path = []
                return self.stop('FRONT_BLOCKED',now)
        if not self.path:
            return self.stop('EXIT_PATH_ERROR' if self.exiting else 'PLANNING', now)
        # A replan may begin in the opposite direction. Never reverse velocity
        # while rolling; require three fresh observations of a measured stop.
        if self.motion_direction != self.last_drive_direction:
            if new_scan:
                self.gear_confirmations = self.gear_confirmations+1 if self.measured_speed < .03 else 0
            if self.gear_confirmations < 3:
                return self.stop('CHANGING_DIRECTION', now)
            self.last_drive_direction = self.motion_direction
            self.gear_confirmations = 0
        # Monotonic closest waypoint avoids jumping to another loop/segment.
        upper = min(self.segment_end+1, self.index + 30)
        self.index = min(range(self.index, upper),
                         key=lambda i: math.hypot(self.path[i][0] - pose[0], self.path[i][1] - pose[1]))
        transform = inverse(pose)
        path = [compose(transform, p) for p in self.path[self.index:self.segment_end+1]]
        cusp = self.segment_end < len(self.path)-1
        if cusp and math.hypot(path[-1][0], path[-1][1]) < .04:
            if new_scan:
                self.gear_confirmations = self.gear_confirmations+1 if self.measured_speed < .03 else 0
            if self.gear_confirmations >= 3:
                self.set_segment(self.segment_end)
                self.last_drive_direction = self.motion_direction
            return self.stop('CHANGING_DIRECTION', now)
        endpoint_threshold = min(.065, position_tolerance*.5) if self.final_approach and not self.exiting else .065
        if self.recovering:
            endpoint_threshold = .025
        if not cusp and (not path or math.hypot(path[-1][0], path[-1][1]) < endpoint_threshold):
            if self.exiting:
                return self.stop('EXIT_PATH_ERROR', now)
            if self.recovering and self.cfg.continuous_replanning:
                self.recovering = False
                self.recovery_finished = True
                return self.stop('PATH_HELD',now)
            if (self.entry_locked or self.cfg.continuous_replanning) and not self.recovering:
                return self.stop('PATH_HELD',now)
            self.path = []
            self.recovering = False
            return self.stop('REOBSERVE', now)
        # Follow the accepted path after the configured front-proximity check.
        aim = path[-1]
        lookahead = self.lookahead_distance
        for point in path:
            if math.hypot(point[0], point[1]) >= lookahead:
                aim = point
                break
        self.lookahead_point = aim
        direction = self.motion_direction
        if direction * aim[0] <= 0.005:
            if self.exiting:
                return self.stop('EXIT_PATH_ERROR', now)
            if self.entry_locked or self.cfg.continuous_replanning:
                return self.stop('PATH_HELD',now)
            self.path = []
            return self.stop('REPLAN', now)
        distance2 = aim[0] ** 2 + aim[1] ** 2
        steering = math.atan(self.cfg.wheelbase * 2 * aim[1] /
                             (max(distance2, 1e-6)*self.cfg.steering_response_scale)) * self.cfg.steering_gain
        steering = max(-self.cfg.steering_limit, min(self.cfg.steering_limit, steering))
        if self.cfg.max_steering_on_curves:
            # Use the current segment, not the distant lookahead point, to
            # switch straight/arc commands. Reverse retraces the same wheel angle.
            steering = 0.
            for a,b in zip(path,path[1:]):
                if math.hypot(b[0]-a[0],b[1]-a[1]) > 1e-6:
                    delta = direction*wrap(b[2]-a[2])
                    if abs(delta) > 1e-6:
                        steering = math.copysign(self.cfg.steering_limit,delta)
                    break
        requested_steering = steering
        if self.max_steering_rate > 0:
            dt = max(0., min(.1, now-self.last_command_time)) if self.last_command_time is not None else .05
            change = self.max_steering_rate*dt
            steering = max(self.last_steering-change, min(steering, self.last_steering+change))
        self.last_steering, self.last_command_time = steering, now
        remaining = math.hypot(path[0][0], path[0][1]) + sum(
            math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))
        # Reserve a scan/control interval for actuator response before braking.
        stop_margin = min(.06, position_tolerance*.5) if self.final_approach or self.exiting else .06
        if cusp or self.recovering:
            stop_margin = .01
        braking = math.sqrt((self.deceleration * .2) ** 2 +
                            2 * self.deceleration * max(0., remaining - stop_margin)) - self.deceleration * .2
        speed = min(self.speed, braking, self.max_yaw_rate * self.cfg.wheelbase /
                    max(abs(math.tan(steering))*self.cfg.steering_response_scale, 1e-6))
        if direction < 0 and not self.exiting:
            speed = min(speed, self.cfg.recovery_speed)
        if self.max_steering_rate > 0 and abs(requested_steering-steering) > .01:
            settling_time = abs(requested_steering-steering)/self.max_steering_rate
            speed = min(speed, .005/settling_time)
        distance = 0.0
        for a, b in zip(path, path[1:]):
            segment = math.hypot(b[0] - a[0], b[1] - a[1])
            if segment < 1e-6:
                continue
            curvature = abs(wrap(b[2] - a[2])) / segment
            if self.max_steering_rate > 0:
                signed_curvature = direction*wrap(b[2]-a[2])/segment
                next_steering = math.atan(self.cfg.wheelbase*signed_curvature/self.cfg.steering_response_scale)
                change_time = abs(next_steering-steering)/self.max_steering_rate
                if change_time > .05 and distance > .02:
                    speed = min(speed, max(.02, (distance-.02)/change_time))
            turn_speed = self.max_yaw_rate / max(curvature, 1e-6)
            speed = min(speed, math.sqrt(turn_speed ** 2 + 2 * self.deceleration *
                                        max(0., distance - self.measured_speed * .2)))
            distance += segment
        # Apply the floor to steering-related slowdown, preserving final braking.
        cruise_limit = min(self.speed, self.cfg.recovery_speed) if direction < 0 and not self.exiting else self.speed
        yaw_limit = self.max_yaw_rate * self.cfg.wheelbase / max(abs(math.tan(steering))*self.cfg.steering_response_scale, 1e-6)
        speed = min(cruise_limit, braking, yaw_limit, max(speed, min(self.min_turn_speed, cruise_limit)))
        terminal=(self.cfg.require_complete_entry_path and self.final_approach and not self.exiting
                  and goal is not None and math.hypot(goal.x,goal.y)<=self.lookahead)
        if terminal:
            # Brake toward a point inside the arrival tolerance, so a preview
            # does not assume cruising past a goal at which we will stop.
            speed=min(speed,max(.03,(math.hypot(goal.x,goal.y)-.8*position_tolerance)/
                                    max(.2,2*self.cfg.control_preview_time)))
        if self.cfg.require_complete_entry_path and self.cfg.continuous_replanning and speed>0:
            checker=CollisionChecker(scan if obstacle_scan is None else obstacle_scan,
                                     replace(self.cfg,initial_clear_radius=self.cfg.max_range+1.))
            velocity=max(speed,self.measured_speed)
            horizon=velocity*self.cfg.control_preview_time+velocity**2/(2*self.deceleration)+self.cfg.collision_step
            if terminal:
                horizon=min(horizon,max(0.,math.hypot(goal.x,goal.y)-position_tolerance)+
                            self.measured_speed*.2+self.cfg.collision_step/4)
            requested=self.cfg.steering_response_scale*math.tan(steering)/self.cfg.wheelbase
            measured=self.measured_curvature if self.measured_speed>.03 else 0.
            count=max(1,math.ceil(horizon/self.cfg.collision_step))
            for curvature in (measured,requested,(measured+requested)/2):
                if any(not checker.free(advance((0.,0.,0.),direction*horizon*i/count,curvature)) for i in range(count+1)):
                    if self.recovering:
                        self.recovering=False
                        self.recovery_finished=True
                        return self.stop('RECOVERY_BLOCKED',now)
                    return self.stop('ROUTE_BLOCKED',now)
        state = ('EXITING' if self.exiting else ('RECOVERING' if self.recovering else
                 ('REVERSING' if direction < 0 else ('PARKING' if self.final_approach else 'APPROACHING'))))
        return direction * speed, steering, state

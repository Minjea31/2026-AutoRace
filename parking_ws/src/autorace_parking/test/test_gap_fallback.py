from concurrent.futures import Future
from dataclasses import replace
import math
import time
from unittest.mock import patch
from pathlib import Path
import json
from dataclasses import fields
import pytest
import yaml

from autorace_parking.core import CollisionChecker, Settings, Scan
from autorace_parking.control import build_plan_report
from autorace_parking.targets import automatic_goals
from test_slot_map import cfg, objects, snapshot
from test_search_stop import node, plan, measurement


def test_two_objects_select_the_gap_without_five_occupied_cells():
    config=cfg(single_empty_slot=True,pair_fallback_enabled=True)
    scan=snapshot(objects([(1.,-.7),(1.8,-.7)]));details={}
    goals=automatic_goals(scan,scan.candidate_clusters(config),config,(0.,0.,0.),details)
    assert goals and goals[0].kind=='pair_midpoint'
    assert (goals[0].x,goals[0].y)==(1.4,-.7)
    assert details['slot_map_fallback'] and not details['full_slot_map_matched']
    assert len(goals[0].support_ids)==2


def test_disabled_fallback_still_requires_the_configured_grid():
    config=cfg(single_empty_slot=True,pair_fallback_enabled=False)
    scan=snapshot(objects([(1.,-.7),(1.8,-.7)]));details={}
    assert not automatic_goals(scan,scan.candidate_clusters(config),config,(0.,0.,0.),details)


def test_real_obstacle_at_gap_center_prevents_a_fake_free_space():
    config=cfg(single_empty_slot=True,pair_fallback_enabled=True)
    scan=snapshot(objects([(1.,-.7),(1.8,-.7)]))
    scan.points.append((1.4,-.7));details={}
    goals=automatic_goals(scan,scan.candidate_clusters(config),config,(0.,0.,0.),details)
    assert not goals


def test_failed_complete_plan_produces_a_checked_approach_to_the_same_gap():
    config=cfg(single_empty_slot=True,pair_fallback_enabled=True,candidate_approach_enabled=True,
               require_complete_entry_path=True,require_goal_heading=False)
    scan=snapshot(objects([(1.,-.7),(1.8,-.7)]))
    with patch('autorace_parking.control.plan_path',return_value=[]):
        report=build_plan_report(scan,config,known_free_poses=[(0.,0.,0.)],recovery_allowed=False)
    assert report['path'] and not report['final']
    assert report['diagnostics']['reason_code']=='candidate_approach_ready'
    assert report['goal'].kind=='pair_midpoint'
    assert all(CollisionChecker(scan,config).free(p) for p in report['path'])
    end=report['path'][-1];goal=report['goal']
    assert math.hypot(goal.x-end[0],goal.y-end[1])<math.hypot(goal.x,goal.y)-.03
    assert sum(math.dist(a[:2],b[:2]) for a,b in zip(report['path'],report['path'][1:]))<=.301


def test_rejected_gap_is_excluded_and_another_gap_can_be_selected():
    config=cfg(single_empty_slot=True,pair_fallback_enabled=True,candidate_approach_enabled=True,
               require_goal_heading=False,planning_timeout=.03)
    scan=snapshot(objects([(1.,-.7),(1.8,-.7),(2.6,-.7)]))
    report=build_plan_report(scan,config,excluded_targets=((1.4,-.7,.15),),recovery_allowed=False)
    assert report['diagnostics']['target']['x']>2.


def test_three_failed_candidate_plans_resume_search_instead_of_waiting_forever(node):
    n,clock=node
    n.cfg=replace(n.cfg,candidate_approach_enabled=True)
    n.search_started=99.
    plan(n,clock)
    for stamp in (100.02,100.04,100.06):measurement(n,clock,stamp)
    for _ in range(3):plan(n,clock,path=False)
    assert n.search_target_phase is None and n.tracked_goal is None
    assert n.search_excluded_targets and not n.follower.path
    assert n.search_command(clock[0],n.tracker.pose,n.scan)==(.2,0.)


def test_candidate_timeout_cancels_a_stuck_job_and_resumes_search(node):
    n,clock=node
    n.cfg=replace(n.cfg,candidate_approach_enabled=True)
    n.search_started=99.
    plan(n,clock)
    for stamp in (100.02,100.04,100.06):measurement(n,clock,stamp)
    n.search_candidate_started=time.monotonic()-n.cfg.candidate_plan_timeout-.1
    future=Future();n.pending=(future,n.epoch,n.tracker.pose,clock[0])
    assert n.search_command(clock[0],n.tracker.pose,n.scan)==(.2,0.)
    assert future.cancelled() and n.search_target_phase is None


def test_checked_partial_route_is_accepted_after_the_required_stop(node):
    n,clock=node
    n.cfg=replace(n.cfg,candidate_approach_enabled=True,require_complete_entry_path=True)
    n.search_started=99.
    plan(n,clock)
    for stamp in (100.02,100.04,100.06):measurement(n,clock,stamp)
    goal=n.tracked_goal
    future=Future();future.set_result({'goal':goal,'path':[(0.,0.,0.),(.1,0.,0.),(.2,0.,0.)],
        'final':False,'diagnostics':{'reason_code':'candidate_approach_ready','candidates':1,
        'target':{'x':goal.x,'y':goal.y,'yaw':goal.yaw}}})
    n.pending=(future,n.epoch,n.tracker.pose,clock[0]);n.poll_plan()
    assert n.follower.path and n.search_target_phase is None
    assert not n.follower.final_approach


@pytest.mark.parametrize('index',range(3))
def test_november_18_recorded_parking_scenes_create_collision_checked_routes(index):
    package=Path(__file__).resolve().parents[1]
    frame=json.loads((package/'test/data/parking_gap_scenes_20261118.json').read_text())[index]
    params=yaml.safe_load((package/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    config=Settings(**{f.name:params[f.name] for f in fields(Settings) if f.name in params})
    scan=Scan(frame['ranges'],frame['angle_min'],frame['angle_increment'],frame['range_min'],frame['range_max'],(.103,0.,0.),config.max_range)
    result=build_plan_report(scan,config,known_free_poses=[(0.,0.,0.)],recovery_allowed=False)
    assert result['path'] and result['goal'] is not None
    assert result['diagnostics']['reason_code'] in ('path_ready','candidate_approach_ready')
    assert all(CollisionChecker(scan,config).free(p) for p in result['path'])

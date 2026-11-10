"""Two anchors locate a gap; new objects promote it to a six-cell map."""
from dataclasses import replace
import math
import json
import pytest
from autorace_parking.core import CollisionChecker
from autorace_parking.control import build_plan_report
from autorace_parking.slot_map import fit_slot_map
from autorace_parking.targets import automatic_goals,update_geometry_goal
from autorace_parking.tracking import compose
from test_slot_map import cfg,objects,scene,snapshot


@pytest.mark.parametrize('yaw',[0.,.4,math.pi/2,-math.pi/2])
@pytest.mark.parametrize('reversed_anchors',[False,True])
def test_two_anchors_match_one_row_and_stop_at_open_midpoint(yaw,reversed_anchors):
    config=cfg();center=(1.4,-1.1);axis=(math.cos(yaw),math.sin(yaw))
    anchors=objects([(center[0]+s*.52*axis[0],center[1]+s*.52*axis[1]) for s in (-1,1)])
    if reversed_anchors:anchors.reverse()
    layout=fit_slot_map(anchors,config)
    json.dumps(layout,allow_nan=False)
    assert layout['method']=='two_object_row_template' and not layout['locked']
    assert layout['matched_cells']==2 and layout['rows']==1 and len(layout['cells'])==3
    empty=[c for c in layout['cells'] if c['eligible_empty']]
    assert len(empty)==1 and empty[0]['center']==pytest.approx(center)
    scan=snapshot(anchors);details={}
    goals=automatic_goals(scan,scan.candidate_clusters(config),config,(0.,0.,0.),details)
    assert len(goals)==1 and goals[0].kind=='pair_midpoint' and not goals[0].confirmed
    assert (goals[0].x,goals[0].y)==pytest.approx(center)
    assert goals[0].heading_options and goals[0].parking_grid is None
    assert goals[0].slot_id==-1  # No fictitious full-grid cell identity.


def test_two_object_map_and_goal_remain_in_same_odom_position_after_vehicle_turns():
    anchors,_=scene((0,2),origin=(1.5,-1.1),yaw=.5);config=cfg();scan=snapshot(anchors);first={}
    initial=automatic_goals(scan,scan.candidate_clusters(config),config,(0.,0.,0.),first)[0]
    pose=(.6,-.2,1.2);scan=snapshot(anchors,pose);details={}
    goal=automatic_goals(scan,scan.candidate_clusters(config),replace(config,row_yaw=-pose[2]),pose,details,first['slot_map'])[0]
    assert compose(pose,(goal.x,goal.y,goal.yaw))[:2]==pytest.approx((initial.x,initial.y))
    assert details['slot_map']['origin']==pytest.approx(first['slot_map']['origin'])


def test_third_object_promotes_partial_pair_to_six_cells_without_jumping_to_other_gap():
    anchors,truth=scene((0,2),origin=(1.5,-1.1));config=cfg();scan=snapshot(anchors);details={}
    previous=automatic_goals(scan,scan.candidate_clusters(config),config,(0.,0.,0.),details)[0]
    more,_=scene((0,2,3),origin=(1.5,-1.1));scan=snapshot(more);after={}
    candidates=automatic_goals(scan,scan.candidate_clusters(config),config,(0.,0.,0.),after,details['slot_map'])
    assert after['slot_map']['method']=='metric_six_slot_template'
    assert len(after['slot_map']['cells'])==6 and after['slot_map']['matched_cells']==3
    upgraded,changed=update_geometry_goal(previous,candidates)
    assert changed and upgraded.kind=='enclosed_center' and upgraded.slot_id>=0
    assert (upgraded.x,upgraded.y)==pytest.approx(truth[1])
    # Occluding back to two objects must retain an already locked full map.
    all_objects,_=scene(origin=(1.5,-1.1));locked=fit_slot_map(all_objects,config)
    assert fit_slot_map(anchors,config,locked)['locked']
    assert len(fit_slot_map(anchors,config,locked)['cells'])==6


def test_two_anchors_off_metric_pitch_use_orientation_independent_gap_geometry():
    config=cfg();anchors=objects([(1.2,-1.5),(1.2,-.75)])
    scan=snapshot(anchors);details={}
    goals=automatic_goals(scan,scan.candidate_clusters(config),config,(0.,0.,0.),details)
    assert goals and goals[0].kind=='pair_midpoint'
    assert (goals[0].x,goals[0].y)==pytest.approx((1.2,-1.125))
    assert details['target_center_method']=='two_object_gap'
    assert details['slot_map'] is None


def test_two_anchor_entry_path_is_complete_and_keeps_ten_cm_clearance():
    from test_obstacle_collision_mode import profile
    config=profile();anchors=objects([(1.0,-.7),(2.04,-.7)])
    scan=snapshot(anchors)
    result=build_plan_report(scan,config,recovery_allowed=False)
    assert result['final'] and result['path'],result['diagnostics']
    goal=result['goal'];assert goal.kind=='pair_midpoint'
    assert (goal.x,goal.y)==pytest.approx((1.52,-.7))
    assert all(CollisionChecker(scan,config).free(p) for p in result['path'])


def test_left_side_pair_and_insufficient_center_clearance_do_not_become_mapped_goals():
    config=cfg();assert fit_slot_map(objects([(1.,.8),(2.04,.8)]),config) is None
    # Broad objects fill the gap even though their centroids fit the template.
    anchors=objects([(1.,-.8),(2.04,-.8)])
    anchors=[(ids,replace(c,points=[(x+dx,y+dy) for x,y in [c.center] for dx in (-.34,.34) for dy in (-.2,.2)])) for ids,c in anchors]
    scan=snapshot(anchors);details={}
    assert not automatic_goals(scan,scan.candidate_clusters(config),config,(0.,0.,0.),details)

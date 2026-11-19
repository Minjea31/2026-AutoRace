"""Known 3x2 layout, partial visibility, right-side gating and RViz output."""
from dataclasses import replace
import math
from pathlib import Path
import pytest
from autorace_parking.core import Settings,Cluster,Scan
from autorace_parking.slot_map import reference_centers,rotation,fit_slot_map
from autorace_parking.targets import automatic_goals
from autorace_parking.obstacle_memory import MemoryScan
from autorace_parking.tracking import compose,inverse


def cfg(**kw):
    values=dict(slot_map_enabled=True,automatic_target_selection=True,enclosed_space_detection=True,
                assume_unobserved_free=True,min_cluster_size=.14,max_gap=.8,pair_fallback_enabled=False)
    values.update(kw)
    return replace(Settings(),**values)


def objects(centers):
    return [((i,),Cluster([(float(x+dx),float(y+dy)) for dx in (-.08,0.,.08) for dy in (-.08,0.,.08)],i))
            for i,(x,y) in enumerate(centers)]


def scene(indices=(0,2,3,4,5),origin=(2.,-1.2),yaw=.2):
    config=cfg();points=reference_centers(config)@rotation(yaw).T+origin
    return objects(points[list(indices)]),points


def snapshot(objs,pose=(0.,0.,0.)):
    scan=MemoryScan.__new__(MemoryScan);scan.__dict__=Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.,(0.,0.,0.)).__dict__.copy()
    tr=inverse(pose);cells=[(compose(tr,(*p,0.))[:2],c.track_id) for _,c in objs for p in c.points]
    scan.points=[p for p,_ in cells];scan.point_owners=dict(cells);return scan


def test_five_objects_identify_inside_empty_cell_and_metric_size():
    objs,truth=scene();m=fit_slot_map(objs,cfg())
    import json
    json.dumps(m,allow_nan=False)
    assert m['locked'] and m['matched_cells']==5
    empty=[c for c in m['cells'] if c['eligible_empty']]
    assert len(empty)==1 and empty[0]['center']==pytest.approx(truth[1],abs=1e-8)
    assert m['cell_width']==.5 and m['cell_depth']==.6 and m['border']==.02


def test_multiple_empty_cells_are_supported_without_five_object_requirement():
    objs,truth=scene((0,2,3,5));m=fit_slot_map(objs,cfg())
    assert m['locked'] and m['matched_cells']==4
    empty=[c for c in m['cells'] if c['eligible_empty']]
    assert len(empty)==2
    assert all(min(math.dist(c['center'],truth[i]) for i in (1,4))<1e-6 for c in empty)


def test_three_object_partial_map_does_not_invent_outer_empty_column():
    objs,truth=scene((0,1,3),yaw=0.);m=fit_slot_map(objs,cfg())
    assert not m['locked'] and m['hypotheses']>=2
    eligible=[c for c in m['cells'] if c['eligible_empty']]
    assert len(eligible)==1 and eligible[0]['center']==pytest.approx(truth[4],abs=1e-6)
    assert any(c['state']=='unconfirmed' for c in m['cells'])


def test_outside_obstacle_and_duplicate_returns_do_not_shift_layout():
    objs,truth=scene();m=fit_slot_map(objs,cfg())
    noisy=objs+objects([(.1,-.5)])
    duplicate=objs+[((99,),replace(objs[0][1],track_id=99))]
    for extra in (noisy,duplicate):
        after=fit_slot_map(extra,cfg())
        assert after['origin']==pytest.approx(m['origin'],abs=1e-6)
        assert after['matched_cells']==5
        assert after['yaw']==pytest.approx(m['yaw'],abs=1e-6)


def test_locked_map_remains_fixed_and_occupied_cells_survive_occlusion():
    objs,truth=scene();m=fit_slot_map(objs,cfg());after=fit_slot_map(objs[:2],cfg(),m)
    assert after['origin']==m['origin'] and after['yaw']==m['yaw']
    assert [c['occupied'] for c in after['cells']]==[c['occupied'] for c in m['cells']]
    assert after['locked'] and len([c for c in after['cells'] if c['eligible_empty']])==1


def test_grid_is_invariant_to_vehicle_motion_and_rotation():
    objs,truth=scene();first={};scan=snapshot(objs)
    a=automatic_goals(scan,scan.candidate_clusters(cfg()),cfg(),(0.,0.,0.),first)[0]
    pose=(.7,-.3,.8);scan=snapshot(objs,pose);details={}
    b=automatic_goals(scan,scan.candidate_clusters(cfg()),replace(cfg(),row_yaw=-pose[2]),pose,details,first['slot_map'])[0]
    assert compose(pose,(b.x,b.y,0.))[:2]==pytest.approx((a.x,a.y),abs=1e-8)
    assert details['slot_map']['origin']==first['slot_map']['origin']


def test_initial_left_grid_is_rejected_and_left_empty_cell_is_never_selected():
    objs,_=scene(origin=(2.,1.2));assert fit_slot_map(objs,cfg()) is None
    objs,_=scene((0,1,2,3,5),origin=(2.,-.1),yaw=0.);m=fit_slot_map(objs,cfg())
    assert m and all(c['center'][1]<0 for c in m['cells'] if c['eligible_empty'])
    assert any(c['state']=='outside_initial_right' for c in m['cells'])


def test_latest_recording_layout_can_match_with_positive_side_obstacle_as_part_of_right_grid():
    # Actual 21:53 bag, mission coordinates. The empty cell is slightly left
    # of the initial vehicle and must not become a right-side parking target.
    centers=[(1.869,-.059),(2.034,-.608),(2.443,-.404),(2.798,.161),(2.931,-.31)]
    objs=objects(centers);m=fit_slot_map(objs,cfg())
    assert m and m['matched_cells']==5 and m['locked']
    empty=[c for c in m['cells'] if not c['occupied']]
    assert len(empty)==1 and .0<empty[0]['center'][1]<.2
    assert empty[0]['state']=='outside_initial_right' and not empty[0]['eligible_empty']


def test_rviz_slot_map_uses_six_distinct_cells_and_reset_clears_it():
    import rclpy
    from rclpy.parameter import Parameter
    from autorace_parking.node import ParkingNode
    from visualization_msgs.msg import Marker
    rclpy.init();n=ParkingNode(parameter_overrides=[Parameter('localization_source',value='scan')])
    class Capture:
        def __init__(self):self.messages=[]
        def publish(self,m):self.messages.append(m)
    output=Capture();n.slot_map_pub=output
    try:
        n.slot_map=fit_slot_map(scene()[0],cfg());n.tracker.valid=True;n.tracker.pose=(.4,.1,.2)
        n.publish_slot_map();markers=output.messages[-1].markers
        boxes=[m for m in markers if m.ns=='slot_grid'];labels=[m for m in markers if m.ns=='slot_labels']
        assert len(boxes)==6 and len(labels)==6 and {m.id for m in boxes}==set(range(6))
        assert all(m.header.frame_id=='base_link' and len(m.points)==5 for m in boxes)
        n.reset_motion();assert n.slot_map is None
        n.clear_visuals();assert output.messages[-1].markers[0].action==Marker.DELETEALL
    finally:n.destroy_node();rclpy.shutdown()


def test_real_rviz_contains_slot_map_display():
    import yaml
    path=Path(__file__).resolve().parents[1]/'config/parking_real.rviz'
    if not path.exists():path=Path(__file__).resolve().parents[3]/'parking_real.rviz'
    data=yaml.safe_load(path.read_text())
    assert any(d.get('Topic',{}).get('Value')=='/parking/slot_map' and d.get('Enabled')
               for d in data['Visualization Manager']['Displays'])


def test_new_occupied_target_is_invalidated_instead_of_retaining_its_old_goal():
    from autorace_parking.control import build_plan_report
    objs,truth=scene();scan=snapshot(objs);config=cfg();details={}
    old=automatic_goals(scan,scan.candidate_clusters(config),config,(0.,0.,0.),details)[0]
    extra=objects([truth[1]])[0]
    occupied=objs+[(extra[0],replace(extra[1],track_id=99))]
    scan=snapshot(occupied)
    result=build_plan_report(scan,config,old,layout_hint=details['slot_map'],recovery_allowed=False)
    assert result['diagnostics']['goal_invalidated_by_slot_map']
    assert result['goal'] is None and not result['path']


@pytest.mark.parametrize('empty', range(6))
@pytest.mark.parametrize('yaw', [0., .2, -.25])
def test_single_empty_mode_all_positions(empty,yaw):
    config=cfg(single_empty_slot=True,block_occupied_slots=True)
    objs,truth=scene(tuple(i for i in range(6) if i!=empty),yaw=yaw)
    layout=fit_slot_map(objs,config)
    assert layout and layout['locked']
    eligible=[c for c in layout['cells'] if c['eligible_empty']]
    assert len(eligible)==1
    assert eligible[0]['center']==pytest.approx(truth[empty],abs=1e-6)
    details={}
    goals=automatic_goals(snapshot(objs),[c for _,c in objs],config,(0.,0.,0.),details)
    assert len(goals)==1 and goals[0].confirmed
    from autorace_parking.regions import with_occupied_slots
    scan=with_occupied_slots(snapshot(objs),layout,(0.,0.,0.),config)
    assert len(scan.occupied_slots)==5
    assert all(r['half_size']==(.110,.161) for r in scan.occupied_slots)
    from autorace_parking.core import CollisionChecker
    from autorace_parking.regions import first_occupied_path_collision
    checker=CollisionChecker(scan,config)
    for region in scan.occupied_slots:
        x,y,angle=region['pose']
        assert not checker.free((x,y,angle),allow_unknown=True)
        path=[(x-.5*math.cos(angle),y-.5*math.sin(angle),angle),
              (x+.5*math.cos(angle),y+.5*math.sin(angle),angle)]
        assert first_occupied_path_collision(checker,path,.02) is not None
    # A remembered occupied layout survives temporary loss of returns.
    held=fit_slot_map(objs[:2],config,layout)
    assert held['locked']
    assert sum(c['occupied'] for c in held['cells'])==5
    assert [c['center'] for c in held['cells'] if c['eligible_empty']]==[eligible[0]['center']]
    filled=fit_slot_map(objects(truth),config,layout)
    assert not any(c['eligible_empty'] for c in filled['cells'])


def test_single_empty_mode_waits_for_five_objects():
    config=cfg(single_empty_slot=True)
    objs,_=scene((0,2,3,5))
    assert fit_slot_map(objs,config) is None

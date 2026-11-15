"""Pairs must not create or restart a parking-grid registration."""
from dataclasses import replace
import math
import pytest
from autorace_parking.slot_map import fit_slot_map
from autorace_parking.targets import automatic_goals
from test_slot_map import cfg,scene,snapshot,objects

@pytest.mark.parametrize('yaw',[0.,.4,math.pi/2,-math.pi/2])
@pytest.mark.parametrize('reverse',[False,True])
def test_two_objects_never_create_a_grid_or_a_parking_goal(yaw,reverse):
    anchors,_=scene((0,2),origin=(1.5,-1.1),yaw=yaw)
    if reverse:anchors=anchors[::-1]
    config=cfg();scan=snapshot(anchors);details={}
    assert fit_slot_map(anchors,config) is None
    assert automatic_goals(scan,scan.candidate_clusters(config),config,(0.,0.,0.),details)==[]
    assert details['slot_map'] is None


def test_locked_full_grid_survives_occlusion_without_refitting_a_pair():
    full,_=scene(origin=(1.5,-1.1));config=cfg();locked=fit_slot_map(full,config)
    assert locked and locked['locked']
    partial=fit_slot_map(full[:2],config,locked)
    assert partial['locked'] and len(partial['cells'])==6
    assert partial['origin']==locked['origin'] and partial['yaw']==locked['yaw']


def test_nonmetric_pair_cannot_fall_back_to_a_gap_in_grid_mode():
    config=cfg();anchors=objects([(1.2,-1.5),(1.2,-.75)])
    scan=snapshot(anchors);details={}
    assert not automatic_goals(scan,scan.candidate_clusters(config),config,(0.,0.,0.),details)

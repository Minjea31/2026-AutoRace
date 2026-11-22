"""Unrelated objects cannot revive the removed two-object fallback."""
import pytest
from autorace_parking.targets import automatic_goals
from test_slot_map import cfg,objects,snapshot

@pytest.mark.parametrize('extras',[(),((2.7,-2.4),),((2.7,-2.4),(.4,-1.9),(3.3,-.2))])
def test_unrelated_points_never_create_a_two_anchor_goal(extras):
    config=cfg();scan=snapshot(objects([(1.,-.7),(2.04,-.7)]+list(extras)));details={}
    goals=automatic_goals(scan,scan.candidate_clusters(config),config,(0.,0.,0.),details)
    assert all(g.kind=='enclosed_center' and g.slot_id>=0 for g in goals)
    assert details.get('pair_map_hypotheses',0)==0


def test_nonmetric_pair_with_unrelated_object_is_not_a_parking_map():
    config=cfg();scan=snapshot(objects([(1.2,-1.5),(1.2,-.75),(2.7,-2.4)]));details={}
    assert not automatic_goals(scan,scan.candidate_clusters(config),config,(0.,0.,0.),details)

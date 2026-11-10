"""Choose supported registrations, independently of unrelated object counts."""
from dataclasses import replace
import json
import math
from pathlib import Path
from unittest.mock import patch

import pytest
from autorace_parking.core import Settings,Gap,CollisionChecker
from autorace_parking.control import build_plan_report
from autorace_parking.targets import automatic_goals
from autorace_parking.slot_map import fit_slot_map
from test_slot_map import cfg,objects,snapshot


@pytest.mark.parametrize('extras', [(), ((2.7,-2.4),), ((2.7,-2.4),(.4,-1.9),(3.3,-.2))])
def test_supported_pair_survives_unrelated_objects_that_do_not_fit_six_cells(extras):
    config=cfg();anchors=objects([(1.,-.7),(2.04,-.7)]+list(extras))
    scan=snapshot(anchors);details={}
    goals=automatic_goals(scan,scan.candidate_clusters(config),config,(0.,0.,0.),details)
    assert any(g.kind=='pair_midpoint' and (g.x,g.y)==pytest.approx((1.52,-.7)) for g in goals)
    assert details['slot_map']['method']=='two_object_row_template'
    assert set(goals[0].support_ids)=={0,1}


def test_nonmetric_pair_axis_is_also_tested_among_unrelated_objects():
    config=cfg();scan=snapshot(objects([(1.2,-1.5),(1.2,-.75),(2.7,-2.4)]));details={}
    goals=automatic_goals(scan,scan.candidate_clusters(config),config,(0.,0.,0.),details)
    assert any(g.kind=='pair_midpoint' and (g.x,g.y)==pytest.approx((1.2,-1.125)) for g in goals)


def test_intervening_object_invalidates_outer_pair_even_when_full_map_is_unmatched():
    config=cfg();scan=snapshot(objects([(1.,-.7),(2.04,-.7),(1.52,-.7),(2.7,-2.4)]));details={}
    goals=automatic_goals(scan,scan.candidate_clusters(config),config,(0.,0.,0.),details)
    assert not any(set(g.support_ids)=={0,1} for g in goals)


def test_live_scene_finds_reachable_pair_and_preserves_all_remembered_collisions():
    from autorace_parking.obstacle_memory import MemoryScan
    from autorace_parking.core import Scan
    d=json.loads((Path(__file__).parent/'data/pair_geometry_live_20261108.json').read_text())
    config=Settings(**d['parameters']);pose=tuple(d['pose'])
    scan=MemoryScan.__new__(MemoryScan);scan.__dict__=Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.).__dict__.copy()
    scan.points=[tuple(p[:2]) for p in d['points']]
    scan.point_owners={tuple(p[:2]):p[2] for p in d['points']}
    report=build_plan_report(scan,config,origin=pose,recovery_allowed=False)
    assert report['final'] and report['path'],report['diagnostics']
    assert report['diagnostics']['enclosure_objects']==5
    assert report['diagnostics']['reason_code']=='path_ready'
    assert report['goal'].kind=='pair_midpoint' and set(report['goal'].support_ids)=={5,8}
    assert set(report['diagnostics']['slot_map']['support_ids'])=={5,8}
    assert config.clearance==.10 and len(scan.points)==1497
    assert all(CollisionChecker(scan,config).free(p) for p in report['path'])
    assert '_pair_maps' not in report['diagnostics']


def test_disabling_pair_fallback_still_rejects_unmatched_layout():
    config=cfg(pair_fallback_enabled=False)
    scan=snapshot(objects([(1.,-.7),(2.04,-.7),(2.7,-2.4)]));details={}
    assert not automatic_goals(scan,scan.candidate_clusters(config),config,(0.,0.,0.),details)

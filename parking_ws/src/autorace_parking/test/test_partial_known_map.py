"""Three measured objects may identify the known five-occupied/six-cell layout."""
import json
import math
from pathlib import Path
from dataclasses import fields,replace
import yaml
from autorace_parking.core import Settings,Scan,Cluster
from autorace_parking.control import build_plan_report
from autorace_parking.slot_map import fit_slot_map,reference_centers
from recorded_profiles import fixed_empty_recording


def fixture():
    root=Path(__file__).resolve().parents[4]
    params=yaml.safe_load((root/'parking_ws/src/autorace_parking/config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    cfg=Settings(**{f.name:params[f.name] for f in fields(Settings) if f.name in params})
    cfg=replace(fixed_empty_recording(cfg),slot_object_box_enabled=False)  # recorded cone surfaces, not LIMO bodies
    s=json.loads((Path(__file__).parent/'data/parking_three_object_20261115.json').read_text())
    return Scan(s['ranges'],s['angle_min'],s['angle_increment'],s['range_min'],s['range_max'],(.103,0.,0.),cfg.max_range),cfg


def report(source,cfg):
    return build_plan_report(source,cfg,None,(0.,0.,0.),[(0.,0.,0.)],True,cfg.blind_reverse_distance,True,source,0,False,None)


def test_real_three_objects_uniquely_register_known_empty_target():
    source,cfg=fixture()
    assert cfg.enclosure_confirm_objects==3
    result=report(source,cfg);d=result['diagnostics'];layout=d['slot_map']
    assert d['enclosure_objects']==3 and d['full_slot_map_matched']
    assert layout['locked'] and layout['hypotheses']==1
    assert result['goal'].slot_id==cfg.fixed_parking_slot_id==4
    assert result['path'] and result['final']


def test_same_scan_was_rejected_by_old_four_object_threshold():
    source,cfg=fixture();result=report(source,replace(cfg,enclosure_confirm_objects=4))
    assert result['diagnostics']['enclosure_objects']==3
    assert result['diagnostics']['slot_map'] is None and not result['path']


def test_collinear_three_objects_do_not_lock_a_two_dimensional_map():
    _,cfg=fixture()
    centers=reference_centers(cfg)
    objects=[]
    for i in (0,1,2):
        x,y=centers[i]+(2.,-1.2)
        points=[(float(x+dx),float(y+dy)) for dx in (-.08,0.,.08) for dy in (-.08,0.,.08)]
        objects.append(((i,),Cluster(points,i)))
    layout=fit_slot_map(objects,cfg)
    assert layout is None or not layout['locked']

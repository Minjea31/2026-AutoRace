"""Finite segment checks use only the rectangle clipped to observed support."""
import json
import math
from pathlib import Path
import random
from dataclasses import fields,replace
import yaml
from autorace_parking.yellow_boundary import YellowBoundary
from autorace_parking.core import CollisionChecker, Settings, Scan
from autorace_parking.control import build_plan_report
from recorded_profiles import fixed_empty_recording


def reference(pose,hl,hw,segments):
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
    corners=[(x+c*a-s*b,y+s*a+c*b) for a,b in ((-hl,-hw),(hl,-hw),(hl,hw),(-hl,hw))]
    for a,b,side in segments:
        dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
        if length<.02:continue
        polygon=[(((px-a[0])*dx+(py-a[1])*dy)/length,
                  side*(dx*(py-a[1])-dy*(px-a[0]))/length) for px,py in corners]
        # Sutherland-Hodgman oracle: clip the whole polygon at each endpoint.
        for bound,sense in ((0.,1),(length,-1)):
            clipped=[]
            if not polygon:break
            previous=polygon[-1]
            for current in polygon:
                inside=sense*(current[0]-bound)>=0
                was_inside=sense*(previous[0]-bound)>=0
                if inside!=was_inside:
                    f=(bound-previous[0])/(current[0]-previous[0])
                    clipped.append((bound,previous[1]+f*(current[1]-previous[1])))
                if inside:clipped.append(current)
                previous=current
            polygon=clipped
        if polygon and min(v for _,v in polygon)<-1e-6:return True
    return False


def test_rotated_footprints_and_finite_segments_match_reference():
    r=random.Random(2026);segments=[]
    for _ in range(15):
        a=(r.uniform(-2,2),r.uniform(-2,2));angle=r.uniform(-math.pi,math.pi)
        b=(a[0]+.4*math.cos(angle),a[1]+.4*math.sin(angle))
        segments.append((a,b,r.choice((-1,1))))
    boundary=YellowBoundary(segments)
    for _ in range(2000):
        p=(r.uniform(-3,3),r.uniform(-3,3),r.uniform(-math.pi,math.pi))
        assert boundary.crosses(p[0],p[1],math.cos(p[2]),math.sin(p[2]),.261,.21)==reference(p,.261,.21,segments)


def test_corner_outside_observed_segment_does_not_block_but_real_crossing_does():
    boundary=YellowBoundary([((.555,.211),(.607,.213),-1)])
    assert not boundary.crosses(.3,0.,1.,0.,.261,.21)
    assert boundary.crosses(.3,.004,1.,0.,.261,.21)
    assert not boundary.crosses(.2,1.,1.,0.,.261,.21)


def test_empty_space_queries_do_not_grow_obstacle_grid():
    scan=Scan([math.inf]*361,-math.pi,math.pi/180,.01,8.)
    cfg=Settings(assume_unobserved_free=True,max_range=8.)
    checker=CollisionChecker(scan,cfg)
    for i in range(1000):assert checker.free((i%100*.04,0.,0.))
    assert len(checker.cells)==0
    assert checker.samples is CollisionChecker(scan,cfg).samples


def recorded_scene():
    root=Path(__file__).resolve().parents[4]
    params=yaml.safe_load((root/'parking_ws/src/autorace_parking/config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    cfg=Settings(**{f.name:params[f.name] for f in fields(Settings) if f.name in params})
    cfg=replace(fixed_empty_recording(cfg),slot_object_box_enabled=False,clearance=.10,path_tracking_margin=.02,
                yellow_boundary_enabled=True,entry_via_short_edge=False)  # legacy yellow-check fixture
    data=json.loads((Path(__file__).parent/'data/parking_yellow_finite_20261115.json').read_text())
    s=data['scan']
    source=Scan(s['ranges'],s['angle_min'],s['angle_increment'],s['range_min'],s['range_max'],(.103,0.,0.),cfg.max_range)
    points=data['yellow'];compact=[points[0]]
    for p in points[1:]:
        if math.dist(compact[-1],p)>=.04:compact.append(p)
    if math.dist(compact[-1],points[-1])>=.02:compact.append(points[-1])
    source.yellow_segments=[(a,b,-1) for a,b in zip(compact,compact[1:]) if math.dist(a,b)>=.02]
    return source,cfg


def test_recorded_scene_search_is_not_blocked_by_segment_extensions():
    source,cfg=recorded_scene()
    checker=CollisionChecker(source,cfg,known_free_poses=[(0.,0.,0.)])
    assert source.yellow_segments
    assert all(checker.free((.3*i/15,0.,0.)) for i in range(16))



def test_recorded_scene_has_complete_safe_entry_with_independent_yellow_margin():
    from dataclasses import replace
    source,cfg=recorded_scene()
    assert cfg.clearance==.10 and cfg.yellow_boundary_clearance==.02
    result=build_plan_report(source,cfg,None,(0.,0.,0.),[(0.,0.,0.)],True,cfg.blind_reverse_distance,True,source,0,False,None)
    assert result['path'] and result['final'] and result['goal'] is not None
    checker=CollisionChecker(source,cfg,known_free_poses=[(0.,0.,0.)])
    assert all(checker.free(p) for p in result['path'])
    conservative=CollisionChecker(source,replace(cfg,yellow_boundary_clearance=.10),known_free_poses=[(0.,0.,0.)])
    assert any(not conservative.free(p) and conservative.last_block=='yellow_boundary' for p in result['path'])


def test_obstacle_clearance_stays_ten_centimeters():
    cfg=Settings(assume_unobserved_free=True,obstacle_min_cluster_size=0.,clearance=.10,yellow_boundary_clearance=.02)
    source=Scan([math.inf]*361,-math.pi,math.pi/180,.01,8.)
    source.points=[(.24,0.)]  # 7.9 cm ahead of the physical body; still blocked.
    checker=CollisionChecker(source,cfg)
    assert not checker.free((0.,0.,0.)) and checker.last_block=='obstacle'

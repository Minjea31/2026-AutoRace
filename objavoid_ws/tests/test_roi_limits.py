from autorace_objavoid.node import *
from autorace_objavoid.start_gate import StartGate


def test_default_roi_and_rank_limits():
    c=Config();g=StartGate()
    assert (g.x_min,g.x_max,g.y_min,g.y_max)==(0.,2.5,-.6,.6)
    z=Sector(float('inf'),20)
    first=(.8,.1,.05,.05,0.)
    side=(1.2,.65,.05,.05,0.)
    far=(2.6,.1,.05,.05,0.)
    scan=Sectors(z,z,z,1.,1.,boxes=(first,side,far))
    assert nearest_objects(scan,c)==[first]
    state,_,angle,checked=plan_avoidance(scan,c)
    assert checked.preview_distance<=2.5
    assert scan.boxes==(first,side,far)


def test_preview_default_and_maximum_are_independent_of_sensor_roi():
    c=Config();z=Sector(float('inf'),20)
    assert c.preview_distance==.5 and c.max_preview_distance==1. and c.far_distance==2.5
    s=Sectors(z,z,z,1.,1.)
    assert plan_avoidance(s,c)[3].preview_distance==.5
    s=replace(s,boxes=((.8,.1,.03,.03,0.),(1.3,-.1,.03,.03,0.)))
    state,_,angle,route=plan_avoidance(s,c)
    assert path_is_valid(state)
    assert route.preview_distance==1.
    assert path_clearance(s,angle,route)>0

from autorace_objavoid.node import *


def test_large_cluster_is_still_an_obstacle():
    z=Sector(float('inf'),20);s=Sectors(z,z,z,1.,1.)
    group=((.4,-.6),(.4,0.),(.4,.6))
    output=object_sectors(s,(group,),Config())
    assert len(output.boxes)==1
    assert path_clearance(output,0.,Config())==0.


def test_lane_target_guides_plan_without_yellow_boundary():
    z=Sector(float('inf'),20);s=Sectors(z,z,z,1.,1.)
    for sign in (-1,1):
        data=(1.,sign*.1,0.,1.,0.,1.,2.,2.,sign*.1,0.,.49)
        state,_,angle,route=plan_avoidance(s,Config(),lane_hint(data,Config()),True,lane_data=data)
        assert path_is_valid(state)
        assert angle*sign>0
        assert path_clearance(s,angle,route)>0

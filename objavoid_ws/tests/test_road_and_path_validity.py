from autorace_objavoid.node import *


def test_lane_offset_does_not_become_road_collision_boundary():
    data=(1.,.05,0.,1.,0.,1.,2.,2.,.05,0.,.2)
    assert current_road(data,None,True,Config()) is None
    z=Sector(float('inf'),20)
    scan=Sectors(z,z,z,1.,1.,road=current_road(data,None,True,Config()))
    state,speed,angle,c=plan_avoidance(scan,Config(),.1,True)
    assert path_is_valid(state)
    assert path_clearance(scan,angle,c)>0


def test_real_road_constraints_and_failed_motion_are_preserved():
    band=RoadBand(0.,1.,-.05,.16,2)
    assert current_road(None,band,True,Config()) is band
    assert current_road(None,band,False,Config()) is None
    z=Sector(float('inf'),20)
    state,speed,angle,c=plan_avoidance(Sectors(z,z,z,1.,1.,road=band),Config())
    assert state=='NO_CLEAR_PATH' and speed==.2
    assert not path_is_valid(state)
    assert path_is_valid('AVOID/SHORT_PREVIEW/LANE_UNCONFIRMED')
    assert not path_is_valid('SCAN_UNAVAILABLE/LANE_UNCONFIRMED')


def test_rviz_clears_failed_path_even_while_moving():
    from types import SimpleNamespace
    from autorace_objavoid.visualization import Visualizer
    sent={}
    def publisher(kind,topic,qos):
        sent[topic]=[]
        return SimpleNamespace(publish=sent[topic].append)
    visual=Visualizer(SimpleNamespace(create_publisher=publisher),preview_points,cluster_bounds)
    visual.publish('AVOID',.2,0.,1.,Config(),(),None,(),'laser')
    assert sent['/objavoid/path'][-1].poses
    visual.publish('NO_CLEAR_PATH',.2,.1,2.,Config(),(),None,(),'laser')
    assert not sent['/objavoid/path'][-1].poses
    curve=next(m for m in sent['/objavoid/markers'][-1].markers if m.ns=='command_path')
    assert curve.action==curve.DELETE

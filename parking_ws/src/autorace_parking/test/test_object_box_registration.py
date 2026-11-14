"""Known LIMO rectangles, not visible-face midpoints, register to cell centers."""
from dataclasses import replace
import math
import numpy as np
import pytest
from autorace_parking.core import Settings,Cluster
from autorace_parking.object_boxes import infer_object_box
from autorace_parking.slot_map import reference_centers,rotation,fit_slot_map


def config():
    return replace(Settings(),slot_map_enabled=True,fixed_parking_slot_id=4,
        enclosure_confirm_objects=3,slot_object_box_enabled=True,slot_initial_right_only=True)


def scene(yaw=0.,face='front',indices=(0,1,2,3,5)):
    cfg=config();rot=rotation(yaw);origin=np.array((2.,-1.2))
    truth=reference_centers(cfg)@rot.T+origin
    objects=[]
    for i in indices:
        if face=='front':local=[(x,cfg.slot_object_length/2) for x in np.linspace(-cfg.slot_object_width/2,cfg.slot_object_width/2,9)]
        else:local=[(-cfg.slot_object_width/2,y) for y in np.linspace(-cfg.slot_object_length/2,cfg.slot_object_length/2,9)]
        points=np.asarray(local)@rot.T+truth[i]
        objects.append(((i,),Cluster([tuple(p) for p in points],i)))
    return objects,truth,origin


@pytest.mark.parametrize('yaw',[0.,.2,-.3])
@pytest.mark.parametrize('face',['front','side'])
def test_surface_only_objects_recover_body_center_and_grid(yaw,face):
    cfg=config();objects,truth,origin=scene(yaw,face)
    before=[list(c.points) for _,c in objects]
    layout=fit_slot_map(objects,cfg,object_yaw=yaw)
    assert layout and layout['locked']
    assert layout['origin']==pytest.approx(origin,abs=.003)
    assert layout['yaw']==pytest.approx(yaw,abs=.003)
    for box in layout['object_boxes']:
        assert box['center']==pytest.approx(truth[box['track_id']],abs=.003)
        assert math.dist(box['corners'][0],box['corners'][1])==pytest.approx(.220)
        assert math.dist(box['corners'][1],box['corners'][2])==pytest.approx(.322)
    assert [c.points for _,c in objects]==before  # collision geometry not moved


def test_visible_front_face_midpoint_bias_is_removed():
    cfg=config();objects,truth,origin=scene()
    old=fit_slot_map(objects,replace(cfg,slot_object_box_enabled=False))
    new=fit_slot_map(objects,cfg)
    assert math.dist(old['origin'],origin)==pytest.approx(.322/2)
    assert math.dist(new['origin'],origin)<.001


def test_full_rectangle_extent_is_not_shifted_twice():
    cfg=config()
    points=[(2.+x,-1.+y) for x,y in ((-.11,-.161),(.11,-.161),(.11,.161),(-.11,.161))]
    result=infer_object_box(Cluster(points,9),0.,(0.,0.),.22,.322)
    assert result['center']==pytest.approx((2.,-1.))

import math
from dataclasses import replace
from pathlib import Path
import cv2
import numpy as np
import pytest
from autorace_objavoid.node import *
from autorace_objavoid.camera_road import camera_road,camera_guidance
from autorace_objavoid.web_view import GroundProjection


def road(edges):
    return MeasuredRoad(0.,1.,0.,0.,2,edges)


def test_single_observed_boundary_never_invents_opposite_boundary():
    z=Sector(math.inf,20);c=Config()
    left=road(((0.,-1.,.45),))
    scan=Sectors(z,z,z,1.,1.,road=left)
    assert path_clearance(scan,.35,c)==0
    assert path_clearance(scan,-.35,c)>0
    assert camera_guidance(left,c)==0
    right=replace(scan,road=road(((0.,1.,.45),)))
    assert path_clearance(right,-.35,c)==0
    assert path_clearance(right,.35,c)>0


def test_both_observed_boundaries_constrain_whole_body():
    z=Sector(math.inf,20);c=Config()
    scan=Sectors(z,z,z,1.,1.,road=road(((0.,-1.,.45),(0.,1.,.45))))
    assert path_clearance(scan,0.,c)>0
    assert path_clearance(scan,.35,c)==0
    assert path_clearance(scan,-.35,c)==0


def test_unknown_boundary_does_not_publish_unconstrained_plan():
    z=Sector(math.inf,20);s=Sectors(z,z,z,1.,1.)
    state,speed,_,_=plan_with_boundaries(s,Config())
    assert state=='ROAD_UNAVAILABLE' and not path_is_valid(state)
    assert speed==.2


def test_broad_yellow_patch_is_not_a_road_boundary():
    projection=GroundProjection.load(Path(__file__).parents[1]/'src/autorace_objavoid/config/camera_projection.yaml')
    mask=np.zeros((480,640),np.uint8)
    cv2.rectangle(mask,(220,250),(400,450),1,-1)
    assert camera_road(mask,projection,Config()) is None


def paint_segment(projection, side, label=1):
    mask = np.zeros((480,640),np.uint8)
    # Only 18 cm of paint is visible, well beyond the 50 cm planning horizon.
    points = np.array([projection.point(x,side*.3,640,480)
                       for x in np.linspace(.85,1.03,40)],np.int32)
    cv2.polylines(mask,[points],False,label,3)
    return mask


@pytest.mark.parametrize('side',[-1,1])
def test_short_yellow_segment_blocks_turn_in_near_camera_blind_area(side):
    projection=GroundProjection.load(Path(__file__).parents[1]/'src/autorace_objavoid/config/camera_projection.yaml')
    c=Config()
    boundary=camera_road(paint_segment(projection,side),projection,c)
    assert boundary is not None and len(boundary.edges)==1
    z=Sector(math.inf,20)
    scan=Sectors(z,z,z,1.,1.)
    assert path_clearance(scan,side*.35,c)>0
    bounded=replace(scan,road=boundary)
    # The entire checked path lies before the observed paint, yet must respect
    # its extrapolation, including the body corners rather than just the centre.
    assert max(x for x,y,yaw in preview_poses(side*.35,c))<.85
    assert path_clearance(bounded,side*.35,c)==0
    assert path_clearance(bounded,-side*.35,c)>0
    data=(1.,side*.3,0.,1.,0.,1.,2.,2.,side*.3,0.,.49)
    state,_,angle,route=plan_in_lane(bounded,c,data)
    assert path_is_valid(state)
    assert path_clearance(bounded,angle,route)>0


def test_white_segment_does_not_create_extended_road_boundary():
    projection=GroundProjection.load(Path(__file__).parents[1]/'src/autorace_objavoid/config/camera_projection.yaml')
    assert camera_road(paint_segment(projection,1,label=2),projection,Config()) is None


def test_finite_boundary_does_not_constrain_body_outside_extended_segment():
    boundary=MeasuredRoad(0.,1.,0.,0.,2,((0.,-1.,.1),),((.8,1.4),))
    z=Sector(math.inf,20)
    scan=Sectors(z,z,z,1.,1.,road=boundary)
    assert path_clearance(scan,0.,Config())>0
    assert path_clearance(scan,0.,replace(Config(),preview_distance=1.))==0
    assert math.isinf(boundary.body_gap(0.,0.,0.,.161,.11))
    assert boundary.body_gap(.9,0.,0.,.161,.11)<0


def test_detected_segment_extension_is_thirty_centimetres_per_end():
    projection=GroundProjection.load(Path(__file__).parents[1]/'src/autorace_objavoid/config/camera_projection.yaml')
    boundary=camera_road(paint_segment(projection,1),projection,Config())
    xmin,xmax=boundary.ranges[0]
    assert xmin==pytest.approx(.55,abs=.04)
    assert xmax==pytest.approx(1.33,abs=.04)

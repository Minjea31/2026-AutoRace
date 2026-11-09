import math
from dataclasses import replace
from pathlib import Path
import cv2
import numpy as np
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

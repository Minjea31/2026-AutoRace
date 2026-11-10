import math
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch
import cv2
import numpy as np
from autorace_objavoid.node import Config,Sector,Sectors,scan_fresh,plan_avoidance
from autorace_objavoid.web_view import GroundProjection,detected_lane_points,render_camera,HTML

CONFIG=Path(__file__).parents[1]/'src/autorace_objavoid/config/camera_projection.yaml'


def test_last_valid_scan_keeps_planning_across_time_gaps():
    sector=Sector(math.inf,20)
    scan=Sectors(sector,sector,sector,1.,1.,boxes=((.8,.12,.05,.05,0.),))
    assert scan_fresh(scan,Config(),300.,300.)
    state,_,_,_=plan_avoidance(scan,Config())
    assert state.startswith('AVOID')
    assert not scan_fresh(None,Config(),300.,300.)
    assert not scan_fresh(scan,replace(Config(),scan_timeout=.25),1.3,1.3)
    assert not scan_fresh(scan,Config(),.1,1.)


def test_actual_lane_pixels_project_back_to_their_ground_position():
    projection=GroundProjection.load(CONFIG)
    mask=np.zeros((480,640),np.uint8)
    x,y=projection.point(.9,.2,640,480)
    mask[y,x]=1
    points=detected_lane_points(mask,projection)
    assert len(points)==1
    assert abs(points[0][0]-.9)<.02
    assert abs(points[0][1]-.2)<.02
    assert detected_lane_points(np.zeros_like(mask),projection)==[]


def test_camera_draws_only_roi_obstacles_and_current_path_geometry():
    data=dict(roi=[(.6,-.4),(1.,-.4)],obstacles=[dict(outline=[(.6,.2),(1.,.2)])],
              path=[(.6,0.),(1.,0.)],footprints=[[(-9.,-9.),(-8.,-8.)]],
              road=[[(-7.,-7.),(-6.,-6.)]],road_dividers=[[(-5.,-5.),(-4.,-4.)]],
              lane=[(-3.,-3.),(-2.,-2.)],diagnostics={'path_held':True})
    projection=GroundProjection.load(CONFIG)
    with patch.object(projection,'point',wraps=projection.point) as project:
        image=render_camera(np.zeros((480,640,3),np.uint8),data,projection,(.2,.8,.86,.92))
    assert len(project.call_args_list)==6
    assert all(c.args[0]>=.6 for c in project.call_args_list)
    # No header banners or projected/estimated centre overlays.
    assert not image[:50].any()
    assert 'd.footprints' not in HTML and 'd.road_dividers' not in HTML and 'd.body' not in HTML
    assert "id=\"cloud\"" not in HTML


def test_camera_shows_detected_lane_mask_without_ground_projection():
    image=np.zeros((32,32,3),np.uint8);mask=np.zeros((32,32),np.uint8)
    mask[8:20,8:12]=2
    result=render_camera(image,{},None,(.2,.8,.86,.92),mask)
    assert result[12,10].min()>180
    assert not image.any()


def test_camera_and_map_show_extended_yellow_boundary():
    projection=GroundProjection.load(CONFIG)
    image=np.zeros((480,640,3),np.uint8)
    data=dict(road=[[(.5,.3),(1.2,.3)]],road_source='yellow_edges')
    result=render_camera(image,data,projection,(.2,.8,.86,.92))
    control=render_camera(image,{},projection,(.2,.8,.86,.92))
    assert np.any(result!=control)
    assert "line(edge,'#ffdc00',2,true)" in HTML

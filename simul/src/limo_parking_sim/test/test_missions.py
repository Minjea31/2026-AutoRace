import json
import math

import numpy as np
import pytest
from PIL import Image

from limo_parking_sim.config import load_config,share_directory
from limo_parking_sim.model import Simulation,overlaps
from limo_parking_sim.world import Cone,cone_course_shape,cone_reference_path,obstacle_reference_path,signs


@pytest.fixture
def cfg(tmp_path):
    # Default-layout checks must not depend on the user's live editor sidecar.
    world = tmp_path/'track.yaml'
    world.write_text((share_directory()/'config/parking_map.yaml').read_text())
    return load_config(world=world)


def test_seven_avoidance_vehicles_and_feasible_ackermann_route(cfg):
    vehicles=[o for o in cfg['world']['obstacles'] if o['name'].startswith('avoid_')]
    assert len(vehicles)==7
    assert {(o['grid_row'],o['grid_column']) for o in vehicles}=={
        ('upper',3),('upper',8),('middle',4),('middle',5),('middle',7),('lower',2),('lower',6)}
    layout=cfg['world']['avoidance_layout']
    for o in vehicles:
        assert o['pose'][0]==pytest.approx(layout['x_min']+(o['grid_column']-.5)
                                          *(layout['x_max']-layout['x_min'])/8)
        assert o['pose'][1]==layout['row_y'][o['grid_row']]
    m=Simulation(cfg)
    path=obstacle_reference_path(m.min_radius)
    assert all(not m.collisions_at(*p) for p in path)
    delta=np.linalg.norm(np.diff(path[:,:2],axis=0),axis=1)
    valid=np.r_[True,delta>1e-8]
    path=path[valid]
    distance=np.r_[0,np.cumsum(np.linalg.norm(np.diff(path[:,:2],axis=0),axis=1))]
    curvature=np.gradient(np.unwrap(path[:,2]),distance)
    assert np.max(np.abs(curvature))<=1/m.min_radius+.01
    assert path[-1,1]==pytest.approx(.68)


def test_s_cones_inside_gate_and_route_clearance(cfg):
    m=Simulation(cfg)
    assert len(m.cones)==48
    course=cfg['world']['cone_course']
    assert min(c.y-c.base_size/2 for c in m.cones)==pytest.approx(course['start_line_y'])
    assert max(c.y+c.base_size/2 for c in m.cones)==pytest.approx(course['end_line_y'])
    for side in ('left','right'):
        row=[c for c in m.cones if c.name.startswith('cone_'+side)]
        assert len(row)==24
        steps=np.linalg.norm(np.diff([[c.x,c.y] for c in row],axis=0),axis=1)
        assert steps.max()<.155 and steps.min()>.08
        # Dense cones should not overlap their square bases on the inner bend.
        separation=np.max(np.abs(np.diff([[c.x,c.y] for c in row],axis=0)),axis=1)
        assert np.all(separation>=course['cone']['base_size'])
        delta=np.diff([c.x for c in row])
        assert np.count_nonzero(delta[:-1]*delta[1:]<0)==4
    assert min(c.x-c.base_size/2 for c in m.cones)<.47
    assert max(c.x+c.base_size/2 for c in m.cones)>.97
    # Four interior extrema form two complete S waves. Check the actual
    # facility shape separately from the smoother feasible vehicle route.
    path=cone_reference_path(course)
    layout_x,_=cone_course_shape(course,path[:,1])
    delta=np.diff(layout_x)
    turns=np.flatnonzero(delta[:-1]*delta[1:]<0)+1
    assert len(turns)==4
    assert layout_x[turns]==pytest.approx([.92,.52,.92,.52],abs=1e-4)
    assert np.ptp(layout_x)==pytest.approx(.4,abs=1e-4)
    # Every paired cone row has this physical double-S shape, rather than
    # drawing a wavy guide while placing the cones somewhere else.
    for c in m.cones:
        x,_=cone_course_shape(course,c.y)
        expected=x+(-1 if c.name.startswith('cone_left') else 1)*course['half_width']
        assert c.x==pytest.approx(expected,abs=2e-6)
    max_deviation=0.
    for x,y,yaw in path:
        assert not m.collisions_at(x,y,yaw)
        corners=m.body(x,y,yaw).corners()
        assert np.all(corners>0) and np.all(corners<np.array(cfg['world']['size']))
        # Sample the whole body perimeter, including between cone centres.
        hull=np.concatenate([a+(b-a)*np.linspace(0,1,21)[:,None]
                            for a,b in zip(corners,np.roll(corners,-1,axis=0))])
        hull=hull[(hull[:,1]>=path[0,1])&(hull[:,1]<=path[-1,1])]
        if len(hull):
            centre_x,_=cone_course_shape(course,hull[:,1])
            max_deviation=max(max_deviation,np.max(np.abs(hull[:,0]-centre_x)))
    assert course['half_width']-course['cone']['base_size']/2-max_deviation>.015
    assert path[[0,-1],0]==pytest.approx([course['center_x']]*2)
    assert cfg['world']['spawns']['cone']==pytest.approx(path[0])
    y=path[:,1]
    frequency=math.pi*course['half_waves']/(path[-1,1]-path[0,1])
    phase=(y-path[0,1])*frequency+math.radians(course['phase_deg'])
    dx=-course['reference_amplitude']*frequency*np.sin(phase)
    ddx=-course['reference_amplitude']*frequency**2*np.cos(phase)
    curvature=np.abs(ddx)/(1+dx*dx)**1.5
    assert curvature.max()<1/m.min_radius
    assert curvature.max()>.85/m.min_radius


def test_cone_height_aware_scan_and_circle_collision(cfg):
    cfg['simulation']['spawn']=[2.,2.,0.]
    m=Simulation(cfg)
    m.obstacles=[]
    m.cones=[Cone('reference',3.103,2.)]
    z=cfg['vehicle']['base_link_height']+cfg['lidar']['xyz'][2]
    radius=m.cones[0].radius_at(z)
    assert m.scan()[len(m.angles)//2]==pytest.approx(1-radius,abs=1e-6)
    assert m.collisions_at(3.103,2.,0)==['reference']
    assert not m.collisions_at(3.103,2.5,0)
    m.cones[0].height=.05
    assert m.scan()[len(m.angles)//2]==0


def test_mission_signs_and_spawn_locations(cfg):
    assets=share_directory()/'assets'
    items=signs(cfg['world'])
    assert {s['name'] for s in items}=={'parking_sign','speed_bump_sign',
             'zigzag_slow_sign','pedestrian_sign','fork_sign'}
    for s in items:
        assert (assets/s['asset']).is_file()
        assert (assets/(s['asset'].removesuffix('.png')+'-print.png')).is_file()
        assert s['panel_width']==pytest.approx(.09)
        assert s['panel_center_height']==pytest.approx(.155)
        assert s['panel_center_height']+s['panel_height']/2==pytest.approx(.20)
        assert s['panel_center_height']-s['panel_height']/2==pytest.approx(.11)
    m=Simulation(cfg)
    for name,pose in cfg['world']['spawns'].items():
        assert not m.collisions_at(*pose),name
    assert cfg['world']['speed_bump']['height']<cfg['vehicle']['ground_clearance']


def test_round_sign_scan_uses_actual_chord_at_lidar_height(cfg):
    m=Simulation(cfg)
    panel=next(b for b in m.obstacles if b.name=='parking_sign_panel')
    assert panel.zmin==pytest.approx(.11) and panel.zmax==pytest.approx(.20)
    assert panel.width_at(.155)==pytest.approx(.09)
    assert panel.width_at(.11)==pytest.approx(0)
    assert panel.width_at(.116)==pytest.approx(2*math.sqrt(.045**2-.039**2))
    assert panel.width_at(.21)==0


def test_marking_texture_restores_zebra_gaps_and_korean_holes():
    assets=share_directory()/'assets'
    data=json.loads((assets/'track_geometry.json').read_text())
    assert data['texture_size_px']==[8000,4800]
    assert data['markings']['slow_text']=='천천히'
    im=Image.open(assets/'track.png')
    def brightness(x,y):
        return im.getpixel((round(x*800),round((6-y)*800)))[0]
    # Both actual staggered crossings: white strip centres and black gaps.
    assert brightness(3.35,4.675)>200
    assert brightness(3.35,4.735)<100
    assert brightness(3.60,4.640)>200
    assert brightness(3.60,4.700)<100
    assert brightness(9.38,2.12)>200
    assert brightness(9.44,2.12)<100
    assert brightness(9.42,2.33)>200
    assert brightness(9.48,2.33)<100
    # Hollow diamond centres must remain road colour, not a filled white blob.
    assert brightness(2.878,5.58)<100
    # The text must contain separated strokes and road-coloured holes/gaps.
    crop=np.array(im.crop((round(3.19*800),round((6-5.807)*800),round(3.34*800),round((6-5.355)*800))))
    assert np.mean(crop[:,:,0]>150)<.65
    assert np.mean(crop[:,:,0]>150)>.12

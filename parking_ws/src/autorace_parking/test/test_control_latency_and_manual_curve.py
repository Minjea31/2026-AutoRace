from dataclasses import replace,fields
from pathlib import Path
import json,math,time
import pytest,yaml
from std_msgs.msg import String
from autorace_parking.core import Settings,Gap,Scan,advance,plan_path
from autorace_parking.control import Follower


def clear():return Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.)


@pytest.fixture
def node():
    import rclpy
    from autorace_parking.node import ParkingNode
    rclpy.init();n=ParkingNode()
    try:yield n
    finally:n.destroy_node();rclpy.shutdown()


def test_sensor_subscriptions_keep_only_latest_sample(node):
    profiles={s.topic_name:s.qos_profile for s in node.subscriptions}
    assert all(profiles[t].depth==1 for t in ('/scan','/odometry/filtered','/localization/health'))


def test_health_timeout_and_unhealthy_input_still_stop_immediately(node):
    clock=[100.1]
    node.now_seconds=lambda:clock[0]
    node.tracker.healthy=lambda now:True
    node.on_fusion_health(String(data=json.dumps({'stamp':100.,'ready':True})))
    assert node.localization_ready(100.1)
    assert not node.localization_ready(100.31)
    clock[0]=100.31
    node.on_fusion_health(String(data=json.dumps({'stamp':100.31,'ready':False,'reason':'missing IMU'})))
    assert not node.localization_ready(100.31)
    assert 'missing IMU' in node.reason


def layout():
    return {'origin':[.7,0.],'yaw':0.,'cell_width':.5,'cell_depth':.6,'border':.02,'locked':True,
            'cells':[{'id':0,'center':[.7,0.],'occupied':False}]}


def test_snapshot_and_map_fit_reused_until_new_input_or_occupancy(node,monkeypatch):
    import autorace_parking.slot_map as maps
    calls=[]
    def fit(objects,cfg,hint,sensor_origin=None):
        assert sensor_origin is not None
        calls.append(1)
        return dict(hint)
    monkeypatch.setattr(maps,'fit_slot_map',fit)
    node.cfg=replace(node.cfg,block_occupied_slots=True)
    node.follower.cfg=node.cfg
    node.scan=clear();node.slot_map=layout()
    first=node.planning_scan()
    for _ in range(5):assert node.planning_scan() is first
    assert len(calls)==1
    node.scan_sequence+=1
    assert node.planning_scan() is not first and len(calls)==2
    node.slot_map['cells'][0]['occupied']=True
    updated=node.planning_scan()
    assert len(calls)==3 and len(updated.occupied_slots)==1


def test_visual_collision_cache_never_hides_newly_occupied_cell(node,monkeypatch):
    import autorace_parking.node as module
    node.cfg=replace(node.cfg,block_occupied_slots=True)
    node.follower.cfg=node.cfg
    node.scan=clear();node.slot_map=layout();node.slot_map['locked']=False
    node.follower.accept(Gap(1.5,0.,0.,.5,0,1),[(0.,0.,0.),(1.5,0.,0.)],True,(0.,0.,0.))
    calls=[];original=module.first_occupied_path_collision
    def check(*args):calls.append(1);return original(*args)
    monkeypatch.setattr(module,'first_occupied_path_collision',check)
    assert node.visible_path();assert node.visible_path();assert len(calls)==1
    node.slot_map['cells'][0]['occupied']=True
    assert not node.visible_path() and node.path_hidden_by_slot


def test_error_gain_increases_correction_without_biasing_nominal_curve():
    cfg=Settings(min_turn_radius=.53,curvature_feedforward=True,assume_unobserved_free=True)
    circle=[advance((0.,0.,0.),.53*.9*i/80,-1/.53) for i in range(81)]
    corrections=[]
    for gain in (1.,2.):
        f=Follower(replace(cfg,steering_gain=gain),lookahead=.2)
        f.accept(Gap(*circle[-1],.6,0,1),circle,True,(0.,0.,0.))
        _,angle,_=f.command(clear(),(0.,0.,0.),now=0.)
        assert angle==pytest.approx(-math.atan(.2/.53),abs=1e-4)
        straight=[(i*.02,0.,0.) for i in range(51)]
        f=Follower(replace(cfg,steering_gain=gain),lookahead=.2)
        f.accept(Gap(1.,0.,0.,.6,0,1),straight,True,(0.,0.,0.))
        corrections.append(abs(f.command(clear(),(0.,.01,0.),now=0.)[1]))
    assert corrections[1]>corrections[0]*1.8


def test_latest_manual_curve_gets_a_compact_entry_without_outward_detour():
    root=Path(__file__).resolve().parents[1]
    values=yaml.safe_load((root/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    cfg=Settings(**{f.name:values[f.name] for f in fields(Settings) if f.name in values})
    recorded=json.loads((root/'test/data/manual_curve_20261108.json').read_text())['points']
    x,y,yaw=recorded[-1]
    goal=Gap(x,y,yaw,.6,0,1,kind='enclosed_center',heading_options=tuple(-i*math.pi/36 for i in range(19)))
    route=plan_path(clear(),goal,cfg)
    assert route and math.dist(route[-1][:2],recorded[-1][:2])<.03
    assert max(p[1] for p in route)<.015
    assert max(min(math.dist(p[:2],q[:2]) for q in recorded) for p in route)<.08
    for a,b in zip(route,route[1:]):
        d=math.dist(a[:2],b[:2]);k=2*abs(math.sin((b[2]-a[2])/2))/d
        if k>1e-4:assert 1/cfg.max_planning_turn_radius-1e-6<=k<=1/cfg.turn_radius+1e-6


def test_position_only_arrival_does_not_require_a_perpendicular_heading():
    cfg=Settings(assume_unobserved_free=True,auto_exit_after_parking=False,arrival_use_rear_axle=True)
    f=Follower(cfg,require_parking_heading=False,parking_position_tolerance=.03)
    goal=Gap(1.,0.,-math.pi/2,.5,0,1,kind='enclosed_center',heading_options=(-math.pi/2,),stop_tolerance=.03)
    f.accept(goal,[(0.,0.,0.),(1.,0.,0.)],True,(0.,0.,0.))
    for t in (0.,.1,.2):result=f.command(clear(),(1.,0.,0.),True,t)
    assert result[2]=='PARKED' and f.parked
    assert f.arrival_error(Gap(0.,0.,-math.pi/2,.5,0,1,kind='enclosed_center'))==(0.,0.)


def test_position_only_arrival_still_rejects_an_occupied_footprint():
    cfg=Settings(assume_unobserved_free=True,auto_exit_after_parking=False,block_occupied_slots=True)
    f=Follower(cfg,require_parking_heading=False,parking_position_tolerance=.03)
    goal=Gap(0.,0.,-math.pi/2,.5,0,1,kind='enclosed_center',heading_options=(-math.pi/2,),stop_tolerance=.03)
    f.accept(goal,[(0.,0.,0.)],True,(0.,0.,0.))
    scan=clear();scan.occupied_slots=[{'id':0,'pose':(0.,0.,0.),'half_size':(.25,.3)}]
    for t in (0.,.1,.2):f.command(scan,(0.,0.,0.),True,t)
    assert not f.parked


def test_quarter_circle_can_finish_on_the_arc_without_a_straight_alignment():
    cfg=Settings(min_turn_radius=.53,require_goal_heading=False,assume_unobserved_free=True,
                 max_planning_turn_radius=.58,entry_outward_penalty=3.)
    goal=Gap(.53,-.53,0.,.5,0,1,kind='enclosed_center')
    route=plan_path(clear(),goal,cfg)
    assert route and math.dist(route[-1][:2],(.53,-.53))<.001
    assert route[-1][2]==pytest.approx(-math.pi/2,abs=.001)
    assert all(abs(b[2]-a[2])>.001 for a,b in zip(route[-5:],route[-4:]))



def test_exit_accepts_body_center_and_lane_heading_without_a_second_yaw_penalty():
    cfg=Settings(assume_unobserved_free=True,arrival_use_rear_axle=True,exit_arrival_yaw_tolerance=.35)
    f=Follower(cfg,require_parking_heading=False)
    f.goal=Gap(-.03,.045,-.2,.5,0,1)
    f.path=[(.2,0.,0.),(-.03,.045,-.2)]
    f.set_segment(0);f.last_drive_direction=f.motion_direction
    f.parked=f.exiting=True;f.entry_trace=[(0.,0.,0.)]
    f.exit_route_ready=True;f.index=1
    for t in (0.,.1,.2):result=f.command(clear(),(0.,0.,0.),True,t)
    assert result[2]=='EXITED' and f.exited

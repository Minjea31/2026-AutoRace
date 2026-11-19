from std_msgs.msg import Bool,String
from test_floor_approach import approach,measured

def send(n,clock,t,value=True):
    clock['wall']=t;n.on_sign(Bool(data=value))

def arm_set(n,clock,t):
    send(n,clock,t);send(n,clock,t+.1)

def test_gap_point_eight_and_false_confirm_without_old_half_second_window(approach):
    n,clock,step,commands,guards=approach
    send(n,clock,1000.);send(n,clock,1000.4,False);send(n,clock,1000.8)
    assert n.floor_gate.armed and not n.activation.active

def test_same_set_and_new_set_do_not_restart_white_wait(approach):
    n,clock,step,commands,guards=approach
    arm_set(n,clock,1000.);stamp=n.floor_gate.armed_stamp;wall=n.floor_gate.armed_wall
    for i in range(20):send(n,clock,1000.2+i*.1)
    arm_set(n,clock,1004.)
    assert n.floor_gate.armed_stamp==stamp and n.floor_gate.armed_wall==wall and n.attempt==0

def test_parking_stop_has_no_fixed_five_second_cooldown(approach):
    n,clock,step,commands,guards=approach
    n.parking_sign_motion_guard=True
    arm_set(n,clock,1000.)
    n.floor_gate.observe(n.floor_gate.armed_stamp+1,.9,0.,0.)
    assert n.start_phase=='STOP' and n.attempt==1 and commands[-1]==(0.,0.)
    assert n.sign_motion.allows(clock['wall']) and n.sign_ignore_until==0.
    arm_set(n,clock,1002.)
    assert n.attempt==1 and n.start_phase=='STOP'

def test_blocked_signal_set_is_not_replayed_after_lane_returns(approach):
    n,clock,step,commands,guards=approach
    n.parking_stop_require_lane_owner=True
    n.on_parking_mux_source(String(data='signal'));arm_set(n,clock,1000.)
    assert not n.floor_gate.armed
    n.on_parking_mux_source(String(data='lane'));send(n,clock,1000.2)
    assert not n.floor_gate.armed
    clock['wall']=1001.3;n.on_parking_mux_source(String(data='lane'))
    arm_set(n,clock,1001.3);assert n.floor_gate.armed

def test_confirmed_set_during_active_mission_is_discarded(approach):
    n,clock,step,commands,guards=approach
    n.activation.active=True;n.attempt=1
    arm_set(n,clock,1000.);assert n.attempt==1 and not n.floor_gate.armed
    n.activation.reset();send(n,clock,1000.2)
    assert not n.floor_gate.armed
    arm_set(n,clock,1002.);assert n.floor_gate.armed

def test_failure_requires_real_stop_and_departure_then_fresh_set(approach):
    n,clock,step,commands,guards=approach
    n.parking_sign_motion_guard=True;n.activation.active=True;n.motion_failed=True
    arm_set(n,clock,1000.);assert not n.floor_gate.armed
    for _ in range(4):measured(n,step,v=0.)
    for _ in range(4):measured(n,step,v=.2)
    assert n.sign_motion.allows(clock['wall'])
    send(n,clock,1000.6);assert not n.floor_gate.armed
    arm_set(n,clock,1002.);assert n.floor_gate.armed and not n.motion_failed

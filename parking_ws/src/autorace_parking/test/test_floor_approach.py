"""Sign-to-floor approach must move before requesting a stationary calibration."""
from concurrent.futures import Future
from types import SimpleNamespace
import json

import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry

from autorace_mux.core import Candidate
from autorace_mux.parking_guard import ParkingGuard
import autorace_parking.floor_gate as floor_module
import autorace_parking.node as module
import autorace_parking.start_sequence as start_module


@pytest.fixture
def approach(monkeypatch):
    context=Context();rclpy.init(context=context,domain_id=195,signal_handler_options=SignalHandlerOptions.NO)
    n=module.ParkingNode(context=context,parameter_overrides=[
        Parameter('localization_source',value='fused'),Parameter('white_roi_enabled',value=True),
        Parameter('parking_calibration_stop',value=True),Parameter('manual_mode_guard',value=False),
        Parameter('prevent_active_retrigger',value=True)])
    n.control_timer.cancel()
    clock={'wall':1000.,'ros':n.now_seconds()}
    fake_time=SimpleNamespace(monotonic=lambda:clock['wall'])
    monkeypatch.setattr(module,'time',fake_time);monkeypatch.setattr(floor_module,'time',fake_time)
    monkeypatch.setattr(start_module,'time',fake_time)
    n.now_seconds=lambda:clock['ros']
    commands=[];guards=[]
    n.publish_command=lambda v,a:commands.append((v,a))
    n.guard_pub=SimpleNamespace(publish=lambda m:guards.append(json.loads(m.data)))
    def step(dt=0.,v=.2):
        clock['wall']+=dt;clock['ros']+=dt
        n.wheel_sample=(clock['ros'],v,0.,clock['wall'])
        n.fusion_health={'stamp':clock['ros'],'ready':True,'calibrating':False,'calibration_generation':1}
        n.fusion_health_received=clock['wall']
    step()
    yield n,clock,step,commands,guards
    n.destroy_node();context.shutdown()


def arm(n):
    n.on_sign(Bool(data=True));n.on_sign(Bool(data=True))
    assert n.floor_gate.armed and not n.activation.active


def allow_lane(n,clock,guards):
    n.publish_guard();guard=ParkingGuard()
    assert guard.receive(guards[-1],clock['wall'],clock['ros'])
    candidate=Candidate('lane',.5,.1,round(clock['ros']*1e9),clock['wall'])
    return guard.apply(candidate,clock['wall'],clock['ros'])


def test_approach_event_keeps_lane_at_point_two_without_starting_gyro_or_ekf(approach):
    n,clock,step,commands,guards=approach
    arm(n)
    n.on_approach_start(Header(stamp=n.get_clock().now().to_msg()))
    n.tick()
    output=allow_lane(n,clock,guards)
    assert output is not None and output.speed==pytest.approx(.2)
    assert output.steering_angle==pytest.approx(.1)
    assert n.state=='WAITING_WHITE_LINE' and n.start_phase=='IDLE'
    assert n.approach_calibration.phase=='IDLE' and not n.approach_calibration.pending
    assert not commands  # No zero parking lease can override the approach.


def test_repeated_signs_and_approach_events_do_not_restart_the_floor_budget(approach):
    n,clock,step,commands,guards=approach
    arm(n);started=n.floor_gate.armed_wall;stamp=n.floor_gate.armed_stamp
    for _ in range(20):
        step(.1)
        n.on_sign(Bool(data=True))
        n.on_approach_start(Header(stamp=n.get_clock().now().to_msg()))
    assert n.floor_gate.armed_wall==started and n.floor_gate.armed_stamp==stamp
    assert n.floor_gate.approach_distance==pytest.approx(.4,abs=1e-5)
    assert n.approach_calibration.phase=='IDLE'


def test_white_line_stops_before_one_gyro_request_and_duplicate_signs_do_not_repeat_it(approach):
    n,clock,step,commands,guards=approach
    arm(n)
    class Client:
        calls=[]
        def service_is_ready(self):return True
        def call_async(self,request):
            self.calls.append(request);future=Future()
            future.set_result(SimpleNamespace(success=True,message='{"generation":1}'))
            return future
    client=Client();n.approach_calibration_client=client
    n.floor_gate.observe(n.floor_gate.armed_stamp+1,.9,0.,0.)
    assert n.activation.active and n.start_phase=='STOP' and commands[-1]==(0.,0.)
    assert not client.calls
    n.publish_guard();assert guards[-1]['phase']=='HOLD' and not guards[-1]['ready']
    step(.05,v=.2);n.tick_start_sequence()
    assert not client.calls
    for _ in range(3):
        step(.05,v=0.);n.tick_start_sequence()
    assert n.start_phase=='GYRO' and len(client.calls)==1
    for _ in range(5):
        n.on_sign(Bool(data=True));n.on_approach_start(Header(stamp=n.get_clock().now().to_msg()))
    assert n.attempt==1 and len(client.calls)==1 and not n.floor_gate.armed
    assert all(v==0. for v,a in commands)


def test_no_white_line_times_out_and_manual_reset_remains_supported(approach):
    n,clock,step,commands,guards=approach
    arm(n);step(20.1,v=0.);n.tick()
    assert n.state=='APPROACH_LIMIT' and n.floor_gate.failure
    assert allow_lane(n,clock,guards) is None and guards[-1]['phase']=='FAILED'
    n.floor_gate.observe(n.floor_gate.armed_stamp+1,.9,0.,0.)
    assert not n.activation.active and commands[-1]==(0.,0.)
    assert n.start_phase=='IDLE' and n.approach_calibration.phase=='IDLE'
    n.on_reset(None,Trigger.Response())
    n.on_approach_start(Header(stamp=n.get_clock().now().to_msg()))
    n.publish_guard()
    assert guards[-1]['phase']=='RESET' and not n.floor_gate.armed and not n.floor_gate.failure
    arm(n);assert allow_lane(n,clock,guards).speed==pytest.approx(.2)


def measured(n,step,dt=.05,v=.2):
    previous=n.wheel_sample
    step(dt,v=v)
    n.wheel_sample=previous
    msg=Odometry();msg.child_frame_id=n.base_frame
    stamp=n.now_seconds();msg.header.stamp.sec=int(stamp)
    msg.header.stamp.nanosec=int((stamp-int(stamp))*1e9)
    msg.twist.twist.linear.x=v
    n.on_measured_wheel(msg)


@pytest.mark.parametrize('previous',['white_timeout','failed','exited','search_exhausted'])
def test_retry_requires_actual_departure_new_sign_and_new_white(approach,previous):
    n,clock,step,commands,guards=approach
    n.parking_sign_motion_guard=True
    if previous=='white_timeout':
        # Arm the original approach without the new sign gate to create its failure.
        n.parking_sign_motion_guard=False;arm(n)
        step(20.1,v=0.);n.tick();assert n.floor_gate.failure
        n.parking_sign_motion_guard=True
    else:
        n.activation.active=True;n.attempt=1
        n.follower.exited=previous=='exited'
        n.motion_failed=previous=='failed'
        if previous=='search_exhausted':
            n.search_started=n.now_seconds()-n.search_timeout-1.
            n.follower.path=[]
    for _ in range(240):
        measured(n,step,v=0.)
        n.on_sign(Bool(data=True))
    assert n.floor_gate.confirmation.count==0
    # Commanded speed alone is deliberately irrelevant.
    n.publish_command(.2,0.)
    n.on_sign(Bool(data=True));assert n.floor_gate.confirmation.count==0
    # Failed-attempt recovery still requires a real departure, but ordinary
    # stop/departure feedback must not create a second five-second timer.
    for _ in range(3):
        measured(n,step)
    assert n.sign_motion.allows(clock['wall'])
    assert n.floor_gate.confirmation.count==0
    n.on_sign(Bool(data=True));assert n.floor_gate.confirmation.count==1
    n.on_sign(Bool(data=True))
    assert n.floor_gate.armed and not n.floor_gate.failure and not n.activation.active
    assert not n.motion_failed and not n.follower.exited
    arm_stamp=n.floor_gate.armed_stamp
    n.floor_gate.observe(arm_stamp-1,.9,0.,0.)
    assert not n.activation.active
    n.floor_gate.observe(arm_stamp+1,.9,0.,0.)
    assert n.activation.active and n.start_phase=='STOP'
    assert commands[-1]==(0.,0.)


def test_healthy_mission_cannot_be_retriggered_after_departure(approach):
    n,clock,step,commands,guards=approach
    n.parking_sign_motion_guard=True
    n.activation.active=True;n.attempt=1
    for _ in range(110):measured(n,step)
    for _ in range(5):n.on_sign(Bool(data=True))
    assert not n.floor_gate.armed and n.attempt==1


def test_failure_cannot_rearm_while_vehicle_is_still_braking(approach):
    n,clock,step,commands,guards=approach
    n.parking_sign_motion_guard=True
    for _ in range(110):measured(n,step)
    arm(n)
    for _ in range(410):
        measured(n,step,v=.1)
        n.on_sign(Bool(data=True))
    assert n.floor_gate.failure and n.sign_motion.await_stop
    assert n.floor_gate.confirmation.count==0
    assert not n.activation.active


def test_measured_three_meters_stops_approach_before_twenty_seconds(approach):
    n,clock,step,commands,guards=approach
    arm(n)
    for _ in range(151):
        step(.1);n.floor_gate.update_approach()
    n.tick()
    assert n.floor_gate.approach_distance>=3. and clock['wall']<1020.
    assert n.state=='APPROACH_LIMIT' and '3m' in n.floor_gate.failure
    assert allow_lane(n,clock,guards) is None and commands[-1]==(0.,0.)


@pytest.mark.parametrize('previous',['exited','failed'])
def test_a_new_approach_retires_the_old_route_before_waiting_for_white(approach,previous):
    n,clock,step,commands,guards=approach
    n.activation.active=True;n.attempt=1
    n.follower.exited=previous=='exited'
    n.motion_failed=previous=='failed'
    n.parking_commands_released=previous=='exited'
    n.follower.path=[(0.,0.,0.),(1.,0.,0.)]
    arm(n);n.tick()
    assert n.attempt==1 and not n.follower.path and not n.follower.exited
    assert not n.parking_commands_released and not n.motion_failed
    assert allow_lane(n,clock,guards).speed==pytest.approx(.2)


def test_stop_line_cooldown_is_scoped_to_the_accepted_parking_stop(approach):
    n,clock,step,commands,guards=approach
    n.parking_sign_motion_guard=True
    for _ in range(5):measured(n,step,v=0.)
    arm(n)  # Startup/ordinary standstill must not reject the parking sign.
    assert n.sign_motion.allows(clock['wall'])
    step(.1,v=.2)
    n.floor_gate.observe(n.floor_gate.armed_stamp+1,.9,0.,0.)
    stopped_at=clock['wall']
    assert commands[-1]==(0.,0.) and n.start_phase=='STOP' and n.attempt==1
    assert n.sign_motion.ignore_until==pytest.approx(stopped_at+5.)
    for _ in range(5):n.on_sign(Bool(data=True))
    assert n.attempt==1 and n.floor_gate.confirmation.count==0
    measured(n,step,dt=4.99,v=0.)
    assert not n.sign_motion.allows(clock['wall'])
    measured(n,step,dt=.01,v=0.)
    assert n.sign_motion.allows(clock['wall'])
    measured(n,step,dt=.1,v=0.)
    assert n.sign_motion.ignore_until==pytest.approx(stopped_at+5.)


def test_ordinary_stop_during_point_two_approach_does_not_start_cooldown(approach):
    n,clock,step,commands,guards=approach
    n.parking_sign_motion_guard=True
    arm(n)
    for _ in range(5):measured(n,step,v=0.)
    assert n.floor_gate.armed and not n.activation.active
    assert n.sign_motion.allows(clock['wall'])
    assert n.sign_motion.ignore_until==float('-inf')
    n.floor_gate.observe(n.floor_gate.armed_stamp+1,.9,0.,0.)
    assert n.start_phase=='STOP' and not n.sign_motion.allows(clock['wall'])


def test_rejected_stop_line_start_does_not_start_cooldown(approach):
    n,clock,step,commands,guards=approach
    n.parking_sign_motion_guard=True
    n.sign_ignore_until=clock['wall']+10.
    n.start_from_floor()
    assert n.attempt==0
    assert n.sign_motion.allows(clock['wall'])


def test_signal_stop_line_before_parking_sign_does_not_start_cooldown(approach):
    n,clock,step,commands,guards=approach
    n.parking_sign_motion_guard=True
    n.parking_stop_require_lane_owner=True
    n.on_parking_mux_source(String(data='signal'))
    for _ in range(5):measured(n,step,v=0.)
    n.on_sign(Bool(data=True));n.on_sign(Bool(data=True))
    assert n.floor_gate.confirmation.count==0
    n.floor_gate.observe(n.get_clock().now().nanoseconds,.9,0.,0.)
    assert not n.activation.active and not n.floor_gate.armed
    assert n.attempt==0 and n.sign_motion.allows(clock['wall'])


@pytest.mark.parametrize('owner',['signal','crosswalk','stop'])
def test_other_mission_white_line_cannot_start_parking_stop_or_cooldown(approach,owner):
    n,clock,step,commands,guards=approach
    n.parking_sign_motion_guard=True
    n.parking_stop_require_lane_owner=True
    n.on_parking_mux_source(String(data='lane'))
    arm(n)
    stamp=n.floor_gate.armed_stamp
    n.on_parking_mux_source(String(data=owner))
    n.floor_gate.observe(stamp+1,.9,0.,0.)
    assert not n.activation.active and n.attempt==0 and n.floor_gate.armed
    assert n.sign_motion.allows(clock['wall']) and not commands
    n.on_parking_mux_source(String(data='lane'))
    n.floor_gate.observe(stamp+2,.9,0.,0.)
    assert n.activation.active and n.start_phase=='STOP' and n.attempt==1
    assert not n.sign_motion.allows(clock['wall']) and commands[-1]==(0.,0.)


def test_stale_lane_owner_cannot_trigger_parking_stop(approach):
    n,clock,step,commands,guards=approach
    n.parking_sign_motion_guard=True
    n.parking_stop_require_lane_owner=True
    n.on_parking_mux_source(String(data='lane'))
    arm(n)
    n.on_parking_mux_source(String(data='lane'))
    step(.31)
    n.floor_gate.observe(n.floor_gate.armed_stamp+1,.9,0.,0.)
    assert n.floor_gate.armed and n.attempt==0 and n.sign_motion.allows(clock['wall'])

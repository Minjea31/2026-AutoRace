from concurrent.futures import Future
from dataclasses import fields,replace
import json,math,os
from pathlib import Path
import subprocess
import pytest,yaml
from autorace_parking.core import Settings,Gap,Scan,CollisionChecker,wrap
from autorace_parking.control import build_plan_report
from autorace_parking.obstacle_memory import MemoryScan
from autorace_parking.regions import with_occupied_slots


def profile():
    values=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/parking_real.yaml').read_text())['parking_node']['ros__parameters']
    return Settings(**{f.name:values[f.name] for f in fields(Settings) if f.name in values})


@pytest.mark.parametrize('initial',[False,True])
def test_manual_recording_generates_a_complete_clear_entry_with_physical_turning_radius(initial):
    data=json.loads((Path(__file__).parent/'data/manual_20261107_225625_scene.json').read_text());config=profile()
    if initial:data=data['initial']
    scan=MemoryScan.__new__(MemoryScan);scan.__dict__=Scan([math.inf]*181,-math.pi/2,math.pi/180,.01,12.).__dict__.copy()
    scan.points=[(x,y) for x,y,_ in data['memory_cells']];scan.point_owners={(x,y):o for x,y,o in data['memory_cells']}
    result=build_plan_report(scan,config,layout_hint=data['slot_map'],recovery_allowed=False)
    assert result['final'] and result['diagnostics']['reason_code']=='path_ready'
    assert result['goal'].slot_id==4 and 1.6<result['goal'].x<1.8
    assert config.turn_radius==pytest.approx(.55) and config.require_steering_calibration
    assert config.steering_limit>math.atan(.2/.55)
    assert config.steering_limit<config.max_steering
    checker=CollisionChecker(with_occupied_slots(scan,result['diagnostics']['slot_map'],(0.,0.,0.),config),config)
    assert all(checker.free(p) for p in result['path'])
    for a,b in zip(result['path'],result['path'][1:]):
        distance=math.dist(a[:2],b[:2])
        if distance>1e-6:assert 2*abs(math.sin(wrap(b[2]-a[2])/2))/distance<=1/.55+1e-6


def test_cpp_command_calibration_has_neutral_symmetry_inverse_fit_and_recorded_cap(tmp_path):
    candidate=Path(__file__).resolve().parents[2]/'platform_working/sensor_ws/src/limo_ros2/limo_base/include/limo_base/steering_command_model.hpp'
    if not candidate.exists():
        candidate=next((parent/'sensor_ws/src/limo_ros2/limo_base/include/limo_base/steering_command_model.hpp'
                        for parent in Path(__file__).resolve().parents
                        if (parent/'sensor_ws/src/limo_ros2/limo_base/include/limo_base/steering_command_model.hpp').exists()),candidate)
    header=Path(os.environ.get('STEERING_MODEL_HEADER',str(candidate)))
    assert header.is_file()
    source=tmp_path/'test.cpp';binary=tmp_path/'test'
    source.write_text('#include "'+str(header)+'"\n'+r'''
#include <cassert>
#include <limits>
int main() {
  AgileX::SteeringCommandModel m;
  assert(std::abs(m.encode(.567,2.47)-.567/2.47)<1e-9);
  m.enabled=true;m.validate();
  assert(m.encode(0.,2.47)==0.);
  assert(std::abs(m.encode(.211,2.47)-.2296875)<1e-8);
  const double physical=std::atan(.2/(.55-.086));
  const double wire=m.encode(physical,2.47);
  assert(wire>0.32 && wire<.35);
  assert(std::abs(m.gain*wire-m.deadband-physical)<1e-10);
  assert(m.encode(-physical,2.47)==-wire);
  assert(m.encode(1.,2.47)==.35 && m.encode(-1.,2.47)==-.35);
  for (double bad : {0.0,-1.0,std::numeric_limits<double>::infinity()}) {
    m.gain=bad;bool rejected=false;
    try {m.validate();} catch (const std::invalid_argument&) {rejected=true;}
    assert(rejected);
  }
}
''')
    subprocess.run(['g++','-std=c++14','-Wall','-Wextra',str(source),'-o',str(binary)],check=True,capture_output=True)
    subprocess.run([str(binary)],check=True,capture_output=True)


@pytest.fixture
def node():
    import rclpy
    from rclpy.parameter import Parameter
    from autorace_parking.node import ParkingNode
    rclpy.init();n=ParkingNode(parameter_overrides=[Parameter('localization_source',value='scan'),
        Parameter('require_steering_calibration',value=True),Parameter('min_turn_radius',value=.55),Parameter('assume_unobserved_free',value=True)])
    try:yield n
    finally:n.destroy_node();rclpy.shutdown()


def test_missing_or_old_driver_never_activates_the_tighter_parking_control(node):
    import time
    node.activation.active=True;node.now_seconds=lambda:20.;node.scan_time=20.
    node.scan=Scan([math.inf]*361,-math.pi,math.pi/180,.01,12.)
    node.follower.accept(Gap(1.,0.,0.,.6,0,1),[(0.,0.,0.),(.5,0.,0.),(1.,0.,0.)],True,(0.,0.,0.))
    commands=[];node.publish_command=lambda v,a:commands.append((v,a))
    node.tick();assert node.state=='WAITING_STEERING' and commands[-1]==(0.,0.)
    node.steering_model={'enabled':False,'gain':1.92,'deadband':.23,'limit':.35};node.steering_model_time=time.monotonic()
    assert not node.steering_model_ready()
    node.steering_model['enabled']=True;assert node.steering_model_ready()
    node.steering_model['gain']=2.47;assert not node.steering_model_ready()
    node.steering_model['gain']=1.92;node.steering_model_time=time.monotonic()-3.1
    assert not node.steering_model_ready()


def test_driver_parameter_reply_is_validated_before_enabling_model(node):
    from rcl_interfaces.msg import ParameterValue
    from rcl_interfaces.srv import GetParameters
    f=Future();f.set_result(GetParameters.Response(values=[ParameterValue(type=1,bool_value=True),
        ParameterValue(type=3,double_value=1.92),ParameterValue(type=3,double_value=.23),ParameterValue(type=3,double_value=.35)]))
    node.steering_query=f;node.poll_steering_model()
    assert node.steering_model_ready()


def test_exit_does_not_stop_at_heading_crossing_far_from_the_original_lane():
    from autorace_parking.control import Follower
    cfg=replace(Settings(),auto_exit_after_parking=True)
    grid={'origin':[1.2,-1.],'yaw':0.,'width':.5,'depth':.6}
    f=Follower(cfg);goal=Gap(1.2,-1.,-1.,.5,0,1,parking_grid=grid)
    f.accept(goal,[(0.,0.,0.),(1.2,-1.,-1.)],True,(0.,0.,0.))
    f.entry_trace=[(0.,0.,0.),(.5,0.,.1),(1.,.3,0.),(1.2,-.5,-.5),(1.2,-1.,-1.)]
    f.prepare_exit((1.2,-1.,-1.),10.)
    assert f.exit_route_ready and f.path[-1]==(0.,0.,0.)

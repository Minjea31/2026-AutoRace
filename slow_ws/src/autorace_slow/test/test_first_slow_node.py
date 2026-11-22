from types import SimpleNamespace
import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
import autorace_slow.node as module


def test_node_first_mission_uses_point_two_for_six_seconds_then_uses_existing_speed(monkeypatch):
    context=Context();rclpy.init(context=context,domain_id=193)
    n=module.SlowNode(context=context,parameter_overrides=[
        Parameter('white_roi_enabled',value=False),Parameter('start_delay_sec',value=0.),
        Parameter('speed_mps',value=.15)])
    n.timer.cancel();clock=[100.];commands=[]
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:clock[0]))
    n.command_pub=SimpleNamespace(publish=commands.append)
    def lane():
        m=AckermannDriveStamped();m.header.stamp=n.get_clock().now().to_msg()
        m.drive.speed=.5;n.on_lane(m)
    def sign(value,stamp):
        clock[0]=stamp;n.on_sign('slow',Bool(data=value))
    try:
        sign(True,100.);assert n.state.slow_missions_started==0
        sign(True,100.1);lane();n.publish()
        assert n.state.slow_missions_started==1 and commands[-1].drive.speed==pytest.approx(.2)
        assert n.state.missions['slow'].deadline==pytest.approx(106.1)
        for stamp in (101.,102.,103.,104.,105.,106.):
            sign(True,stamp);lane();n.publish()
            assert n.state.slow_missions_started==1
        count=len(commands);clock[0]=106.1;lane();n.publish()
        assert len(commands)==count
        sign(False,107.);sign(True,107.1);sign(True,107.2);lane();n.publish()
        assert n.state.slow_missions_started==2
        assert commands[-1].drive.speed==pytest.approx(.15)
        assert n.state.missions['slow'].deadline==pytest.approx(122.2)
    finally:
        n.destroy_node();context.shutdown()

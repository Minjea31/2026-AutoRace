import json
import time
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String
from test_localization_rematch import odom,scan,health
from autorace_parking.node import ParkingNode


def test_short_outage_stops_retains_map_and_requires_new_scan_before_resume():
    rclpy.init();n=ParkingNode(parameter_overrides=[Parameter('localization_preserve_timeout',value=1.)])
    clock=[100.];n.now_seconds=lambda:clock[0];commands=[];n.publish_command=lambda v,a:commands.append((v,a))
    try:
        n.activation.active=True
        n.on_localization(odom(100.));n.on_scan(scan(100.));n.on_fusion_health(health(100.))
        tracker=n.tracker;n.follower.path=[(0.,0.,0.),(.5,0.,0.)];layout={'retained':True};n.slot_map=layout
        clock[0]=100.4
        assert not n.localization_ready(clock[0])
        assert commands[-1]==(0.,0.) and n.tracker is tracker and n.slot_map is layout and n.follower.path
        for t in (100.41,100.45):
            clock[0]=t;n.on_localization(odom(t));n.on_fusion_health(health(t))
        assert not n.localization_ready(clock[0])
        n.on_scan(scan(clock[0]))
        assert n.localization_ready(clock[0]) and n.localization_hold_since is None
        assert n.tracker is tracker and n.slot_map is layout
        n.on_localization(odom(clock[0]));n.on_localization(odom(100.42))
        assert n.tracker is tracker and not n.localization_recovering
    finally:n.destroy_node();rclpy.shutdown()


def test_long_outage_or_pose_jump_discards_old_frame():
    rclpy.init();n=ParkingNode(parameter_overrides=[Parameter('localization_preserve_timeout',value=1.)])
    clock=[100.];n.now_seconds=lambda:clock[0];n.publish_command=lambda *args:None
    try:
        n.on_localization(odom(100.));n.on_scan(scan(100.));n.on_fusion_health(health(100.))
        original=n.tracker
        clock[0]=100.4;n.localization_ready(clock[0])
        clock[0]=101.5;n.localization_ready(clock[0])
        assert n.localization_recovering and n.tracker is not original and not n.odometry.samples
        clock[0]=101.6;n.on_localization(odom(101.6));n.on_scan(scan(101.6));n.on_fusion_health(health(101.6))
        old=n.tracker;clock[0]=101.7;n.on_localization(odom(101.7,5.))
        assert n.tracker is not old and not n.odometry.samples
    finally:n.destroy_node();rclpy.shutdown()

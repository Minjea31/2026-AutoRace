"""Map epochs clear viewer state and reject queued markers from an old map."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import threading
import json

from std_msgs.msg import String
from visualization_msgs.msg import Marker,MarkerArray

spec=importlib.util.spec_from_file_location('obj_web',Path(__file__).with_name('web_viewer.py'))
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

def viewer():
    return SimpleNamespace(lock=threading.Lock(),missions={},markers=[{'ns':'old'}],frame='odom',
        marker_at=1.,mapping_epoch=None,marker_floor_stamp_ns=0)

def status(v,epoch,stamp):
    module.Viewer.on_status(v,'/objavoid_v3/state',String(data=json.dumps(dict(
        mapping_epoch=epoch,mapping_started_at=stamp,state='RUN',t=stamp))))

def markers(stamp):
    m=Marker(ns='yellow_observed',type=Marker.POINTS,action=Marker.ADD)
    m.header.stamp.sec=stamp;m.header.frame_id='base_link'
    return MarkerArray(markers=[m])

def test_reset_clears_points_and_old_packets_cannot_restore_them():
    v=viewer();status(v,2,20.)
    assert not v.markers and v.marker_at is None
    module.Viewer.on_markers(v,markers(19));assert not v.markers
    module.Viewer.on_markers(v,markers(21));assert v.markers[0]['ns']=='yellow_observed'
    status(v,2,20.);assert v.markers
    status(v,3,22.);assert not v.markers
    module.Viewer.on_markers(v,markers(21));assert not v.markers

def test_delete_all_and_clock_rewind_clear_the_display():
    v=viewer();status(v,2,20.);module.Viewer.on_markers(v,markers(21))
    m=Marker(action=Marker.DELETEALL);m.header.stamp.sec=22
    module.Viewer.on_markers(v,MarkerArray(markers=[m]));assert not v.markers
    status(v,3,1.);module.Viewer.on_markers(v,markers(2));assert v.markers

"""Geometry and stale-data contracts; no vehicle publishers are created."""
import json
import math
import struct
import time
from types import SimpleNamespace as NS

from parking_bridge import ParkingState, cloud_points, finite_json, transform_point


def test_mounting_transform_rotates_and_translates_scan_points():
    tf = NS(translation=NS(x=.103,y=.2,z=0.),rotation=NS(x=0.,y=0.,z=math.sqrt(.5),w=math.sqrt(.5)))
    point = transform_point((1.,0.,0.),tf)
    assert abs(point[0]-.103)<1e-12 and abs(point[1]-1.2)<1e-12
    upside_down=NS(translation=NS(x=0.,y=0.,z=0.),rotation=NS(x=1.,y=0.,z=0.,w=0.))
    assert transform_point((1.,2.,0.),upside_down)==[1.,-2.]


def test_point_cloud_handles_endianness_row_padding_and_nonfinite_points():
    for endian in ('<','>'):
        msg=NS(fields=[NS(name='x',datatype=7,offset=0),NS(name='y',datatype=7,offset=4)],
               width=1,height=2,point_step=8,row_step=12,is_bigendian=endian=='>',
               data=struct.pack(endian+'ff',1.,2.)+b'PAD!'+struct.pack(endian+'ff',3.,4.)+b'PAD!')
        assert cloud_points(msg)==[[1.,2.],[3.,4.]]
    msg.data=struct.pack('>ff',float('nan'),2.)+b'PAD!'+struct.pack('>ff',3.,4.)+b'PAD!'
    assert cloud_points(msg)==[[3.,4.]]


def state(active=True,valid=True,attempt=1,restarts=0):
    return NS(data=json.dumps({'active':active,'mission_attempt':attempt,
              'synchronization':{'localization_restart_count':restarts},
              'localization':{'valid':valid}}))


def test_mission_overlays_disappear_on_inactive_invalid_or_stale_status():
    store=ParkingState();store.on_status(state());store.put('scan',[[1.,1.]]);store.put('path',[[1.,2.]])
    assert 'path' in store.snapshot()['layers']
    store.on_status(state(active=False))
    assert store.snapshot()['layers']=={'scan':[[1.,1.]]}
    store.on_status(state(valid=False))
    assert 'path' not in store.snapshot()['layers']
    store.status_time=time.monotonic()-3.
    data=store.snapshot();assert data['status'] is None and not data['connected']
    assert 'path' not in data['layers']


def test_new_mission_or_localization_restart_discards_all_prior_geometry():
    store=ParkingState();store.on_status(state());store.put('path',[[1.,2.]])
    store.on_status(state(attempt=2));assert not store.snapshot()['layers']
    store.put('path',[[1.,2.]])
    store.on_status(state(attempt=2,restarts=1));assert not store.snapshot()['layers']


def test_layer_ttl_and_nonfinite_numbers_never_leak_stale_or_invalid_json():
    store=ParkingState();store.on_status(state());store.layers['path']=(time.monotonic()-2.,[[1.,2.]])
    assert 'path' not in store.snapshot()['layers']
    assert finite_json({'a':[float('nan'),float('inf'),.5]})=={'a':[None,None,.5]}
    json.dumps(finite_json({'bad':float('nan')}),allow_nan=False)


def test_throttled_complete_marker_update_does_not_clear_previous_geometry():
    store=ParkingState();store.on_status(state());store.put('obstacles',[{'id':1}])
    store.received['obstacles']=time.monotonic()
    message=NS(markers=[NS(action=3),NS(action=0)])
    store.on_markers('obstacles',message)
    assert store.snapshot()['layers']['obstacles']==[{'id':1}]
    store.on_markers('obstacles',NS(markers=[NS(action=3)]))
    assert store.snapshot()['layers']['obstacles']==[]

import math
import numpy as np
import pytest

from limo_parking_sim.view_controls import OrbitView
from limo_parking_sim.config import load_config
from limo_parking_sim.model import Simulation, overlaps


def test_observer_pan_rotate_zoom_and_limits():
    view = OrbitView((0,-3,3),(0,0,0))
    eye,target,distance = view.eye().copy(),view.target.copy(),view.distance
    view.pan(.2,.1,60,40)
    assert np.linalg.norm(view.target-target)>.1
    assert view.eye()-eye == pytest.approx(view.target-target)
    target = view.target.copy()
    view.rotate(.2,.15)
    assert view.target == pytest.approx(target)
    assert np.linalg.norm(view.eye()-target) == pytest.approx(distance)
    view.zoom(1)
    assert view.distance<distance
    view.rotate(0,100)
    assert view.elevation == pytest.approx(math.radians(89.5))
    view.rotate(0,-100)
    assert view.elevation == pytest.approx(math.radians(5))
    view.zoom(100)
    assert view.distance == .2
    view.zoom(-100)
    assert view.distance == 30


def test_full_inner_row_and_sign_beside_line():
    cfg = load_config()
    vehicles = [o for o in cfg['world']['obstacles'] if o['name'].startswith('parked_')]
    assert len(vehicles)==5
    assert len([o for o in vehicles if o['name'].startswith('parked_inner_')])==3
    model = Simulation(cfg)
    boxes = [b for b in model.obstacles if b.name.startswith('parked_')]
    for i,a in enumerate(boxes):
        for b in boxes[i+1:]:
            assert not overlaps(a,b)
    assert not model.collisions_at(model.x,model.y,model.yaw)
    assert not model.collisions_at(6.88,4.27,-math.pi/2)
    # Sign is adjacent to the white parking entrance line; board cannot protrude
    # over it into the approach lane or touch any parked vehicle.
    sign = next(b for b in model.obstacles if b.name=='parking_sign_panel')
    assert sign.corners()[:,1].max()<4.570188
    assert 0<4.570188-sign.corners()[:,1].max()<.02
    for b in boxes:
        assert not overlaps(sign,b)

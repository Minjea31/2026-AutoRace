from autorace_mux.parking_guard import ParkingGuard
from autorace_mux.core import Candidate


def state(phase, stamp=10., ready=True):return dict(stamp=stamp,phase=phase,ready=ready)
def lane(t=10.):return Candidate('lane',.8,.1,int(t*1e9),t)
def parking(t=10.):return Candidate('parking',-.12,.2,int(t*1e9),t)


def test_approach_is_capped_until_explicit_release_even_after_ten_seconds():
    g=ParkingGuard();g.arm(10.,10.,10.)
    for t in (10.1,20.2,40.):
        assert g.receive(state('APPROACH',t),t,t)
        assert g.apply(lane(t),t,t).speed==.3
    assert g.receive(state('RELEASE',40.1),40.1,40.1)
    assert g.apply(lane(),40.1,40.1).speed==.8


def test_planning_expired_command_never_falls_through_to_lane():
    g=ParkingGuard();g.receive(state('DRIVE'),10.,10.)
    assert g.apply(parking(),10.,10.).speed<0
    assert g.apply(lane(),10.1,10.1) is None
    assert g.apply(parking(),10.31,10.31) is None


def test_restart_idle_and_old_release_cannot_unlock_guard():
    g=ParkingGuard();g.arm(10.,10.,10.)
    assert not g.receive(state('RELEASE',9.9),10.,10.)
    assert g.receive(state('IDLE',10.1),10.1,10.1)
    assert g.apply(lane(),10.1,10.1) is None
    assert not g.receive(state('RELEASE',10.),10.1,10.1)
    assert g.receive(state('RESET',10.2),10.2,10.2)
    assert not g.armed


def test_not_ready_future_malformed_and_manual_failure_hold_zero():
    g=ParkingGuard();g.arm(10.,10.,10.)
    assert not g.receive(state('RELEASE',float('nan')),10.,10.)
    assert not g.receive(state('RELEASE',11.),10.,10.)
    assert not g.receive(state('DRIVE',10.,1),10.,10.)
    assert g.receive(state('APPROACH',10.1,False),10.1,10.1)
    assert g.apply(lane(),10.1,10.1) is None
    assert g.receive(state('FAILED',10.2),10.2,10.2)
    assert g.apply(parking(),10.2,10.2) is None

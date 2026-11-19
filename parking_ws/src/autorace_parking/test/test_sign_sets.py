import pytest
from autorace_parking.sign_sets import ParkingSignSet

@pytest.mark.parametrize('delta',[0.1,0.8,1.0])
def test_same_set_confirms_twice(delta):
    s=ParkingSignSet();assert not s.receive(True,10.);assert s.receive(True,10.+delta)
    assert not s.receive(True,10.+delta+.1)

def test_gap_above_one_needs_two_new_messages():
    s=ParkingSignSet();assert not s.receive(True,0.);assert not s.receive(True,1.001)
    assert s.receive(True,1.1)
    assert not s.receive(True,2.101);assert s.receive(True,2.2)

def test_continuous_ignored_true_keeps_last_time():
    s=ParkingSignSet();events=sum(s.receive(True,i*.1) for i in range(10000))
    assert events==1;assert s.last_true==pytest.approx(999.9)
    assert not s.receive(True,1000.);assert not s.receive(True,1001.001)
    assert s.receive(True,1001.1)

def test_false_does_not_split_or_reset():
    s=ParkingSignSet();s.receive(True,10.)
    assert not s.receive(False,10.5);assert s.count==1 and s.last_true==10.
    assert s.receive(True,10.8)
    s.receive(False,11.);assert not s.receive(True,11.5)
    s.receive(False,13.);assert not s.receive(True,13.1);assert s.receive(True,13.2)

@pytest.mark.parametrize('gap',[0.,-1.,float('nan'),float('inf')])
def test_invalid_gap(gap):
    with pytest.raises(ValueError):ParkingSignSet(gap)

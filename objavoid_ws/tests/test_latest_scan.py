import threading
from autorace_objavoid.latest_scan import LatestScan


def test_keeps_only_latest_without_mutating_planner_snapshot():
    buffer=LatestScan()
    first=object()
    buffer.put(first,10.)
    planning=buffer.snapshot()
    for i in range(1,101):buffer.put(i,10.+i*.1)
    latest=buffer.snapshot()
    assert latest.message==100
    assert latest.sequence==101
    assert abs(latest.period-.1)<1e-12
    assert planning.message is first
    assert planning.received==10.
    assert planning.sequence==1


def test_reception_continues_while_planner_is_blocked():
    buffer=LatestScan();started=threading.Event();release=threading.Event()
    buffer.put('initial',1.)
    seen=[]
    def planner():
        current=buffer.snapshot()
        started.set()
        release.wait(2)
        seen.append(current.message)
    thread=threading.Thread(target=planner);thread.start()
    assert started.wait(1)
    try:
        for i in range(50):buffer.put(i,2.+i*.01)
        assert buffer.snapshot().message==49
        assert buffer.snapshot().sequence==51
    finally:
        release.set();thread.join(2)
    assert seen==['initial']

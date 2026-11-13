"""One planner process with one running job and one replaceable pending snapshot."""
from concurrent.futures import ProcessPoolExecutor
from concurrent.futures.process import BrokenProcessPool
import multiprocessing
import time


class PlanningBudgetExceeded(Exception):
    pass


def check_budget(parameters):
    deadline = parameters.get('_planning_deadline')
    if deadline is not None and time.monotonic() >= deadline:
        raise PlanningBudgetExceeded()


def compute_path(payload):
    # Import in the child; no ROS node or executor is created here.
    from cone_lane_follower.node import RoiNode
    left, right, pairs, measured, width, parameters, budget = payload
    parameters = dict(parameters, _planning_deadline=time.monotonic()+budget)
    diagnostics = {}
    start = time.monotonic()
    try:
        check_budget(parameters)
        path = RoiNode.unified_path(left, right, pairs, measured, width, parameters, diagnostics)
    except PlanningBudgetExceeded:
        path = measured
        diagnostics['planning_budget'] = 'exceeded; retained measured path'
    diagnostics['planning_ms'] = round((time.monotonic()-start)*1000, 2)
    return path, diagnostics


class AsyncPlanner:
    def __init__(self, executor=None):
        self.executor = executor
        self.future = None
        self.pending = None
        self.running = None
        self.generation = 0

    def request(self, payload, metadata):
        self.pending = (self.generation, payload, metadata)
        self._start()

    def _start(self):
        if self.future is not None or self.pending is None:
            return
        generation, payload, metadata = self.pending
        self.pending = None
        if self.executor is None:
            self.executor = ProcessPoolExecutor(
                max_workers=1, mp_context=multiprocessing.get_context('spawn'))
        self.running = (generation, metadata)
        self.future = self.executor.submit(compute_path, payload)

    def poll(self):
        if self.future is None or not self.future.done():
            return None
        generation, metadata = self.running
        try:
            value = self.future.result()
        except Exception as exc:
            value = ([], {'planner_error': str(exc)})
            if isinstance(exc, BrokenProcessPool):
                self.executor.shutdown(wait=False, cancel_futures=True)
                self.executor = None
        self.future = self.running = None
        self._start()
        return (metadata, value) if generation == self.generation else None

    def reset(self):
        self.generation += 1
        self.pending = None

    def close(self):
        self.reset()
        if self.executor is not None:
            self.executor.shutdown(wait=False, cancel_futures=True)
            self.executor = None

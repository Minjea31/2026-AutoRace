"""One immutable input snapshot; reception never waits for planning."""
from dataclasses import dataclass
import threading


@dataclass(frozen=True)
class ScanInput:
    message: object
    received: float
    period: float | None
    sequence: int


class LatestScan:
    def __init__(self):
        self._lock = threading.Lock()
        self._latest = None

    def put(self, message, received):
        with self._lock:
            previous = self._latest
            self._latest = ScanInput(message, received,
                received-previous.received if previous else None,
                previous.sequence+1 if previous else 1)

    def snapshot(self):
        with self._lock:
            return self._latest

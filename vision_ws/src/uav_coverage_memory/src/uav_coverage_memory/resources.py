"""Bounded input retention and cooperative CPU pacing; independent of ROS."""
from collections import deque
import math


class ByteWindow:
    """Retain whole messages only. Overflow evicts oldest, never slices data."""
    def __init__(self, max_items, max_bytes):
        if any(int(x) != x or x <= 0 for x in (max_items, max_bytes)):
            raise ValueError('positive window bounds required')
        self.max_items, self.max_bytes = max_items, max_bytes
        self.items = deque()
        self.bytes = 0

    def append(self, message):
        size = len(message.data)
        if size > self.max_bytes:
            self.clear()
            return False
        while self.items and (len(self.items) >= self.max_items or self.bytes+size > self.max_bytes):
            old = self.items.popleft()
            self.bytes -= len(old.data)
        self.items.append(message)
        self.bytes += size
        return True

    def clear(self):
        self.items.clear()
        self.bytes = 0

    def __iter__(self):
        return iter(self.items)

    def __len__(self):
        return len(self.items)


class WorkBudget:
    """Pace heavy work using wall monotonic time, independent of ROS speed.

    This is a soft duty budget for measured work, not a real-time preemption
    mechanism or an OS-wide CPU quota. Existing multi-frame freshness rules
    remain in force if throttling makes observations too sparse.
    """
    def __init__(self, max_hz=3., cpu_fraction=.15, warn_wall_ms=150.):
        if (not all(math.isfinite(x) for x in (max_hz, cpu_fraction, warn_wall_ms)) or
                not 0 < max_hz <= 10 or not 0 < cpu_fraction <= 1 or warn_wall_ms <= 0):
            raise ValueError('invalid work budget')
        self.period = 1./max_hz
        self.cpu_fraction = cpu_fraction
        self.warn_wall_ms = warn_wall_ms
        self.next_ready = -math.inf
        self.skipped = self.completed = self.overruns = 0

    def ready(self, now):
        # Timer periods and accumulated doubles can differ by sub-microseconds.
        # Do not throw away an entire timer period at that numerical boundary.
        if now < self.next_ready-1e-6:
            self.skipped += 1
            return False
        return True

    def finish(self, start, end, cpu_seconds):
        if not all(math.isfinite(x) for x in (start, end, cpu_seconds)) or end < start or cpu_seconds < 0:
            raise ValueError('invalid measured work interval')
        # Charge measured process CPU, including native library threads and
        # callbacks; one long frame is followed by rest, never a catch-up burst.
        self.next_ready = max(end, start+self.period, start+cpu_seconds/self.cpu_fraction)
        self.completed += 1
        overrun = (end-start)*1000 > self.warn_wall_ms
        self.overruns += int(overrun)
        return dict(work_wall_ms=(end-start)*1000, work_cpu_ms=cpu_seconds*1000,
                    budget_overrun=overrun, budget_overruns=self.overruns,
                    budget_skips=self.skipped, budget_completed=self.completed,
                    cpu_budget_fraction=self.cpu_fraction,
                    cooldown_ms=max(0., self.next_ready-end)*1000)

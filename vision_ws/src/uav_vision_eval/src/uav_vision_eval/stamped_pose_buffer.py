"""Thread-safe zero-order-hold history for stamped evaluation poses."""

import copy
import threading
from collections import deque


class StampedPoseBuffer:
    def __init__(self, max_length=512):
        if int(max_length) < 2:
            raise ValueError("max_length must be at least two")
        self._messages = deque(maxlen=int(max_length))
        self._lock = threading.RLock()

    def add(self, message):
        stamp_ns = int(message.header.stamp.to_nsec())
        if stamp_ns <= 0:
            return False
        with self._lock:
            if self._messages:
                last_ns = int(self._messages[-1].header.stamp.to_nsec())
                if stamp_ns < last_ns:
                    # A Gazebo/reset time jump invalidates the old history.
                    self._messages.clear()
                elif stamp_ns == last_ns:
                    self._messages[-1] = copy.deepcopy(message)
                    return True
            self._messages.append(copy.deepcopy(message))
        return True

    def at_or_before(self, stamp):
        requested_ns = int(stamp.to_nsec())
        with self._lock:
            for message in reversed(self._messages):
                source_ns = int(message.header.stamp.to_nsec())
                if source_ns <= requested_ns:
                    age_sec = (requested_ns - source_ns) / 1.0e9
                    return copy.deepcopy(message), age_sec
        return None, None

    def clear(self):
        with self._lock:
            self._messages.clear()

    def __len__(self):
        with self._lock:
            return len(self._messages)

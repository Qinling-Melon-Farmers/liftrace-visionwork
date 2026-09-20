"""Position-based dwell confirmation for mission terminal states."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PositionSettleResult:
    ready: bool
    reason: str
    elapsed_ns: int = 0
    displacement_m: float = 0.0


class PositionSettleWindow:
    """Confirm that stamped positions remain inside one radius for a dwell."""

    def __init__(self, dwell_ns, radius_m, max_sample_gap_ns):
        self.dwell_ns = int(dwell_ns)
        self.radius_m = float(radius_m)
        self.max_sample_gap_ns = int(max_sample_gap_ns)
        if self.dwell_ns <= 0 or self.max_sample_gap_ns <= 0:
            raise ValueError("settle durations must be positive")
        if not math.isfinite(self.radius_m) or self.radius_m <= 0.0:
            raise ValueError("settle radius must be finite and positive")
        self._anchor = None
        self._last_position = None
        self._start_ns = 0
        self._last_ns = 0
        self._result = PositionSettleResult(False, "inactive")

    @property
    def result(self):
        return self._result

    def reset(self, reason="inactive"):
        self._anchor = None
        self._last_position = None
        self._start_ns = 0
        self._last_ns = 0
        self._result = PositionSettleResult(False, str(reason))
        return self._result

    def _start(self, stamp_ns, position, reason, displacement_m=0.0):
        self._anchor = position
        self._last_position = position
        self._start_ns = stamp_ns
        self._last_ns = stamp_ns
        self._result = PositionSettleResult(
            False, reason, 0, float(displacement_m))
        return self._result

    def update(self, stamp_ns, x, y, z):
        stamp_ns = int(stamp_ns)
        position = (float(x), float(y), float(z))
        if stamp_ns <= 0 or not all(math.isfinite(value) for value in position):
            return self.reset("sample_invalid")

        if self._last_ns:
            if stamp_ns < self._last_ns:
                return self.reset("sample_out_of_order")
            if stamp_ns == self._last_ns:
                if position == self._last_position:
                    return self._result
                return self.reset("sample_stamp_conflict")
            if stamp_ns - self._last_ns > self.max_sample_gap_ns:
                return self._start(stamp_ns, position, "sample_gap_restart")

        if self._anchor is None:
            return self._start(stamp_ns, position, "settle_started")

        displacement = math.sqrt(sum(
            (value - anchor) ** 2
            for value, anchor in zip(position, self._anchor)
        ))
        if displacement > self.radius_m:
            return self._start(
                stamp_ns, position, "motion_restart", displacement)

        self._last_position = position
        self._last_ns = stamp_ns
        elapsed_ns = stamp_ns - self._start_ns
        ready = elapsed_ns >= self.dwell_ns
        self._result = PositionSettleResult(
            ready,
            "settled" if ready else "settling",
            elapsed_ns,
            displacement,
        )
        return self._result


__all__ = ["PositionSettleResult", "PositionSettleWindow"]

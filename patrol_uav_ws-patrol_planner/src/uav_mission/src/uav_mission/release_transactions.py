"""Release facts and stable action identity, independent of ROS runtime."""

EXECUTION_UNKNOWN = 0
NOT_STARTED = 1
RAW_CALL_STARTED = 2
COMPLETED = 3


def stamp_ns(value):
    return int(value.to_nsec()) if hasattr(value, "to_nsec") else (
        int(value.secs) * 1_000_000_000 + int(value.nsecs))


def action_identity(message):
    """Return a complete fence, or None for the old isolated bool interface."""
    mission = str(getattr(message, "mission_id", ""))
    seq = int(getattr(message, "decision_seq", 0))
    first = getattr(message, "target_first_seen", None)
    if not mission or seq <= 0 or first is None or stamp_ns(first) <= 0:
        return None
    return (mission, seq, int(message.attempt), int(message.payload_slot),
            int(message.target_id), stamp_ns(first), str(message.target_class))


def execution_fact(message):
    state = int(getattr(message, "execution_state", EXECUTION_UNKNOWN))
    if state not in (EXECUTION_UNKNOWN, NOT_STARTED, RAW_CALL_STARTED, COMPLETED):
        raise ValueError("invalid release execution state")
    # A legacy false response contains no physical-start information.
    if state == EXECUTION_UNKNOWN and bool(message.success):
        return COMPLETED
    if bool(message.success) != (state == COMPLETED):
        raise ValueError("release success/fact mismatch")
    return state


def result_terminal(message):
    return (bool(getattr(message, "terminal", False)) or
            int(getattr(message, "execution_state", 0)) == EXECUTION_UNKNOWN)


class SemanticContradictionWindow:
    """Only independent, recent, persistent conflicting observations can revoke."""
    def __init__(self, min_frames=3, min_span_ns=150_000_000, max_gap_ns=500_000_000):
        self.min_frames = int(min_frames)
        self.min_span_ns = int(min_span_ns)
        self.max_gap_ns = int(max_gap_ns)
        if self.min_frames < 2 or self.min_span_ns <= 0 or self.max_gap_ns <= 0:
            raise ValueError("invalid contradiction window")
        self.reset()

    def reset(self):
        self.class_name = ""
        self.first_ns = self.last_ns = self.count = 0

    def update(self, class_name, stamp_ns, now_ns):
        stamp_ns, now_ns = int(stamp_ns), int(now_ns)
        if stamp_ns <= 0 or stamp_ns > now_ns or now_ns - stamp_ns > self.max_gap_ns:
            self.reset()
            return False
        if stamp_ns <= self.last_ns:
            return False
        if (class_name != self.class_name or
                stamp_ns - self.last_ns > self.max_gap_ns):
            self.reset()
            self.class_name, self.first_ns = str(class_name), stamp_ns
        self.last_ns = stamp_ns
        self.count += 1
        return (self.count >= self.min_frames and
                self.last_ns - self.first_ns >= self.min_span_ns)

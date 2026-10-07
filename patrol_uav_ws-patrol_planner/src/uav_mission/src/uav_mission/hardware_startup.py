"""ROS-independent admission checks for an initially empty board map.

Pose tuples are (x, y, z, yaw, source_stamp), both at the FC center in the
same fixed mission frame. Never clear/restart a map while airborne.
"""
import math


class PoseAgreement:
    def __init__(self, config):
        self.age = float(config['pose_max_age'])
        self.skew = float(config['pair_max_skew'])
        self.position = float(config['position_tolerance'])
        self.yaw = math.radians(float(config['yaw_tolerance_deg']))
        self.duration = float(config['stable_seconds'])
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.age, self.skew, self.position, self.yaw, self.duration)):
            raise ValueError('invalid mapping startup thresholds')
        if self.skew > self.age:
            raise ValueError('pair skew exceeds pose age')
        self.since = None
        self.last_now = None

    def update(self, fc, lio, now, disarmed):
        reason = None
        info = {}
        if not math.isfinite(now) or (self.last_now is not None and now < self.last_now):
            reason = 'clock_reset'
        elif not disarmed:
            reason = 'vehicle_not_fresh_disarmed'
        elif not fc or not lio:
            reason = 'pose_missing'
        else:
            # Compare a common source time, not newest FC against a delayed LIO.
            # Both latest streams must still be fresh; no old pair can mask loss.
            latest = (fc[-1], lio[-1])
            if fc[-1][4] <= lio[-1][4]:
                a = fc[-1]
                b = min(lio, key=lambda v: abs(v[4] - a[4]))
            else:
                b = lio[-1]
                a = min(fc, key=lambda v: abs(v[4] - b[4]))
            info = dict(latest_age_sec=[now-v[4] for v in latest],
                        paired_age_sec=[now-v[4] for v in (a,b)])
            if not all(math.isfinite(v) for v in (*a, *b)):
                reason = 'pose_nonfinite'
            elif not all(v[4] > 0 and 0 <= now-v[4] <= self.age for v in (*latest, a, b)):
                reason = 'pose_stale_or_future'
            elif abs(a[4]-b[4]) > self.skew:
                reason = 'pose_stamp_skew'
            else:
                distance = math.dist(a[:3], b[:3])
                yaw = abs(math.atan2(math.sin(a[3]-b[3]), math.cos(a[3]-b[3])))
                info.update(position_delta_m=distance, yaw_delta_deg=math.degrees(yaw))
                if distance > self.position or yaw > self.yaw:
                    reason = 'fc_lio_disagreement'
        self.last_now = now
        if reason is not None:
            self.since = None
            return False, dict(reason=reason, **info)
        if self.since is None:
            self.since = now
        ready = now-self.since >= self.duration
        return ready, dict(reason='stable' if ready else 'settling',
                           stable_for=now-self.since, **info)


class MapWarmup:
    def __init__(self, start, config):
        self.start = start
        self.age = float(config['map_max_age'])
        self.duration = float(config['map_warmup_seconds'])
        if not all(math.isfinite(v) and v > 0 for v in (self.age, self.duration)):
            raise ValueError('invalid mapping warmup thresholds')
        self.first = None
        self.last = None
        self.count = 0

    def observe(self, stamp, now, valid):
        if not valid or not math.isfinite(stamp) or not self.start < stamp <= now or now-stamp > self.age:
            return
        if self.last is not None and stamp <= self.last:
            return
        if self.last is None or stamp-self.last > self.age:
            self.first = stamp
            self.count = 0
        self.last = stamp
        self.count += 1

    def ready(self, now):
        return (self.last is not None and self.count >= 2 and
                0 <= now-self.last <= self.age and
                self.last-self.first >= self.duration)


class VisionReadiness:
    def __init__(self, topics, max_age):
        self.age = float(max_age)
        if not topics or len(set(topics)) != len(topics) or not math.isfinite(self.age) or self.age <= 0:
            raise ValueError('invalid vision readiness configuration')
        self.streams = {t: [None, 0] for t in topics}

    def observe(self, topic, stamp, now):
        if not math.isfinite(stamp) or not 0 < stamp <= now or now-stamp > self.age:
            return
        row = self.streams[topic]
        if row[0] is None or stamp > row[0]:
            row[1] = 1 if row[0] is None or stamp-row[0] > self.age else min(2, row[1]+1)
            row[0] = stamp

    def missing(self, now):
        return [topic for topic, (stamp, count) in self.streams.items()
                if stamp is None or count < 2 or not 0 <= now-stamp <= self.age]


def startup_transport_pending(detail):
    """Wait within the existing startup deadline, never grant READY from a gap."""
    reason=detail.get('reason')
    if reason=='settling':return True
    if reason not in ('pose_stale_or_future','pose_stamp_skew'):return False
    ages=detail.get('latest_age_sec',[])+detail.get('paired_age_sec',[])
    return len(ages)==4 and all(math.isfinite(v) and v>=0 for v in ages)

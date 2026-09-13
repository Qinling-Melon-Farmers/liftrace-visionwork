"""Bounded survey catalog and explicit-cost proposals, independent of flight.

Source timestamps never become fresh through replay. A ready hint is only a
revisit hypothesis; there is deliberately no conversion to TargetCandidate,
NavigationDecision or release permission here. Times use integer nanoseconds.
"""
from collections import deque
from dataclasses import dataclass
from itertools import permutations
import math

NS = 1000000000


@dataclass(frozen=True)
class Epoch:
    mission: str
    localization: str
    calibration: str

    def __post_init__(self):
        if not all(isinstance(x, str) and x for x in
                   (self.mission, self.localization, self.calibration)):
            raise ValueError('explicit mission/localization/calibration required')


@dataclass(frozen=True, order=True)
class Key:
    target_id: int
    first_seen_ns: int


@dataclass(frozen=True)
class Observation:
    epoch: Epoch
    key: Key
    stamp_ns: int
    frame: str
    class_name: str
    xy: tuple
    confidence: float
    quality: float
    uncertainty_m: float
    fc_agl: float
    map_valid: bool = True
    association_valid: bool = True
    transform_age_ns: int = 0


@dataclass(frozen=True)
class Config:
    frame: str = 'camera_init'
    capacity: int = 16
    observations: int = 8
    min_hits: int = 3
    min_span_ns: int = 500000000
    min_interval_ns: int = 250000000
    input_max_age_ns: int = 500000000
    tf_max_age_ns: int = 100000000
    hint_ttl_ns: int = 60000000000
    max_observation_gap_ns: int = 2000000000
    vote_fraction: float = .8
    min_confidence: float = .7
    min_quality: float = .5
    max_uncertainty_m: float = .25
    high_min_agl: float = 2.0
    high_max_agl: float = 3.0
    max_revisits: int = 2

    def __post_init__(self):
        counts = (self.capacity, self.observations, self.min_hits, self.max_revisits)
        times = (self.min_span_ns, self.min_interval_ns, self.input_max_age_ns,
                 self.tf_max_age_ns, self.hint_ttl_ns, self.max_observation_gap_ns)
        if (not self.frame or any(type(v) is not int or v <= 0 for v in counts+times)
                or not 3 <= self.min_hits <= self.observations <= 8
                or self.capacity > 16 or self.max_revisits > 2):
            raise ValueError('invalid bounds')
        values = (self.vote_fraction, self.min_confidence, self.min_quality,
                  self.max_uncertainty_m, self.high_min_agl, self.high_max_agl)
        if (not all(math.isfinite(x) for x in values)
                or not .5 < self.vote_fraction <= 1
                or not 0 <= self.min_confidence <= 1 or not 0 <= self.min_quality <= 1
                or self.max_uncertainty_m <= 0
                or not 0 < self.high_min_agl <= self.high_max_agl <= 3):
            raise ValueError('invalid thresholds')


@dataclass
class Entry:
    samples: deque
    attempts: int = 0


@dataclass(frozen=True)
class Hint:
    epoch: Epoch
    key: Key
    class_name: str
    xy: tuple
    uncertainty_m: float
    last_seen_ns: int
    vote_fraction: float
    evidence_count: int
    # It is intentionally impossible to infer release eligibility from this.
    role: str = 'REVISIT_HINT_ONLY'


class Catalog:
    def __init__(self, config, weights):
        self.config = config
        self.weights = dict(weights)
        if (not self.weights or len(self.weights) > 5
                or any(not isinstance(k, str) or not k or not math.isfinite(v) or v <= 0
                       for k, v in self.weights.items())):
            raise ValueError('one explicit profile with at most five classes required')
        self.epoch = None
        self.mission = None
        self.invalid_epoch = None
        self.entries = {}
        self.last_now_ns = None
        self.delivered = {}  # slot -> class; preserved across spatial resets
        self.retired = set()  # bounded source keys cannot refill attempts via eviction

    def reset(self, epoch):
        if not isinstance(epoch, Epoch):
            raise ValueError('epoch required')
        if epoch == self.invalid_epoch:
            raise ValueError('new epoch required after clock rewind')
        if self.epoch != epoch:
            if self.mission != epoch.mission:
                self.delivered.clear()
            self.mission = epoch.mission
            self.entries.clear()
            self.retired.clear()
            self.last_now_ns = None
            self.epoch = epoch
            self.invalid_epoch = None

    def tick(self, now_ns):
        if type(now_ns) is not int or now_ns <= 0:
            raise ValueError('positive integer source clock required')
        if self.last_now_ns is not None and now_ns < self.last_now_ns:
            self.entries.clear()
            # A caller must supply a new epoch after a source-clock rewind.
            self.invalid_epoch = self.epoch or self.invalid_epoch
            self.epoch = None
            self.last_now_ns = now_ns
            return False
        self.last_now_ns = now_ns
        for key, e in list(self.entries.items()):
            if now_ns-e.samples[-1].stamp_ns > self.config.hint_ttl_ns:
                # Keep a bounded tombstone instead of allowing a new identity
                # with the same key to bypass the revisit limit.
                self.retired.add(key)
                del self.entries[key]
        return self.epoch is not None

    def observe(self, obs, now_ns):
        if not self.tick(now_ns):
            return 'context_unavailable'
        c = self.config
        if obs.epoch != self.epoch:
            return 'epoch_mismatch'
        if obs.frame != c.frame:
            return 'frame_mismatch'
        if (type(obs.stamp_ns) is not int or obs.stamp_ns <= 0
                or not 0 <= now_ns-obs.stamp_ns <= c.input_max_age_ns
                or type(obs.transform_age_ns) is not int
                or not 0 <= obs.transform_age_ns <= c.tf_max_age_ns):
            return 'stale_or_future'
        # TargetCandidate.id is uint32 and target_memory starts at zero.
        # has_target in a motion contract, not numeric ID 0, is the sentinel.
        if (type(obs.key.target_id) is not int or not 0 <= obs.key.target_id < 2**32
                or type(obs.key.first_seen_ns) is not int
                or not 0 < obs.key.first_seen_ns <= obs.stamp_ns):
            return 'invalid_identity'
        if obs.class_name not in self.weights:
            return 'unsupported_class'
        if obs.class_name in self.delivered.values() or obs.key in self.retired:
            return 'retired_or_delivered'
        values = tuple(obs.xy)+(obs.confidence, obs.quality, obs.uncertainty_m, obs.fc_agl)
        if len(obs.xy) != 2 or not all(math.isfinite(v) for v in values):
            return 'nonfinite'
        if (obs.map_valid is not True or obs.association_valid is not True
                or not c.min_confidence <= obs.confidence <= 1
                or not c.min_quality <= obs.quality <= 1
                or not 0 < obs.uncertainty_m <= c.max_uncertainty_m):
            return 'quality_rejected'
        if not c.high_min_agl <= obs.fc_agl <= c.high_max_agl:
            return 'outside_survey_height'
        e = self.entries.get(obs.key)
        if e is None:
            # A total lifetime identity cap also bounds retired memory.
            if len(self.entries)+len(self.retired) >= c.capacity:
                return 'capacity_reached'
            e = Entry(deque(maxlen=c.observations))
            self.entries[obs.key] = e
        if e.samples:
            delta = obs.stamp_ns-e.samples[-1].stamp_ns
            if delta < c.min_interval_ns:
                return 'duplicate_or_too_close'
            if delta > c.max_observation_gap_ns:
                e.samples.clear()
        e.samples.append(obs)
        return 'accepted'

    def hints(self, now_ns):
        if not self.tick(now_ns):
            return []
        c = self.config
        candidates = []
        for key, e in self.entries.items():
            s = list(e.samples)
            if (e.attempts >= c.max_revisits or len(s) < c.min_hits
                    or s[-1].stamp_ns-s[0].stamp_ns < c.min_span_ns):
                continue
            votes = {}
            for o in s:
                votes[o.class_name] = votes.get(o.class_name, 0)+1
            cls, hits = max(votes.items(), key=lambda item: (item[1], item[0]))
            agreement = hits/len(s)
            if agreement < c.vote_fraction or cls in self.delivered.values():
                continue
            # Only winning-class positions contribute. Spread cannot shrink
            # below the supplied, uncalibrated per-observation error floor.
            positions = [o for o in s if o.class_name == cls]
            xy = tuple(sum(o.xy[i] for o in positions)/len(positions) for i in (0, 1))
            uncertainty = max(o.uncertainty_m+math.hypot(o.xy[0]-xy[0], o.xy[1]-xy[1])
                              for o in positions)
            if uncertainty <= c.max_uncertainty_m:
                candidates.append(Hint(self.epoch, key, cls, xy, uncertainty, positions[-1].stamp_ns,
                                       agreement, hits))
        ambiguous = set()
        for i, a in enumerate(candidates):
            for b in candidates[i+1:]:
                if math.hypot(a.xy[0]-b.xy[0], a.xy[1]-b.xy[1]) <= a.uncertainty_m+b.uncertainty_m:
                    ambiguous.update((a.key, b.key))
        return sorted((h for h in candidates if h.key not in ambiguous), key=lambda h: h.key)

    def begin_revisit(self, key, now_ns):
        if key not in {h.key for h in self.hints(now_ns)}:
            return False
        self.entries[key].attempts += 1
        return True

    def record_delivery(self, slot, class_name):
        # Adapter must call only on the original committed-slot acknowledgement.
        if type(slot) is not int or not 1 <= slot <= 3 or class_name not in self.weights:
            raise ValueError('invalid committed delivery')
        if slot in self.delivered:
            return self.delivered[slot] == class_name
        if slot != len(self.delivered)+1 or class_name in self.delivered.values():
            return False
        self.delivered[slot] = class_name
        return True


@dataclass(frozen=True)
class Edge:
    source: str
    target: str
    seconds: float
    stamp_ns: int
    epoch: Epoch
    traversable: bool


def label(key):
    return '%d:%d' % (key.target_id, key.first_seen_ns)


@dataclass(frozen=True)
class RouteProposal:
    keys: tuple
    classes: tuple
    weight_sum: float
    seconds: float
    permutations_checked: int
    role: str = 'OFFLINE_COST_PROPOSAL_NOT_PLANNER_ACCEPTANCE'


def rank_routes(hints, weights, edges, epoch, now_ns, slots, service_seconds,
                remaining_seconds, reserve_seconds, fixed_seconds=0., edge_max_age_ns=2*NS):
    """Enumerate at most 5P3 routes; no assumed edge for unknown/free space.

    service_seconds maps (key-label, next physical slot) to measured/declared
    approach+reacquisition+release+recovery cost. Edges are directed and include
    '@start' (verified descent completion point) and '@exit' (corridor entry).
    Fixed cost accounts for remaining survey/descent; reserve is the tail after
    corridor entry. The caller must avoid double counting these quantities.
    """
    if (type(slots) is not int or not 0 < slots <= 3 or not isinstance(epoch, Epoch)
            or not all(math.isfinite(v) and v >= 0 for v in
                       (remaining_seconds, reserve_seconds, fixed_seconds))):
        raise ValueError('invalid route budget')
    if any(not math.isfinite(v) or v <= 0 for v in weights.values()):
        raise ValueError('invalid weights')
    if len(hints) > 16 or len(weights) > 5 or len(edges) > 64:
        raise ValueError('bounded inputs exceeded')
    if len({h.key for h in hints}) != len(hints):
        raise ValueError('duplicate physical key')
    by_class = {}
    for h in hints:
        if h.epoch != epoch or not 0 <= now_ns-h.last_seen_ns <= 60*NS:
            continue
        by_class.setdefault(h.class_name, []).append(h)
    # Do not arbitrarily choose between spatially distinct same-class hints.
    unique = [v[0] for k, v in by_class.items() if len(v) == 1 and k in weights]
    edge_map = {}
    for e in edges:
        pair = e.source, e.target
        if pair in edge_map:
            raise ValueError('duplicate directed edge')
        edge_map[pair] = e
    candidates, checked = [], 0
    for route in permutations(unique, slots):
        checked += 1
        names = ['@start']+[label(h.key) for h in route]+['@exit']
        total = fixed_seconds+reserve_seconds
        valid = True
        for source, target in zip(names, names[1:]):
            e = edge_map.get((source, target))
            if (e is None or e.epoch != epoch or e.traversable is not True
                    or not math.isfinite(e.seconds) or e.seconds < 0
                    or not 0 <= now_ns-e.stamp_ns <= edge_max_age_ns):
                valid = False
                break
            total += e.seconds
        for index, h in enumerate(route, 4-slots):
            value = service_seconds.get((label(h.key), index))
            if value is None or not math.isfinite(value) or value < 0:
                valid = False
                break
            total += value
        if valid and total <= remaining_seconds:
            candidates.append((sum(weights[h.class_name] for h in route), total, route))
    if not candidates:
        return None
    weight, total, route = min(candidates, key=lambda p: (-p[0], p[1], tuple(h.key for h in p[2])))
    return RouteProposal(tuple(h.key for h in route), tuple(h.class_name for h in route),
                         weight, total, checked)


@dataclass(frozen=True)
class DescentEvidence:
    epoch: Epoch
    stamp_ns: int
    source: str
    swept_volume_verified: bool

    def valid(self, epoch, now_ns):
        return (self.epoch == epoch and self.swept_volume_verified is True
                and self.source in ('verified_transit', 'sensor_swept_volume')
                and 0 <= now_ns-self.stamp_ns <= 2*NS)


def survey_decision(elapsed_seconds, budget_seconds, proposal, desired_weight,
                    epoch, now_ns, descent=None, return_descent=None, p0_passed=False):
    """Pure proposal only. P0 defaults false; no flight action is constructed."""
    if (not all(math.isfinite(v) for v in (elapsed_seconds, budget_seconds, desired_weight))
            or elapsed_seconds < 0 or not 0 < budget_seconds <= 60 or desired_weight <= 0):
        raise ValueError('invalid survey budget')
    if not p0_passed:
        return 'OBSERVATION_ONLY_P0_PENDING'
    finish = elapsed_seconds >= budget_seconds or (proposal is not None and proposal.weight_sum >= desired_weight)
    if not finish:
        return 'CONTINUE_SURVEY'
    if descent is not None and descent.valid(epoch, now_ns):
        return 'PROPOSE_DESCENT_AND_REVISIT' if proposal else 'PROPOSE_DESCENT_AND_LOW_SEARCH'
    if return_descent is not None and return_descent.valid(epoch, now_ns):
        return 'PROPOSE_RETURN_VERIFIED_DESCENT'
    return 'NO_DESCENT_EVIDENCE_HOLD_FOR_EXISTING_SUPERVISOR'

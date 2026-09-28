"""Finite navigation hypotheses, distinct-image support and low-view resolution.

Hints are never MissionCore candidates or release evidence. Source timestamps
remain sensor timestamps even when a stronger hypothesis replaces an old one.
"""
from dataclasses import dataclass, field, replace
import math


@dataclass
class _Hypothesis:
    hint: object
    anchor: tuple
    samples: list = field(default_factory=list)
    pair: tuple = ()
    low_verified: bool = False
    support_count: int = 0  # Bounded independent observations; revisit ranking only.

    @property
    def level(self):
        return 3 if self.low_verified else (2 if self.hint.key.source != 'bbox' else 0)


class NavigationMemory:
    def __init__(self, classes, ttl_ns, merge_radius=.6, *,
                 coarse_min_interval_ns=100000000, coarse_max_gap_ns=1000000000,
                 coarse_consistency_m=.5):
        self.classes=set(classes);self.ttl_ns=ttl_ns;self.merge_radius=merge_radius
        self.min_interval_ns=coarse_min_interval_ns
        self.max_gap_ns=coarse_max_gap_ns
        self.consistency_m=coarse_consistency_m
        if not (0 < self.min_interval_ns <= self.max_gap_ns and
                0 < self.consistency_m <= self.merge_radius):
            raise ValueError('invalid navigation support window')
        self.epoch=None;self.last_now=None
        self.saved={};self.suspended=set();self.events=[];self.conflict_hints={}
        self._rows={};self._resolved=[];self._retired=[]

    def _event(self, reason, now_ns, **data):
        self.events.append(dict(reason=reason,time_ns=now_ns,**data))
        self.events=self.events[-64:]

    def _valid(self,h,now_ns):
        return (h.epoch==self.epoch and h.class_name in self.classes and
                0<=now_ns-h.last_seen_ns<=self.ttl_ns and
                len(h.xy)==2 and all(math.isfinite(v) for v in h.xy))

    def _near(self,a,b):
        return math.dist(a,b)<=self.merge_radius

    def _advance(self,epoch,now_ns):
        if epoch!=self.epoch or (self.last_now is not None and now_ns<self.last_now):
            self._rows.clear();self._resolved.clear();self._retired.clear()
            self._event('epoch_or_clock_reset',now_ns)
            self.epoch=epoch
        self.last_now=now_ns
        for cls,rows in list(self._rows.items()):
            kept=[r for r in rows if self._valid(r.hint,now_ns)]
            if len(kept)!=len(rows):self._event('hint_expired',now_ns,class_name=cls)
            if kept:self._rows[cls]=kept
            else:del self._rows[cls]
        self._resolved=[h for h in self._resolved if self._valid(h,now_ns)]
        self._retired=[r for r in self._retired if 0<=now_ns-r[2]<=self.ttl_ns]

    def _sample(self,row,h):
        # A re-published image, even with a new target ID, cannot add support.
        if row.samples and h.last_seen_ns<=row.samples[-1][0]:return
        sample=(h.last_seen_ns,tuple(h.xy))
        if row.pair and any(math.dist(xy,h.xy)>self.consistency_m for _,xy in row.pair):
            row.pair=()
        if row.samples:
            stamp,xy=row.samples[-1]
            gap=h.last_seen_ns-stamp
            if gap<self.min_interval_ns:return
            if gap<=self.max_gap_ns and math.dist(xy,h.xy)<=self.consistency_m:
                row.pair=(row.samples[-1],sample)
        if row.samples and math.dist(row.samples[-1][1],h.xy)<=self.consistency_m:
            row.support_count=min(255,row.support_count+1)
        else:
            row.support_count=1
        row.samples=(row.samples+[sample])[-2:]

    def _rank(self,row):
        # Source type outranks weak frame counts; recency breaks equal ranks.
        return (row.level,bool(row.pair),row.hint.evidence_count,row.hint.last_seen_ns)

    def _put(self,h,now_ns,low=False):
        # A low-view confirmed label rejects stale competing high-view labels
        # at this location, including catalog re-publications after resolution.
        if not low and any(r.class_name!=h.class_name and self._near(r.xy,h.xy)
                           for r in self._resolved):return
        retired=[r for r in self._retired if r[0]==h.class_name and self._near(r[1],h.xy)]
        if retired:
            # Re-publication or more weak boxes cannot undo a completed low check.
            # A new formally refined observation may restore this location.
            if h.key.source=='bbox' or h.last_seen_ns<=max(r[2] for r in retired):return
            self._retired=[r for r in self._retired if r not in retired]
        rows=self._rows.setdefault(h.class_name,[])
        near=[r for r in rows if self._near(r.hint.xy,h.xy) and self._near(r.anchor,h.xy)]
        row=min(near,key=lambda r:math.dist(r.hint.xy,h.xy)) if near else None
        level=3 if low else (2 if h.key.source!='bbox' else 0)
        if row is not None:
            if level<row.level:return
            if level==row.level and h.last_seen_ns<=row.hint.last_seen_ns:return
            if h.key.source=='bbox':self._sample(row,h)
            row.hint=h;row.low_verified=low or row.low_verified
        else:
            row=_Hypothesis(h,tuple(h.xy),low_verified=low)
            if h.key.source=='bbox':self._sample(row,h)
            rows.append(row)
        if row.hint.key.source=='bbox':
            row.hint=replace(row.hint,evidence_count=2 if row.pair else 1)
        if len(rows)>2:
            removed=min(rows,key=self._rank)
            rows.remove(removed)
            if removed is not row:
                self._event('weaker_location_replaced',now_ns,class_name=h.class_name,
                            old_xy=removed.hint.xy,new_xy=h.xy)

    def _refresh(self,now_ns):
        previous=set(self.suspended)
        self.saved={};self.suspended=set();self.conflict_hints={}
        # A stronger label at one location can suppress its weak competing
        # label, without discarding that class's other spatial hypothesis.
        rows=[r for values in self._rows.values() for r in values]
        eligible={}
        for cls,values in self._rows.items():
            active=[r for r in values if not any(
                other.hint.class_name!=cls and self._near(r.hint.xy,other.hint.xy)
                and other.level>r.level for other in rows)]
            if not active:continue
            strongest=max(r.level for r in active)
            finalists=[r for r in active if r.level==strongest]
            # Two coarse places remain ambiguous, however many frames repeat.
            best=max(finalists,key=self._rank)
            eligible[cls]=finalists
            self.saved[cls]=best.hint
            if len(finalists)>1:self.suspended.add(cls)
        for cls,values in eligible.items():
            for r in values:
                if any(other.hint.class_name!=cls and
                       (other.hint.key==r.hint.key or self._near(other.hint.xy,r.hint.xy))
                       and other.level==r.level
                       for others in eligible.values() for other in others):
                    self.suspended.add(cls)
        for cls in self.suspended:
            self.conflict_hints[cls]=[r.hint for r in eligible[cls]]
        for cls in self.suspended-previous:
            self._event('navigation_evidence_conflict',now_ns,class_name=cls,
                        locations=[h.xy for h in self.conflict_hints[cls]])
        for cls in previous-self.suspended:
            self._event('navigation_conflict_resolved',now_ns,class_name=cls)
        return {c:h for c,h in self.saved.items() if c not in self.suspended}

    def update(self,hints,epoch,now_ns):
        self._advance(epoch,now_ns)
        if epoch is not None:
            for h in hints:
                if self._valid(h,now_ns):self._put(h,now_ns)
        return self._refresh(now_ns)

    def retire_location(self,hint,now_ns):
        """Retire one unconfirmed location; this never rejects an entire class."""
        self._advance(self.epoch,now_ns)
        if not self._valid(hint,now_ns):return False
        cls=hint.class_name
        self._rows[cls]=[r for r in self._rows.get(cls,[]) if not self._near(r.hint.xy,hint.xy)]
        self._resolved=[h for h in self._resolved
                        if h.class_name!=cls or not self._near(h.xy,hint.xy)]
        self._retired=[r for r in self._retired if r[0]!=cls or not self._near(r[1],hint.xy)]
        self._retired.append((cls,tuple(hint.xy),now_ns))
        self._retired=self._retired[-2*len(self.classes):]
        self._event('unconfirmed_location_retired',now_ns,class_name=cls,xy=hint.xy)
        self._refresh(now_ns)
        return True

    def resolve_low(self,hint,now_ns):
        """Caller must supply a fresh formally validated low-view observation."""
        self._advance(self.epoch,now_ns)
        if not self._valid(hint,now_ns) or hint.key.source=='bbox' or hint.evidence_count<3:
            return False
        if any(r[0]==hint.class_name and self._near(r[1],hint.xy) and
               hint.last_seen_ns<=r[2] for r in self._retired):return False
        near=[h for h in self._resolved if self._near(h.xy,hint.xy)]
        if near and hint.last_seen_ns<=max(h.last_seen_ns for h in near):
            return any(h==hint for h in near)
        for cls,rows in self._rows.items():
            if cls!=hint.class_name:
                self._rows[cls]=[r for r in rows if not self._near(r.hint.xy,hint.xy)]
        self._resolved=[h for h in self._resolved if not self._near(h.xy,hint.xy)]
        self._resolved.append(hint)
        self._resolved=self._resolved[-2*len(self.classes):]
        self._put(hint,now_ns,low=True)
        self._event('low_view_location_resolved',now_ns,class_name=hint.class_name,xy=hint.xy)
        self._refresh(now_ns)
        return True

    def revisit_hints(self,now_ns):
        """Best supported place per class, even when early exit is suspended.

        Navigation only: competing labels/places stay in verification_hints.
        A conflicted single image cannot displace a supported location; repeated
        publication of one source image cannot increase its ranking.
        """
        visible=self.update((),self.epoch,now_ns)
        for cls,hints in self.conflict_hints.items():
            supported=[r for r in self._rows.get(cls,()) if r.hint in hints and
                       (r.hint.key.source!='bbox' or r.pair)]
            if supported:
                best=max(supported,key=lambda r:(r.level,r.support_count,
                                                 r.hint.evidence_count,r.hint.last_seen_ns))
                visible[cls]=best.hint
        return visible

    def verification_hints(self,now_ns):
        self.update((),self.epoch,now_ns)
        return {c:tuple(hs) for c,hs in self.conflict_hints.items()}

    def interrupt_hints(self,now_ns):
        visible=self.update((),self.epoch,now_ns)
        return {c:h for c,h in visible.items()
                if h.key.source!='bbox' or h.evidence_count>=2}

    def support_status(self):
        return {c:[dict(xy=r.hint.xy,source=r.hint.key.source,
                       source_stamp_ns=r.hint.last_seen_ns,
                       distinct_image_stamps=[t for t,_ in r.pair or r.samples],
                       low_verified=r.low_verified,
                       revisit_support_count=r.support_count) for r in rows]
                for c,rows in self._rows.items()}

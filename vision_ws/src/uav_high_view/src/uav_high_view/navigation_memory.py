"""Bounded persistent *navigation* hints; never fabricates fresh observations."""
import math


class NavigationMemory:
    def __init__(self, classes, ttl_ns, merge_radius=.6):
        self.classes=set(classes);self.ttl_ns=ttl_ns;self.merge_radius=merge_radius
        self.epoch=None;self.last_now=None;self.saved={};self.suspended=set();self.events=[]
        self.conflict_hints={}

    def _remember_conflict(self, hint):
        rows=self.conflict_hints.setdefault(hint.class_name,[])
        # Two spatial hypotheses per class, no live queues or unbounded history.
        if any(math.dist(hint.xy,old.xy)<=self.merge_radius for old in rows):return
        if len(rows)<2:rows.append(hint)

    def verification_hints(self, now_ns):
        """Ambiguous locations for low-view checking, never release permission."""
        return {c:tuple(h for h in rows if h.epoch==self.epoch and 0<=now_ns-h.last_seen_ns<=self.ttl_ns)
                for c,rows in self.conflict_hints.items() if c in self.suspended}

    def update(self, hints, epoch, now_ns):
        if epoch!=self.epoch or (self.last_now is not None and now_ns<self.last_now):
            self.saved.clear();self.suspended.clear()
            self.conflict_hints.clear()
            self.events.append(dict(reason='epoch_or_clock_reset',time_ns=now_ns))
            self.epoch=epoch
        self.last_now=now_ns
        if epoch is None:return {}
        for cls,h in list(self.saved.items()):
            if not 0<=now_ns-h.last_seen_ns<=self.ttl_ns:
                del self.saved[cls]
                self.events.append(dict(reason='hint_expired',class_name=cls,time_ns=now_ns))
        for h in hints:
            if (h.epoch!=epoch or h.class_name not in self.classes or
                    not 0<=now_ns-h.last_seen_ns<=self.ttl_ns):continue
            cls=h.class_name
            conflict=[c for c,old in self.saved.items() if
                      (old.key==h.key and c!=cls) or
                      (c==cls and math.dist(old.xy,h.xy)>self.merge_radius) or
                      (c!=cls and (old.key.source=="bbox" or h.key.source=="bbox") and
                       math.dist(old.xy,h.xy)<=self.merge_radius)]
            if conflict:
                self._remember_conflict(h)
                for c in conflict+[cls]:
                    if c in self.saved:self._remember_conflict(self.saved[c])
                    if c not in self.suspended:
                        self.events.append(dict(reason='confirmed_evidence_conflict',class_name=c,time_ns=now_ns,
                                                locations=[old.xy for old in self.conflict_hints.get(c,())]))
                    self.suspended.add(c)
                continue
            if cls in self.suspended:continue
            old=self.saved.get(cls)
            if old is not None:
                # Prefer refined geometry and never refresh with an older sample.
                if old.key.source!="bbox" and h.key.source=="bbox":continue
                if old.key.source==h.key.source and h.last_seen_ns<=old.last_seen_ns:continue
            if cls not in self.saved:
                self.events.append(dict(reason='coarse_navigation_hint_saved' if h.key.source=='bbox' else 'navigation_hint_confirmed',class_name=cls,time_ns=now_ns,xy=h.xy))
            self.saved[cls]=h
        self.events=self.events[-64:]
        return {c:h for c,h in self.saved.items() if c not in self.suspended}

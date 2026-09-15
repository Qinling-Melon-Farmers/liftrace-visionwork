"""Bounded persistent *navigation* hints; never fabricates fresh observations."""
import math


class NavigationMemory:
    def __init__(self, classes, ttl_ns, merge_radius=.6):
        self.classes=set(classes);self.ttl_ns=ttl_ns;self.merge_radius=merge_radius
        self.epoch=None;self.last_now=None;self.saved={};self.suspended=set();self.events=[]

    def update(self, hints, epoch, now_ns):
        if epoch!=self.epoch or (self.last_now is not None and now_ns<self.last_now):
            self.saved.clear();self.suspended.clear()
            self.events.append(dict(reason='epoch_or_clock_reset',time_ns=now_ns))
            self.epoch=epoch
        self.last_now=now_ns
        if epoch is None:return {}
        for cls,h in list(self.saved.items()):
            if not 0<=now_ns-h.last_seen_ns<=self.ttl_ns:
                del self.saved[cls]
                self.events.append(dict(reason='hint_expired',class_name=cls,time_ns=now_ns))
        for h in hints:
            if h.epoch!=epoch or h.class_name not in self.classes:continue
            cls=h.class_name
            conflict=[c for c,old in self.saved.items() if
                      (old.key==h.key and c!=cls) or
                      (c==cls and math.dist(old.xy,h.xy)>self.merge_radius)]
            if conflict:
                for c in conflict+[cls]:
                    if c not in self.suspended:
                        self.events.append(dict(reason='confirmed_evidence_conflict',class_name=c,time_ns=now_ns))
                    self.suspended.add(c)
                continue
            if cls in self.suspended:continue
            if cls not in self.saved:
                self.events.append(dict(reason='navigation_hint_confirmed',class_name=cls,time_ns=now_ns,xy=h.xy))
            self.saved[cls]=h
        self.events=self.events[-64:]
        return {c:h for c,h in self.saved.items() if c not in self.suspended}

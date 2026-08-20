"""辅助主动搜索的纯策略对象。

本模块不发布 ROS 控制话题，只管理评测外挂中的粗候选生命周期。公开消息暂不增加
跨组字段；来源、状态和交接结果先在 V-EXP-01 隔离原型及报告内验证。
"""

import math


SOURCE_AUX_CV = "AUX_CV"
SOURCE_AUX_YOLO = "AUX_YOLO"
SOURCE_DOWNWARD_YOLO = "DOWNWARD_YOLO"

STATUS_DETECTED = "DETECTED"
STATUS_APPROACHING = "APPROACHING"
STATUS_VERIFYING = "VERIFYING"
STATUS_CONFIRMED = "CONFIRMED"
STATUS_REJECTED = "REJECTED"

TERMINAL_STATUSES = {STATUS_CONFIRMED, STATUS_REJECTED}


class AuxCandidateRecord:
    """一个按地图邻近关系聚类的辅助粗候选。"""

    def __init__(self, candidate_id, source_id, x, y, confidence,
                 map_quality, stamp_sec, source, class_hint):
        self.id = int(candidate_id)
        self.source_id = int(source_id)
        self.x = float(x)
        self.y = float(y)
        self.weight = max(float(map_quality), 0.01)
        self.confidence = float(confidence)
        self.map_quality = float(map_quality)
        self.source = str(source)
        self.class_hint = str(class_hint)
        self.first_seen_sec = float(stamp_sec)
        self.last_seen_sec = float(stamp_sec)
        self.observations = 1
        self.status = STATUS_DETECTED
        self.approach_started_sec = None
        self.verify_started_sec = None
        self.terminal_sec = None
        self.reject_reason = ""
        self.confirmed_target_id = None
        self.confirmed_class = ""
        self.confirmed_distance_m = None

    def update(self, x, y, confidence, map_quality, stamp_sec, source_id):
        quality = max(float(map_quality), 0.01)
        total = self.weight + quality
        self.x = (self.x * self.weight + float(x) * quality) / total
        self.y = (self.y * self.weight + float(y) * quality) / total
        self.weight = total
        self.confidence = max(self.confidence, float(confidence))
        self.map_quality = max(self.map_quality, float(map_quality))
        self.last_seen_sec = max(self.last_seen_sec, float(stamp_sec))
        self.source_id = int(source_id)
        self.observations += 1

    def start_approach(self, stamp_sec):
        if self.status in TERMINAL_STATUSES:
            return False
        self.status = STATUS_APPROACHING
        self.approach_started_sec = float(stamp_sec)
        self.reject_reason = ""
        return True

    def start_verify(self, stamp_sec):
        if self.status != STATUS_APPROACHING:
            return False
        self.status = STATUS_VERIFYING
        self.verify_started_sec = float(stamp_sec)
        return True

    def confirm(self, stamp_sec, target_id, class_name, distance_m):
        if self.status != STATUS_VERIFYING:
            return False
        self.status = STATUS_CONFIRMED
        self.terminal_sec = float(stamp_sec)
        self.confirmed_target_id = int(target_id)
        self.confirmed_class = str(class_name)
        self.confirmed_distance_m = float(distance_m)
        return True

    def reject(self, stamp_sec, reason):
        if self.status in TERMINAL_STATUSES:
            return False
        self.status = STATUS_REJECTED
        self.terminal_sec = float(stamp_sec)
        self.reject_reason = str(reason)
        return True

    def to_dict(self):
        return {
            "id": self.id,
            "source_id": self.source_id,
            "source": self.source,
            "class_hint": self.class_hint,
            "x": self.x,
            "y": self.y,
            "confidence": self.confidence,
            "map_quality": self.map_quality,
            "first_seen_sec": self.first_seen_sec,
            "last_seen_sec": self.last_seen_sec,
            "observations": self.observations,
            "status": self.status,
            "approach_started_sec": self.approach_started_sec,
            "verify_started_sec": self.verify_started_sec,
            "terminal_sec": self.terminal_sec,
            "reject_reason": self.reject_reason,
            "confirmed_target_id": self.confirmed_target_id,
            "confirmed_class": self.confirmed_class,
            "confirmed_distance_m": self.confirmed_distance_m,
        }


class AuxCandidateBook:
    """维护稳定 ID、地图聚类和最近邻访问顺序。"""

    def __init__(self, match_distance_m):
        self._match_distance_m = float(match_distance_m)
        self._records = []
        self._next_id = 1

    @property
    def records(self):
        return tuple(self._records)

    def observe(self, source_id, x, y, confidence, map_quality, stamp_sec,
                source=SOURCE_AUX_CV, class_hint="circle"):
        match = None
        for record in self._records:
            distance = math.hypot(record.x - float(x), record.y - float(y))
            if distance <= self._match_distance_m:
                match = record
                break
        if match is None:
            match = AuxCandidateRecord(
                self._next_id, source_id, x, y, confidence, map_quality,
                stamp_sec, source, class_hint)
            self._next_id += 1
            self._records.append(match)
        else:
            match.update(x, y, confidence, map_quality, stamp_sec, source_id)
        return match

    def visit_order(self, start_x, start_y):
        remaining = [record for record in self._records
                     if record.status == STATUS_DETECTED]
        ordered = []
        x = float(start_x)
        y = float(start_y)
        while remaining:
            selected = min(
                remaining,
                key=lambda item: math.hypot(item.x - x, item.y - y))
            remaining.remove(selected)
            ordered.append(selected)
            x, y = selected.x, selected.y
        return ordered

    def state_counts(self):
        result = {
            STATUS_DETECTED: 0,
            STATUS_APPROACHING: 0,
            STATUS_VERIFYING: 0,
            STATUS_CONFIRMED: 0,
            STATUS_REJECTED: 0,
        }
        for record in self._records:
            result[record.status] += 1
        return result


def fresh_spatial_match(candidate, downward_targets, now_sec,
                        max_distance_m, max_age_sec, preverify_grace_sec=0.0):
    """返回当前候选附近、且在复核窗口内真正更新过的最近下视目标。"""
    if candidate is None or candidate.status != STATUS_VERIFYING or \
            candidate.verify_started_sec is None:
        return None
    matches = []
    for target in downward_targets:
        last_seen = float(target["last_seen_sec"])
        if last_seen < candidate.verify_started_sec - preverify_grace_sec:
            continue
        if float(now_sec) - last_seen > float(max_age_sec):
            continue
        distance = math.hypot(
            candidate.x - float(target["x"]),
            candidate.y - float(target["y"]))
        if distance <= float(max_distance_m):
            matches.append((distance, target))
    return min(matches, key=lambda item: item[0]) if matches else None


def handoff_gate_status(records, triggered, min_success_rate=0.80):
    """按已完成访问的交接率判子 Gate，不与返航/规划终态混为一谈。"""
    if not triggered:
        return "NOT_EXERCISED"
    statuses = [record.status for record in records]
    if STATUS_APPROACHING in statuses or STATUS_VERIFYING in statuses:
        return "PENDING"
    confirmed = statuses.count(STATUS_CONFIRMED)
    rejected = statuses.count(STATUS_REJECTED)
    terminal = confirmed + rejected
    if terminal == 0:
        return "PENDING"
    return "PASS" if confirmed / float(terminal) >= min_success_rate else "FAIL"

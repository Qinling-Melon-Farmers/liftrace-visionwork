#!/usr/bin/env python3
"""Pure policy helpers for coverage navigation and candidate scheduling."""

from dataclasses import dataclass
import math


RULE_WEIGHTS = {
    "tent": 1.0,
    "pillbox": 1.5,
    "bridge": 2.0,
    "panzer": 2.5,
    "tank": 5.0,
    # 随机投放区红十字：与标准靶共用同一候选队列，仅权重更高（面积小是唯一差异，
    # 不需要独立任务模式；内部 drop_cross 只是对准几何来源不同）。
    "red_cross": 10.0,
}

STANDARD_CLASSES = ("tent", "pillbox", "bridge", "panzer", "tank")
# Gate 的“发现完整”口径：五类标准靶 + （摆放时）红十字。
DISCOVERY_CLASSES = STANDARD_CLASSES + ("red_cross",)

# 类目 profile：任务队列允许调度的标准靶集合。
# - full：五类标准靶（含 tank），对应历史固定场（toudi4）口径，回归基线；
# - r2026：2026 规则书场地只设 4 个 1m 标准投放区（无 tank），随机红十字另计。
#   profile 只约束标准靶准入；red_cross 始终允许入队（发现与否由真值/搜索决定）。
CLASS_PROFILES = {
    "full": STANDARD_CLASSES,
    "r2026": ("tent", "pillbox", "bridge", "panzer"),
}


def profile_standard_classes(profile):
    """按名称取 profile 的标准靶集合；未知名称回退 full 保持历史口径。"""
    return CLASS_PROFILES.get(profile, CLASS_PROFILES["full"])


def profile_allowed_classes(profile):
    """任务队列允许调度的类目：profile 标准靶 + 随机投放区红十字。"""
    return tuple(profile_standard_classes(profile)) + ("red_cross",)


def accumulate_run_facts(discovered_by_class, discovered_ids,
                         selection_accum, payload):
    """Merge discovered/selection facts from a manager status payload.

    发现/选择属于全程事实，不随最终快照变化：任务成功后 align_mode 会切到
    landing，target_memory 按模式过滤会清空最终快照里的标准靶，因此 Gate 必须
    按每个状态快照累计，而不是只看最后一个。三个参数就地更新并返回。
    """
    for item in payload.get("discovered") or []:
        class_name = item.get("class")
        target_id = item.get("id")
        if class_name in DISCOVERY_CLASSES and target_id is not None:
            discovered_by_class[class_name] = int(target_id)
            discovered_ids.add(int(target_id))
    for item in payload.get("selection_sequence") or []:
        key = (item.get("id"), item.get("class"))
        if not any(existing[0] == key[0] and existing[1] == key[1]
                   for existing in selection_accum):
            selection_accum.append(key)
    return discovered_by_class, discovered_ids, selection_accum


@dataclass(frozen=True)
class CoveragePoint:
    index: int
    row: int
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class CandidateData:
    target_id: int
    class_name: str
    confidence: float
    first_seen: float
    last_seen: float
    state: int
    map_valid: bool
    map_frame: str
    association_valid: bool
    reject_reason: str
    x: float
    y: float


@dataclass(frozen=True)
class CaptureEvidence:
    class_name: str
    x: float
    y: float
    last_seen: float
    confidence: float


def expected_capture_class(candidate_class):
    """Return the near-field geometry required before entering ALIGN."""
    return "red_cross" if candidate_class == "red_cross" else "circle"


def capture_evidence_matches(candidate, evidence, now, max_age, radius):
    """Match fresh near-field evidence to the selected semantic candidate."""
    if candidate is None:
        return False
    expected_class = expected_capture_class(candidate.class_name)
    for observation in evidence:
        if observation.class_name != expected_class:
            continue
        if (observation.last_seen <= 0.0 or
                now - observation.last_seen > max_age):
            continue
        if math.hypot(observation.x - candidate.x,
                      observation.y - candidate.y) <= radius:
            return True
    return False


def generate_serpentine(min_x, max_x, min_y, max_y, safety_margin,
                        spacing, height):
    if min_x >= max_x or min_y >= max_y:
        raise ValueError("field bounds must have positive area")
    if safety_margin < 0.0 or spacing <= 0.0 or height <= 0.0:
        raise ValueError("margin, spacing and height must be valid")
    safe_min_x = min_x + safety_margin
    safe_max_x = max_x - safety_margin
    safe_min_y = min_y + safety_margin
    safe_max_y = max_y - safety_margin
    if safe_min_x >= safe_max_x or safe_min_y >= safe_max_y:
        raise ValueError("safety margin consumes field")

    rows = []
    y = safe_min_y
    while y <= safe_max_y + 1e-9:
        rows.append(min(y, safe_max_y))
        y += spacing
    if safe_max_y - rows[-1] > 1e-6:
        rows.append(safe_max_y)

    points = []
    for row, row_y in enumerate(rows):
        endpoints = ((safe_min_x, safe_max_x) if row % 2 == 0
                     else (safe_max_x, safe_min_x))
        for x in endpoints:
            points.append(CoveragePoint(
                index=len(points), row=row, x=x, y=row_y, z=height))
    return points


def select_serpentine_entry(points, start_x, start_y):
    """Choose the nearest of four equivalent full serpentine routes."""
    if not points:
        return []
    center_x = 0.5 * (min(point.x for point in points) +
                      max(point.x for point in points))
    mirrored = [CoveragePoint(
        index=point.index,
        row=point.row,
        x=2.0 * center_x - point.x,
        y=point.y,
        z=point.z) for point in points]
    variants = [points, mirrored, list(reversed(points)),
                list(reversed(mirrored))]
    selected = min(variants, key=lambda route: (
        math.hypot(route[0].x - start_x, route[0].y - start_y),
        variants.index(route)))
    return [CoveragePoint(
        index=index,
        row=index // 2,
        x=point.x,
        y=point.y,
        z=point.z) for index, point in enumerate(selected)]


def point_inside_safe_bounds(point, bounds, safety_margin):
    min_x, max_x, min_y, max_y = bounds
    return (
        min_x + safety_margin <= point[0] <= max_x - safety_margin and
        min_y + safety_margin <= point[1] <= max_y - safety_margin
    )


def _clear(point, occupied, clearance, vertical_tolerance):
    clearance_sq = clearance * clearance
    for obstacle in occupied:
        if abs(obstacle[2] - point[2]) > vertical_tolerance:
            continue
        dx = obstacle[0] - point[0]
        dy = obstacle[1] - point[1]
        if dx * dx + dy * dy < clearance_sq:
            return False
    return True


def resolve_safe_waypoint(point, occupied, bounds, safety_margin,
                          clearance=0.35, vertical_tolerance=0.5,
                          search_step=0.25, max_adjustment=1.0):
    """Return the nearest known-clear endpoint, or None if none is found."""
    point = tuple(float(value) for value in point)
    if (point_inside_safe_bounds(point, bounds, safety_margin) and
            _clear(point, occupied, clearance, vertical_tolerance)):
        return point

    candidates = []
    steps = int(math.ceil(max_adjustment / search_step))
    for dx_step in range(-steps, steps + 1):
        for dy_step in range(-steps, steps + 1):
            if dx_step == 0 and dy_step == 0:
                continue
            dx = dx_step * search_step
            dy = dy_step * search_step
            distance = math.hypot(dx, dy)
            if distance > max_adjustment + 1e-9:
                continue
            candidate = (point[0] + dx, point[1] + dy, point[2])
            candidates.append((distance, candidate))
    for _, candidate in sorted(candidates, key=lambda item: item[0]):
        if (point_inside_safe_bounds(candidate, bounds, safety_margin) and
                _clear(candidate, occupied, clearance, vertical_tolerance)):
            return candidate
    return None


def candidate_valid(candidate, now, mission_frame="camera_init",
                    max_age=0.5, allowed_classes=None):
    age = max(0.0, now - candidate.last_seen)
    return (
        candidate.class_name in RULE_WEIGHTS and
        (allowed_classes is None or
         candidate.class_name in allowed_classes) and
        candidate.state >= 2 and
        candidate.map_valid and
        candidate.map_frame == mission_frame and
        candidate.association_valid and
        not candidate.reject_reason and
        age <= max_age
    )


def candidate_rank(candidate):
    return (
        -RULE_WEIGHTS[candidate.class_name],
        -candidate.confidence,
        candidate.first_seen,
        candidate.target_id,
    )


def interrupt_eligible(pending, min_weight):
    """队列头部候选权重达到阈值时允许中断搜索先行投递。"""
    if not pending:
        return False
    top = pending[0]
    weight = RULE_WEIGHTS.get(top.class_name, 0.0)
    return weight >= float(min_weight)


def expected_delivery_classes(discovered_by_class, count=3):
    """按已发现候选的规则权重动态给出期望投递序列（权重降序，同权重按 ID）。"""
    ranked = sorted(
        discovered_by_class.items(),
        key=lambda item: (-RULE_WEIGHTS.get(item[0], 0.0), item[1]))
    return [class_name for class_name, _target_id in ranked[:count]]


class CandidateQueue:
    def __init__(self):
        self._pending = {}
        self._terminal_ids = set()

    @property
    def terminal_ids(self):
        return set(self._terminal_ids)

    @property
    def pending(self):
        return sorted(
            (candidate for target_id, candidate in self._pending.items()
             if target_id not in self._terminal_ids),
            key=candidate_rank)

    def update(self, candidates, now, mission_frame="camera_init",
               max_age=0.5, allowed_classes=None):
        for candidate in candidates:
            if candidate.target_id in self._terminal_ids:
                continue
            if candidate_valid(candidate, now, mission_frame, max_age,
                               allowed_classes):
                self._pending[candidate.target_id] = candidate

    def retain(self, target_ids):
        active_ids = {int(target_id) for target_id in target_ids}
        for target_id in list(self._pending):
            if target_id not in active_ids:
                self._pending.pop(target_id, None)

    def pop(self):
        available = self.pending
        if not available:
            return None
        selected = sorted(available, key=candidate_rank)[0]
        self._pending.pop(selected.target_id, None)
        return selected

    def mark_terminal(self, target_id):
        self._terminal_ids.add(int(target_id))
        self._pending.pop(int(target_id), None)


class GoalRetryPolicy:
    def __init__(self, retry_interval=5.0, unreachable_timeout=20.0,
                 max_retries=2):
        self.retry_interval = float(retry_interval)
        self.unreachable_timeout = float(unreachable_timeout)
        self.max_retries = int(max_retries)
        self.started_at = None
        self.last_publish_at = None
        self.retries = 0

    def start(self, now):
        self.started_at = float(now)
        self.last_publish_at = float(now)
        self.retries = 0

    def note_progress(self, now):
        if self.started_at is None:
            raise RuntimeError("goal policy has not started")
        self.started_at = float(now)
        self.last_publish_at = float(now)

    def decision(self, now):
        if self.started_at is None:
            raise RuntimeError("goal policy has not started")
        now = float(now)
        if now - self.started_at >= self.unreachable_timeout:
            return "timeout"
        if (self.retries < self.max_retries and
                now - self.last_publish_at >= self.retry_interval):
            self.retries += 1
            self.last_publish_at = now
            return "retry"
        return "wait"

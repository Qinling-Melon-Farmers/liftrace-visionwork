#!/usr/bin/env python3
"""target_memory: 图像域候选确认与状态管理。

订阅 /uav_vision/detections，跨帧匹配同类目标，管理候选生命周期：
- 连续 confirm_frames 帧同类同区域命中 → CONFIRMED
- 超 TTL 未观测 → EXPIRED
- 被拒绝目标冷却期内不再作为候选

发布：
- /uav_vision/targets        — 所有活跃候选 (TargetCandidateArray)
- /uav_vision/selected_target — 最高优先级已确认目标 (TargetCandidate)
"""
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
from uav_vision.msg import (TargetDetection, TargetDetectionArray,
                             TargetCandidate, TargetCandidateArray)
from sensor_msgs.msg import RegionOfInterest

# 候选状态
(ST_DETECTED, ST_OBSERVING, ST_CONFIRMED, ST_REJECTED, ST_EXPIRED) = range(5)
STATE_NAMES = ["DETECTED", "OBSERVING", "CONFIRMED", "REJECTED", "EXPIRED"]
VALID_ALIGN_MODES = {"disabled", "drop_circle", "drop_cross", "landing"}
STANDARD_CLASSES = {"bridge", "panzer", "pillbox", "tent", "tank"}


class CandidateRecord:
    """单个候选的内部记录。"""
    __slots__ = ('id', 'class_name', 'class_confidence', 'geometry_confidence',
                 'roi', 'center_px', 'state', 'observe_count',
                 'consecutive_observe_count', 'first_seen', 'last_seen',
                 'last_center', 'center_refined', 'center_source',
                 'association_valid', 'reject_reason', 'map_valid',
                 'map_point', 'map_frame', 'map_quality', 'map_weight',
                 'transform_age_sec', 'class_votes', 'class_max_confidence',
                 'pending_class', 'pending_class_count')

    def __init__(self, cid, det, now):
        self.id = cid
        self.class_name = det.class_name
        self.class_confidence = det.class_confidence
        self.geometry_confidence = det.geometry_confidence
        self.roi = det.roi
        self.center_px = det.center_px
        self.state = ST_DETECTED
        self.observe_count = 1
        self.consecutive_observe_count = 1
        self.first_seen = now
        self.last_seen = now
        self.last_center = (det.center_px.x, det.center_px.y)
        self.center_refined = det.center_refined
        self.center_source = det.center_source
        self.association_valid = det.association_valid
        self.reject_reason = det.reject_reason
        self.map_valid = det.map_valid
        self.map_point = Point(det.map_point.x, det.map_point.y, det.map_point.z)
        self.map_frame = det.map_frame
        self.map_quality = det.map_quality
        self.map_weight = max(float(det.map_quality), 0.01) if det.map_valid else 0.0
        self.transform_age_sec = det.transform_age_sec
        initial_vote = self._class_vote_weight(det)
        self.class_votes = {det.class_name: initial_vote}
        self.class_max_confidence = {det.class_name: det.class_confidence}
        self.pending_class = ""
        self.pending_class_count = 0

    @staticmethod
    def _class_vote_weight(det):
        return max(0.001, float(det.class_confidence) *
                   float(det.geometry_confidence))

    def _update_class(self, det, switch_frames, switch_min_confidence,
                      switch_vote_ratio):
        vote = self._class_vote_weight(det)
        self.class_votes[det.class_name] = \
            self.class_votes.get(det.class_name, 0.0) + vote
        self.class_max_confidence[det.class_name] = max(
            self.class_max_confidence.get(det.class_name, 0.0),
            float(det.class_confidence))

        if det.class_name == self.class_name:
            self.class_confidence = max(
                self.class_confidence, det.class_confidence)
            self.pending_class = ""
            self.pending_class_count = 0
            return

        if (det.class_name not in STANDARD_CLASSES or
                self.class_name not in STANDARD_CLASSES or
                det.class_confidence < switch_min_confidence):
            self.pending_class = ""
            self.pending_class_count = 0
            return

        if det.class_name == self.pending_class:
            self.pending_class_count += 1
        else:
            self.pending_class = det.class_name
            self.pending_class_count = 1

        candidate_vote = self.class_votes.get(det.class_name, 0.0)
        current_vote = self.class_votes.get(self.class_name, 0.0)
        if (self.pending_class_count >= switch_frames and
                candidate_vote >= current_vote * switch_vote_ratio):
            self.class_name = det.class_name
            self.class_confidence = self.class_max_confidence[det.class_name]
            self.pending_class = ""
            self.pending_class_count = 0

    def _update_map(self, det):
        if not det.map_valid:
            return
        new_weight = max(float(det.map_quality), 0.01)
        if not self.map_valid or self.map_frame != det.map_frame:
            self.map_valid = True
            self.map_point = Point(
                det.map_point.x, det.map_point.y, det.map_point.z)
            self.map_frame = det.map_frame
            self.map_quality = det.map_quality
            self.map_weight = new_weight
            return
        total_weight = self.map_weight + new_weight
        self.map_point = Point(
            (self.map_point.x * self.map_weight +
             det.map_point.x * new_weight) / total_weight,
            (self.map_point.y * self.map_weight +
             det.map_point.y * new_weight) / total_weight,
            (self.map_point.z * self.map_weight +
             det.map_point.z * new_weight) / total_weight,
        )
        self.map_quality = (
            self.map_quality * self.map_weight +
            float(det.map_quality) * new_weight) / total_weight
        self.map_weight = total_weight

    def update(self, det, now, confirm_frames, switch_frames,
               switch_min_confidence, switch_vote_ratio):
        self._update_class(det, switch_frames, switch_min_confidence,
                           switch_vote_ratio)
        self.geometry_confidence = max(self.geometry_confidence, det.geometry_confidence)
        self.roi = det.roi
        self.center_px = det.center_px
        self.last_center = (det.center_px.x, det.center_px.y)
        self.last_seen = now
        self.observe_count += 1
        self.consecutive_observe_count += 1
        self.center_refined = det.center_refined
        self.center_source = det.center_source
        self.association_valid = det.association_valid
        self.reject_reason = det.reject_reason
        self.transform_age_sec = det.transform_age_sec
        self._update_map(det)
        self._advance_state(confirm_frames)

    def merge_from(self, other):
        """Merge a converged duplicate without inventing extra hit streaks."""
        if self.map_valid and other.map_valid:
            total_weight = self.map_weight + other.map_weight
            if total_weight > 0.0:
                self.map_point = Point(
                    (self.map_point.x * self.map_weight +
                     other.map_point.x * other.map_weight) / total_weight,
                    (self.map_point.y * self.map_weight +
                     other.map_point.y * other.map_weight) / total_weight,
                    (self.map_point.z * self.map_weight +
                     other.map_point.z * other.map_weight) / total_weight)
                self.map_quality = (
                    self.map_quality * self.map_weight +
                    other.map_quality * other.map_weight) / total_weight
                self.map_weight = total_weight

        for class_name, vote in other.class_votes.items():
            self.class_votes[class_name] = \
                self.class_votes.get(class_name, 0.0) + vote
        for class_name, confidence in other.class_max_confidence.items():
            self.class_max_confidence[class_name] = max(
                self.class_max_confidence.get(class_name, 0.0), confidence)
        standard_votes = {
            name: vote for name, vote in self.class_votes.items()
            if name in STANDARD_CLASSES}
        if standard_votes:
            self.class_name = max(
                standard_votes, key=lambda name: (
                    standard_votes[name],
                    self.class_max_confidence.get(name, 0.0)))
            self.class_confidence = self.class_max_confidence[
                self.class_name]

        if other.last_seen.to_sec() > self.last_seen.to_sec():
            self.roi = other.roi
            self.center_px = other.center_px
            self.last_center = other.last_center
            self.center_refined = other.center_refined
            self.center_source = other.center_source
            self.association_valid = other.association_valid
            self.reject_reason = other.reject_reason
            self.transform_age_sec = other.transform_age_sec
        self.first_seen = min(self.first_seen, other.first_seen)
        self.last_seen = max(self.last_seen, other.last_seen)
        self.observe_count = max(self.observe_count, other.observe_count)
        self.consecutive_observe_count = max(
            self.consecutive_observe_count,
            other.consecutive_observe_count)
        self.geometry_confidence = max(
            self.geometry_confidence, other.geometry_confidence)
        self.state = max(self.state, other.state)

    def _advance_state(self, confirm_frames):
        if self.state == ST_REJECTED or self.state == ST_EXPIRED:
            return
        if self.consecutive_observe_count >= confirm_frames:
            self.state = ST_CONFIRMED
        elif self.consecutive_observe_count >= max(confirm_frames - 1, 1):
            self.state = ST_OBSERVING

    def mark_missed(self):
        self.consecutive_observe_count = 0
        if self.state != ST_CONFIRMED:
            self.state = ST_DETECTED

    def age(self, now, ttl):
        """返回是否已过期。"""
        if self.state == ST_REJECTED or self.state == ST_EXPIRED:
            return True
        if (now - self.last_seen).to_sec() > ttl:
            self.state = ST_EXPIRED
            return True
        return False

    def to_msg(self, now):
        msg = TargetCandidate()
        msg.header.stamp = now
        msg.id = self.id
        msg.class_name = self.class_name
        msg.class_confidence = self.class_confidence
        msg.geometry_confidence = self.geometry_confidence
        msg.roi = self.roi
        msg.center_px = self.center_px
        msg.center_refined = self.center_refined
        msg.center_source = self.center_source
        msg.association_valid = self.association_valid
        msg.reject_reason = self.reject_reason
        msg.map_valid = self.map_valid
        msg.map_point = self.map_point
        msg.map_frame = self.map_frame
        msg.map_quality = self.map_quality
        msg.transform_age_sec = self.transform_age_sec
        msg.state = self.state
        msg.observe_count = self.observe_count
        msg.consecutive_observe_count = self.consecutive_observe_count
        msg.first_seen = self.first_seen
        msg.last_seen = self.last_seen
        return msg


class TargetMemory:
    def __init__(self):
        rospy.init_node("target_memory")
        self._detections_topic = rospy.get_param("~detections_topic", "/uav_vision/detections")

        # 发布
        self._targets_pub = rospy.Publisher("/uav_vision/targets",
                                            TargetCandidateArray, queue_size=1)
        self._selected_pub = rospy.Publisher("/uav_vision/selected_target",
                                             TargetCandidate, queue_size=1)

        # ---- 匹配参数 ----
        self._confirm_frames = rospy.get_param("~confirm_frames", 3)
        self._candidate_ttl = rospy.get_param("~candidate_ttl", 3.0)
        self._reject_cooldown = rospy.get_param("~reject_cooldown", 5.0)
        self._match_distance_px = rospy.get_param("~match_distance_px", 80.0)
        self._map_match_distance_m = rospy.get_param("~map_match_distance_m", 0.5)
        self._map_merge_distance_m = rospy.get_param(
            "~map_merge_distance_m", 0.6)
        self._map_memory_ttl = rospy.get_param("~map_memory_ttl", 0.0)
        self._require_map_for_candidates = bool(
            rospy.get_param("~require_map_for_candidates", False))
        self._selected_max_age = float(rospy.get_param("~selected_max_age", 0.5))
        self._class_switch_confirm_frames = max(
            1, int(rospy.get_param("~class_switch_confirm_frames", 2)))
        self._class_switch_min_confidence = float(
            rospy.get_param("~class_switch_min_confidence", 0.70))
        self._class_switch_vote_ratio = float(
            rospy.get_param("~class_switch_vote_ratio", 1.0))
        self._reset_service_name = rospy.get_param(
            "~reset_service", "/uav_vision/reset_memory")
        self._align_mode_topic = rospy.get_param(
            "~align_mode_topic", "/uav_vision/align_mode")
        self._align_mode = "disabled"

        # ---- 优先级权重 ----
        self._priority = {
            "red_cross": rospy.get_param("~priority_red_cross", 10.0),
            "panzer":    rospy.get_param("~priority_panzer", 2.5),
            "pillbox":   rospy.get_param("~priority_pillbox", 1.5),
            "tent":      rospy.get_param("~priority_tent", 1.0),
            "tank":      rospy.get_param("~priority_tank", 5.0),
            "bridge":    rospy.get_param("~priority_bridge", 0.0),
            "landing_pad": rospy.get_param("~priority_landing_pad", 0.0),
            "circle":    rospy.get_param("~priority_circle", 0.0),
        }

        # ---- 视觉中断阈值 ----
        self._cross_conf = rospy.get_param("~cross_class_confidence", 0.70)
        self._cross_geom = rospy.get_param("~cross_geometry_confidence", 0.85)
        self._std_conf = rospy.get_param("~std_class_confidence", 0.60)
        self._std_geom = rospy.get_param("~std_geometry_confidence", 0.70)
        self._aux_geom = rospy.get_param("~aux_geometry_confidence", 0.85)
        self._suppress_bridge_on_red_cross = rospy.get_param("~suppress_bridge_on_red_cross", True)
        self._suppress_bridge_on_landing_pad = rospy.get_param("~suppress_bridge_on_landing_pad", True)

        # ---- 内部状态 ----
        self._candidates = {}        # id → CandidateRecord
        self._next_id = 0
        self._rejected = {}          # (class_name, roi_hash) → reject_time

        # 订阅
        rospy.Subscriber(self._detections_topic, TargetDetectionArray,
                         self._on_detections, queue_size=2)
        rospy.Subscriber(self._align_mode_topic, String,
                         self._on_align_mode, queue_size=2)
        self._reset_service = rospy.Service(self._reset_service_name,
                                            Empty, self._on_reset)

        self._publish_empty()

        rospy.loginfo("[TargetMemory] ready  detections_topic=%s  confirm=%d  ttl=%.1fs  cooldown=%.1fs  "
                      "match_dist=%.0fpx",
                      self._detections_topic,
                      self._confirm_frames, self._candidate_ttl, self._reject_cooldown,
                      self._match_distance_px)
        rospy.loginfo("  map_match=%.2fm  map_memory_ttl=%.1fs (0=until reset)",
                      self._map_match_distance_m, self._map_memory_ttl)
        rospy.loginfo("  require_map_for_candidates=%s",
                      self._require_map_for_candidates)
        rospy.loginfo("  class_switch=%d consecutive frames min_conf=%.2f vote_ratio=%.2f",
                      self._class_switch_confirm_frames,
                      self._class_switch_min_confidence,
                      self._class_switch_vote_ratio)
        rospy.loginfo("  suppress_bridge_on_red_cross=%s  suppress_bridge_on_landing_pad=%s  aux_geom=%.2f",
                      self._suppress_bridge_on_red_cross,
                      self._suppress_bridge_on_landing_pad,
                      self._aux_geom)
        for cls, pri in self._priority.items():
            rospy.loginfo("  priority[%s] = %.1f", cls, pri)

    # ------------------------------------------------------------------
    def _on_align_mode(self, message):
        mode = message.data.strip()
        new_mode = mode if mode in VALID_ALIGN_MODES else "disabled"
        if new_mode != self._align_mode:
            self._align_mode = new_mode
            self._publish(rospy.Time.now())

    def _allowed_in_current_mode(self, class_name):
        if self._align_mode == "landing":
            return class_name == "landing_pad"
        if self._align_mode == "drop_cross":
            return class_name == "red_cross"
        if self._align_mode == "drop_circle":
            return class_name == "circle" or class_name in STANDARD_CLASSES
        return class_name != "landing_pad"

    # ------------------------------------------------------------------
    def _on_detections(self, msg):
        now = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()
        frame_has_red_cross = any(
            det.class_name == "red_cross" and
            det.geometry_verified and
            det.geometry_confidence >= self._cross_geom
            for det in msg.detections
        )
        frame_has_landing_pad = any(
            det.class_name == "landing_pad" and
            det.geometry_verified and
            det.geometry_confidence >= self._aux_geom
            for det in msg.detections
        )

        matched_ids = set()
        ordered_detections = sorted(
            msg.detections,
            key=lambda detection: (
                float(detection.class_confidence) *
                float(detection.geometry_confidence)),
            reverse=True)
        for det in ordered_detections:
            if not self._allowed_in_current_mode(det.class_name):
                continue
            if det.class_name == "bridge":
                if frame_has_red_cross and self._suppress_bridge_on_red_cross:
                    continue
                if frame_has_landing_pad and self._suppress_bridge_on_landing_pad:
                    continue
            if not self._pass_threshold(det):
                continue
            if self._is_rejected(det, now):
                continue

            cid = self._match_or_create(det, now, matched_ids)
            if cid is not None:
                matched_ids.add(cid)

        self._merge_spatial_duplicates(matched_ids)

        # 老化未匹配的候选
        stale = []
        for cid, cand in self._candidates.items():
            if cid not in matched_ids:
                cand.mark_missed()
                was_confirmed = (cand.state == ST_CONFIRMED)
                ttl = self._map_memory_ttl if cand.map_valid else self._candidate_ttl
                if ttl > 0.0 and cand.age(now, ttl):
                    stale.append(cid)
                    if was_confirmed:
                        # 确认后丢失 → 加入拒绝冷却
                        self._add_rejected(cand, now)
        for cid in stale:
            del self._candidates[cid]

        self._cleanup_rejects(now)
        self._publish(now)

    def _merge_spatial_duplicates(self, matched_ids):
        """Collapse mapped standard records that converge to one location."""
        changed = True
        while changed:
            changed = False
            ids = sorted(self._candidates)
            for left_index, left_id in enumerate(ids):
                if left_id not in self._candidates:
                    continue
                left = self._candidates[left_id]
                if (left.class_name not in STANDARD_CLASSES or
                        not left.map_valid):
                    continue
                for right_id in ids[left_index + 1:]:
                    if right_id not in self._candidates:
                        continue
                    right = self._candidates[right_id]
                    if (right.class_name not in STANDARD_CLASSES or
                            not right.map_valid or
                            right.map_frame != left.map_frame):
                        continue
                    distance = (
                        (left.map_point.x - right.map_point.x) ** 2 +
                        (left.map_point.y - right.map_point.y) ** 2 +
                        (left.map_point.z - right.map_point.z) ** 2) ** 0.5
                    if distance > self._map_merge_distance_m:
                        continue
                    left.merge_from(right)
                    if right_id in matched_ids:
                        matched_ids.add(left_id)
                    matched_ids.discard(right_id)
                    del self._candidates[right_id]
                    rospy.loginfo(
                        "[TargetMemory] merged duplicate id=%u into id=%u distance=%.3f",
                        right_id, left_id, distance)
                    changed = True
                    break
                if changed:
                    break

    # ------------------------------------------------------------------
    def _pass_threshold(self, det):
        """检测是否满足对应类别的置信度阈值。"""
        if self._require_map_for_candidates and not det.map_valid:
            return False
        if det.class_name == "red_cross":
            return (det.geometry_verified and det.center_refined and
                    det.class_confidence >= self._cross_conf and
                    det.geometry_confidence >= self._cross_geom)
        if det.class_name in ("landing_pad", "circle"):
            return (det.geometry_verified and det.center_refined and
                    det.geometry_confidence >= self._aux_geom)
        # 标准投放区必须同时有类别和蓝环关联；未关联框仅保留在原始
        # detections 供调试，不能升级成可操作候选。
        return (det.geometry_verified and det.center_refined and
                det.class_confidence >= self._std_conf and
                det.geometry_confidence >= self._std_geom)

    def _is_rejected(self, det, now):
        """检查是否在拒绝冷却期。"""
        key = (det.class_name, self._roi_hash(det.roi))
        if key in self._rejected:
            if (now - self._rejected[key]).to_sec() < self._reject_cooldown:
                return True
        return False

    @staticmethod
    def _same_map_identity_group(left_class, right_class):
        if left_class in STANDARD_CLASSES and right_class in STANDARD_CLASSES:
            return True
        return left_class == right_class

    def _update_candidate(self, candidate, det, now):
        candidate.update(
            det, now, self._confirm_frames,
            self._class_switch_confirm_frames,
            self._class_switch_min_confidence,
            self._class_switch_vote_ratio)

    def _match_or_create(self, det, now, matched_ids):
        """优先用地图距离跨视角匹配，无地图时退回像素近邻。"""
        if det.map_valid:
            best_id, best_dist = None, self._map_match_distance_m
            for cid, cand in self._candidates.items():
                if (not self._same_map_identity_group(
                        cand.class_name, det.class_name) or
                        not cand.map_valid or
                        cand.map_frame != det.map_frame):
                    continue
                if cand.state in (ST_REJECTED, ST_EXPIRED):
                    continue
                dx = cand.map_point.x - det.map_point.x
                dy = cand.map_point.y - det.map_point.y
                dz = cand.map_point.z - det.map_point.z
                distance = (dx * dx + dy * dy + dz * dz) ** 0.5
                if distance < best_dist:
                    best_dist = distance
                    best_id = cid
            if best_id is not None:
                # 同一帧中，同一地图目标的多个分类框只计为一次观测，绝不能冒充多帧命中。
                if best_id not in matched_ids:
                    self._update_candidate(self._candidates[best_id], det, now)
                return best_id

        cx, cy = det.center_px.x, det.center_px.y
        best_id, best_dist = None, self._match_distance_px

        for cid, cand in self._candidates.items():
            if cand.class_name != det.class_name:
                continue
            # 两次观测都有地图坐标后，地图匹配失败就表示它们是不同物理目标。不能仅因相机
            # 再次经过该区域、像素位置接近，就错误地把它们合并。
            if det.map_valid and cand.map_valid:
                continue
            if cand.state in (ST_REJECTED, ST_EXPIRED):
                continue
            lx, ly = cand.last_center
            d = ((cx - lx) ** 2 + (cy - ly) ** 2) ** 0.5
            if d < best_dist:
                best_dist = d
                best_id = cid

        if best_id is not None:
            if best_id not in matched_ids:
                self._update_candidate(self._candidates[best_id], det, now)
            return best_id

        # 新建候选
        cid = self._next_id
        self._next_id += 1
        self._candidates[cid] = CandidateRecord(cid, det, now)
        self._candidates[cid]._advance_state(self._confirm_frames)
        return cid

    def _on_reset(self, _request):
        self._candidates.clear()
        self._rejected.clear()
        self._next_id = 0
        self._publish_empty()
        rospy.loginfo("[TargetMemory] memory reset")
        return EmptyResponse()

    def _add_rejected(self, cand, now):
        key = (cand.class_name, self._roi_hash(cand.roi))
        self._rejected[key] = now

    def _cleanup_rejects(self, now):
        expired = [k for k, t in self._rejected.items()
                   if (now - t).to_sec() > self._reject_cooldown]
        for k in expired:
            del self._rejected[k]

    def _publish(self, now):
        # 所有活跃候选
        arr = TargetCandidateArray()
        arr.header.stamp = now
        for cand in self._candidates.values():
            if cand.state not in (ST_REJECTED, ST_EXPIRED) and \
                    self._allowed_in_current_mode(cand.class_name):
                arr.targets.append(cand.to_msg(now))

        # 按优先级排序
        arr.targets.sort(key=lambda t: self._priority.get(t.class_name, 0),
                         reverse=True)
        self._targets_pub.publish(arr)

        # 选最优已确认目标（跳过 priority <= 0 的类别）
        best = None
        for t in arr.targets:
            observation_age = max(0.0, (now - t.last_seen).to_sec())
            if (t.state >= ST_CONFIRMED and
                    observation_age <= self._selected_max_age and
                    self._priority.get(t.class_name, 0) > 0):
                if best is None or self._priority.get(t.class_name, 0) > \
                   self._priority.get(best.class_name, 0):
                    best = t
        if best is not None:
            self._selected_pub.publish(best)

    def _publish_empty(self):
        arr = TargetCandidateArray()
        arr.header.stamp = rospy.Time.now()
        self._targets_pub.publish(arr)

    @staticmethod
    def _roi_hash(roi):
        return (roi.x_offset // 40, roi.y_offset // 40, roi.width, roi.height)

    # ------------------------------------------------------------------
    # 调试接口
    # ------------------------------------------------------------------
    def debug_dump(self):
        """返回当前候选表的可读摘要。"""
        lines = []
        for cid, cand in sorted(self._candidates.items()):
            lines.append(
                f"  [{cid}] {cand.class_name}  state={STATE_NAMES[cand.state]}  "
                f"obs={cand.observe_count} streak={cand.consecutive_observe_count}  "
                f"conf={cand.class_confidence:.2f}  "
                f"geom={cand.geometry_confidence:.2f}  "
                f"age={(rospy.Time.now() - cand.last_seen).to_sec():.1f}s"
            )
        return "\n".join(lines) if lines else "  (empty)"


def main():
    node = TargetMemory()
    rospy.spin()


if __name__ == "__main__":
    main()

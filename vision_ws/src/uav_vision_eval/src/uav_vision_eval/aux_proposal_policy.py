"""辅助候选 Provider 的纯校验与不确定度估算逻辑。"""

import math


SOURCE_AUX_CV = "AUX_CV"
SOURCE_AUX_YOLO = "AUX_YOLO"


def validate_aux_proposal(*, class_hint, confidence, map_valid, map_frame,
                          map_quality, state, reject_reason, x, y,
                          source_stamp_sec, now_sec, accepted_classes,
                          expected_frame, min_confidence, min_map_quality,
                          min_state, max_age_sec, max_future_skew_sec):
    """返回候选是否可供任务层消费及事实准确的拒绝原因。"""
    if not map_valid:
        return False, "map_invalid"
    if expected_frame and map_frame != expected_frame:
        return False, "map_frame_mismatch"
    if reject_reason:
        return False, "source_rejected:%s" % reject_reason
    if int(state) < int(min_state):
        return False, "candidate_not_confirmed"
    if accepted_classes and class_hint not in set(accepted_classes):
        return False, "class_not_accepted"
    if float(confidence) < float(min_confidence):
        return False, "confidence_below_threshold"
    if float(map_quality) < float(min_map_quality):
        return False, "map_quality_below_threshold"
    if not math.isfinite(float(x)) or not math.isfinite(float(y)):
        return False, "map_point_non_finite"
    if float(source_stamp_sec) <= 0.0:
        return False, "source_stamp_missing"
    if float(source_stamp_sec) - float(now_sec) > float(max_future_skew_sec):
        return False, "source_stamp_in_future"
    if (float(max_age_sec) > 0.0 and
            float(now_sec) - float(source_stamp_sec) > float(max_age_sec)):
        return False, "source_candidate_stale"
    return True, ""


def map_quality_uncertainty(map_quality, floor_m, scale_m):
    """在真实投影协方差接入前给出保守、可替换的质量代理。"""
    quality = min(1.0, max(0.0, float(map_quality)))
    return max(0.0, float(floor_m)) + max(0.0, float(scale_m)) * (1.0 - quality)

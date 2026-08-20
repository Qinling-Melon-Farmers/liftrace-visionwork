#!/usr/bin/env python3
"""把 OpenCV/YOLO 辅助候选归一化为单一、零控制输出的 proposal 流。"""

import json

import rospy
from std_msgs.msg import String

from uav_vision.msg import TargetCandidateArray
from uav_vision_eval.aux_proposal_policy import (
    SOURCE_AUX_CV,
    SOURCE_AUX_YOLO,
    map_quality_uncertainty,
    validate_aux_proposal,
)
from uav_vision_eval.msg import AuxProposal, AuxProposalArray


class AuxProposalProvider:
    def __init__(self):
        rospy.init_node("aux_proposal_provider")
        self._output_topic = rospy.get_param(
            "~output_topic", "/uav_vision/aux/proposals")
        self._status_topic = rospy.get_param(
            "~status_topic", "/uav_vision/aux/proposal_status")
        self._expected_frame = rospy.get_param("~expected_frame", "camera_init")
        self._min_state = int(rospy.get_param("~min_state", 2))
        self._max_age = float(rospy.get_param("~max_age_sec", 1.0))
        self._max_future_skew = float(
            rospy.get_param("~max_future_skew_sec", 0.10))
        self._publisher = rospy.Publisher(
            self._output_topic, AuxProposalArray, queue_size=1)
        self._status_publisher = rospy.Publisher(
            self._status_topic, String, queue_size=1, latch=True)
        self._source_stats = {}

        self._configure_source(
            "cv", SOURCE_AUX_CV, ["circle"], 0.35, 0.10, 0.35, 0.65)
        self._configure_source(
            "yolo", SOURCE_AUX_YOLO,
            ["bridge", "panzer", "pillbox", "tent", "tank", "red_cross"],
            0.45, 0.10, 0.45, 0.75)
        self._publish_status()

    def _configure_source(self, key, source, default_classes,
                          default_confidence, default_quality,
                          default_uncertainty_floor,
                          default_uncertainty_scale):
        enabled = bool(rospy.get_param("~%s/enabled" % key, key == "cv"))
        config = {
            "key": key,
            "source": source,
            "accepted_classes": set(rospy.get_param(
                "~%s/accepted_classes" % key, default_classes)),
            "min_confidence": float(rospy.get_param(
                "~%s/min_confidence" % key, default_confidence)),
            "min_map_quality": float(rospy.get_param(
                "~%s/min_map_quality" % key, default_quality)),
            "uncertainty_floor_m": float(rospy.get_param(
                "~%s/uncertainty_floor_m" % key,
                default_uncertainty_floor)),
            "uncertainty_scale_m": float(rospy.get_param(
                "~%s/uncertainty_scale_m" % key,
                default_uncertainty_scale)),
        }
        self._source_stats[source] = {
            "messages": 0, "valid": 0, "rejected": 0,
            "last_reject_reasons": {},
        }
        if not enabled:
            return
        topic = rospy.get_param(
            "~%s/input_topic" % key,
            "/uav_vision/aux/%s_targets" % key)
        rospy.Subscriber(
            topic, TargetCandidateArray, self._on_candidates,
            callback_args=config, queue_size=1)
        rospy.loginfo(
            "[AuxProposalProvider] source=%s input=%s output=%s",
            source, topic, self._output_topic)

    @staticmethod
    def _source_stamp(target, message):
        stamp = target.last_seen
        if stamp.to_sec() <= 0.0:
            stamp = message.header.stamp
        return stamp

    def _to_proposal(self, target, message, config, now):
        stamp = self._source_stamp(target, message)
        valid, reason = validate_aux_proposal(
            class_hint=target.class_name,
            confidence=target.class_confidence,
            map_valid=target.map_valid,
            map_frame=target.map_frame,
            map_quality=target.map_quality,
            state=target.state,
            reject_reason=target.reject_reason,
            x=target.map_point.x,
            y=target.map_point.y,
            source_stamp_sec=stamp.to_sec(),
            now_sec=now.to_sec(),
            accepted_classes=config["accepted_classes"],
            expected_frame=self._expected_frame,
            min_confidence=config["min_confidence"],
            min_map_quality=config["min_map_quality"],
            min_state=self._min_state,
            max_age_sec=self._max_age,
            max_future_skew_sec=self._max_future_skew)
        proposal = AuxProposal()
        proposal.header = target.header
        proposal.source_id = target.id
        proposal.source = config["source"]
        proposal.class_hint = target.class_name
        proposal.confidence = target.class_confidence
        proposal.map_point = target.map_point
        proposal.map_frame = target.map_frame
        proposal.map_quality = target.map_quality
        proposal.position_uncertainty_m = map_quality_uncertainty(
            target.map_quality,
            config["uncertainty_floor_m"],
            config["uncertainty_scale_m"])
        proposal.uncertainty_source = "map_quality_proxy"
        proposal.transform_age_sec = target.transform_age_sec
        proposal.state = target.state
        proposal.observe_count = target.observe_count
        proposal.first_seen = target.first_seen
        proposal.last_seen = stamp
        proposal.valid = valid
        proposal.reject_reason = reason
        return proposal

    def _on_candidates(self, message, config):
        now = rospy.Time.now()
        output = AuxProposalArray()
        output.header = message.header
        if output.header.stamp.to_sec() <= 0.0:
            output.header.stamp = now
        stats = self._source_stats[config["source"]]
        stats["messages"] += 1
        for target in message.targets:
            proposal = self._to_proposal(target, message, config, now)
            output.proposals.append(proposal)
            if proposal.valid:
                stats["valid"] += 1
            else:
                stats["rejected"] += 1
                reasons = stats["last_reject_reasons"]
                reasons[proposal.reject_reason] = (
                    reasons.get(proposal.reject_reason, 0) + 1)
        self._publisher.publish(output)
        self._publish_status()

    def _publish_status(self):
        payload = {
            "output_topic": self._output_topic,
            "expected_frame": self._expected_frame,
            "sources": self._source_stats,
        }
        self._status_publisher.publish(String(
            data=json.dumps(payload, ensure_ascii=False, sort_keys=True)))


if __name__ == "__main__":
    AuxProposalProvider()
    rospy.spin()

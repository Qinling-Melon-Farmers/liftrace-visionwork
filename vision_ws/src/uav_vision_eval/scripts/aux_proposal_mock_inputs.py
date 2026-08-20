#!/usr/bin/env python3
"""为 Aux Proposal Provider L0 发布确定性 CV/YOLO 候选。"""

import rospy

from uav_vision.msg import TargetCandidate, TargetCandidateArray


class MockInputs:
    def __init__(self):
        self._cv_pub = rospy.Publisher(
            "/mock/aux/cv_targets", TargetCandidateArray, queue_size=1)
        self._yolo_pub = rospy.Publisher(
            "/mock/aux/yolo_targets", TargetCandidateArray, queue_size=1)
        self._timer = rospy.Timer(rospy.Duration(0.10), self._publish)

    @staticmethod
    def _candidate(stamp, candidate_id, class_name, confidence,
                   x, y, quality, frame="camera_init"):
        target = TargetCandidate()
        target.header.stamp = stamp
        target.header.frame_id = frame
        target.id = candidate_id
        target.class_name = class_name
        target.class_confidence = confidence
        target.map_valid = True
        target.map_point.x = x
        target.map_point.y = y
        target.map_frame = frame
        target.map_quality = quality
        target.transform_age_sec = 0.02
        target.state = 2
        target.observe_count = 3
        target.consecutive_observe_count = 3
        target.first_seen = stamp - rospy.Duration(0.2)
        target.last_seen = stamp
        return target

    def _publish(self, _event):
        stamp = rospy.Time.now()
        cv = TargetCandidateArray()
        cv.header.stamp = stamp
        cv.header.frame_id = "camera_init"
        cv.targets = [
            self._candidate(stamp, 3, "circle", 0.78, 1.0, 2.0, 0.80),
            self._candidate(stamp, 4, "circle", 0.10, 3.0, 2.0, 0.80),
        ]
        self._cv_pub.publish(cv)

        yolo = TargetCandidateArray()
        yolo.header.stamp = stamp
        yolo.header.frame_id = "camera_init"
        yolo.targets = [
            self._candidate(
                stamp, 8, "red_cross", 0.91, -2.0, 1.0, 0.72),
        ]
        self._yolo_pub.publish(yolo)


if __name__ == "__main__":
    rospy.init_node("aux_proposal_mock_inputs")
    MockInputs()
    rospy.spin()

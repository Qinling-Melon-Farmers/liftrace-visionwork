#!/usr/bin/env python3
"""Publish YOLO overlays on the exact source frame, independent of control."""
from collections import OrderedDict
from threading import Lock

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from uav_vision.msg import TargetDetectionArray


class YoloLiveView:
    def __init__(self):
        self._bridge = CvBridge()
        self._lock = Lock()
        self._images = OrderedDict()
        self._results = OrderedDict()
        self._capacity = max(1, int(rospy.get_param("~buffer_frames", 30)))
        self._pub = rospy.Publisher(
            rospy.get_param("~output_topic", "/uav_vision/yolo_debug"),
            Image, queue_size=1)
        self._image_sub = rospy.Subscriber(
            rospy.get_param("~image_topic", "/camera/image_raw"), Image,
            self._on_image, queue_size=1, buff_size=2**24)
        self._result_sub = rospy.Subscriber(
            rospy.get_param("~detections_topic", "/uav_vision/detections"),
            TargetDetectionArray, self._on_result, queue_size=10)

    def _on_image(self, msg):
        self._accept(msg, True)

    def _on_result(self, msg):
        # This topic also carries geometry detector arrays. Both YOLO
        # backends identify themselves as target_detector.
        if msg.source == "target_detector":
            self._accept(msg, False)

    def _accept(self, msg, is_image):
        key = (msg.header.stamp.to_nsec(), msg.header.frame_id)
        with self._lock:
            if self._pub.get_num_connections() == 0:
                self._images.clear()
                self._results.clear()
                return
            cache = self._images if is_image else self._results
            cache[key] = msg
            while len(cache) > self._capacity:
                cache.popitem(last=False)
            if key not in self._images or key not in self._results:
                return
            image_msg = self._images.pop(key)
            result = self._results.pop(key)
        try:
            frame = self._bridge.imgmsg_to_cv2(image_msg, "bgr8").copy()
            height, width = frame.shape[:2]
            for det in result.detections:
                roi = det.roi
                x1 = max(0, min(width - 1, int(roi.x_offset)))
                y1 = max(0, min(height - 1, int(roi.y_offset)))
                x2 = max(0, min(width - 1, int(roi.x_offset + roi.width)))
                y2 = max(0, min(height - 1, int(roi.y_offset + roi.height)))
                if x2 <= x1 or y2 <= y1:
                    continue
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, "%s %.2f" % (
                    det.class_name, det.class_confidence),
                    (x1, max(20, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 0), 2)
            cv2.putText(frame, "YOLO detections: %d" % len(result.detections),
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 255), 2)
            output = self._bridge.cv2_to_imgmsg(frame, "bgr8")
            output.header = image_msg.header
            self._pub.publish(output)
        except (CvBridgeError, cv2.error) as exc:
            rospy.logwarn_throttle(5, "YOLO overlay conversion failed: %s", exc)


if __name__ == "__main__":
    rospy.init_node("yolo_live_view")
    YoloLiveView()
    rospy.spin()

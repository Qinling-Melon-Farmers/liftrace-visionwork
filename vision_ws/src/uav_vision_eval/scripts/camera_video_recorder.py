#!/usr/bin/env python3
"""Read-only image-topic video with source timestamps; no bag or flight outputs."""
import csv
import json
import threading
from pathlib import Path
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Recorder:
    def __init__(self):
        self.path = Path(rospy.get_param('~output_path'))
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.topic = rospy.get_param('~image_topic')
        self.fps = float(rospy.get_param('~fps', 10.0))
        if not 0 < self.fps <= 60:
            raise ValueError('fps must be in (0,60]')
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.writer = None
        self.closed = False
        self.received = self.frames = 0
        self.first = self.last = None
        self.error = None
        self.stamps = self.path.with_suffix('.csv').open('w', buffering=1)
        self.csv = csv.writer(self.stamps)
        self.csv.writerow(['frame', 'image_stamp_ros_sec', 'receipt_ros_sec'])
        rospy.on_shutdown(self.close)
        self.sub = rospy.Subscriber(self.topic, Image, self.image, queue_size=1,
                                    buff_size=8*1024*1024)

    def image(self, msg):
        with self.lock:
            if self.closed:
                return
            self.received += 1
            stamp = msg.header.stamp.to_sec()
            if self.last is not None and stamp >= self.last and stamp-self.last < 1/self.fps-1e-6:
                return
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                if self.writer is None:
                    self.size = (frame.shape[1], frame.shape[0])
                    self.writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*'mp4v'),
                                                  self.fps, self.size)
                    if not self.writer.isOpened():
                        raise RuntimeError('VideoWriter could not open output')
                if (frame.shape[1], frame.shape[0]) != self.size:
                    raise RuntimeError('Camera resolution changed during recording')
                self.writer.write(frame)
                self.csv.writerow([self.frames, stamp, rospy.get_time()])
                self.frames += 1
                self.first = stamp if self.first is None else self.first
                self.last = stamp
            except Exception as exc:
                self.error = str(exc)
                rospy.logerr_throttle(5, 'camera recorder: %s', self.error)

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            if self.writer is not None:
                self.writer.release()
            self.stamps.close()
            self.path.with_suffix('.json').write_text(json.dumps({
                'status': 'RECORDED' if self.frames and not self.error else 'ERROR',
                'image_topic': self.topic, 'frames': self.frames, 'received': self.received,
                'encoded_fps': self.fps, 'first_image_stamp': self.first,
                'last_image_stamp': self.last, 'error': self.error,
                'timing': 'Constant-rate review video; CSV image timestamps are authoritative for gaps.'
            }, indent=2))


if __name__ == '__main__':
    rospy.init_node('camera_video_recorder')
    recorder = Recorder()
    rospy.spin()

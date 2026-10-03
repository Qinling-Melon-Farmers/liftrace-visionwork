#!/usr/bin/env python3
"""Replay recorded H images/TF through vision only; never publish flight inputs."""
import collections
import json
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rosbag
import rospy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from uav_vision.msg import (TargetDetectionArray, TargetCandidateArray,
                           DropOffset, DropReady)


def main():
    rospy.init_node("landing_bag_replay")
    path = Path(rospy.get_param("~bag"))
    output = Path(rospy.get_param("~output_dir"))
    output.mkdir(parents=True, exist_ok=True)
    camera = rospy.get_param("~source_camera_topic", "/board_trials/recording/image/compressed")
    camera_info = rospy.get_param("~source_camera_info_topic", "/camera/camera_info")
    frames, context = [], []
    with rosbag.Bag(str(path)) as source:
        start = source.get_start_time()
        for topic, msg, receipt in source.read_messages(topics=[camera, camera_info, "/tf", "/tf_static", "/uav_vision/align_mode"]):
            if topic == camera:
                frames.append((msg, receipt))
            else:
                context.append((topic, msg, receipt))
    if not frames:
        raise RuntimeError("recorded camera is missing")
    publishers = {
        camera_info: rospy.Publisher("/replay/camera_info", CameraInfo, queue_size=1, latch=True),
        "/tf": rospy.Publisher("/tf", TFMessage, queue_size=100),
        "/tf_static": rospy.Publisher("/tf_static", TFMessage, queue_size=10, latch=True),
        "/uav_vision/align_mode": rospy.Publisher("/uav_vision/align_mode", String, queue_size=1, latch=True),
    }
    images = rospy.Publisher("/replay/image", Image, queue_size=1)
    clock = rospy.Publisher("/clock", Clock, queue_size=1, latch=True)
    cv2.setNumThreads(1)
    lock = threading.Condition()
    received = collections.defaultdict(set)
    records, subscriptions = [], []
    active_time = [frames[0][1]]
    types = {
        "/replay/baseline": TargetDetectionArray,
        "/uav_vision/detections": TargetDetectionArray,
        "/uav_vision/detections_resolved": TargetDetectionArray,
        "/uav_vision/detections_refined": TargetDetectionArray,
        "/uav_vision/detections_mapped": TargetDetectionArray,
        "/uav_vision/targets": TargetCandidateArray,
        "/uav_vision/drop_offset": DropOffset,
        "/uav_vision/drop_ready": DropReady,
    }
    def callback(msg, topic):
        with lock:
            records.append((topic, msg, active_time[0]))
            if isinstance(msg, TargetDetectionArray):
                received[msg.header.stamp.to_nsec()].add(topic)
            lock.notify_all()
    for topic, typ in types.items():
        subscriptions.append(rospy.Subscriber(topic, typ, callback, callback_args=topic, queue_size=100))
    clock.publish(Clock(frames[0][1]))
    deadline = time.monotonic() + 30
    while images.get_num_connections() < 2:
        if rospy.is_shutdown() or time.monotonic() > deadline:
            raise RuntimeError("H detector pair did not start")
        time.sleep(.05)
    tf_context = [row for row in context if row[0] == "/tf"]
    static_tf = {}
    for topic, message, _ in context:
        if topic == "/tf_static":
            for transform in message.transforms:
                static_tf[transform.child_frame_id] = transform
    publishers["/tf_static"].publish(TFMessage(list(static_tf.values())))
    time.sleep(.2)
    tf_cursor = 0
    cursor = 0
    for index, (compressed, receipt) in enumerate(frames):
        active_time[0] = receipt
        clock.publish(Clock(receipt))
        while cursor < len(context) and context[cursor][2] <= receipt:
            topic, message, _ = context[cursor]
            # TF is sent once by the look-ahead cursor; static TF was latched above.
            if topic not in ("/tf", "/tf_static"):
                publishers[topic].publish(message)
            cursor += 1
        # Supply the recorded right-hand TF interpolation sample. Offline
        # look-ahead is explicit and must not be interpreted as online latency.
        while tf_cursor < len(tf_context) and tf_context[tf_cursor][2].to_sec() <= receipt.to_sec()+.25:
            publishers["/tf"].publish(tf_context[tf_cursor][1])
            tf_cursor += 1
        time.sleep(.03)
        image = cv2.imdecode(np.frombuffer(compressed.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None: raise RuntimeError("bad compressed frame")
        msg = Image(header=compressed.header, height=image.shape[0], width=image.shape[1],
                    encoding="bgr8", step=image.shape[1]*3, data=image.tobytes())
        images.publish(msg)
        expected = {"/replay/baseline", "/uav_vision/detections"}
        deadline = time.monotonic() + 10
        with lock:
            while not expected <= received[msg.header.stamp.to_nsec()]:
                if rospy.is_shutdown() or time.monotonic() > deadline:
                    raise RuntimeError("frame %d timeout: %s" % (index, received[msg.header.stamp.to_nsec()]))
                lock.wait(.02)
        time.sleep(.04)
        if index % 100 == 0: print("H replay %d/%d" % (index+1, len(frames)), flush=True)
    time.sleep(.3)
    counts = collections.Counter()
    first = {}
    with rosbag.Bag(str(output / "landing_reinferred.bag"), "w") as bag:
        for topic, msg, receipt in records:
            bag.write(topic, msg, receipt)
            for det in getattr(msg, "detections", []):
                key = topic + ":" + det.class_name
                counts[key] += 1
                first.setdefault(key, receipt.to_sec()-start)
                if getattr(det, "map_valid", False): counts[key+":map_valid"] += 1
            for target in getattr(msg, "targets", []):
                key = topic+":"+target.class_name+":"+str(target.state)
                counts[key] += 1
                first.setdefault(key, receipt.to_sec()-start)
    report = dict(kind="OFFLINE_H_VISION_REPLAY_NOT_CLOSED_LOOP_FLIGHT", bag=str(path),
                  camera_frames=len(frames), counts=dict(counts), first_sec=first,
                  tf_lookahead_sec=.25,
                  note="Recorded camera/TF/mode only. Counterfactual perception; actual flight not changed.")
    (output/"reinference.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()

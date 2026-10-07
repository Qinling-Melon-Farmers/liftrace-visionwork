#!/usr/bin/env python3
"""Observe an existing run ONLY; no control/GT publishing and no simulator start.

Use the ROS interpreter after sourcing the H overlays. A missing/changed
CameraInfo, image stream or Gazebo world SDF returns nonzero for the runner.
"""
import argparse
import json
from pathlib import Path
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from camera_tools import validate_camera_info, validate_sdf, validate_world_sdf, write_new_json


def observe(args, contract):
    import rospy
    from sensor_msgs.msg import CameraInfo, Image
    rospy.init_node('drop_research_camera_observer', anonymous=True, disable_signals=True)
    errors, infos, images = [], [], []
    lock = threading.Lock()
    started = time.monotonic()

    def info_callback(msg):
        info = dict(width=msg.width, height=msg.height, frame_id=msg.header.frame_id,
                    distortion_model=msg.distortion_model, K=list(msg.K), P=list(msg.P),
                    R=list(msg.R), D=list(msg.D), binning_x=msg.binning_x, binning_y=msg.binning_y,
                    roi={k: getattr(msg.roi, k) for k in ('x_offset', 'y_offset', 'width', 'height')})
        try:
            validate_camera_info(info, contract)
            if msg.header.stamp.to_sec() <= 0:
                raise ValueError('CameraInfo has no source timestamp')
            with lock:
                infos.append((time.monotonic(), msg.header.stamp.to_sec(), info))
        except ValueError as exc:
            with lock:
                errors.append(str(exc))

    def image_callback(msg):
        with lock:
            if (msg.width, msg.height, msg.header.frame_id) != (
                    contract['width'], contract['height'], contract['frame_id']):
                errors.append('Image size/frame mismatch')
            elif msg.header.stamp.to_sec() <= 0:
                errors.append('Image has no source timestamp')
            else:
                images.append((time.monotonic(), msg.header.stamp.to_sec()))

    info_sub = rospy.Subscriber(args.camera_info_topic, CameraInfo, info_callback, queue_size=1)
    image_sub = rospy.Subscriber(args.image_topic, Image, image_callback, queue_size=1)
    try:
        deadline = started + args.duration_wall
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with lock:
                if errors:
                    raise ValueError(errors[0])
                now = time.monotonic()
                for stream, name in ((infos, 'CameraInfo'), (images, 'Image')):
                    if stream and now - stream[-1][0] > args.max_gap_wall:
                        raise ValueError(name + ' stream stalled')
            time.sleep(.05)
        with lock:
            if errors:
                raise ValueError(errors[0])
            if len(infos) < args.min_samples or len(images) < args.min_samples:
                raise ValueError('Insufficient runtime CameraInfo/Image samples')
            if len(set(row[1] for row in infos)) < args.min_samples or len(set(row[1] for row in images)) < args.min_samples:
                raise ValueError('Repeated source stamps are not fresh runtime samples')
            info_stamps = {row[1] for row in infos}
            paired = sum(row[1] in info_stamps for row in images)
            if paired < args.min_samples:
                raise ValueError('Insufficient exact source-stamp Image/CameraInfo pairs')
        # Request the server's loaded SDF rather than trusting the disk path.
        response = subprocess.run([str(args.world_sdf_helper), args.world],
                                  capture_output=True, text=True, timeout=20, check=True)
        # Preserve the raw server response before any parsing/validation can fail.
        if args.world_sdf_output:
            with args.world_sdf_output.open('x', encoding='utf-8') as stream:
                stream.write(response.stdout)
        runtime_sdf = validate_world_sdf(response.stdout, contract, args.model_name)
        return dict(status='PASS', scope='RUNTIME_CONFIG_OBSERVATION_NOT_PIXEL_CALIBRATION',
                    simulation_started_by_this_tool=False, camera_info_samples=len(infos),
                    image_samples=len(images), exact_stamp_pairs=paired,
                    observation_wall_s=time.monotonic()-started,
                    camera_info=infos[-1][2], runtime_model=runtime_sdf,
                    camera_info_topic=args.camera_info_topic, image_topic=args.image_topic)
    finally:
        info_sub.unregister()
        image_sub.unregister()
        rospy.signal_shutdown('Read-only camera observation finished')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observe-existing-run', action='store_true', required=True)
    parser.add_argument('--contract', type=Path, required=True)
    parser.add_argument('--world-sdf-helper', type=Path, required=True)
    parser.add_argument('--world', required=True)
    parser.add_argument('--model-name', default='iris_mid360')
    parser.add_argument('--camera-info-topic', default='/downward_camera/camera_info')
    parser.add_argument('--image-topic', default='/downward_camera/image_raw')
    parser.add_argument('--duration-wall', type=float, default=30.)
    parser.add_argument('--max-gap-wall', type=float, default=10.)
    parser.add_argument('--min-samples', type=int, default=3)
    parser.add_argument('--world-sdf-output', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.duration_wall <= 0 or args.max_gap_wall <= 0 or args.min_samples < 3:
        parser.error('Positive durations and at least three distinct samples are required')
    try:
        contract = json.loads(args.contract.read_text())
        validate_sdf(ET.parse(contract['generated_sdf']), contract)
        result = observe(args, contract)
        code = 0
    except Exception as exc:
        result = dict(status='FAIL', scope='RUNTIME_CAMERA_CONTRACT', reason=str(exc),
                      simulation_started_by_this_tool=False)
        code = 1
    write_new_json(args.output, result)
    print(json.dumps(result, indent=2))
    raise SystemExit(code)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Standalone research subscriber: publishes observation state and diagnostics only."""
import json
import threading
import time
from collections import deque
from dataclasses import fields, MISSING
import numpy as np
import cv2
import rospy
import tf2_ros
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from uav_coverage_memory.memory import Camera, Config, Memory, State
from uav_coverage_memory.recording import Recorder
from uav_coverage_memory.cloud import decode_xyz
from uav_coverage_memory.resources import ByteWindow, WorkBudget


def transform_matrix(message):
    p, q = message.transform.translation, message.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    if not np.isfinite([x,y,z,w,p.x,p.y,p.z]).all() or abs(x*x+y*y+z*z+w*w-1) > 1e-4:
        raise ValueError('invalid TF quaternion/translation')
    result = np.eye(4)
    result[:3,:3] = [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    result[:3,3] = [p.x,p.y,p.z]
    return result


class Shadow:
    def __init__(self):
        cv2.setNumThreads(1)
        values = {}
        for field in fields(Config):
            name = '~memory/'+field.name
            values[field.name] = rospy.get_param(name) if field.default is MISSING else rospy.get_param(name, field.default)
        self.memory = Memory(Config(**values))
        self.lock = threading.RLock()
        self.input_lock = threading.RLock()
        self.bridge = CvBridge()
        self.tf = tf2_ros.Buffer(cache_time=rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.tf)
        self.tf_timeout = float(rospy.get_param('~tf_timeout', .03))
        self.sample_hz = float(rospy.get_param('~sample_hz', 3.0))
        if not 0 < self.sample_hz <= 10 or not 0 <= self.tf_timeout <= .2:
            raise ValueError('invalid sampling / TF timeout')
        self.budget = WorkBudget(self.sample_hz, float(rospy.get_param('~resources/cpu_fraction', .15)),
                                 float(rospy.get_param('~resources/warn_wall_ms', 150.)))
        self.max_image_bytes = int(rospy.get_param('~resources/max_image_bytes', 8*1024*1024))
        self.max_cloud_bytes = int(rospy.get_param('~resources/max_cloud_bytes', 16*1024*1024))
        if min(self.max_image_bytes, self.max_cloud_bytes) <= 0:
            raise ValueError('positive message byte limits required')
        self.image = self.info = self.cloud = None
        # Match delayed images to a recent past snapshot, never silently use a
        # newer map/CameraInfo merely because it arrived last. Images remain latest-only.
        self.infos = deque(maxlen=8)
        self.clouds = ByteWindow(int(rospy.get_param('~resources/cloud_cache_count', 4)),
                                 int(rospy.get_param('~resources/cloud_cache_bytes', 32*1024*1024)))
        self.last_processed = None
        self.mission_id = None
        self.epoch_start = rospy.Time.now().to_sec()
        self.last_clock = self.epoch_start
        self.map_reason = 'no_map'
        self.input_map_reason = 'no_map'
        self.input_image_reason = 'image_or_calibration_missing'
        self.map_input_points = 0
        record_dir = rospy.get_param('~record_dir', '')
        self.recorder = Recorder(record_dir, float(rospy.get_param('~record_interval',6.)),
                                 int(rospy.get_param('~record_limit',120))) if record_dir else None
        if self.recorder:
            rospy.on_shutdown(self.recorder.close)
        self.state_pub = rospy.Publisher('~state_grid', OccupancyGrid, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('~status', String, queue_size=1, latch=True)
        rospy.Subscriber(rospy.get_param('~image_topic'), Image, self.on_image, queue_size=1, buff_size=2**24)
        rospy.Subscriber(rospy.get_param('~camera_info_topic'), CameraInfo, self.on_info, queue_size=1)
        rospy.Subscriber(rospy.get_param('~obstacle_topic'), PointCloud2, self.on_cloud, queue_size=1, buff_size=2**24)
        rospy.Subscriber(rospy.get_param('~mission_status_topic'), String, self.on_mission, queue_size=1)
        self.reset_service = rospy.Service('~reset', Trigger, self.on_reset)
        self.timer = rospy.Timer(rospy.Duration(1/self.sample_hz), self.tick, reset=True)

    def on_image(self, message):
        with self.input_lock:
            if (message.width*message.height > self.memory.config.max_image_pixels or
                    len(message.data) > self.max_image_bytes):
                self.image = None
                self.input_image_reason = 'image_resource_limit'
                return
            self.image = message
            self.input_image_reason = 'image_or_calibration_missing'

    def on_info(self, message):
        with self.input_lock:
            self.info = message
            self.infos.append(message)

    def on_cloud(self, message):
        with self.input_lock:
            self.map_input_points = message.width*message.height
            if message.width*message.height > self.memory.config.max_cloud_points:
                self.input_map_reason = 'cloud_point_limit'
                self.cloud = None
                self.clouds.clear()
                return
            if len(message.data) > self.max_cloud_bytes or not self.clouds.append(message):
                self.input_map_reason = 'cloud_byte_limit'
                self.cloud = None
                self.clouds.clear()
                return
            self.cloud = message
            self.input_map_reason = 'snapshot_received'

    def reset(self, reason, epoch=None):
        self.memory.reset(reason, epoch)
        with self.input_lock:
            self.image = self.cloud = None
            self.infos.clear()
            self.info = None
            self.clouds.clear()
            self.input_map_reason = 'no_map'
            self.input_image_reason = 'image_or_calibration_missing'
        self.last_processed = None
        self.epoch_start = rospy.Time.now().to_sec()

    def on_reset(self, _request):
        with self.lock:
            self.reset('explicit_reset')
            return TriggerResponse(success=True, message='research memory reset; external map not reset')

    def on_mission(self, message):
        try:
            data = json.loads(message.data)
            epoch = data.get('mission_id', '')
            if not isinstance(epoch, str):
                return
        except (ValueError, AttributeError):
            return
        with self.lock:
            if epoch != self.mission_id:
                self.mission_id = epoch
                self.reset('mission_changed', epoch)

    def lookup(self, target, source, stamp):
        if not source or stamp.to_sec() <= 0:
            raise ValueError('missing source frame/stamp')
        transform = self.tf.lookup_transform(target, source, stamp, rospy.Duration(self.tf_timeout))
        # Exact lookup only. A zero output stamp is allowed for a static-only chain.
        delta = transform.header.stamp.to_sec()
        if delta and abs(delta-stamp.to_sec()) > self.memory.config.max_tf_delta:
            raise ValueError('TF source-time mismatch')
        return transform_matrix(transform), delta

    def map_snapshot(self, stamp, messages=None):
        c = self.memory.config
        if messages is None:
            with self.input_lock:
                messages = tuple(self.clouds)
                self.map_reason = self.input_map_reason
        cloud = self.as_of(messages, stamp, c.max_map_age)
        if cloud is None:
            if self.map_reason not in ('cloud_point_limit', 'cloud_byte_limit'):
                self.map_reason = 'no_source_time_matched_map'
            return None, None
        cloud_stamp = cloud.header.stamp.to_sec()
        if cloud_stamp < self.epoch_start or not -c.future_tolerance <= stamp-cloud_stamp <= c.max_map_age:
            return None, None
        # Do not truncate an oversized obstacle cloud and then mistake missing
        # obstacles for visibility. Oversized/empty/invalid inputs remain unknown.
        if cloud.width*cloud.height > c.max_cloud_points:
            return None, None
        points = decode_xyz(cloud,c.max_cloud_points,self.max_cloud_bytes)
        if points.size == 0:
            return None, None
        matrix, _ = self.lookup(c.frame_id, cloud.header.frame_id, cloud.header.stamp)
        self.map_reason = 'source_time_matched'
        return points @ matrix[:3,:3].T + matrix[:3,3], cloud_stamp

    def as_of(self, messages, stamp, max_age):
        valid = [m for m in messages if m.header.stamp.to_sec() >= self.epoch_start and
                 -self.memory.config.future_tolerance <= stamp-m.header.stamp.to_sec() <= max_age]
        return max(valid, key=lambda m:m.header.stamp.to_sec()) if valid else None

    def tick(self, _event):
        with self.lock:
            now = rospy.Time.now().to_sec()
            if now < self.last_clock-self.memory.config.future_tolerance:
                self.reset('clock_rollback')
            self.last_clock = now
            begin = time.perf_counter()
            if not self.budget.ready(begin):
                result = self.memory.summary(now, accepted=False, reason='work_budget_cooldown',
                    budget_skips=self.budget.skipped, cooldown_ms=(self.budget.next_ready-begin)*1000)
                self.publish(result, now)
                return
            cpu_begin = time.process_time()
            try:
                result = self.process(now)
            except (ValueError, TypeError, KeyError, AttributeError, tf2_ros.TransformException) as error:
                result = self.memory.reject('adapter_rejected:'+str(error), now)
            except Exception as error:
                # Diagnostic-only node must not silently retain an active streak
                # if transport/OpenCV decoding fails.
                result = self.memory.reject('processing_error:'+type(error).__name__, now)
                rospy.logwarn_throttle(5, 'coverage shadow: %s', error)
            result['processing_ms'] = (time.perf_counter()-begin)*1000
            result.update(self.budget.finish(begin, time.perf_counter(), time.process_time()-cpu_begin))
            self.publish(result, rospy.Time.now().to_sec())

    def process(self, now):
        # Transport callbacks take only this short lock. Processing and resets
        # are serialized by self.lock; no frame can be committed across a reset.
        with self.input_lock:
            image, latest_info = self.image, self.info
            infos, clouds = tuple(self.infos), tuple(self.clouds)
            self.map_reason = self.input_map_reason
            input_points = self.map_input_points
            cloud_cache_bytes = self.clouds.bytes
            image_reason = self.input_image_reason
        if image is None or latest_info is None:
            return self.memory.reject(image_reason, now)
        stamp = image.header.stamp.to_sec()
        if stamp < self.epoch_start:
            return self.memory.reject('image_before_epoch', now)
        if stamp == self.last_processed:
            # A timer tick is not an additional image observation.
            return self.memory.summary(now, accepted=False, reason='no_new_image')
        self.last_processed = stamp
        if not -self.memory.config.future_tolerance <= now-stamp <= self.memory.config.max_image_age:
            return self.memory.reject('image_stale_or_future', now)
        info = self.as_of(infos, stamp, self.memory.config.max_camera_age)
        if info is None:
            return self.memory.reject('no_calibration_at_image_time', now)
        if (info.binning_x not in (0,1) or info.binning_y not in (0,1) or
                info.roi.x_offset or info.roi.y_offset or
                info.roi.width not in (0,info.width) or info.roi.height not in (0,info.height)):
            raise ValueError('cropped/binned CameraInfo requires calibrated full-image input')
        camera = Camera(info.width, info.height, tuple(info.K), tuple(info.D), info.header.frame_id, info.distortion_model)
        matrix, tf_stamp = self.lookup(self.memory.config.frame_id, image.header.frame_id, image.header.stamp)
        begin = time.perf_counter()
        pixels = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        image_decode_ms = (time.perf_counter()-begin)*1000
        begin = time.perf_counter()
        try:
            points, cloud_stamp = self.map_snapshot(stamp, clouds)
        except Exception as error:
            self.map_reason = 'map_error:'+str(error)
            points, cloud_stamp = None, None
        map_decode_transform_ms = (time.perf_counter()-begin)*1000
        # Keep a small transaction checkpoint (five cell arrays, no images/maps)
        # so a slow frame cannot add visibility credit after it has expired.
        checkpoint = self.memory.__dict__.copy()
        for name in ('state', 'last_seen', 'last_good', 'first_good', 'count'):
            checkpoint[name] = checkpoint[name].copy()
        begin = time.perf_counter()
        try:
            result = self.memory.observe(pixels, camera, matrix, stamp, now, tf_stamp,
                info.header.stamp.to_sec(), image.header.frame_id,
                points, cloud_stamp, self.memory.config.frame_id)
        except Exception:
            self.memory.__dict__.update(checkpoint)
            raise
        observe_ms = (time.perf_counter()-begin)*1000
        completed_ros = rospy.Time.now().to_sec()
        if result['accepted'] and not -self.memory.config.future_tolerance <= completed_ros-stamp <= self.memory.config.max_image_age:
            self.memory.__dict__.update(checkpoint)
            if completed_ros < now-self.memory.config.future_tolerance:
                self.memory.reset('clock_rollback')
            result = self.memory.reject('image_expired_during_processing', completed_ros)
        result.update(camera_stamp=info.header.stamp.to_sec(), tf_stamp=tf_stamp,
                      map_stamp=cloud_stamp, map_points=0 if points is None else len(points),
                      map_input_points=input_points,map_reason=self.map_reason,
                      cloud_cache_bytes=cloud_cache_bytes,
                      image_age=now-stamp, completion_image_age=completed_ros-stamp,
                      image_decode_ms=image_decode_ms, map_decode_transform_ms=map_decode_transform_ms,
                      observe_ms=observe_ms)
        if self.recorder and result['accepted']:
            begin = time.perf_counter()
            self.recorder.sample(result,pixels,camera,matrix,points,cloud_stamp,
                                 self.memory.state.reshape(self.memory.ny,self.memory.nx),self.memory.config)
            result['sample_write_ms'] = (time.perf_counter()-begin)*1000
        return result

    def publish(self, result, now):
        c = self.memory.config
        grid = OccupancyGrid()
        grid.header.stamp = rospy.Time.from_sec(max(0,now))
        grid.header.frame_id = c.frame_id
        grid.info.resolution = c.resolution
        grid.info.width, grid.info.height = self.memory.nx, self.memory.ny
        grid.info.origin.position.x, grid.info.origin.position.y = c.min_x, c.min_y
        grid.info.origin.position.z = c.ground_z
        grid.info.origin.orientation.w = 1.0
        grid.data = self.memory.state.tolist()
        result['state_codes'] = {s.name:int(s) for s in State}
        result['not_an_occupancy_or_navigation_map'] = True
        result['receipt_ros'] = now
        result['grid_geometry'] = {key:getattr(c,key) for key in
            ('min_x','max_x','min_y','max_y','resolution','ground_z','frame_id')}
        if self.recorder:
            self.recorder.status(result)
        self.state_pub.publish(grid)
        self.status_pub.publish(String(data=json.dumps(result, sort_keys=True)))


if __name__ == '__main__':
    rospy.init_node('coverage_memory_shadow')
    Shadow()
    rospy.spin()

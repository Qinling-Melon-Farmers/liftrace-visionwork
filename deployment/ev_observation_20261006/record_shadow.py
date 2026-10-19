#!/usr/bin/env python3
"""Extra bounded shadow bag. Preview by default; no publishers or flight calls."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys

from ev_suite.bootstrap import HERE, ROOT, message_checks

TOPICS = {
    '/livox/imu': 'sensor_msgs/Imu',
    '/laserMapping/prediction_state': 'fast_lio/PredictionState',
    '/laserMapping/realtime': 'diagnostic_msgs/DiagnosticArray',
    '/Odometry': 'nav_msgs/Odometry',
    '/mavros/vision_pose/pose': 'geometry_msgs/PoseStamped',
    '/mavros/local_position/odom': 'nav_msgs/Odometry',
    '/mavros/state': 'mavros_msgs/State',
    '/ev_shadow/raw_pose': 'geometry_msgs/PoseStamped',
    '/ev_shadow/smooth_pose': 'geometry_msgs/PoseStamped',
    '/ev_shadow/status': 'std_msgs/String',
    '/ev_task_boundary/status': 'std_msgs/String',
}


def recorder_module():
    path = ROOT/'deployment/low_hover_observation/record_diagnostics.py'
    spec = importlib.util.spec_from_file_location('ev_observation_bounded_recorder', str(path))
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    value.SAFE_TYPES = value.SAFE_TYPES | {'fast_lio/PredictionState'}
    original_forbidden = value.FORBIDDEN_NAME
    class ExactImuException:
        def search(self, name):
            return None if name == '/livox/imu' else original_forbidden.search(name)
    value.FORBIDDEN_NAME = ExactImuException()
    original_validate = value.validate_topic
    def validate(name, types):
        if name not in TOPICS or types != [TOPICS[name]]:
            raise ValueError('shadow recorder exact topic/type whitelist rejected: '+str(name))
        original_validate(name, types)
    value.validate_topic = validate
    return value


def config_check(module, path):
    cfg = module.load_config(path)
    if {x['name'] for x in cfg['topics']} != set(TOPICS):
        raise ValueError('shadow recording config must include the full exact topic set')
    return cfg


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--duration', type=float, default=600)
    p.add_argument('--config', type=Path, default=HERE/'config/recording_shadow.yaml')
    p.add_argument('--start', action='store_true')
    args = p.parse_args()
    module = recorder_module()
    cfg = config_check(module, args.config)
    duration = module.effective_duration(args.duration, cfg['max_duration_s'])
    if not args.start:
        print(json.dumps(dict(preview=True, output=str(args.output), duration=duration,
            topics=cfg['topics'], implementation=str(ROOT/'deployment/low_hover_observation/record_diagnostics.py'),
            publishers=[], flight_calls=[], master_started=False), indent=2))
        return 0
    # Require the same generated messages and existing local master as observers.
    message_checks(prediction=True)
    from observer import master
    master()
    import rospy
    def forbidden(*args, **kwargs):
        raise RuntimeError('read-only recorder cannot publish or call ROS services')
    rospy.Publisher = forbidden
    rospy.ServiceProxy = forbidden
    return module.main(['--output', str(args.output), '--config', str(args.config),
                        '--duration', str(duration)])


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ImportError, ValueError, RuntimeError, OSError) as exc:
        print('BLOCKED shadow recorder: '+str(exc), file=sys.stderr)
        sys.exit(3)

"""Exact publisher whitelist, checked even after rospy name resolution/remapping."""
import json

SHADOW = {
    '/ev_shadow/raw_pose': 'geometry_msgs/PoseStamped',
    '/ev_shadow/smooth_pose': 'geometry_msgs/PoseStamped',
    '/ev_shadow/status': 'std_msgs/String',
}
RESET = {
    '/ev_task_boundary/'+name: kind for name, kind in (
        ('task_pose', 'geometry_msgs/PoseStamped'),
        ('task_odom', 'nav_msgs/Odometry'),
        ('camera_pose', 'geometry_msgs/PoseStamped'),
        ('fc_setpoint', 'geometry_msgs/PoseStamped'),
        ('fc_hold_request', 'geometry_msgs/PoseStamped'),
        ('release_permission', 'uav_mission/ReleasePermission'),
        ('status', 'std_msgs/String'))
}
SHADOW_INPUTS = {'imu_topic', 'state_topic'}
RESET_INPUTS = {'fc_odom_input', 'lio_body_pose_input', 'state_input',
                'reset_input', 'lio_health_input', 'task_setpoint_input', 'permission_input'}


def config_check(kind, config):
    if not isinstance(config, dict):
        raise ValueError('config must be a mapping')
    allowed = ({'limits', 'output_hz', 'status_hz', 'imu_to_body_xyz'} | SHADOW_INPUTS
               if kind == 'shadow' else {'reference', 'limits'} | RESET_INPUTS)
    if set(config) - allowed:
        raise ValueError('unsupported parameter/output override: '+str(sorted(set(config)-allowed)))
    if kind == 'reset':
        reference = config.get('reference', {})
        if reference.get('calibration_verified') is not False:
            raise ValueError('observation suite requires calibration_verified=false; no online READY')
    else:
        if config.get('imu_to_body_xyz') != [0.0, 0.0, 0.0]:
            raise ValueError('this suite reports IMU reference only; no guessed body calibration')
        for key in SHADOW_INPUTS:
            if not isinstance(config.get(key), str) or not config[key].startswith('/'):
                raise ValueError('explicit absolute input required: '+key)
    for key in RESET_INPUTS & set(config):
        if not isinstance(config[key], str) or not config[key].startswith('/'):
            raise ValueError('explicit absolute input required: '+key)


class PublisherGuard:
    def __init__(self, rospy, kind):
        self.rospy, self.kind = rospy, kind
        self.allowed = SHADOW if kind == 'shadow' else RESET
        self.original = rospy.Publisher
        self.original_resolve = rospy.resolve_name
        self.created = set()

    def resolve(self, name, *args, **kwargs):
        canonical = ('/ev_shadow/'+name[1:] if name.startswith('~') and self.kind == 'shadow'
                     else name)
        topic = self.original_resolve(name, *args, **kwargs)
        if canonical in self.allowed and topic != canonical:
            raise ValueError('output remapping rejected before publisher creation: '+canonical)
        return topic

    def install(self):
        self.rospy.resolve_name = self.resolve
        self.rospy.Publisher = self
        # Validate every output first, including status, before creating any.
        for topic in self.allowed:
            if self.resolve(topic) != topic:
                raise ValueError('whitelist name did not resolve canonically')

    def __call__(self, name, message, *args, **kwargs):
        topic = self.resolve(name)
        # Check canonical pre-remap and resolved names, not only a prefix.
        canonical = ('/ev_shadow/'+name[1:] if name.startswith('~') and self.kind == 'shadow'
                     else name)
        if canonical not in self.allowed or topic != canonical:
            raise ValueError('output remap or non-whitelisted publisher rejected: '+name+' -> '+topic)
        if getattr(message, '_type', None) != self.allowed[topic]:
            raise ValueError('publisher message type rejected: '+topic)
        if args or kwargs != {'queue_size': 1}:
            raise ValueError('publisher configuration rejected: '+topic)
        pub = self.original(topic, message, queue_size=1)
        self.created.add(topic)
        if topic.endswith('/status'):
            original_publish = pub.publish

            def publish_status(value):
                data = json.loads(value.data)
                data['suite_observe_only'] = True
                if self.kind == 'shadow':
                    data['output_reference'] = 'lio_imu_no_body_calibration'
                else:
                    data['suite_blocked_reasons'] = [
                        'calibration_not_verified',
                        'authoritative_reset_producer_not_validated',
                        'independent_lio_health_producer_not_validated']
                    if data.get('ready') is not False:
                        raise ValueError('unverified reset suite may never be READY')
                value.data = json.dumps(data, allow_nan=False)
                return original_publish(value)

            pub.publish = publish_status
        return pub

    def complete(self):
        if self.created != set(self.allowed):
            raise ValueError('observer publisher set differs from whitelist')

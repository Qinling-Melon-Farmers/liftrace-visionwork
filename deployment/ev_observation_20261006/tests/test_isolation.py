import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from ev_suite.bootstrap import HERE, SOURCE
from ev_suite.isolation import SHADOW, RESET, PublisherGuard, config_check
from parallel_check import check


def fake_ros(remaps=None):
    remaps = remaps or {}
    def resolve(name, *args, **kwargs):
        name = '/ev_shadow/'+name[1:] if name.startswith('~') else name
        return remaps.get(name, name)
    return SimpleNamespace(resolve_name=resolve, Publisher=Mock(return_value=Mock()))


class IsolationTests(unittest.TestCase):
    def test_all_shadow_outputs_and_status_reject_formal_remap_before_publish(self):
        for topic in SHADOW:
            ros = fake_ros({topic: '/mavros/vision_pose/pose'})
            original = ros.Publisher
            with self.assertRaisesRegex(ValueError, 'remapping rejected'):
                PublisherGuard(ros, 'shadow').install()
            original.assert_not_called()

    def test_shadow_rejects_remap_even_inside_observer_namespace(self):
        ros = fake_ros({'/ev_shadow/status': '/ev_shadow/status_other'})
        with self.assertRaises(ValueError): PublisherGuard(ros, 'shadow').install()

    def test_all_reset_outputs_reject_formal_and_internal_remap(self):
        for topic in RESET:
            for destination in ('/mavros/setpoint_position/local', '/ev_task_boundary/other'):
                ros = fake_ros({topic: destination})
                original = ros.Publisher
                with self.assertRaises(ValueError): PublisherGuard(ros, 'reset').install()
                original.assert_not_called()

    def test_reset_swapped_outputs_rejected_before_creation(self):
        ros = fake_ros({'/ev_task_boundary/task_pose': '/ev_task_boundary/camera_pose'})
        with self.assertRaises(ValueError): PublisherGuard(ros, 'reset').install()

    def test_publisher_cannot_create_flight_topic(self):
        ros = fake_ros()
        with self.assertRaises(ValueError):
            PublisherGuard(ros, 'shadow')('/mavros/vision_pose/pose',
                SimpleNamespace(_type='geometry_msgs/PoseStamped'), queue_size=1)
        ros.Publisher.assert_not_called()

    def test_publisher_exact_type_required(self):
        ros = fake_ros()
        with self.assertRaises(ValueError):
            PublisherGuard(ros, 'shadow')('~status',
                SimpleNamespace(_type='geometry_msgs/PoseStamped'), queue_size=1)

    def test_shadow_guard_creates_exact_whitelist_and_annotates_status(self):
        ros = fake_ros(); original = ros.Publisher
        guard = PublisherGuard(ros, 'shadow'); guard.install()
        for name, kind in SHADOW.items():
            guard(name, SimpleNamespace(_type=kind), queue_size=1)
        guard.complete()
        self.assertEqual(original.call_count, 3)
        self.assertEqual(guard.created, set(SHADOW))

    def test_reset_status_cannot_report_unverified_ready(self):
        ros = fake_ros(); guard = PublisherGuard(ros, 'reset')
        pub = guard('/ev_task_boundary/status', SimpleNamespace(_type='std_msgs/String'), queue_size=1)
        with self.assertRaisesRegex(ValueError, 'never be READY'):
            pub.publish(SimpleNamespace(data=json.dumps({'ready': True})))

    def test_reset_status_explicitly_reports_missing_producers(self):
        ros = fake_ros(); pub = Mock(); ros.Publisher = Mock(return_value=pub)
        original = pub.publish
        wrapped = PublisherGuard(ros, 'reset')('/ev_task_boundary/status',
                    SimpleNamespace(_type='std_msgs/String'), queue_size=1)
        wrapped.publish(SimpleNamespace(data=json.dumps({'ready': False})))
        value = json.loads(original.call_args[0][0].data)
        self.assertEqual(len(value['suite_blocked_reasons']), 3)
        self.assertFalse(value['ready'])

    def test_config_rejects_all_output_overrides(self):
        for key in ('observer_namespace', 'status_output', 'raw_pose_output',
                    'fc_setpoint_output', 'release_permission_output', '__name'):
            with self.assertRaises(ValueError): config_check('reset', {'reference':
                {'calibration_verified': False}, key: '/mavros/vision_pose/pose'})

    def test_reset_calibration_true_or_string_rejected(self):
        for value in (True, 'false', None, 0):
            with self.assertRaises(ValueError): config_check('reset', {'reference':
                {'calibration_verified': value}})

    def test_shadow_requires_explicit_imu_reference_and_inputs(self):
        config = dict(imu_to_body_xyz=[0.,0.,0.], imu_topic='/livox/imu',
                      state_topic='/laserMapping/prediction_state')
        config_check('shadow', config)
        with self.assertRaises(ValueError): config_check('shadow', dict(config, imu_to_body_xyz=[0.,0.,-.05]))

    def test_unknown_ros_remap_cli_rejected_without_ros_import(self):
        result = subprocess.run([sys.executable, str(HERE/'observer.py'), 'shadow',
            '/ev_shadow/status:=/mavros/vision_pose/pose'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn('unrecognized arguments', result.stderr)

    def test_default_preview_does_not_require_ros(self):
        result = subprocess.run([sys.executable, str(HERE/'observer.py'), 'preview'],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)['default'], 'no nodes started')

    def test_sources_remain_byte_identical_to_recorded_origin(self):
        manifest = json.loads((HERE/'manifest.json').read_text())
        root = Path(manifest['source_root_recorded'])
        if not root.is_dir(): self.skipTest('original source checkout not present on this machine')
        for entry in manifest['files']:
            if entry['destination'].startswith('source/'):
                self.assertEqual((HERE/entry['destination']).read_bytes(),
                                 (root/entry['origin_path']).read_bytes())

    def test_snapshot_patch_preserves_thread_and_diagnostic_export(self):
        patch = (HERE/'integration/fast_lio_snapshot.patch').read_text()
        changes = [line for line in patch.splitlines() if line.startswith(('+', '-'))
                   and not line.startswith(('+++', '---'))]
        self.assertFalse(any('MP_EN' in line or 'MP_PROC_NUM' in line or
                             'FAST_LIO_MATCH_THREADS' in line for line in changes))
        self.assertFalse(any('CATKIN_DEPENDS' in line for line in changes))
        self.assertIn('prediction_state_en = false', patch)

    def test_optional_capture_patch_adds_no_nodes(self):
        patch = (HERE/'integration/low_hover_prediction_optional.patch').read_text()
        additions = [line for line in patch.splitlines() if line.startswith('+') and not line.startswith('+++')]
        self.assertFalse(any('<node ' in line for line in additions))
        self.assertTrue(any('enable_prediction_state' in line and 'default="false"' in line for line in additions))
        self.assertFalse(any('OMP_' in line for line in additions))

    def test_build_parallel_is_not_runtime_confirmation(self):
        result = check('-DMP_EN -DMP_PROC_NUM=3 -fopenmp', 'FAST_LIO_MATCH_THREADS:STRING=3')
        self.assertTrue(result['build_parallel'])
        self.assertFalse(result['runtime_parallel_confirmed'])

    def test_serial_or_wrong_worker_build_rejected(self):
        for flags in ('-DMP_PROC_NUM=1 -fopenmp', '-DMP_EN -DMP_PROC_NUM=2 -fopenmp'):
            with self.assertRaises(ValueError): check(flags)

    def test_parallel_evidence_requires_team_and_actual_binary_path(self):
        result = check('-DMP_EN -DMP_PROC_NUM=3 -fopenmp', 'FAST_LIO_MATCH_THREADS:STRING=3',
            'PASS: 10 production matching cases; workers=3\n',
            'launch executable: /board/devel/lib/fast_lio/fastlio_mapping',
            '/board/devel/lib/fast_lio/fastlio_mapping')
        self.assertTrue(result['runtime_parallel_confirmed'])


if __name__ == '__main__': unittest.main()

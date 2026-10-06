import copy
import importlib
import json
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from uav_mission.msg import ReleasePermission
from ev_suite.isolation import PublisherGuard, RESET
from ev_suite.bootstrap import MISSION_TESTS
import ev_task_boundary
import observer


class WrapperRosTests(unittest.TestCase):
    def test_new_0928_permission_fields_survive_copy_and_serialization(self):
        import io
        original = ReleasePermission()
        original.mission_id = 'offline-only'
        original.decision_seq = 17; original.attempt = 2
        original.permission_epoch = 'offline-arbiter'; original.permission_revision = 41
        result = copy.deepcopy(original)
        out = io.BytesIO(); result.serialize(out)
        decoded = ReleasePermission().deserialize(out.getvalue())
        for field in ('mission_id', 'decision_seq', 'attempt', 'permission_epoch', 'permission_revision'):
            self.assertEqual(getattr(decoded, field), getattr(original, field))

    def test_real_adapter_constructs_only_guarded_outputs_with_false_reference(self):
        config = observer.load_config('reset')
        def resolve(name): return name
        ros = ev_task_boundary.rospy
        with patch.object(ros, 'resolve_name', side_effect=resolve), \
             patch.object(ros, 'Publisher', side_effect=lambda *a, **k: Mock()) as pubs, \
             patch.object(ros, 'Subscriber') as subs, patch.object(ros, 'Timer'), \
             patch.object(ros, 'get_param', side_effect=lambda key, default=None: config.get(key[1:], default)):
            guard = PublisherGuard(ros, 'reset')
            with patch.object(ros, 'Publisher', side_effect=guard):
                node = ev_task_boundary.EvTaskBoundary()
                guard.complete()
                self.assertFalse(node.core.calibration_verified)
                self.assertFalse(node.core.ready)
                self.assertIsNone(node.health)
            self.assertEqual(pubs.call_count, 7)
            self.assertEqual(subs.call_count, 7)

    def test_start_flag_required_before_master_or_init_node(self):
        with patch('sys.argv', ['observer.py', 'reset']), \
             patch('observer.message_checks', return_value={}), \
             patch('observer.components'), patch('observer.start') as start:
            self.assertEqual(observer.main(), 0)
            start.assert_not_called()

    def test_shadow_start_disables_rosout_and_registers_only_exact_publishers(self):
        import rospy
        master = Mock(); master.getSystemState.return_value = ([], [], [])
        def resolve(name, *a, **k):
            return '/ev_shadow/'+name[1:] if name.startswith('~') else name
        with patch('observer.master', return_value=master), \
             patch.dict('os.environ', {'ROS_NAMESPACE': '/'}), \
             patch.object(rospy, 'init_node') as init, \
             patch.object(rospy, 'get_name', return_value='/ev_shadow'), \
             patch.object(rospy, 'set_param'), patch.object(rospy, 'get_param'), \
             patch.object(rospy, 'resolve_name', side_effect=resolve), \
             patch.object(rospy, 'Publisher', side_effect=lambda *a, **k: Mock()) as pubs, \
             patch.object(rospy, 'Subscriber'), patch.object(rospy, 'Timer'), patch.object(rospy, 'spin'):
            observer.start('shadow', observer.load_config('shadow'))
            self.assertIs(init.call_args.kwargs['disable_rosout'], True)
            self.assertEqual({call.args[0] for call in pubs.call_args_list},
                {'/ev_shadow/raw_pose', '/ev_shadow/smooth_pose', '/ev_shadow/status'})

    def test_graph_check_accepts_exact_sets_and_rejects_rosout(self):
        master = Mock()
        from ev_suite.isolation import SHADOW
        pubs = [(name, ['/ev_shadow']) for name in SHADOW]
        master.getSystemState.return_value = (pubs, [], [])
        with patch('observer.master', return_value=master):
            self.assertTrue(observer.graph_check()['/ev_shadow']['present'])
            master.getSystemState.return_value = (pubs+[('/rosout', ['/ev_shadow'])], [], [])
            with self.assertRaisesRegex(RuntimeError, 'unexpected publisher'):
                observer.graph_check()

    def test_runtime_message_preflight_rejects_disposable_fixture_path(self):
        from ev_suite.bootstrap import message_checks
        module = importlib.import_module(ReleasePermission.__module__)
        if 'generated_schema' not in module.__file__:
            self.skipTest('this check is specific to the explicit schema-fixture runner')
        with self.assertRaisesRegex(RuntimeError, 'current 0928 workspace'):
            message_checks(prediction=True)


if __name__ == '__main__': unittest.main()

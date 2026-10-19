"""FC index API mocks and reused trajectory semantics; never starts ROS."""
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch
import unittest
import numpy as np

VIEWER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(VIEWER))
sys.path.insert(0, str(VIEWER.parent))
from trajectory import position_samples, enu
from px4_log_index import collect_entries


class IndexTests(unittest.TestCase):
    def run_index(self, messages, ground=None, success=True):
        callbacks = {}
        clock = [0.]
        subscriber = Mock()
        rospy = Mock()
        rospy.Subscriber.side_effect = lambda topic, cls, callback, **kw: (callbacks.update(receive=callback) or subscriber)
        end = Mock(return_value=SimpleNamespace(success=True))
        def request(start, stop):
            self.assertEqual((start, stop), (0, 65534))
            for message in messages:
                callbacks['receive'](message)
            return SimpleNamespace(success=success)
        listing = Mock(side_effect=request)
        rospy.ServiceProxy.side_effect = lambda name, cls: end if name.endswith('log_request_end') else listing
        rospy.is_shutdown.return_value = False
        modules = {'mavros_msgs': SimpleNamespace(), 'mavros_msgs.msg': SimpleNamespace(LogEntry=object),
                   'mavros_msgs.srv': SimpleNamespace(LogRequestList=object, LogRequestEnd=object)}
        def sleep(seconds):
            clock[0] += seconds
        with patch.dict(sys.modules, modules):
            try:
                return collect_entries(rospy, '/fixture/mavros', 1, ground or Mock(),
                                       clock=lambda: clock[0], sleep=sleep)
            finally:
                end.assert_called_once_with()
                subscriber.unregister.assert_called_once_with()
                self.assertFalse(any('param' in str(c) or 'command' in str(c) for c in rospy.method_calls))

    def entry(self, identifier=17, count=2, size=1234):
        return SimpleNamespace(id=identifier, num_logs=count, size=size, last_log_num=42,
                               time_utc=SimpleNamespace(to_sec=lambda: 1791400000.))

    def test_real_ids_duplicate_order_and_completeness(self):
        entries = self.run_index([self.entry(42), self.entry(17), self.entry(42)])
        self.assertEqual([r['log_id'] for r in entries], [17, 42])
        self.assertTrue(all(r['index_complete'] for r in entries))
        self.assertTrue(entries[0]['time_beijing'].endswith('+08:00'))

    def test_partial_index_and_empty_card(self):
        self.assertFalse(self.run_index([self.entry()])[0]['index_complete'])
        self.assertEqual(self.run_index([self.entry(count=0, size=0)]), [])

    def test_missing_response_service_failure_and_arming_always_end(self):
        for args, message in [(([],), 'No FC'), (([self.entry()], None, False), 'failed'),
                              (([self.entry()], Mock(side_effect=RuntimeError('armed'))), 'armed')]:
            with self.assertRaisesRegex(RuntimeError, message):
                self.run_index(*args)

    def test_end_service_unavailable_still_unregisters_subscription(self):
        rospy = Mock()
        subscriber = Mock()
        rospy.Subscriber.return_value = subscriber
        rospy.wait_for_service.side_effect = RuntimeError('END unavailable')
        modules = {'mavros_msgs': SimpleNamespace(), 'mavros_msgs.msg': SimpleNamespace(LogEntry=object),
                   'mavros_msgs.srv': SimpleNamespace(LogRequestList=object, LogRequestEnd=object)}
        with patch.dict(sys.modules, modules), self.assertRaisesRegex(RuntimeError, 'END unavailable'):
            collect_entries(rospy, '/fixture/mavros', 1, Mock())
        subscriber.unregister.assert_called_once_with()
        rospy.ServiceProxy.assert_not_called()


class TrajectoryTests(unittest.TestCase):
    def test_mapping_gaps_resets_and_invalid_samples(self):
        data = {'timestamp': np.array([1., 1.1, 1.2, 2.])*1e6,
                'x': [10, 11, 12, 13], 'y': [20, 21, 22, 23], 'z': [-1, -2, -3, -4],
                'xy_valid': [1, 0, 1, 1], 'z_valid': [1, 1, 1, 1], 'z_reset_counter': [0, 0, 1, 1]}
        times, points, breaks = position_samples(data, 1000000, True)
        self.assertEqual(breaks, 2)
        self.assertTrue(np.all(np.isnan(points[1:])))
        np.testing.assert_equal(enu(np.array([[10, 20, -1], [11, 22, -3]]), points[0], True), [[0, 0, 0], [2, 1, 2]])

    def test_empty_position_is_unknown(self):
        times, points, breaks = position_samples({'timestamp': [], 'x': [], 'y': [], 'z': []}, 0, True)
        self.assertEqual(len(times), 0); self.assertEqual(breaks, 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)

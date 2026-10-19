"""Review regressions. Every board action is mocked; no ROS or SSH is started."""
import copy
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

TOOL = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL))
import server
import wb_board
import wb_ssh
import wb_status


class ReviewTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.profile = patch.object(server, 'PROFILE_DIR', self.temp.name)
        self.profile.start()
        self.addCleanup(self.profile.stop)
        self.wb = server.Workbench(copy.deepcopy(wb_board.load_config()), {'transport': 'ssh'})
        self.wb.board = Mock()
        self.wb.sessions = Mock()
        self.wb.sessions.get.return_value = None
        self.wb.sessions.sessions = {}
        self.wb.sessions.open.return_value.snapshot.return_value = {}

    def test_real_flight_from_actual_browser_payload(self):
        for gid in ('site1', 'site2', 'site4', 'site5'):
            group = next(g for g in self.wb.config['groups'] if g['id'] == gid)
            command, _ = wb_board.build_group_command(self.wb.config, group, 'flight')
            result = self.wb.start_trial(dict(group_id=gid, mode='flight', real_release=True,
                                             confirm='实投', expected_body=command))
            self.assertTrue(result['ok'])
            self.assertEqual(result['trial']['release'], 'real')

    def test_site_real_release_cannot_be_bypassed_by_false_flag(self):
        with self.assertRaisesRegex(ValueError, '实投'):
            self.wb.start_trial(dict(group_id='site5', route='site', mode='flight',
                                     confirm='启动试飞', real_release=False))

    def test_capture_speed_preview_and_flight(self):
        for mode in ('preview', 'flight'):
            for speed in (.5, 1., 1.2):
                result = self.wb.start_trial(dict(group_id='site6', mode=mode,
                    capture_speed=speed, confirm='启动试飞'))
                self.assertIn('09_high_speed_capture/start.sh %s' % mode, result['trial']['command'])
                self.assertIn('--capture-speed %s' % speed, result['trial']['command'])

    def test_preview_never_selects_real_release(self):
        group = next(g for g in self.wb.config['groups'] if g['id'] == 'mod08')
        command, _ = wb_board.build_group_command(self.wb.config, group, 'preview', real_release=True)
        self.assertIn('/start.sh preview', command)
        self.assertNotIn('start_real', command)
        result = self.wb.start_trial(dict(group_id='site5', mode='preview', real_release=True))
        self.assertEqual(result['trial']['release'], 'none')

    def test_config_check_no_flight_confirmation(self):
        result = self.wb.start_trial(dict(group_id='site5', mode='flight', check_config=True))
        self.assertTrue(result['trial']['command'].endswith('--check-config'))
        self.assertEqual(result['trial']['release'], 'none')

    def test_concurrent_trial_requests_open_only_one_session(self):
        def opened(*args, **kwargs):
            self.wb.sessions.get.return_value = Mock(state='running')
            return Mock(snapshot=lambda: {})
        self.wb.sessions.open.side_effect = opened
        results = []
        def start():
            try:
                results.append(self.wb.start_trial(dict(group_id='site3', mode='flight', confirm='启动试飞')))
            except ValueError:
                results.append(None)
        threads = [threading.Thread(target=start) for _ in range(2)]
        for t in threads: t.start()
        for t in threads: t.join()
        self.assertEqual(sum(r is not None for r in results), 1)
        self.wb.sessions.open.assert_called_once()

    def test_failed_device_keeps_next_step_pending(self):
        terms = self.wb.config['terminals'][:2]
        self.wb.orchestration['steps'] = [dict(id=t['id'], state='pending') for t in terms]
        self.wb.board_state['preflight'] = {'leftovers': {}}
        with patch.object(self.wb, 'ensure_probe'), patch.object(self.wb, '_wait_ready', return_value=(False, 'timeout')):
            self.wb._run_orchestration(terms)
        self.assertEqual([s['state'] for s in self.wb.orchestration['steps']], ['failed', 'pending'])
        self.wb.sessions.open.assert_called_once()

    def test_reuse_camera_and_lidar_from_external_terminals(self):
        terms = [t for t in self.wb.config['terminals'] if t['id'] in ('lidar', 'camera')]
        self.wb.orchestration['steps'] = [dict(id=t['id'], state='pending') for t in terms]
        self.wb.board_state['preflight'] = {'leftovers': {}}
        self.wb.telemetry = {'at': time.time(), 'master': True, 'topics': {
            '/livox/lidar': dict(age=.1, count=5), '/camera/image_raw': dict(age=.1, count=5)}}
        with patch.object(self.wb, 'ensure_probe'), patch.object(self.wb, '_wait_ready', return_value=(True, 'fresh')):
            self.wb._run_orchestration(terms)
        self.wb.sessions.open.assert_not_called()

    def test_stale_telemetry_not_ready(self):
        self.wb.telemetry = {'at': time.time()-60, 'master': True}
        self.assertFalse(self.wb._wait_ready(dict(kind='ros_master'), timeout=.01)[0])
        self.assertFalse(wb_status.ready_check('mavros_connected', {}, {'state': {'connected': True},
            'topics': {'/mavros/state': {'age': 60}}})[0])

    def test_no_master_wait_before_roscore(self):
        terms = self.wb.config['terminals'][:1]
        self.wb.orchestration['steps'] = [dict(id='roscore', state='pending')]
        self.wb.board_state['preflight'] = {'leftovers': {}}
        with patch.object(self.wb, 'ensure_probe'), patch.object(self.wb, '_wait_ready', return_value=(True, 'ok')), \
                patch.object(self.wb, '_wait_for') as waiting:
            self.wb._run_orchestration(terms)
        waiting.assert_not_called()

    def test_no_host_switch_while_session_running(self):
        self.wb.sessions.sessions = {'probe': Mock(state='running')}
        original = self.wb.target.host
        with self.assertRaises(ValueError): self.wb.update_config({'host': 'test@other'})
        self.assertEqual(self.wb.target.host, original)

    def test_unsaved_password_stays_in_memory(self):
        with patch.object(self.wb, '_save_profile') as saved:
            self.wb.update_config({'password': 'test-only', 'save_password': False})
        saved.assert_not_called()
        self.assertEqual(self.wb.target.password, 'test-only')

    def test_explicit_password_save_remains_supported(self):
        with patch.object(self.wb, '_save_profile') as saved:
            self.wb.update_config({'password': 'test-only', 'save_password': True})
        saved.assert_called_once_with(password=True)

    def test_auto_mission_false_is_error(self):
        tracker = wb_status.StageTracker()
        tracker.feed('AUTO_MISSION_START False rejected\n')
        self.assertEqual(tracker.timeline[-1]['level'], 'error')
        self.assertTrue(any(a['level'] == 'error' for a in tracker.alerts))

    def test_dead_supervisor_cannot_leave_ready_lit(self):
        self.wb.stage.name = 'READY'
        self.wb._on_state('trial',dict(state='failed',exit_code=1))
        self.assertEqual(self.wb.stage.name,'FAILED')
        self.wb.stage.name = 'READY'
        self.wb._on_state('trial',dict(state='exited',exit_code=0))
        self.assertEqual(self.wb.stage.name,'STOPPED')

    def test_manual_start_rejects_auto_preview_and_disarmed(self):
        self.wb.sessions.get.return_value = Mock(state='running')
        for mode, stage, auto in [('preview', 'READY', None), ('flight', 'DISARMED', None),
                                 ('flight', 'IN_FLIGHT', 'manual arm -> auto mission')]:
            self.wb.trial.update(mode=mode)
            self.wb.stage.name = stage
            self.wb.stage.auto_sequence = auto
            with self.assertRaises(ValueError): self.wb.mission_start({'confirm': '启动任务'})
        self.wb.board.run.assert_not_called()

    def test_manual_start_only_once(self):
        self.wb.sessions.get.return_value = Mock(state='running')
        self.wb.trial['mode'] = 'flight'
        self.wb.stage.name = 'READY'
        self.wb.board.run.return_value = (0, 'success: true')
        self.assertTrue(self.wb.mission_start({'confirm': '启动任务'})['ok'])
        with self.assertRaises(ValueError): self.wb.mission_start({'confirm': '启动任务'})
        self.wb.board.run.assert_called_once()

    def test_shutdown_supervisor_before_devices(self):
        manager = wb_ssh.SessionManager(self.temp.name)
        manager.sessions = dict(roscore=Mock(), mavros=Mock(), trial=Mock(), probe=Mock())
        with patch.object(manager, 'close') as close:
            manager.close_all()
        self.assertEqual([c.args[0] for c in close.call_args_list], ['trial', 'probe', 'mavros', 'roscore'])
        self.assertEqual(close.call_args_list[0].kwargs['wait'], 120.)


if __name__ == '__main__':
    unittest.main(verbosity=2)

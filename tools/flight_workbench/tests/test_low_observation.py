"""Low observation integration; all process, ROS and SSH actions mocked."""
import copy
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

TOOL = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL))
import server
import wb_board
import wb_ssh
import wb_status


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        p = patch.object(server, 'PROFILE_DIR', self.tmp.name)
        p.start()
        self.addCleanup(p.stop)
        self.config = copy.deepcopy(wb_board.load_config())
        self.groups = [g for g in self.config['groups'] if wb_board.is_observation(g)]
        self.wb = server.Workbench(self.config, {'transport': 'ssh'})
        self.wb.board = Mock()
        self.wb.sessions = Mock()
        self.wb.sessions.get.return_value = None
        self.wb.sessions.open.return_value.snapshot.return_value = {}

    def start(self, **kwargs):
        return self.wb.start_trial(dict(group_id='observation_hover', mode='flight',
                                       confirm='启动试飞', **kwargs))

    def active(self):
        self.start()
        session = Mock(state='running', graceful_only=True)
        self.wb.sessions.get.return_value = session
        return session

    def test_card_order_and_existing_profiles(self):
        self.assertEqual([g['profile'] for g in self.groups], ['hover','forward','square'])
        self.assertTrue(all(g['release']=='none' and not g['needs_servo'] for g in self.groups))
        self.assertTrue(all(not g.get('survey_patterns') for g in self.groups))

    def test_actual_cli_preview_has_no_ros_or_hardware(self):
        env = dict(os.environ, BOARD_PYTHON=sys.executable)
        for g in self.groups:
            cmd, _ = wb_board.build_group_command(self.config, g, 'preview')
            result = subprocess.run(cmd.split(), cwd=TOOL.parents[1], env=env,
                                    text=True, capture_output=True, check=True)
            value = json.loads(result.stdout)
            self.assertEqual(value['profile'], g['profile'])
            self.assertEqual(value['fc_agl'], .6)
            self.assertAlmostEqual(value['relative_local_z_target'], .38)
            for key in ('camera','yolo','planner','servo','auto_arm','auto_mode'):
                self.assertFalse(value[key])

    def test_check_config_uses_preview_and_no_unsupported_flags(self):
        for g in self.groups:
            command, _ = wb_board.build_group_command(self.config, g, 'flight', check_config=True)
            self.assertEqual(command, 'bash deployment/low_hover_observation/start.sh preview '+g['profile'])

    def test_reject_trial_options_and_route_override(self):
        for kwargs in (dict(real_release=True),dict(motion_optimized=True),dict(capture_speed=.5),
                       dict(survey_pattern='rectangle'),dict(resume_survey='on'),
                       dict(capture_lighting='dim'),dict(site_geometry={}),dict(route='module')):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                wb_board.build_group_command(self.config, self.groups[0], 'flight', **kwargs)

    def test_confirmation_and_parity_before_session_open(self):
        for body in (dict(group_id='observation_hover',mode='flight'),
                     dict(group_id='observation_hover',mode='flight',confirm='启动试飞',expected_body='wrong')):
            with self.assertRaises(ValueError): self.wb.start_trial(body)
        self.wb.sessions.open.assert_not_called()

    def test_flight_uses_trial_terminal_and_protected_session(self):
        self.start()
        self.assertEqual(self.wb.sessions.open.call_args.args[0], 'trial')
        self.assertTrue(self.wb.sessions.open.call_args.kwargs['graceful_only'])
        self.assertTrue(self.wb.stage.observation_mode)

    def test_repeat_blocked_until_process_exit(self):
        session = self.active()
        self.wb.stop_trial()
        session.send_key.assert_called_once_with('C-c')
        with self.assertRaises(ValueError): self.start()
        session.state = 'exited'
        self.start()
        self.assertEqual(self.wb.sessions.open.call_count, 2)

    def test_full_stop_disconnect_and_dependency_close_rejected(self):
        self.active()
        for action in (self.wb.stop_all, self.wb.disconnect):
            with self.assertRaisesRegex(ValueError, 'OBSERVATION_CLOSED'): action()
        for sid in ('roscore','mavros','lidar','observation_localization'):
            with self.assertRaises(ValueError): self.wb.protect_observation_dependencies(sid)
        self.wb.sessions.close_all.assert_not_called()
        self.wb.protect_observation_dependencies('trial')

    def test_mission_service_never_used(self):
        self.active()
        with self.assertRaises(ValueError): self.wb.mission_start({'confirm':'启动任务'})
        self.wb.board.run.assert_not_called()

    def test_observation_device_order_no_camera_servo(self):
        with patch.object(server.threading, 'Thread'):
            result = self.wb.open_all_devices(dict(confirm='启动设备',group_id='observation_hover'))
        self.assertEqual([s['id'] for s in result['orchestration']['steps']],
                         ['roscore','mavros','lidar','observation_localization'])
        self.wb.orchestration['running'] = False
        with self.assertRaises(ValueError):
            self.wb.open_all_devices(dict(confirm='启动设备',group_id='observation_hover',include_servo=True))

    def test_generic_devices_exclude_observation_localization(self):
        with patch.object(server.threading, 'Thread'):
            result = self.wb.open_all_devices(dict(confirm='启动设备',group_id='site1'))
        self.assertNotIn('observation_localization', [s['id'] for s in result['orchestration']['steps']])

    def test_ready_marker_not_status_ready(self):
        self.start()
        self.wb.stage.feed('WAIT_GROUND_REFERENCE {}\n'+json.dumps(dict(stage='READY',armed=False))+'\n')
        self.assertEqual(self.wb.stage.name,'INITIALIZING')
        self.wb.stage.feed('READY_FOR_MANUAL_ARM_AND_OFFBOARD (pilot checks still apply)\n')
        self.assertEqual(self.wb.stage.name,'READY')
        self.assertIsNone(self.wb.stage.run_dir)

    def test_topic_json_updates_hold_and_takeover_no_mission_success(self):
        self.active()
        for stage in ('RUN','FINISHED_HOVER','HOLD_FOR_PILOT','TAKEN_OVER'):
            self.wb._feed_probe(json.dumps(dict(observe=dict(low_hover=dict(
                stage=stage,reason='diagnostic',armed=True,mode='OFFBOARD'))))+'\n')
            self.assertEqual(self.wb.stage.phase,stage)
        self.assertIsNone(self.wb.stage.outcome)
        self.assertIn('落地上锁',self.wb.stage.pilot_action)
        self.wb.stage.feed('OBSERVATION_CLOSED /board/logs/low_hover_hover_test\n')
        self.assertEqual(self.wb.stage.name,'STOPPED')

    def test_ready_to_hold_revokes_ready_and_finished_stays_live(self):
        self.start()
        self.wb.stage.feed('READY_FOR_MANUAL_ARM_AND_OFFBOARD\n')
        self.wb.stage.feed_line(json.dumps(dict(stage='HOLD_FOR_PILOT',armed=False,reason='reference_lost')))
        self.assertNotEqual(self.wb.stage.name, 'READY')
        self.wb.stage.feed('READY_FOR_MANUAL_ARM_AND_OFFBOARD\n')
        self.assertNotEqual(self.wb.stage.name, 'READY')
        self.wb.stage.feed_line(json.dumps(dict(stage='FINISHED_HOVER',armed=True)))
        self.assertEqual(self.wb.stage.name, 'IN_FLIGHT')
        self.assertIsNone(self.wb.stage.outcome)

    def test_http_close_and_key_cannot_bypass_dependency_guard(self):
        self.active()
        for route in ('/api/session/close','/api/session/key'):
            for sid in ('roscore','mavros','lidar','observation_localization'):
                handler = server.Handler.__new__(server.Handler)
                handler.workbench = self.wb
                handler.path = route
                handler._body = Mock(return_value={'id':sid,'key':'C-c'})
                handler._json = Mock()
                handler._error = Mock()
                handler.do_POST()
                handler._error.assert_called_once()
                handler._json.assert_not_called()
        self.wb.sessions.close.assert_not_called()

    def test_preflight_checks_actual_observation_entry(self):
        board = wb_board.BoardClient(self.config, self.wb.target)
        board.run = Mock(return_value=(0,''))
        board.preflight()
        script = board.run.call_args.args[0]
        self.assertIn('/deployment/low_hover_observation/start.sh', script)
        self.assertIn('/deployment/low_hover_observation/profiles.yaml',script)
        self.assertNotIn('board_trials_4x4/low_hover_observation',script)

    def test_session_close_never_escalates_observation(self):
        session = wb_ssh.Session('trial','test','mock',self.wb.target,graceful_only=True)
        session.child = Mock()
        session.child.isalive.return_value = True
        session.state = 'running'
        session.close(wait=0)
        session.child.terminate.assert_not_called()
        self.assertEqual(session.state,'running')

    def test_manager_close_all_and_direct_dependency_close_keep_links(self):
        manager = wb_ssh.SessionManager(self.tmp.name)
        trial = Mock(state='running',graceful_only=True)
        mavros = Mock(state='running',graceful_only=False)
        manager.sessions = {'trial':trial,'mavros':mavros}
        with self.assertRaises(ValueError): manager.close('mavros')
        with self.assertRaises(ValueError): manager.close_all()
        mavros.close.assert_not_called()
        trial.state = 'exited'
        manager.close_all()
        mavros.close.assert_called_once()


if __name__ == '__main__': unittest.main()

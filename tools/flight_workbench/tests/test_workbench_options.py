"""Exercise launch contracts and observed mission outcomes without a board."""
import copy
import json
import shlex
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import wb_board
import wb_status
import test_review


class LaunchOptionsTest(unittest.TestCase):
    def setUp(self):
        self.config = copy.deepcopy(wb_board.load_config())
        self.groups = {g['id']: g for g in self.config['groups']}

    def command(self, gid, **kwargs):
        return wb_board.build_group_command(self.config, self.groups[gid], 'flight', **kwargs)[0]

    def test_delivery_choice_and_fixed_mock_alias(self):
        for gid in ('site1', 'site2', 'site4', 'site5', 'mod08'):
            self.assertIn('/start.sh flight', self.command(gid, real_release=False))
            self.assertIn('/start_real.sh', self.command(gid, real_release=True))
        with self.assertRaises(ValueError):
            self.command('mod06mock', real_release=True)

    def test_all_groups_preserve_defaults_and_accept_explicit_motion(self):
        for gid in self.groups:
            default = self.command(gid)
            self.assertNotIn('--motion-optimized', default)
            self.assertNotIn('--resume-survey', default)
            self.assertNotIn('--survey-pattern', default)
            if wb_board.is_observation(self.groups[gid]):
                with self.assertRaises(ValueError): self.command(gid, motion_optimized=True)
            else:
                self.assertIn('--motion-optimized', self.command(gid, motion_optimized=True))

    def test_pattern_and_resume_capabilities(self):
        for gid in ('site3', 'site4', 'site5', 'site6', 'mod08', 'mod06mock'):
            self.assertIn('--survey-pattern snake3', self.command(gid, survey_pattern='snake3'))
        for gid in ('site5', 'mod08', 'mod06mock'):
            for value in ('on', 'off'):
                self.assertIn('--resume-survey '+value, self.command(gid, resume_survey=value))
        for gid in ('site1', 'mod03'):
            with self.assertRaises(ValueError):
                self.command(gid, survey_pattern='snake2')
        with self.assertRaises(ValueError):
            self.command('site4', resume_survey='on')

    def test_dynamic_site_dir_is_one_shell_argument(self):
        self.config['connection']['site_dir'] = "deployment/field test'; echo INVALID; #"
        for gid, name in [('site5', 'test_area.yaml'), ('mod03', 'h_landing_test_area.yaml'),
                          ('mod04', 'corridor_landing_test_area.yaml'), ('mod08', 'full_mission_test_area.yaml')]:
            argv = shlex.split(self.command(gid))
            self.assertEqual(argv[argv.index('--site-config')+1],
                             self.config['connection']['site_dir']+'/'+name)
            self.assertNotIn('echo', argv)

    def test_capture_labels_and_invalid_inputs(self):
        self.assertIn('--capture-speed 0.5 --capture-lighting dim',
                      self.command('site6', capture_speed=.5, capture_lighting='dim'))
        for kwargs in (dict(capture_speed=True), dict(capture_speed=2),
                       dict(capture_lighting=''), dict(capture_lighting='dark'),
                       dict(motion_optimized='false'), dict(resume_survey=True)):
            with self.assertRaises(ValueError):
                self.command('site6', **kwargs)
        with self.assertRaises(ValueError):
            self.command('site5', capture_lighting='dim')

    def test_configured_observe_topics_survive_shell_quoting(self):
        board = wb_board.BoardClient(self.config, None)
        argv = shlex.split(board.probe_command())
        mapping = json.loads(argv[argv.index('--observe-topics')+1])
        self.assertEqual(mapping, self.config['probe']['observe_topics'])
        self.assertNotIn('image', ' '.join(mapping.values()))
        self.assertIn('&& exec', board.probe_command())


class LaunchRequestTest(unittest.TestCase):
    setUp = test_review.ReviewTests.setUp

    def test_selected_options_reach_command_and_report(self):
        result = self.wb.start_trial(dict(group_id='site5', mode='flight', real_release=False,
            confirm='启动试飞', motion_optimized=True, survey_pattern='snake2', resume_survey='off'))
        self.assertEqual(result['trial']['release'], 'mock')
        self.assertIn('--motion-optimized --survey-pattern snake2 --resume-survey off', result['trial']['command'])
        self.assertIn('snake2', self.wb.stage.report(trial=result['trial']))

    def test_entire_mission_real_choice_still_requires_real_confirmation(self):
        with self.assertRaisesRegex(ValueError, '实投'):
            self.wb.start_trial(dict(group_id='mod08', mode='flight', real_release=True, confirm='启动试飞'))
        self.wb.sessions.open.assert_not_called()

    def test_config_check_carries_options_without_flight_ack(self):
        result = self.wb.start_trial(dict(group_id='site5', mode='preview', check_config=True,
                                         real_release=False, resume_survey='on'))
        self.assertTrue(result['trial']['command'].endswith('--resume-survey on --check-config'))
        self.assertEqual(result['trial']['release'], 'none')

    def test_parity_and_type_rejections_execute_nothing(self):
        for body in [dict(check_config='true'), dict(motion_optimized='false'),
                     dict(real_release='false'), dict(expected_body='bash WRONG')]:
            with self.assertRaises(ValueError):
                self.wb.start_trial(dict(group_id='site5', mode='preview', **body))
        self.wb.sessions.open.assert_not_called()

    def test_external_nodes_are_observed_without_starting_trial(self):
        tel = dict(master=True, state=dict(armed=True, mode='OFFBOARD'),
                   observe=dict(low_hover=dict(profile='hover', stage='FINISHED_HOVER')),
                   topics={})
        self.wb._feed_probe(json.dumps(tel)+'\n')
        snapshot = self.wb.snapshot()
        self.assertEqual(snapshot['telemetry']['observe']['low_hover']['profile'], 'hover')
        self.assertEqual(snapshot['stage']['name'], 'IDLE')
        self.assertEqual([p['id'] for p in snapshot['observation']['profiles']], ['hover','forward','square'])
        self.wb.sessions.open.assert_not_called()
        self.wb.board.run.assert_not_called()


class MissionObservationTest(unittest.TestCase):
    def setUp(self):
        self.tracker = wb_status.StageTracker()
        self.tracker.set_stage('READY', at=time.time()-1)

    def row(self, phase='HIGH_VIEW_SEARCH', armed=True, mode='OFFBOARD', age=.1):
        return dict(at=time.time(), master=True, state=dict(armed=armed, mode=mode),
                    mission=dict(phase=phase), topics={
                        '/mavros/state': dict(age=age), '/navigation/mission_status': dict(age=age),
                        '/custom/hover': dict(age=age)})

    def test_manual_offboard_instruction_and_handoff(self):
        self.tracker.observe_telemetry(self.row(armed=False, mode='POSCTL'))
        self.assertIn('人工解锁', self.tracker.pilot_action)
        self.tracker.observe_telemetry(self.row(mode='POSCTL'))
        self.assertIn('人工拨入 OFFBOARD', self.tracker.pilot_action)
        row = self.row()
        row['terminal_hover'] = dict(stage='PILOT_HANDOFF')
        self.tracker.observe_telemetry(row, '/custom/hover')
        self.assertIn('接管落地', self.tracker.pilot_action)
        self.assertIsNone(self.tracker.outcome)

    def test_abort_cannot_be_success_after_disarm(self):
        self.tracker.observe_telemetry(self.row(phase='ABORTED'))
        self.assertEqual(self.tracker.outcome, 'aborted')
        self.tracker.observe_telemetry(self.row(phase='COMPLETE', armed=False))
        self.assertEqual(self.tracker.name, 'DISARMED')
        self.assertEqual(self.tracker.outcome, 'aborted')
        self.assertEqual(sum('ABORTED' in a['text'] for a in self.tracker.alerts), 1)

    def test_complete_requires_explicit_fresh_terminal_phase(self):
        row = self.row(phase='COMPLETE', age=10)
        self.tracker.observe_telemetry(row)
        self.assertIsNone(self.tracker.outcome)
        row = self.row()
        row['mission']['done'] = True
        self.tracker.observe_telemetry(row)
        self.assertIsNone(self.tracker.outcome)
        self.tracker.observe_telemetry(self.row(phase='COMPLETE'))
        self.assertEqual(self.tracker.outcome, 'complete')
        self.assertEqual(self.tracker.name, 'IN_FLIGHT')

    def test_no_master_or_old_run_payload_is_unknown(self):
        row = self.row(phase='COMPLETE')
        row['master'] = False
        self.tracker.observe_telemetry(row)
        self.assertIsNone(self.tracker.outcome)
        row = self.row(phase='COMPLETE')
        self.tracker.ready_at = time.time()+1
        self.tracker.observe_telemetry(row)
        self.assertIsNone(self.tracker.outcome)

    def test_stdout_abort_and_cancel_markers(self):
        self.tracker.feed("FLIGHT_STATUS {'armed': True, 'mode': 'OFFBOARD', 'phase': 'ABORTED', 'reason': 'pose_jump'}\n")
        self.assertEqual(self.tracker.outcome, 'aborted')
        self.tracker.feed('AUTO_SEQUENCE_CANCELLED_PILOT_MODE_CHANGE\n')
        self.tracker.observe_telemetry(self.row(mode='POSCTL'))
        self.assertIn('自动任务已取消', self.tracker.pilot_action)


if __name__ == '__main__':
    unittest.main(verbosity=2)

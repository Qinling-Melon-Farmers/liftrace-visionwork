"""Independent competition commands and existing safety gates; no board calls."""
import copy
import shlex
import sys
from pathlib import Path
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import wb_board
import test_review


class CompetitionCommandTests(unittest.TestCase):
    def setUp(self):
        self.config = copy.deepcopy(wb_board.load_config())
        self.group = next(g for g in self.config['groups'] if g['id'] == 'competition')

    def command(self, **options):
        return wb_board.build_group_command(self.config, self.group, 'preview', **options)[0]

    def test_independent_default_is_not_test_site(self):
        command = self.command()
        self.assertEqual(command, 'bash deployment/competition/start.sh preview --site-config deployment/competition/field.example.yaml')
        self.assertNotIn('board_trials', command)
        self.assertNotIn('site_20260928', command)
        self.assertNotIn('--motion-', command)
        self.assertNotIn('--obstacle-', command)
        self.assertEqual(self.group['release_options'], ['real'])

    def test_on_off_and_inheritance_in_check_preview_flight(self):
        for mode in ('preview', 'flight'):
            for check in (False, True):
                for motion in (None, 'on', 'off'):
                    for columns in (None, 'on', 'off'):
                        command, _ = wb_board.build_group_command(self.config, self.group, mode,
                            real_release=True, check_config=check,
                            motion_optimization=motion, obstacle_columns=columns)
                        argv = shlex.split(command)
                        self.assertEqual(argv[2], 'preview' if check else mode)
                        for flag, value in (('--motion-optimization', motion), ('--obstacle-columns', columns)):
                            self.assertEqual(flag in argv, value is not None)
                            if value is not None:
                                self.assertEqual(argv[argv.index(flag)+1], value)
                        self.assertEqual('--check-config' in argv, check)

    def test_resume_does_not_toggle_motion(self):
        for resume in ('on','off'):
            argv=shlex.split(self.command(resume_survey=resume))
            self.assertEqual(argv[argv.index('--resume-survey')+1],resume)
            self.assertNotIn('--motion-optimization',argv)

    def test_untrusted_config_path_remains_single_argument(self):
        path = "deployment/test field'; echo INVALID; #.yaml"
        argv = shlex.split(self.command(competition_config=path))
        self.assertEqual(argv[argv.index('--site-config')+1], path)
        self.assertNotIn('echo', argv)

    def test_reject_fake_options_and_mock_flight(self):
        for options in (dict(motion_optimization=True), dict(resume_survey=True), dict(obstacle_columns='true'),
                        dict(motion_optimized=True), dict(competition_config=''),
                        dict(competition_config='field\n.yaml'), dict(route='module'),
                        dict(survey_pattern='snake3'), dict(site_geometry={})):
            with self.assertRaises(ValueError):
                self.command(**options)
        with self.assertRaises(ValueError):
            wb_board.build_group_command(self.config, self.group, 'flight', real_release=False)
        site = next(g for g in self.config['groups'] if g['id'] == 'site5')
        with self.assertRaises(ValueError):
            wb_board.build_group_command(self.config, site, 'preview', obstacle_columns='off')

    def test_remote_paths_are_posix_on_every_host(self):
        self.config['connection']['board_root'] = '/home/operator/root space'
        wrapped = wb_board.terminal_wrapped_command(self.config, 'true')
        self.assertNotIn('\\', wrapped)
        self.assertEqual(wb_board.BoardClient(self.config, None).abs_path('logs/a.txt'), '/home/operator/root space/logs/a.txt')


class CompetitionRequestTests(unittest.TestCase):
    setUp = test_review.ReviewTests.setUp

    def test_real_confirmation_and_mutex_preserved(self):
        for options in (dict(real_release=False, confirm='启动试飞'),
                        dict(real_release=True, confirm='启动试飞')):
            with self.assertRaises(ValueError):
                self.wb.start_trial(dict(group_id='competition', mode='flight', **options))
        self.wb.sessions.open.assert_not_called()
        self.wb.sessions.get.return_value = Mock(state='running')
        with self.assertRaises(ValueError):
            self.wb.start_trial(dict(group_id='competition', mode='flight', real_release=True, confirm='实投'))
        self.wb.sessions.open.assert_not_called()

    def test_selected_overrides_reach_actual_command_and_trial(self):
        body = dict(group_id='competition', mode='flight', real_release=True, confirm='实投',
                    motion_optimization='off', resume_survey='on', obstacle_columns='on')
        body['expected_body'] = self.wb.trial_command(body)['body']
        result = self.wb.start_trial(body)
        self.assertEqual(result['trial']['command'], body['expected_body'])
        self.assertEqual(result['trial']['motion_optimization'], 'off')
        self.assertEqual(result['trial']['resume_survey'], 'on')
        self.assertEqual(result['trial']['obstacle_columns'], 'on')
        self.assertEqual(result['trial']['release'], 'real')

    def test_check_does_not_require_flight_or_real_confirmation(self):
        result = self.wb.start_trial(dict(group_id='competition', mode='flight', check_config=True,
                                         obstacle_columns='off'))
        self.assertEqual(result['trial']['release'], 'none')
        self.assertIn('start.sh preview', result['trial']['command'])
        self.assertTrue(result['trial']['command'].endswith('--check-config'))

    def test_option_drift_refused_before_execution(self):
        with self.assertRaises(ValueError):
            self.wb.start_trial(dict(group_id='competition', mode='preview', obstacle_columns='off',
                                    expected_body='bash WRONG'))
        self.wb.sessions.open.assert_not_called()


if __name__ == '__main__':
    unittest.main(verbosity=2)

"""OFFBOARD candidate is explicit, quoted, and shared by preview/check paths."""
import copy
from pathlib import Path
import shlex
import sys
import unittest

sys.path[:0] = [str(Path(__file__).resolve().parents[1]), str(Path(__file__).resolve().parent)]
import wb_board
import test_review

PATCH = 'deployment/competition/candidates/h_offboard_mode4.override.yaml'


class LandingCandidateCommandTests(unittest.TestCase):
    def setUp(self):
        self.config = copy.deepcopy(wb_board.load_config())
        self.groups = {group['id']:group for group in self.config['groups']}

    def test_h_and_formal_preview_paths_are_opt_in_and_quoted(self):
        path = "deployment/test H'; echo INVALID; #.yaml"
        for gid in ('mod03', 'mod04', 'mod08', 'competition'):
            for check in (False, True):
                command, _ = wb_board.build_group_command(self.config, self.groups[gid], 'preview',
                                                         check_config=check, landing_config=path)
                argv = shlex.split(command)
                self.assertEqual(argv[argv.index('--landing-config') + 1], path)
                self.assertNotIn('echo', argv)
                self.assertEqual('--check-config' in argv, check)
            command, _ = wb_board.build_group_command(self.config, self.groups[gid], 'preview')
            self.assertNotIn('--landing-config', command)

    def test_non_h_or_site_shortcut_and_invalid_path_rejected(self):
        for gid in ('site1', 'site3', 'mod06mock'):
            with self.assertRaises(ValueError):
                wb_board.build_group_command(self.config, self.groups[gid], 'preview', landing_config=PATCH)
        for path in ('', 'bad\n.yaml', 'bad\x00.yaml', True):
            with self.assertRaises(ValueError):
                wb_board.build_group_command(self.config, self.groups['mod03'], 'preview', landing_config=path)

    def test_geometry_recursion_retains_candidate(self):
        import wb_geometry
        from unittest.mock import patch
        with patch.object(wb_geometry, 'overlay_command', return_value=('logs/site.yaml', 'prepare', {})):
            command, _ = wb_board.build_group_command(self.config, self.groups['mod08'], 'preview',
                landing_config=PATCH, site_geometry={}, geometry_revision='test', check_config=True)
        self.assertIn('--landing-config '+PATCH, command)


class LandingCandidateRequestTests(unittest.TestCase):
    setUp = test_review.ReviewTests.setUp

    def test_server_preview_and_config_check_keep_same_candidate(self):
        body = dict(group_id='mod03', mode='preview', landing_config=PATCH, check_config=True)
        preview = self.wb.trial_command(body)
        self.wb.board.run.assert_not_called()
        self.wb.sessions.open.assert_not_called()
        result = self.wb.start_trial(dict(body, expected_body=preview['body']))
        self.assertEqual(result['trial']['command'], preview['body'])
        self.assertEqual(result['trial']['landing_config'], PATCH)
        self.assertEqual(result['trial']['release'], 'none')


if __name__ == '__main__':
    unittest.main()

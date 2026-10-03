"""Exercise the real site router with inert module scripts; never start ROS."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[5]
LEGACY_GROUPS = (
    ('1', 'single', '01_visual_interrupt'),
    ('2', 'multi', '05_low_multi'),
    ('3', 'memory', '07_memory_only'),
    ('4', 'revisit', '02_high_view_revisit'),
    ('5', 'priority', '06_high_priority'),
    ('6', 'capture', '09_high_speed_capture'),
)


class SiteEntrypointTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='site route test ')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.site = self.root / 'deployment/site_20260928'
        self.site.mkdir(parents=True)
        self.entry = self.site / 'start_test.sh'
        shutil.copyfile(ROOT / 'deployment/site_20260928/start_test.sh', self.entry)
        self.modules = self.root / 'deployment/board_trials_4x4'
        stub = '#!/usr/bin/env bash\nprintf \'%s\\0\' "${BASH_SOURCE[0]}" "$@"\n'
        for folder in [g[2] for g in LEGACY_GROUPS] + ['03_h_landing']:
            target = self.modules / folder
            target.mkdir(parents=True)
            for name in ('start.sh', 'start_real.sh'):
                (target / name).write_text(stub)

    def invoke(self, *args):
        return subprocess.run(['bash', str(self.entry), *args],
                              cwd=self.root, capture_output=True, timeout=5)

    def assert_route(self, args, folder, script, expected_args):
        result = self.invoke(*args)
        self.assertEqual(result.returncode, 0, result.stderr.decode())
        fields = result.stdout.split(b'\0')
        self.assertEqual(fields[-1], b'', result.stdout)
        # Legacy real-delivery routing prints an informational line first.
        selected = fields[0].decode().splitlines()[-1]
        self.assertEqual(Path(selected).resolve(), self.modules / folder / script)
        self.assertEqual([v.decode() for v in fields[1:-1]], expected_args)

    def test_h_aliases_default_preview_and_explicit_modes_use_plain_entry(self):
        for alias in ('h', 'landing'):
            for mode in (None, 'preview', 'flight'):
                with self.subTest(alias=alias, mode=mode):
                    args = [alias] + ([mode] if mode else [])
                    self.assert_route(args, '03_h_landing', 'start.sh',
                                      [mode or 'preview', '--site-config',
                                       str(self.site / 'h_landing_test_area.yaml')])

    def test_h_forwards_config_check_and_model_path_without_word_splitting(self):
        for mode in ('preview', 'flight'):
            with self.subTest(mode=mode):
                extra = ['--check-config', '--model', '/test models/h.rknn']
                self.assert_route(['h', mode, *extra], '03_h_landing', 'start.sh',
                                  [mode, *extra, '--site-config',
                                   str(self.site / 'h_landing_test_area.yaml')])

    def test_h_profile_is_the_final_site_config_argument(self):
        extra = ['--site-config', '/test/other.yaml', '--check-config']
        self.assert_route(['landing', 'preview', *extra], '03_h_landing', 'start.sh',
                          ['preview', *extra, '--site-config',
                           str(self.site / 'h_landing_test_area.yaml')])

    def test_existing_numbers_and_aliases_keep_preview_and_flight_routes(self):
        for number, alias, folder in LEGACY_GROUPS:
            for key in (number, alias):
                for mode in ('preview', 'flight'):
                    with self.subTest(key=key, mode=mode):
                        real = mode == 'flight' and number not in ('3', '6')
                        script = 'start_real.sh' if real else 'start.sh'
                        expected = ([] if real else [mode]) + [
                            '--site-config', str(self.site / 'test_area.yaml')]
                        self.assert_route([key, mode], folder, script, expected)

    def test_existing_groups_still_default_to_preview(self):
        for number, alias, folder in LEGACY_GROUPS:
            for key in (number, alias):
                with self.subTest(key=key):
                    self.assert_route([key], folder, 'start.sh',
                                      ['preview', '--site-config',
                                       str(self.site / 'test_area.yaml')])

    def test_capture_options_still_forward_unchanged(self):
        for alias in ('6', 'capture'):
            with self.subTest(alias=alias):
                extra = ['--capture-speed', '1.2', '--capture-lighting', 'dim',
                         '--check-config']
                self.assert_route([alias, 'preview', *extra],
                                  '09_high_speed_capture', 'start.sh',
                                  ['preview', '--site-config',
                                   str(self.site / 'test_area.yaml'), *extra])

    def test_first_five_groups_still_reject_extra_arguments(self):
        for number, alias, _ in LEGACY_GROUPS[:5]:
            for key in (number, alias):
                with self.subTest(key=key):
                    result = self.invoke(key, 'flight', '--check-config')
                    self.assertEqual(result.returncode, 2)
                    self.assertNotIn(b'\0', result.stdout)

    def test_unknown_or_missing_selector_does_not_dispatch(self):
        for args in ([], ['7'], ['03'], ['unknown']):
            with self.subTest(args=args):
                result = self.invoke(*args)
                self.assertEqual(result.returncode, 2)
                self.assertNotIn(b'\0', result.stdout)


if __name__ == '__main__':
    unittest.main(verbosity=2)

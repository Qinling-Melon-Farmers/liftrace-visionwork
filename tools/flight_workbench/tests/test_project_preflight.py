"""Project-specific preflight and config persistence; no SSH, ROS or board actions."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

TOOL = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL))
import server
import wb_board


class ProjectPreflightTests(unittest.TestCase):
    def setUp(self):
        self.config = copy.deepcopy(wb_board.load_config())
        self.config['connection'].update(
            board_root='/home/orangepi/competition root',
            env_script=wb_board.COMPETITION_ENV_SCRIPT,
            site_dir='deployment/competition')
        self.board = wb_board.BoardClient(self.config, None)

    def inspect(self, field=None, exists=True, exit_code=0):
        rows = ['SITECFG=OK' if exists else 'SITECFG=MISSING', 'COMPETITION=OK', 'DISK|10485760']
        rows += ['PROC|%s|0' % name for name in self.config['checks']['process_names']]
        rows += ['VERSION|' + json.dumps(dict(source=dict(head='abc123', dirty=False), resume_cli=True,
                    interfaces={name: dict(ok=True) for name in ('ReleaseAuthorization', 'ReleasePermission', 'ServoAction')}))]
        if field is not None:
            rows.append('FIELD|' + json.dumps(field))
        with patch.object(self.board, 'run', return_value=(exit_code, '\n'.join(rows))) as run:
            result = self.board.preflight()
        return result, run.call_args.args[0]

    def test_independent_project_does_not_require_trial_files(self):
        result, command = self.inspect(dict(ok=True, site_confirmed=False))
        subprocess.run(['bash', '-n'], input=command, text=True, check=True)
        self.assertNotIn('test_area.yaml', command)
        self.assertNotIn('GROUP|', command)
        self.assertIn('/deployment/competition/field.example.yaml', command)
        self.assertIn('competition_supervisor.py', command.split("<<'WBVERSION'", 1)[0])
        self.assertTrue(all(check['ok'] for check in result['checks']), result['checks'])
        self.assertEqual(result['groups'], {})
        self.assertEqual(result['field_template']['site_confirmed'], False)
        state = next(c for c in result['checks'] if c['name'].startswith('正赛模板状态'))
        self.assertIn('site_confirmed=false', state['detail'])
        self.assertIn('未验证', state['detail'])

    def test_confirmed_is_only_a_file_declaration(self):
        result, _ = self.inspect(dict(ok=True, site_confirmed=True))
        state = next(c for c in result['checks'] if c['name'].startswith('正赛模板状态'))
        self.assertIn('site_confirmed=true', state['detail'])
        self.assertIn('仅文件声明', state['detail'])
        self.assertNotIn('generation_ready', result)

    def test_missing_unreadable_or_invalid_template_is_not_success(self):
        for field in (None, dict(ok=False, detail='invalid YAML'), dict(ok=True, site_confirmed='true')):
            with self.subTest(field=field):
                result, _ = self.inspect(field)
                state = next(c for c in result['checks'] if c['name'].startswith('正赛模板状态'))
                self.assertFalse(state['ok'])
                self.assertIn('不能认定', state['detail'])
        result, _ = self.inspect(dict(ok=False, detail='missing'), exists=False)
        file_check = next(c for c in result['checks'] if c['name'] == '正赛场地模板文件')
        self.assertFalse(file_check['ok'])

    def test_trial_environment_preserves_trial_preflight(self):
        self.config['connection'].update(env_script='deployment/site_20260928/environment.sh',
                                         site_dir='deployment/custom_site')
        result, command = self.inspect()
        self.assertIn('/deployment/custom_site/test_area.yaml', command)
        self.assertIn('GROUP|08_full_mission|', command)
        self.assertNotIn("<<'WBFIELD'", command)
        self.assertIn('现场范围配置', [c['name'] for c in result['checks']])

    def test_field_reader_really_parses_file_without_environment(self):
        _, command = self.inspect()
        payload = command.split("<<'WBFIELD'\n", 1)[1].split('\nWBFIELD', 1)[0]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'field space.yaml'
            for text, expected in [('site_confirmed: false\n', False), ('site_confirmed: true\n', True),
                                   ('site_confirmed: "true"\n', None), ('site_confirmed: [\n', None),
                                   ('[]\n', None)]:
                with self.subTest(text=text):
                    path.write_text(text, encoding='utf-8')
                    output = subprocess.check_output([sys.executable, '-', str(path)], input=payload, text=True)
                    report = json.loads(output.partition('FIELD|')[2])
                    self.assertEqual(report['ok'], expected is not None)
                    if expected is not None:
                        self.assertEqual(report['site_confirmed'], expected)
            path.unlink()
            output = subprocess.check_output([sys.executable, '-', str(path)], input=payload, text=True)
            self.assertFalse(json.loads(output.partition('FIELD|')[2])['ok'])

    def test_competition_cli_is_read_from_independent_supervisor(self):
        _, command = self.inspect()
        payload = command.split("<<'WBVERSION'\n", 1)[1].split('\nWBVERSION', 1)[0]
        relative = 'patrol_uav_ws-patrol_planner/src/uav_mission/scripts/competition_supervisor.py'
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / relative
            path.parent.mkdir(parents=True)
            path.write_text("p.add_argument('--resume-survey')\n")
            output = subprocess.check_output([sys.executable, '-', str(root), relative], input=payload, text=True)
            self.assertTrue(json.loads(output.partition('VERSION|')[2])['resume_cli'])
            self.assertFalse((root / 'deployment/board_trials_4x4').exists())


class ProjectConfigTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        for module, name in ((server, 'PROFILE_DIR'), (wb_board, 'DEFAULT_PROFILE_DIR')):
            patched = patch.object(module, name, directory.name)
            patched.start()
            self.addCleanup(patched.stop)
        self.wb = server.Workbench(copy.deepcopy(wb_board.load_config()), {'transport': 'ssh'})
        self.switch = dict(host='orangepi@192.168.3.126',
                           board_root='/home/orangepi/liftrace_competition_20261008',
                           site_dir='deployment/competition', env_script=wb_board.COMPETITION_ENV_SCRIPT)

    def test_relative_assets_survive_switch_and_are_persisted(self):
        with patch.object(self.wb.board, 'run') as run:
            result = self.wb.update_config(self.switch)
        run.assert_not_called()
        saved = wb_board.load_profile()
        for key in ('model', 'metadata'):
            self.assertFalse(saved[key].startswith('/'))
            self.assertEqual(saved[key], result['connection'][key])
        self.assertEqual(saved['board_root'], self.switch['board_root'])
        self.assertFalse(self.wb.sessions.sessions)
        self.assertEqual(result['connection']['state'], 'unknown')

    def test_same_root_absolute_assets_are_saved_as_relative(self):
        body = dict(self.switch, model=self.switch['board_root'] + '/runtime_models/model.rknn',
                    metadata=self.switch['board_root'] + '/config/model.yaml')
        result = self.wb.update_config(body)
        self.assertEqual(result['connection']['model'], 'runtime_models/model.rknn')
        self.assertEqual(wb_board.load_profile()['metadata'], 'config/model.yaml')
        self.assertTrue(body['model'].startswith('/'))  # Caller request is unchanged.

    def test_other_root_and_traversal_are_rejected_without_partial_save(self):
        for key in ('model', 'metadata'):
            for path in ('/home/orangepi/liftrace_board_trials_20260928/old',
                         '../old', 'runtime_models/../../old', 'C:/old/model', 'config\\old'):
                with self.subTest(key=key, path=path):
                    before = copy.deepcopy(self.wb.config['connection'])
                    with self.assertRaises(ValueError):
                        self.wb.update_config(dict(self.switch, **{key: path}))
                    self.assertEqual(self.wb.config['connection'], before)
                    self.assertFalse(Path(wb_board.profile_path()).exists())

    def test_inherited_old_absolute_path_requires_explicit_root_local_replacement(self):
        self.wb.config['connection']['model'] = '/home/orangepi/liftrace_board_trials_20260928/runtime_models/old.rknn'
        with self.assertRaises(ValueError):
            self.wb.update_config(self.switch)
        result = self.wb.update_config(dict(self.switch, model='runtime_models/deployed.rknn'))
        self.assertEqual(result['connection']['model'], 'runtime_models/deployed.rknn')

    def test_running_session_still_blocks_project_switch(self):
        from types import SimpleNamespace
        self.wb.sessions.sessions['test'] = SimpleNamespace(state='running')
        with self.assertRaisesRegex(ValueError, '会话仍在运行'):
            self.wb.update_config(self.switch)
        self.assertFalse(Path(wb_board.profile_path()).exists())


if __name__ == '__main__':
    unittest.main(verbosity=2)

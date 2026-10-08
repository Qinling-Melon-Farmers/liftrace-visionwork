"""Explicit SSH key routing and persistence, using fixtures only."""
import copy
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import server
import wb_board
import wb_native_ssh
import wb_ssh


class IdentityFileTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        for module, name in ((server, 'PROFILE_DIR'), (wb_board, 'DEFAULT_PROFILE_DIR')):
            mocked = patch.object(module, name, self.directory.name)
            mocked.start()
            self.addCleanup(mocked.stop)

    def test_openssh_routes_explicit_key_as_single_argument(self):
        target = wb_ssh.Target('fixture@127.0.0.1', identity_file='/fixture/key with spaces')
        argv = target.base_argv()
        self.assertEqual(argv[argv.index('-i') + 1], '/fixture/key with spaces')
        self.assertIn('IdentitiesOnly=yes', argv)
        self.assertIn('BatchMode=yes', argv)
        self.assertIsNone(target.password)

    def test_default_and_local_transport_do_not_add_key_flags(self):
        self.assertNotIn('-i', wb_ssh.Target().base_argv())
        self.assertEqual(wb_ssh.Target(transport='local', identity_file='/fixture/key').base_argv(), ['bash'])

    def test_native_adapter_uses_explicit_key_or_default_agent(self):
        client = Mock()
        paramiko = SimpleNamespace(SSHClient=Mock(return_value=client), AutoAddPolicy=Mock(), RejectPolicy=Mock())
        with patch.dict(sys.modules, {'paramiko': paramiko}), \
                patch.object(wb_native_ssh.Path, 'home', return_value=Path(self.directory.name)):
            for path in ('/fixture/key with spaces', ''):
                wb_native_ssh.connect(wb_ssh.Target('fixture@127.0.0.1', identity_file=path))
                options = client.connect.call_args.kwargs
                self.assertEqual(options['key_filename'], str(Path(path).expanduser()) if path else None)
                self.assertEqual(options['look_for_keys'], not bool(path))
                self.assertEqual(options['allow_agent'], not bool(path))

    def test_api_saves_and_clears_key_without_connecting(self):
        workbench = server.Workbench(copy.deepcopy(wb_board.load_config()), {'transport': 'ssh'})
        with patch.object(workbench.board, 'run') as run:
            workbench.update_config(dict(identity_file='/fixture/key'))
            self.assertEqual(workbench.target.identity_file, '/fixture/key')
            self.assertEqual(workbench.snapshot()['connection']['identity_file'], '/fixture/key')
            self.assertEqual(wb_board.load_profile()['identity_file'], '/fixture/key')
            workbench.update_config(dict(identity_file=''))
        run.assert_not_called()
        self.assertEqual(workbench.target.identity_file, '')
        self.assertEqual(wb_board.load_profile()['identity_file'], '')
        self.assertFalse(workbench.sessions.sessions)

    def test_saved_key_is_loaded_and_empty_profile_overrides_config(self):
        config = wb_board.load_config()
        wb_board.apply_profile(config, dict(identity_file='/fixture/key'))
        workbench = server.Workbench(config, {'transport': 'ssh'})
        self.assertEqual(workbench.target.identity_file, '/fixture/key')
        wb_board.apply_profile(config, dict(identity_file=''))
        self.assertEqual(config['connection']['identity_file'], '')

    def test_running_session_rejects_key_change_and_clear(self):
        workbench = server.Workbench(copy.deepcopy(wb_board.load_config()), {'transport': 'ssh'})
        workbench.update_config(dict(identity_file='/fixture/key'))
        workbench.sessions.sessions['fixture'] = SimpleNamespace(state='running')
        for value in ('/fixture/newkey', ''):
            with self.assertRaises(ValueError):
                workbench.update_config(dict(identity_file=value))
        self.assertEqual(workbench.target.identity_file, '/fixture/key')
        self.assertEqual(wb_board.load_profile()['identity_file'], '/fixture/key')

    def test_invalid_key_field_does_not_partially_change_host(self):
        workbench = server.Workbench(copy.deepcopy(wb_board.load_config()), {'transport': 'ssh'})
        host = workbench.target.host
        for value in (None, 42, '/key\nother', '/key\x00other'):
            with self.assertRaises(ValueError):
                workbench.update_config(dict(identity_file=value, host='fixture@other'))
        self.assertEqual(workbench.target.host, host)
        self.assertFalse(Path(wb_board.profile_path()).exists())


if __name__ == '__main__':
    unittest.main(verbosity=2)

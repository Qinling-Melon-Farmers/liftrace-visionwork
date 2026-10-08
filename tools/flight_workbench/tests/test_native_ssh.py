"""Native Windows SSH against an in-process localhost fixture, never a board."""
import os
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import wb_ssh
import wb_native_ssh


@unittest.skipUnless(os.name == 'nt', 'Native Windows adapter is tested on Windows')
class NativeSSHTests(unittest.TestCase):
    def setUp(self):
        import paramiko
        self.paramiko = paramiko
        self.tmp = tempfile.TemporaryDirectory(prefix='wb-native-fixture-')
        self.addCleanup(self.tmp.cleanup)
        home = patch.object(wb_native_ssh.Path, 'home', return_value=Path(self.tmp.name))
        home.start()
        self.addCleanup(home.stop)
        self.key = paramiko.RSAKey.generate(2048)

    def fixture(self, payload=b'fixture\x00\xff\r\n\n', code=0, delay=0):
        pm = self.paramiko
        listener = socket.socket()
        listener.bind(('127.0.0.1', 0))
        listener.listen(1)
        listener.settimeout(5)
        port = listener.getsockname()[1]
        requested = []
        ended = threading.Event()
        errors = []
        class Fixture(pm.ServerInterface):
            def check_auth_password(self, username, password):
                return pm.AUTH_SUCCESSFUL if username == 'fixture' and password == 'fixture-only-secret' else pm.AUTH_FAILED
            def get_allowed_auths(self, username):
                return 'password'
            def check_channel_request(self, kind, chanid):
                return pm.OPEN_SUCCEEDED if kind == 'session' else pm.OPEN_FAILED_ADMINISTRATIVELY_PROHIBITED
            def check_channel_pty_request(self, *args):
                return True
            def check_channel_window_change_request(self, *args):
                return True
            def check_channel_exec_request(self, channel, command):
                requested.append(command)
                return True
        def serve():
            transport = None
            try:
                conn, _ = listener.accept()
                transport = pm.Transport(conn)
                transport.add_server_key(self.key)
                transport.start_server(server=Fixture())
                channel = transport.accept(5)
                deadline = time.monotonic() + 5
                while not requested and time.monotonic() < deadline:
                    time.sleep(.01)
                if channel is None or not requested:
                    raise AssertionError('fixture did not receive exec request')
                time.sleep(delay)
                channel.sendall(payload)
                if code:
                    channel.send_stderr(b'fixture failed fixture-only-secret')
                channel.send_exit_status(code)
                channel.close()
                time.sleep(.1)
            except (OSError, EOFError):
                pass  # Timeout tests intentionally close the local channel.
            except Exception as error:
                errors.append(error)
            finally:
                if transport:
                    transport.close()
                listener.close()
                ended.set()
        worker = threading.Thread(target=serve, daemon=True)
        worker.start()
        self.addCleanup(lambda: ended.wait(5))
        target = wb_ssh.Target('fixture@127.0.0.1', port=port,
            password='fixture-only-secret', ssh_options=['-o','StrictHostKeyChecking=accept-new'])
        return target, requested, errors

    def test_binary_bytes_and_host_key_persistence(self):
        payload = bytes(range(256)) * 1024
        target, requested, errors = self.fixture(payload)
        code, result = wb_ssh.run_bytes(target, 'fixture-binary', timeout=5)
        self.assertEqual(code, 0)
        self.assertEqual(result, payload)
        self.assertEqual(requested, [b'fixture-binary'])
        known = (Path(self.tmp.name) / '.ssh' / 'known_hosts').read_text()
        self.assertNotIn(target.password, known)
        self.assertIn('127.0.0.1', known)
        self.assertEqual(errors, [])

    def test_failure_discards_partial_stdout_and_redacts_error(self):
        target, _, _ = self.fixture(b'PARTIAL FILE', code=7)
        code, result = wb_ssh.run_bytes(target, 'fixture-failure', timeout=5)
        self.assertEqual(code, 7)
        self.assertNotIn(b'PARTIAL FILE', result)
        self.assertNotIn(target.password.encode(), result)

    def test_timeout_returns_no_partial_bytes(self):
        target, _, _ = self.fixture(b'PARTIAL FILE', delay=.5)
        code, result = wb_ssh.run_bytes(target, 'fixture-timeout', timeout=.1)
        self.assertEqual(code, 124)
        self.assertNotIn(b'PARTIAL FILE', result)

    def test_real_channel_session_redacts_secret_and_collects_exit(self):
        target, _, errors = self.fixture(b'fixture-only-secret\r\nSESSION_FIXTURE_DONE\r\n', delay=.1)
        output = []
        session = wb_ssh.Session('fixture', 'fixture', 'fixture-terminal', target,
            on_output=lambda sid, seq, data: output.append(data))
        session.start()
        try:
            session.resize(40, 120)
            session.thread.join(5)
            self.assertFalse(session.thread.is_alive())
            self.assertEqual(session.exit_code, 0)
            self.assertIn('SESSION_FIXTURE_DONE', ''.join(output))
            self.assertNotIn(target.password, ''.join(output))
            self.assertEqual(errors, [])
        finally:
            session.close(wait=.1)

    def test_native_local_cannot_invoke_bash(self):
        target = wb_ssh.Target(transport='local')
        code, output = wb_ssh.run_once(target, 'roscore', timeout=1)
        self.assertNotEqual(code, 0)
        self.assertIn('UI-only', output)


if __name__ == '__main__':
    unittest.main(verbosity=2)

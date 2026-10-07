"""No SSH/sudo/hardware: mock decisions and a real local PTY authentication fixture."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import server
import wb_board
import wb_ssh

FAKE_PASSWORD = 'mock-memory-only-secret'


class PromptTests(unittest.TestCase):
    def responder(self, allow=True, **kwargs):
        target = wb_ssh.Target(password=FAKE_PASSWORD, **kwargs)
        child = Mock()
        child.waitnoecho.return_value = True
        notes = []
        return wb_ssh.PromptResponder(target, child, allow, notes.append), child, notes

    def test_english_chinese_fullwidth_and_ansi(self):
        for prompt in ('[sudo] password for orangepi: ', '[sudo] orangepi 的密码：',
                       '[sudo] orangepi 的密码:', '\x1b[31m[sudo] orangepi 的密碼：\x1b[0m'):
            responder, child, notes = self.responder()
            self.assertTrue(responder.answer(prompt))
            child.sendline.assert_called_once_with(FAKE_PASSWORD)
            self.assertTrue(responder.answer(prompt))
            child.sendline.assert_called_once()  # Never repeat sudo credentials.
            self.assertNotIn(FAKE_PASSWORD, ''.join(notes))

    def test_ssh_and_sudo_have_independent_budgets(self):
        responder, child, _ = self.responder()
        for _ in range(5):
            responder.answer("orangepi@fixture-only's password: ")
        self.assertEqual(child.sendline.call_count, 3)
        responder.answer('[sudo] password for orangepi: ')
        self.assertEqual(child.sendline.call_count, 4)
        self.assertEqual(responder.attempts, dict(host=0, ssh=3, sudo=1))

    def test_other_sessions_one_off_and_application_prompts_do_not_get_sudo_secret(self):
        responder, child, _ = self.responder(allow=False)
        for prompt in ('[sudo] password for orangepi: ', '[sudo] orangepi 的密码：',
                       'Password: ', "Enter passphrase for key '/fixture-only': "):
            responder.answer(prompt)
        child.sendline.assert_not_called()

    def test_key_passphrase_keeps_manual_stdin_without_reusing_login_password(self):
        target = wb_ssh.Target(password=FAKE_PASSWORD)
        session = wb_ssh.Session('fixture', 'fixture', 'fixture', target)
        session.child = Mock()
        session.child.isalive.return_value = True
        session._responder = wb_ssh.PromptResponder(target, session.child)
        session._consume("Enter passphrase for key '/fixture-only': ")
        session.child.sendline.assert_not_called()
        session.write('mock-key-passphrase\n')
        session.child.send.assert_called_once_with('mock-key-passphrase\n')

    def test_local_disabled_absent_and_echoing_pty_are_manual(self):
        for kwargs in (dict(transport='local'), dict(auto_password=False), {}):
            responder, child, _ = self.responder(**kwargs)
            if not kwargs:
                responder.target.password = None
            responder.answer('[sudo] orangepi 的密码：')
            child.sendline.assert_not_called()
        responder, child, _ = self.responder()
        child.waitnoecho.return_value = False
        responder.answer('[sudo] password for orangepi: ')
        child.sendline.assert_not_called()
        self.assertEqual(responder.attempts['sudo'], 1)

    def test_echo_check_failure_is_bounded_without_secret_in_note(self):
        responder, child, notes = self.responder()
        child.waitnoecho.side_effect = OSError('PTY closed')
        responder.answer('[sudo] orangepi 的密码：')
        responder.answer('[sudo] orangepi 的密码：')
        child.sendline.assert_not_called()
        self.assertNotIn(FAKE_PASSWORD, ''.join(notes))

    def test_prompt_split_across_chunks_and_output_redaction(self):
        target = wb_ssh.Target(password=FAKE_PASSWORD)
        out, notes = [], []
        session = wb_ssh.Session('servo_init', 'fixture', 'fixture', target,
                                 on_output=lambda sid, seq, data: out.append(data),
                                 on_note=lambda sid, text: notes.append(text), allow_sudo_password=True)
        session.child = Mock()
        session.child.waitnoecho.return_value = True
        session._responder = wb_ssh.PromptResponder(target, session.child, True, session._note)
        for data in ('\x1b[31m[sud', 'o] orangepi 的密', '码：\x1b[0m'):
            session._consume(data)
        session.child.sendline.assert_called_once_with(FAKE_PASSWORD)
        for data in ('\nremote echo: mock-mem', 'ory-only-', 'secret\n'):
            session._consume(data)
        self.assertNotIn(FAKE_PASSWORD, ''.join(out) + ''.join(notes))
        self.assertIn('[口令已隐藏]', ''.join(out))
        self.assertNotIn(FAKE_PASSWORD, json.dumps(session.snapshot()))
        self.assertNotIn(FAKE_PASSWORD, ''.join(d for _, d in session.history_since(0)))

    def test_local_pty_ssh_then_chinese_sudo_no_echo_secret_in_file_or_sse(self):
        # A local Python process mimics SSH and sudo prompts; it never executes
        # ssh, sudo, ROS, PWM or any board command. It deliberately echoes the
        # received password in split output to exercise the redactor too.
        script = r'''
import sys, termios, time
fd = sys.stdin.fileno()
attrs = termios.tcgetattr(fd)
attrs[3] &= ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, attrs)
sys.stdout.write("orangepi@fixture-only's password: "); sys.stdout.flush()
password = sys.stdin.readline().rstrip('\r\n')
sys.stdout.write('\n[sud'); sys.stdout.flush(); time.sleep(.05)
sys.stdout.write('o] orangepi 的密码：'); sys.stdout.flush()
assert sys.stdin.readline().rstrip('\r\n') == password
sys.stdout.write('\nremote echo: ' + password[:7]); sys.stdout.flush(); time.sleep(.05)
sys.stdout.write(password[7:] + '\nAUTH_FIXTURE_DONE\n'); sys.stdout.flush()
'''
        target = wb_ssh.Target(password=FAKE_PASSWORD)
        target.remote_argv = lambda command: [sys.executable, '-u', '-c', script]
        outputs, notes = [], []
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / 'fixture.log'
            session = wb_ssh.Session('servo_init', 'fixture', 'local PTY fixture', target,
                                     log_path=str(log), on_output=lambda sid, seq, data: outputs.append(data),
                                     on_note=lambda sid, text: notes.append(text), allow_sudo_password=True)
            session.start()
            try:
                session.thread.join(timeout=5)
                self.assertFalse(session.thread.is_alive(), 'Fixture did not finish')
                self.assertEqual(session.exit_code, 0)
                self.assertIn('AUTH_FIXTURE_DONE', ''.join(outputs))
                self.assertEqual(session._responder.attempts['ssh'], 1)
                self.assertEqual(session._responder.attempts['sudo'], 1)
                self.assertNotIn(FAKE_PASSWORD, log.read_text() + ''.join(outputs) + ''.join(notes))
            finally:
                if session.child.isalive():
                    session.child.close(force=True)

    def test_partial_secret_at_eof_stays_redacted(self):
        session = wb_ssh.Session('fixture', 'fixture', 'fixture', wb_ssh.Target())
        session._responder = Mock(secrets={FAKE_PASSWORD})
        self.assertEqual(session._redact('mock-memory-'), '')
        self.assertEqual(session._redact('', final=True), '[口令已隐藏]')

    def test_run_once_only_answers_ssh_and_redacts_returned_output(self):
        child = Mock(exitstatus=0, signalstatus=None)
        child.waitnoecho.return_value = True
        child.isalive.return_value = False
        child.read_nonblocking.side_effect = ["orangepi@fixture-only's password: ",
                                              FAKE_PASSWORD[:8], FAKE_PASSWORD[8:],
                                              '\n[sudo] orangepi 的密码：', wb_ssh.pexpect.EOF('fixture EOF')]
        with patch.object(wb_ssh.pexpect, 'spawn', return_value=child):
            code, output = wb_ssh.run_once(wb_ssh.Target(password=FAKE_PASSWORD), 'fixture')
        self.assertEqual(code, 0)
        child.sendline.assert_called_once_with(FAKE_PASSWORD)
        self.assertNotIn(FAKE_PASSWORD, output)
        self.assertIn('[口令已隐藏]', output)

    def test_active_session_duplicate_open_does_not_start_command_twice(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(wb_ssh, 'Session') as factory:
            existing = factory.return_value
            existing.start.return_value = existing
            existing.state = 'running'
            manager = wb_ssh.SessionManager(tmp)
            first = manager.open('servo_init', 'fixture', 'fixture', wb_ssh.Target())
            second = manager.open('servo_init', 'fixture', 'fixture', wb_ssh.Target())
            self.assertIs(first, second)
            factory.assert_called_once()
            existing.start.assert_called_once()


class AuthorizationTests(unittest.TestCase):
    def test_confirmation_precedes_sudo_permission_and_other_terminals_cannot_opt_in(self):
        config = copy.deepcopy(wb_board.load_config())
        with tempfile.TemporaryDirectory() as tmp, patch.object(server, 'PROFILE_DIR', tmp), \
                patch.object(wb_board, 'load_profile', return_value={}):
            wb = server.Workbench(config, dict(transport='ssh', password=FAKE_PASSWORD))
            wb.sessions = Mock()
            wb.sessions.open.return_value.snapshot.return_value = {}
            with self.assertRaises(ValueError):
                wb.open_session(dict(id='servo_init', allow_sudo_password=True))
            wb.sessions.open.assert_not_called()
            with patch.object(wb, '_journal'):
                wb.open_session(dict(id='servo_init', confirm='确认'))
                self.assertTrue(wb.sessions.open.call_args.kwargs['allow_sudo_password'])
                for sid in ('servo', 'monitor', 'roscore'):
                    wb.open_session(dict(id=sid, confirm='确认', allow_sudo_password=True))
                    self.assertFalse(wb.sessions.open.call_args.kwargs['allow_sudo_password'])


if __name__ == '__main__':
    unittest.main(verbosity=2)

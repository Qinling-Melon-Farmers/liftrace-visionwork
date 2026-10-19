"""Real local subprocess/askpass fixtures; never invoke SSH or a board."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import wb_ssh

SECRET = 'mock-download-memory-secret'


class DownloadTests(unittest.TestCase):
    def fixture(self, script, password=SECRET, auto=True):
        target = wb_ssh.Target(password=password, auto_password=auto)
        target.remote_argv = lambda command, force_tty=False: [sys.executable, '-u', '-c', script]
        return target

    def test_password_askpass_and_exact_binary_no_prompt_or_stderr(self):
        script = r'''
import os, subprocess, sys
helper = os.environ['SSH_ASKPASS']
answer = subprocess.check_output([helper, "orangepi@fixture-only's password: "]).rstrip(b'\n')
assert answer == b'mock-download-memory-secret'
assert answer not in open(helper, 'rb').read()
assert all(answer.decode() not in value for value in os.environ.values())
sys.stderr.write("authentication prompt / warnings do not belong to file\n")
sys.stdout.buffer.write(bytes(range(256))*8192 + b'\n\r\n\0mock-download-memory-secret')
'''
        code, output = wb_ssh.run_bytes(self.fixture(script), 'fixture', timeout=10)
        self.assertEqual(code, 0, output[:200])
        self.assertEqual(output, bytes(range(256))*8192 + b'\n\r\n\0' + SECRET.encode())

    def test_askpass_rejects_sudo_key_passphrase_and_fourth_login(self):
        script = r'''
import os, subprocess, sys
helper = os.environ['SSH_ASKPASS']
for prompt in ('[sudo] password for orangepi: ', "Enter passphrase for key '/fixture': ", 'Password: '):
    p = subprocess.run([helper, prompt], capture_output=True)
    assert p.returncode != 0 and not p.stdout
for i in range(4):
    p = subprocess.run([helper, "orangepi@fixture-only's password: "], capture_output=True)
    assert (p.returncode == 0) == (i < 3)
sys.stdout.buffer.write(b'fixture-ok\x00\n')
'''
        code, data = wb_ssh.run_bytes(self.fixture(script), 'fixture', timeout=10)
        self.assertEqual((code, data), (0, b'fixture-ok\x00\n'))

    def test_failure_and_timeout_discard_partial_binary_and_credentials(self):
        code, data = wb_ssh.run_bytes(self.fixture(
            "import sys; sys.stdout.buffer.write(b'partial-file'); sys.stderr.write(%r); sys.exit(7)" % SECRET), 'fixture')
        self.assertEqual(code, 7); self.assertNotIn(b'partial-file', data); self.assertNotIn(SECRET.encode(), data)
        code, data = wb_ssh.run_bytes(self.fixture(
            "import sys,time; sys.stdout.buffer.write(b'partial-file'); sys.stdout.flush(); time.sleep(3)"),
            'fixture', timeout=.1)
        self.assertEqual(code, 124); self.assertNotIn(b'partial-file', data)

    def test_local_and_disabled_auth_download_without_askpass(self):
        for password, auto in ((None, True), (SECRET, False)):
            code, data = wb_ssh.run_bytes(self.fixture("import sys; sys.stdout.buffer.write(b'ULog\\x00\\n')", password, auto), 'fixture')
            self.assertEqual((code, data), (0, b'ULog\x00\n'))
        code, data = wb_ssh.run_bytes(wb_ssh.Target(transport='local'), "printf 'fixture\\n'", timeout=2)
        self.assertEqual((code, data), (0, b'fixture\n'))


if __name__ == '__main__': unittest.main(verbosity=2)

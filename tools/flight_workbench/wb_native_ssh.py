"""Native Windows SSH adapter; optional Paramiko, no WSL or local shell.

Keep the existing Session lifecycle and callbacks. Password authentication is
in memory. Remote PTY echo cannot be verified, so sudo passwords remain manual.
"""
import codecs
from pathlib import Path
import socket
import time


def connect(target):
    import paramiko
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    known = Path.home() / '.ssh' / 'known_hosts'
    known.parent.mkdir(parents=True, exist_ok=True)
    known.touch(exist_ok=True)
    client.load_host_keys(str(known))
    accept_new = 'StrictHostKeyChecking=accept-new' in target.ssh_options
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy() if accept_new else paramiko.RejectPolicy())
    host = target.host.split('@', 1)
    if len(host) != 2 or not all(host):
        raise ValueError('Native SSH requires user@host')
    try:
        client.connect(host[1], port=target.port, username=host[0],
                       password=target.password if target.auto_password else None,
                       key_filename=str(Path(target.identity_file).expanduser()) if target.identity_file else None,
                       timeout=target.connect_timeout, auth_timeout=target.connect_timeout,
                       banner_timeout=target.connect_timeout,
                       look_for_keys=not bool(target.identity_file), allow_agent=not bool(target.identity_file))
        client.get_transport().set_keepalive(15)
        return client
    except Exception:
        client.close()
        raise


class ChannelChild:
    """Small pexpect-compatible surface consumed by wb_ssh.Session."""
    def __init__(self, target, command, dimensions=(36, 140)):
        if target.transport != 'ssh':
            raise ValueError('Native Windows local mode is UI-only; local Linux commands are unavailable')
        import pexpect  # Only exception types; no Unix PTY calls.
        self.exceptions = pexpect
        self.client = connect(target)
        try:
            self.channel = self.client.get_transport().open_session(timeout=target.connect_timeout)
            self.channel.get_pty(term='xterm-256color', width=dimensions[1], height=dimensions[0])
            self.channel.exec_command(command)
        except Exception:
            self.client.close()
            raise
        self.decoder = codecs.getincrementaldecoder('utf-8')('replace')
        self.exitstatus = None
        self.signalstatus = None

    def read_nonblocking(self, size=4096, timeout=.4):
        self.channel.settimeout(timeout)
        try:
            data = self.channel.recv(size)
        except socket.timeout:
            raise self.exceptions.TIMEOUT('SSH channel timeout')
        if not data:
            raise self.exceptions.EOF('SSH channel closed')
        return self.decoder.decode(data)

    def isalive(self):
        return self.channel.recv_ready() or not self.channel.exit_status_ready() and not self.channel.closed

    def send(self, text):
        self.channel.sendall(text.encode('utf-8'))

    def sendline(self, text):
        self.send(text + '\n')

    def waitnoecho(self, timeout=1):
        return False  # Fail closed: cannot query remote termios over SSH.

    def setwinsize(self, rows, cols):
        self.channel.resize_pty(width=cols, height=rows)

    def close(self, force=False):
        if self.channel.exit_status_ready():
            code = self.channel.recv_exit_status()
            self.exitstatus = code if code >= 0 else 1
        else:
            self.exitstatus = 1
        self.channel.close()
        self.client.close()

    def terminate(self, force=False):
        self.close(force=force)


def run_bytes(target, command, timeout):
    """Non-PTY stdout/stderr, bounded transfer; never return partial bytes."""
    client = None
    channel = None
    try:
        if target.transport != 'ssh':
            return 1, b'Native Windows local mode is UI-only'
        client = connect(target)
        channel = client.get_transport().open_session(timeout=target.connect_timeout)
        channel.exec_command(command)
        output, errors = bytearray(), bytearray()
        deadline = time.monotonic() + float(timeout)
        while True:
            if time.monotonic() >= deadline:
                return 124, b'Download timed out; incomplete data discarded'
            # Drain both streams each iteration, even if stdout stays busy.
            if channel.recv_ready():
                output.extend(channel.recv(65536))
            if channel.recv_stderr_ready():
                errors.extend(channel.recv_stderr(65536))
            if channel.exit_status_ready() and not channel.recv_ready() and not channel.recv_stderr_ready():
                code = channel.recv_exit_status()
                if code == 0:
                    return 0, bytes(output)
                error = bytes(errors) or b'SSH failed; incomplete data discarded'
                if target.password:
                    error = error.replace(target.password.encode('utf-8'), b'[redacted]')
                return code if code > 0 else 1, error[-2000:]
            if channel.closed and not channel.exit_status_ready():
                return 1, b'SSH disconnected; incomplete data discarded'
            time.sleep(.01)
    except Exception as error:
        message = ('Native SSH failed: ' + type(error).__name__).encode('ascii', 'replace')
        return 1, message
    finally:
        if channel is not None:
            channel.close()
        if client is not None:
            client.close()

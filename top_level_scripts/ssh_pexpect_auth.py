#!/usr/bin/env python3
"""Small pexpect SSH helper with key, environment, or interactive auth."""

import getpass
import os

import pexpect


DEFAULT_HOST = "orangepi@192.168.3.15"


def board_host():
    return os.environ.get("ORANGEPI_SSH_HOST", DEFAULT_HOST)


def _password():
    value = os.environ.get("ORANGEPI_SSH_PASSWORD")
    if value:
        return value
    password_file = os.environ.get("ORANGEPI_SSH_PASSWORD_FILE")
    if password_file:
        with open(os.path.expanduser(password_file), "r", encoding="utf-8") as handle:
            return handle.readline().rstrip("\r\n")
    return getpass.getpass("OrangePi SSH password: ")


def spawn_and_wait(program, arguments, timeout):
    child = pexpect.spawn(
        program, arguments, timeout=timeout, encoding="utf-8")
    while True:
        index = child.expect([
            r"[Pp]assword:", pexpect.EOF, pexpect.TIMEOUT])
        if index == 0:
            child.sendline(_password())
            continue
        if index == 2:
            output = child.before or ""
            child.close(force=True)
            raise TimeoutError(output[-2000:])
        output = child.before or ""
        child.close()
        code = child.exitstatus
        if code is None and child.signalstatus is not None:
            code = 128 + int(child.signalstatus)
        return int(code if code is not None else 1), output


def ssh_arguments(remote_command, connect_timeout=10):
    return [
        "-o", "StrictHostKeyChecking=accept-new",
        "-o", "ConnectTimeout=%d" % int(connect_timeout),
        board_host(), remote_command,
    ]

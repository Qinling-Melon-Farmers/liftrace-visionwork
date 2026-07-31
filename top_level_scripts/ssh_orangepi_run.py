#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通过 pexpect 以密码方式在 OrangePi 5 Plus 上执行只读命令。

用法:
    python3 ssh_orangepi_run.py '<remote command>'

说明:
    - 本机未安装 sshpass/expect（见 AGENTS.md 2.6），故用 pexpect 自动输入密码;
    - 仅用于板端评测目录的只读检查, 不执行任何硬件动作。
"""
import os
import sys

import pexpect

HOST = "orangepi@192.168.3.15"
PASSWORD = "orangepi"
TIMEOUT = int(os.environ.get("ORANGEPI_SSH_TIMEOUT", "60"))


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: ssh_orangepi_run.py '<remote command>'", file=sys.stderr)
        return 2
    # Accept both one quoted shell command and multiple argv tokens. The latter
    # is convenient from WSL/PowerShell where nested quoting is fragile.
    remote_cmd = " ".join(sys.argv[1:])
    child = pexpect.spawn(
        "ssh",
        [
            "-o", "StrictHostKeyChecking=no",
            "-o", "ConnectTimeout=10",
            HOST,
            remote_cmd,
        ],
        timeout=TIMEOUT,
        encoding="utf-8",
    )
    idx = child.expect(["[Pp]assword:", pexpect.EOF, pexpect.TIMEOUT])
    if idx == 0:
        child.sendline(PASSWORD)
        child.expect(pexpect.EOF)
    output = child.before if child.before else ""
    child.close()
    sys.stdout.write(output)
    return child.exitstatus if child.exitstatus is not None else 1


if __name__ == "__main__":
    sys.exit(main())

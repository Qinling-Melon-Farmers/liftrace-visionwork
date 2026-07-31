#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用 pexpect 将少量评测脚本复制到 OrangePi；不执行远程命令。"""
import pexpect
import sys


HOST = "orangepi@192.168.3.15"
PASSWORD = "orangepi"


def main():
    if len(sys.argv) < 3:
        print("usage: ssh_orangepi_copy.py LOCAL... REMOTE_DIR", file=sys.stderr)
        return 2
    sources = sys.argv[1:-1]
    remote_dir = sys.argv[-1]
    child = pexpect.spawn(
        "scp",
        ["-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=10",
         *sources, HOST + ":" + remote_dir],
        timeout=60,
        encoding="utf-8",
    )
    while True:
        index = child.expect(["[Pp]assword:", pexpect.EOF, pexpect.TIMEOUT])
        if index == 0:
            child.sendline(PASSWORD)
        elif index == 1:
            break
        else:
            print(child.before or "", end="")
            print("copy timeout", file=sys.stderr)
            return 1
    print(child.before or "", end="")
    child.close()
    return child.exitstatus if child.exitstatus is not None else 1


if __name__ == "__main__":
    raise SystemExit(main())

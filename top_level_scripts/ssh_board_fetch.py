#!/usr/bin/env python3
"""从香橙派拉取文件到本地（pexpect scp 带密码，密码不落盘）。

用法: python3 ssh_board_fetch.py <板端路径> <本地路径>
"""
import sys
import pexpect

HOST = "orangepi@192.168.3.15"
PASSWORD = "orangepi"


def fetch(remote_path, local_path, timeout=300):
    child = pexpect.spawn(
        f"scp -o StrictHostKeyChecking=no -o ConnectTimeout=8 "
        f"{HOST}:{remote_path} {local_path}",
        encoding="utf-8", timeout=timeout)
    try:
        idx = child.expect([r"[Pp]assword:", pexpect.EOF, pexpect.TIMEOUT])
        if idx == 0:
            child.sendline(PASSWORD)
            child.expect(pexpect.EOF, timeout=timeout)
        print(child.before)
    except pexpect.TIMEOUT:
        print("TIMEOUT:", child.before[-2000:] if child.before else "")
    finally:
        child.close()


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("usage: ssh_board_fetch.py <remote_path> <local_path>")
        sys.exit(1)
    fetch(sys.argv[1], sys.argv[2])

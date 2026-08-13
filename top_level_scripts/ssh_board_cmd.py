#!/usr/bin/env python3
"""SSH 到香橙派执行只读命令（pexpect 带密码，密码不落盘）。

用法: python3 ssh_board_cmd.py "<远程命令>"
安全: 仅执行传入命令；密码仅存于本进程内存。
"""
import sys
import pexpect

HOST = "orangepi@192.168.3.15"
PASSWORD = "orangepi"


def run(cmd, timeout=60):
    child = pexpect.spawn(
        f"ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 {HOST} "
        f'"{cmd}"',
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


def upload(local_path, remote_path, timeout=600):
    """scp 上传本地文件到板端（pexpect 带密码）。"""
    child = pexpect.spawn(
        f"scp -o StrictHostKeyChecking=no -o ConnectTimeout=8 "
        f"{local_path} {HOST}:{remote_path}",
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


def download(remote_path, local_path, timeout=600):
    """scp 从板端下载文件到本地。"""
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
    if len(sys.argv) < 2:
        print("usage: ssh_board_cmd.py '<remote cmd>' | --upload <local> <remote> | --download <remote> <local>")
        sys.exit(1)
    if sys.argv[1] == "--upload":
        upload(sys.argv[2], sys.argv[3])
    elif sys.argv[1] == "--download":
        download(sys.argv[2], sys.argv[3])
    else:
        run(sys.argv[1])

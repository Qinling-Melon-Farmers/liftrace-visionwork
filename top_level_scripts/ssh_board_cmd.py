#!/usr/bin/env python3
"""SSH command/upload/download helper for the OrangePi lab session."""

import os
import sys

from ssh_pexpect_auth import board_host, spawn_and_wait, ssh_arguments


def _run(program, arguments, timeout):
    try:
        code, output = spawn_and_wait(program, arguments, timeout)
    except TimeoutError as error:
        print("timeout: %s" % error, file=sys.stderr)
        return 1
    sys.stdout.write(output)
    return code


def main():
    timeout = int(os.environ.get("ORANGEPI_SSH_TIMEOUT", "60"))
    if len(sys.argv) < 2:
        print(
            "usage: ssh_board_cmd.py '<remote cmd>' | "
            "--upload LOCAL REMOTE | --download REMOTE LOCAL",
            file=sys.stderr)
        return 2
    if sys.argv[1] == "--upload" and len(sys.argv) == 4:
        return _run("scp", [
            "-o", "StrictHostKeyChecking=accept-new",
            "-o", "ConnectTimeout=10", sys.argv[2],
            board_host() + ":" + sys.argv[3]], max(timeout, 600))
    if sys.argv[1] == "--download" and len(sys.argv) == 4:
        return _run("scp", [
            "-o", "StrictHostKeyChecking=accept-new",
            "-o", "ConnectTimeout=10",
            board_host() + ":" + sys.argv[2], sys.argv[3]],
            max(timeout, 600))
    if sys.argv[1].startswith("--"):
        print("invalid ssh_board_cmd arguments", file=sys.stderr)
        return 2
    return _run("ssh", ssh_arguments(" ".join(sys.argv[1:])), timeout)


if __name__ == "__main__":
    raise SystemExit(main())

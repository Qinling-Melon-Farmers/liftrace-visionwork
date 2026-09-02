#!/usr/bin/env python3
"""Copy local files to the OrangePi without storing a password in source."""

import sys

from ssh_pexpect_auth import board_host, spawn_and_wait


def main():
    if len(sys.argv) < 3:
        print("usage: ssh_orangepi_copy.py LOCAL... REMOTE_DIR", file=sys.stderr)
        return 2
    arguments = [
        "-o", "StrictHostKeyChecking=accept-new",
        "-o", "ConnectTimeout=10",
        *sys.argv[1:-1], board_host() + ":" + sys.argv[-1],
    ]
    try:
        code, output = spawn_and_wait("scp", arguments, 600)
    except TimeoutError as error:
        print("copy timeout: %s" % error, file=sys.stderr)
        return 1
    sys.stdout.write(output)
    return code


if __name__ == "__main__":
    raise SystemExit(main())

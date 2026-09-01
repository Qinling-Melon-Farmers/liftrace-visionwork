#!/usr/bin/env python3
"""Run one explicitly supplied OrangePi command over SSH."""

import os
import sys

from ssh_pexpect_auth import spawn_and_wait, ssh_arguments


def main():
    if len(sys.argv) < 2:
        print("usage: ssh_orangepi_run.py '<remote command>'", file=sys.stderr)
        return 2
    command = " ".join(sys.argv[1:])
    timeout = int(os.environ.get("ORANGEPI_SSH_TIMEOUT", "60"))
    try:
        code, output = spawn_and_wait(
            "ssh", ssh_arguments(command), timeout)
    except TimeoutError as error:
        print("ssh timeout: %s" % error, file=sys.stderr)
        return 1
    sys.stdout.write(output)
    return code


if __name__ == "__main__":
    raise SystemExit(main())

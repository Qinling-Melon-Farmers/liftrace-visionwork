#!/usr/bin/env python3
"""Read existing build/runtime evidence. Never starts LIO or runs a matching test."""
import argparse
import json
from pathlib import Path
import re
import sys


def check(flags, cache=None, matching_log=None, launch_log=None, expected_binary=None):
    if not re.search(r'(?<!\w)-DMP_EN(?:\s|$)', flags):
        raise ValueError('MP_EN is absent; libgomp alone is not parallel matching evidence')
    if not re.search(r'-DMP_PROC_NUM=3(?:\s|$)', flags) or '-fopenmp' not in flags:
        raise ValueError('explicit three-worker/OpenMP flags required')
    if cache is not None and not re.search(r'^FAST_LIO_MATCH_THREADS:[^=]+=3$', cache, re.M):
        raise ValueError('CMake cache must explicitly select FAST_LIO_MATCH_THREADS=3')
    runtime_team = bool(matching_log and re.search(
        r'PASS: 10 production matching cases; workers=3(?:\s|$)', matching_log))
    runtime_path = bool(expected_binary and launch_log and expected_binary in launch_log)
    return dict(build_parallel=True, configured_workers=3, matching_team_exercised=runtime_team,
                actual_launch_path_confirmed=runtime_path,
                runtime_parallel_confirmed=runtime_team and runtime_path,
                limitation='A build flag is not proof the board launched this binary; require matching test log and current launch executable path.')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--flags', type=Path, required=True)
    p.add_argument('--cache', type=Path)
    p.add_argument('--matching-log', type=Path)
    p.add_argument('--launch-log', type=Path)
    p.add_argument('--expected-binary')
    p.add_argument('--require-runtime', action='store_true')
    p.add_argument('--out', type=Path)
    args = p.parse_args()
    def read(path): return path.read_text(encoding='utf-8') if path else None
    result = check(read(args.flags), read(args.cache), read(args.matching_log),
                   read(args.launch_log), args.expected_binary)
    print(json.dumps(result, indent=2))
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    if args.require_runtime and not result['runtime_parallel_confirmed']:
        return 3
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, OSError) as exc:
        print('FAIL: '+str(exc), file=sys.stderr)
        sys.exit(1)

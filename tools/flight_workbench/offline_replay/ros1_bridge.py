"""Read through an existing ROS1 interpreter when native rosbags is unavailable."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

TOOL = Path(__file__).resolve().parent


def available():
    return os.name != 'nt' and bool(shutil.which('bash')) and Path(
        os.environ.get('ROS_SETUP', '/opt/ros/noetic/setup.bash')).is_file()


def run(arguments, **options):
    if not available():
        raise RuntimeError('No existing ROS1 reading environment; use an existing rosbags environment.')
    setup = os.environ.get('ROS_SETUP', '/opt/ros/noetic/setup.bash')
    # Paths and options are separate argv elements, never interpolated shell text.
    shell = 'set +u; source "$1"; shift; exec "${ROS_PYTHON:-/usr/bin/python3}" "$@"'
    return subprocess.run(['bash', '-e', '-c', shell, 'offline-ros1', setup,
                           str(TOOL / 'ros1_bridge.py'), *arguments],
                          check=True, **options)


def probe():
    run(['probe'], capture_output=True, timeout=20)


def analysis_data(bag, topics=None):
    with tempfile.TemporaryDirectory(prefix='offline-analysis-') as folder:
        output = Path(folder) / 'data.json'
        args = ['analysis', str(bag), str(output)]
        if topics:
            args.extend(['--topics', str(topics)])
        run(args)
        return json.loads(output.read_text(encoding='utf-8'))


def main():
    if sys.argv[1] == 'probe':
        import rosbag
        import numpy
        return
    if sys.argv[1] == 'analysis':
        from flight_analysis import read_data
        topics = sys.argv[5] if len(sys.argv) > 5 else None
        data = read_data(sys.argv[2], reader='ros1', topics=topics)
        Path(sys.argv[3]).write_text(json.dumps(data, ensure_ascii=False), encoding='utf-8')
        return
    import argparse
    import bag_replay
    parser = argparse.ArgumentParser()
    parser.add_argument('operation', choices=['export'])
    parser.add_argument('bag')
    parser.add_argument('out')
    parser.add_argument('--topics')
    args = parser.parse_args()
    args.reader = 'ros1'
    bag_replay.export(args)


if __name__ == '__main__':
    main()

"""Cross-platform ROS1 bag replay entry point (Python 3.10+)."""
import argparse
import importlib.util
import shutil
import sys
from pathlib import Path

import bag_replay
from ffmpeg_support import configure_ffmpeg
from render_options import add_render_options
import ros1_bridge


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag',nargs='?')
    parser.add_argument('out',nargs='?')
    parser.add_argument('--fps',type=int,default=10)
    parser.add_argument('--frame',default='map')
    parser.add_argument('--topics')
    parser.add_argument('--reader',choices=['auto','rosbags','ros1'],default='auto')
    parser.add_argument('--keep-frames',action='store_true')
    add_render_options(parser)
    parser.add_argument('--check-deps',action='store_true',help='Check dependencies without reading or writing bag data')
    args=parser.parse_args()
    if not 1<=args.fps<=30:parser.error('fps must be 1..30')
    if sys.version_info<(3,8):parser.error('Use Python 3.8 or newer for the existing ROS1 bridge')
    configure_ffmpeg()
    missing=[name for name in ('numpy','scipy','cv2') if importlib.util.find_spec(name) is None]
    if missing:
        raise RuntimeError('Missing Python dependencies: '+', '.join(missing)+
                           '. Run: python -m pip install -r requirements.txt')
    try:
        backend=bag_replay.select_reader(args.reader)
    except RuntimeError:
        if args.reader != 'auto' or not ros1_bridge.available():
            raise
        ros1_bridge.probe()
        backend='ros1-bridge'
    missing=[name for name in ('ffmpeg','ffprobe') if shutil.which(name) is None]
    if missing:raise RuntimeError('Missing '+', '.join(missing)+'. Install FFmpeg and add its bin directory to PATH.')
    print(f'Python: {sys.executable}\nBag reader: {backend}\nFFmpeg: {shutil.which("ffmpeg")}',flush=True)
    if args.check_deps:return
    if not args.bag or not args.out:parser.error('Provide INPUT.bag and OUTPUT_DIR, or use --check-deps')
    bag=Path(args.bag).resolve()
    out=Path(args.out).resolve()
    if not bag.is_file():parser.error(f'Bag file does not exist: {bag}')
    if out==bag or (out.exists() and not out.is_dir()):parser.error('Output must be a directory')
    if args.topics and not Path(args.topics).is_file():parser.error(f'Topics file does not exist: {args.topics}')
    args.reader=backend
    print('Stage: export',flush=True)
    if backend == 'ros1-bridge':
        bridge_args=['export', str(bag), str(out)]
        if args.topics:bridge_args += ['--topics', args.topics]
        ros1_bridge.run(bridge_args)
    else:
        bag_replay.export(args)
    print('Stage: render',flush=True)
    bag_replay.render(args)
    print('Stage: verify',flush=True)
    bag_replay.verify(args)
    print(f'Done. Open: {out / "index.html"}',flush=True)


if __name__=='__main__':
    try:
        main()
    except (RuntimeError,ValueError,OSError,ImportError) as exc:
        print(f'Replay failed: {exc}',file=sys.stderr)
        sys.exit(1)

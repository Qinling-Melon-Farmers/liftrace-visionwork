"""Encoder contracts and an opt-in short synthetic replay benchmark (no ROS)."""
import argparse
import cProfile
import json
import math
import pstats
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np

from bag_replay import TOPICS, render, verify
from video_encoder import VideoEncoders, command, select_encoder


class EncoderContracts(unittest.TestCase):
    sizes = dict(camera=(64, 48), dashboard=(96, 64))

    def test_cpu_does_not_probe(self):
        with patch('video_encoder.probe_nvenc') as probe:
            selection = select_encoder('cpu', self.sizes, 10)
        probe.assert_not_called()
        self.assertEqual(selection['effective'], 'cpu')

    def test_auto_preserves_concurrency_failure_reason(self):
        with patch('video_encoder.probe_nvenc', return_value=(False, 'session capacity exceeded', .2)) as probe:
            selection = select_encoder('auto', self.sizes, 10)
        probe.assert_called_once_with(self.sizes, 10)
        self.assertEqual(selection['effective'], 'cpu')
        self.assertEqual(selection['reason'], 'session capacity exceeded')
        self.assertEqual(selection['probe_sessions'], 2)

    def test_explicit_nvenc_does_not_hide_probe_failure(self):
        with patch('video_encoder.probe_nvenc', return_value=(False, 'driver unavailable', .1)):
            with self.assertRaisesRegex(RuntimeError, 'driver unavailable'):
                select_encoder('nvenc', self.sizes, 10)

    def test_actual_probe_failure_falls_back_and_reaps_children(self):
        children = []
        real_popen = subprocess.Popen
        def launch(*args, **kwargs):
            proc = real_popen(*args, **kwargs)
            children.append(proc)
            return proc
        fake = [sys.executable, '-c', 'import sys; sys.stderr.write("NVENC session limit"); sys.exit(9)']
        with patch('video_encoder.command', return_value=fake), patch('video_encoder.subprocess.Popen', side_effect=launch):
            selection = select_encoder('auto', self.sizes, 10)
        self.assertEqual(selection['effective'], 'cpu')
        self.assertIn('NVENC session limit', selection['reason'])
        self.assertEqual(len(children), 2)
        self.assertTrue(all(p.poll() is not None for p in children))

    def test_cpu_outputs_complete_frames(self):
        with tempfile.TemporaryDirectory() as tmp:
            with VideoEncoders(tmp, self.sizes, 10, 'cpu', 3) as group:
                for i in range(3):
                    for name, (w, h) in self.sizes.items():
                        group.write(name, np.full((h, w, 3), i*80, np.uint8))
            for name in self.sizes:
                path = str(Path(tmp) / (name + '.mp4'))
                subprocess.run(['ffmpeg', '-v', 'error', '-xerror', '-i', path, '-f', 'null', '-'], check=True)
                info = json.loads(subprocess.check_output(['ffprobe', '-v', 'error', '-count_frames', '-show_entries', 'stream=nb_read_frames', '-of', 'json', path]))
                self.assertEqual(int(info['streams'][0]['nb_read_frames']), 3)
            self.assertFalse(list(Path(tmp).glob('*.partial.mp4')))

    def test_early_eof_with_zero_exit_is_not_promoted(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(RuntimeError, 'frames=1/2'):
                with VideoEncoders(tmp, self.sizes, 10, 'cpu', 2) as group:
                    for name, (w, h) in self.sizes.items():
                        group.write(name, bytes(w*h*3))
            self.assertFalse(list(Path(tmp).glob('camera.mp4')))
            self.assertTrue(all(p.poll() is not None for p in group.processes.values()))

    def test_midstream_failure_invalidates_summary_and_retains_images(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            synthetic_replay(out, .5)
            (out / 'validation.json').write_text('{"old":"success"}')
            args = SimpleNamespace(out=tmp, fps=10, frame='map', encoder='cpu', keep_frames=False)
            # A real child consumes one camera frame then fails. Only our children are involved.
            fake = [sys.executable, '-c', 'import sys; sys.stdin.buffer.read(1280*720*3); sys.stderr.write("synthetic device failure"); sys.exit(7)']
            with patch('video_encoder.command', return_value=fake):
                with self.assertRaises(RuntimeError):
                    render(args)
            summary = json.loads((out / 'summary.json').read_text())
            self.assertEqual(summary['render_status'], 'failed')
            self.assertEqual(summary['video_files'], [])
            self.assertIn('synthetic device failure', summary['error'])
            with self.assertRaisesRegex(ValueError, 'did not complete'):
                verify(args)
            self.assertFalse((out / 'validation.json').exists())
            self.assertTrue(list((out / 'frames').glob('*.jpg')))

    def test_decode_frame_mismatch_prevents_success_and_cleanup(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            synthetic_replay(out, .2)
            args = SimpleNamespace(out=tmp, fps=10, frame='map', encoder='cpu', keep_frames=False)
            render(args)
            summary = json.loads((out / 'summary.json').read_text())
            summary['frames'] += 1
            (out / 'summary.json').write_text(json.dumps(summary))
            (out / 'validation.json').write_text('{"old":"success"}')
            with self.assertRaisesRegex(ValueError, 'frame count mismatch'):
                verify(args)
            self.assertFalse((out / 'validation.json').exists())
            self.assertTrue(list((out / 'frames').glob('*.jpg')))


def synthetic_replay(out, duration=8.):
    """Recorded-style fixture; does not read bags or use live nodes/model inference."""
    out = Path(out)
    (out / 'frames').mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(4060)
    image = rng.integers(0, 90, (540, 960, 3), dtype=np.uint8)
    cv2.rectangle(image, (200, 100), (700, 450), (200, 180, 80), 3)
    cv2.circle(image, (450, 270), 100, (240, 230, 210), 8)
    frames = []
    for i, t in enumerate(np.arange(0, duration, .5)):
        im = image.copy()
        cv2.putText(im, f'Synthetic camera {i}', (40, 80), 0, 1.5,
                    (255, 255, 255), 2, cv2.LINE_AA)
        file = f'frames/{i:06d}.jpg'
        cv2.imwrite(str(out / file), im)
        frames.append(dict(t=float(t), stamp=float(t), file=file))
    rows = {k: [] for k in TOPICS}
    for t in np.arange(0, duration, .01):
        pos = dict(x=2*math.cos(t), y=2*math.sin(t), z=1.5+.3*math.sin(t))
        msg = dict(header=dict(frame_id='map'), pose=dict(position=pos))
        rows['pose'].append(dict(t=float(t), m=msg))
    points = rng.uniform([-2, -2, 1.1], [2, 2, 1.9], (500, 3)).tolist()
    rows['cloud'] = [dict(t=0., m=dict(header=dict(frame_id='map'), points=points))]
    meta = dict(bag='synthetic-encoder-benchmark', start=0., duration=duration,
                frames=frames, rows=rows, missing=[k for k in TOPICS if not rows[k] and k != 'camera'])
    (out / 'data.json').write_text(json.dumps(meta))


def benchmark(output, duration=8., cpu_only=False):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    results = {}
    for encoder in (['cpu'] if cpu_only else ['cpu', 'nvenc']):
        folder = output / encoder
        synthetic_replay(folder, duration)
        args = SimpleNamespace(out=str(folder), fps=10, frame='map',
                               encoder=encoder, keep_frames=True)
        profile = cProfile.Profile()
        start = time.perf_counter()
        profile.runcall(render, args)
        elapsed = time.perf_counter() - start
        validation_started = time.perf_counter()
        verify(args)
        validation_seconds = time.perf_counter() - validation_started
        summary = json.loads((folder / 'summary.json').read_text())
        results[encoder] = dict(render_wall_seconds=elapsed, validation_seconds=validation_seconds, summary=summary)
        with (folder / 'profile.txt').open('w') as stream:
            pstats.Stats(profile, stream=stream).strip_dirs().sort_stats('cumtime').print_stats(25)
    if not cpu_only:
        # Same preloaded BGR frames for both codecs: isolates encoding/IPC from drawing.
        samples = {}
        sizes = results['cpu']['summary']['video_sizes']
        for name in sizes:
            capture = cv2.VideoCapture(str(output / 'cpu' / (name + '.mp4')))
            ok, image = capture.read()
            capture.release()
            if not ok:
                raise RuntimeError('Cannot preload benchmark frame: ' + name)
            samples[name] = image
        for encoder in ('cpu', 'nvenc'):
            folder = output / ('encode_only_' + encoder)
            folder.mkdir()
            count = math.ceil(duration * 10)
            started = time.perf_counter()
            with VideoEncoders(folder, sizes, 10, encoder, count) as writers:
                for i in range(count):
                    for name, image in samples.items():
                        writers.write(name, image)
            results[encoder]['encode_only_seconds'] = time.perf_counter() - started
            meta = dict(duration=duration, fps=10, frames=count,
                        video_files=[name + '.mp4' for name in sizes], video_sizes=sizes)
            (folder / 'summary.json').write_text(json.dumps(meta))
            started = time.perf_counter()
            verify(SimpleNamespace(out=str(folder), keep_frames=True))
            results[encoder]['encode_only_validation_seconds'] = time.perf_counter() - started
    (output / 'benchmark.json').write_text(json.dumps(results, indent=2))
    print(json.dumps({k: dict(render_wall_seconds=v['render_wall_seconds'],
                             validation_seconds=v['validation_seconds'],
                             encode_only_seconds=v.get('encode_only_seconds'),
                             timings=v['summary'].get('timings')) for k, v in results.items()}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--benchmark', type=Path)
    parser.add_argument('--duration', type=float, default=8.)
    parser.add_argument('--cpu-only', action='store_true')
    args, rest = parser.parse_known_args()
    if args.benchmark:
        if not .1 <= args.duration <= 30:
            parser.error('synthetic benchmark duration must be 0.1..30 seconds')
        benchmark(args.benchmark, args.duration, args.cpu_only)
    else:
        unittest.main(argv=[__file__, *rest])

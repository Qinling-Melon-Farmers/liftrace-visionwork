"""FFmpeg CPU/NVENC encoding only; OpenCV drawing stays on the CPU."""
import json
import subprocess
import tempfile
import threading
import time
from pathlib import Path


def command(path, size, fps, encoder):
    codec = (['-c:v', 'libx264', '-threads', '2', '-preset', 'veryfast', '-crf', '23']
             if encoder == 'cpu' else
             ['-c:v', 'h264_nvenc', '-threads', '2', '-preset', 'fast',
              '-rc', 'vbr', '-cq', '23', '-b:v', '0'])
    return ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-filter_threads', '2',
            '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', f'{size[0]}x{size[1]}',
            '-r', str(fps), '-i', '-', '-an', *codec, '-pix_fmt', 'yuv420p',
            '-movflags', '+faststart', str(path)]


class VideoEncoders:
    """Own only our child processes; promote output only after every stream finishes."""
    def __init__(self, out, sizes, fps, encoder, expected_frames):
        self.out = Path(out)
        self.sizes = sizes
        self.fps = fps
        self.encoder = encoder
        self.expected_frames = expected_frames
        self.processes = {}
        self.logs = {}
        self.frames = {name: 0 for name in sizes}
        self.finish_seconds = 0.

    def __enter__(self):
        try:
            for name, size in self.sizes.items():
                self.logs[name] = tempfile.TemporaryFile()
                self.processes[name] = subprocess.Popen(
                    command(self.out / (name + '.partial.mp4'), size, self.fps, self.encoder),
                    stdin=subprocess.PIPE, stderr=self.logs[name])
        except BaseException:
            self.kill()
            self.__exit__(RuntimeError, None, None)
            raise
        return self

    def kill(self):
        for proc in list(self.processes.values()):
            if proc.poll() is None:
                try:
                    proc.kill()
                except ProcessLookupError:
                    pass

    def write(self, name, image):
        proc = self.processes[name]
        if proc.poll() is not None:
            raise RuntimeError(f'{name}: {self.encoder} exited during encoding; rerun with --encoder cpu')
        # ndarray and bytes both support the buffer protocol; avoid a full-frame copy.
        data = memoryview(image).cast('B')
        size = self.sizes[name]
        if len(data) != size[0] * size[1] * 3:
            raise ValueError(f'{name}: wrong raw frame size')
        try:
            while data:
                written = proc.stdin.write(data)
                if not written:
                    raise RuntimeError(f'{name}: incomplete frame write')
                data = data[written:]
        except (BrokenPipeError, OSError) as error:
            raise RuntimeError(f'{name}: {self.encoder} pipe failed; rerun with --encoder cpu') from error
        self.frames[name] += 1

    def __exit__(self, exc_type, exc_value, traceback):
        started = time.perf_counter()
        errors = []
        try:
            # Close every input before waiting; no child is left waiting for EOF.
            for name, proc in self.processes.items():
                try:
                    proc.stdin.close()
                except (BrokenPipeError, OSError) as error:
                    errors.append(f'{name}: closing input: {error}')
            for name, proc in self.processes.items():
                try:
                    code = proc.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    code = proc.wait()
                    errors.append(f'{name}: encoder finalization timed out')
                log = self.logs[name]
                log.seek(0, 2)
                length = log.tell()
                log.seek(max(0, length - 4000))
                detail = log.read().decode('utf-8', errors='replace').strip()
                if code or self.frames[name] != self.expected_frames:
                    errors.append(f'{name}: exit={code}, frames={self.frames[name]}/{self.expected_frames}; {detail}')
            if exc_type is None:
                if errors:
                    raise RuntimeError('Video encoding failed (no midstream fallback): ' + '; '.join(errors))
                for name in self.processes:
                    (self.out / (name + '.partial.mp4')).replace(self.out / (name + '.mp4'))
            elif errors:
                # Keep stderr in the original propagated exception and failure summary.
                if exc_value is not None:
                    exc_value.args = (*exc_value.args, 'Encoder cleanup: ' + '; '.join(errors))
        finally:
            for log in self.logs.values():
                log.close()
            self.finish_seconds = time.perf_counter() - started
        return False


def probe_nvenc(sizes, fps):
    """Encode two frames per actual-sized stream, holding all sessions open together."""
    started = time.perf_counter()
    expired = threading.Event()
    try:
        with tempfile.TemporaryDirectory(prefix='bag_replay_nvenc_') as tmp:
            group = VideoEncoders(tmp, sizes, fps, 'nvenc', 2)
            def timeout():
                expired.set()
                group.kill()
            timer = threading.Timer(20, timeout)
            timer.daemon = True
            try:
                with group:
                    timer.start()
                    for name, size in sizes.items():
                        frame = bytes(size[0] * size[1] * 3)
                        group.write(name, frame)
                        group.write(name, frame)
                    # No stdin closes until all streams have received both frames.
            finally:
                timer.cancel()
            for name in sizes:
                info = json.loads(subprocess.check_output(
                    ['ffprobe', '-v', 'error', '-select_streams', 'v:0', '-count_frames',
                     '-show_entries', 'stream=nb_read_frames', '-of', 'json',
                     str(Path(tmp) / (name + '.mp4'))], timeout=10))
                if int(info['streams'][0]['nb_read_frames']) != 2:
                    raise RuntimeError(f'{name}: NVENC probe did not decode two complete frames')
        return True, f'NVENC encoded and decoded 2 frames on each of {len(sizes)} concurrent actual-sized streams', time.perf_counter() - started
    except Exception as error:
        reason = 'NVENC concurrent probe failed: ' + ('timeout; ' if expired.is_set() else '') + str(error)
        return False, reason, time.perf_counter() - started


def select_encoder(requested, sizes, fps):
    if requested not in ('auto', 'cpu', 'nvenc'):
        raise ValueError('encoder must be auto, cpu or nvenc')
    ok, reason, seconds = (False, 'CPU selected; NVENC probe skipped', 0.)
    if requested != 'cpu':
        ok, reason, seconds = probe_nvenc(sizes, fps)
        if not ok and requested == 'nvenc':
            raise RuntimeError(reason + '; explicit NVENC requested; use --encoder cpu or auto')
    effective = 'nvenc' if ok else 'cpu'
    selection = dict(requested=requested, effective=effective,
                     codec='h264_nvenc' if ok else 'libx264', reason=reason,
                     probe_sessions=len(sizes) if requested != 'cpu' else 0,
                     probe_seconds=seconds, rendering='CPU OpenCV',
                     midstream_fallback=False)
    print(f'Encoder {requested} -> {effective} ({selection["codec"]}): {reason}. Drawing: CPU OpenCV.', flush=True)
    return selection

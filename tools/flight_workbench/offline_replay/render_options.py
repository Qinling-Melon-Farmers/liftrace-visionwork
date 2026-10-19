"""Shared rendering controls and a real hardware-encoder availability probe."""
import subprocess
import os

def process_options():
    return {'creationflags': subprocess.CREATE_NO_WINDOW} if os.name == 'nt' else {}


def add_render_options(parser):
    parser.add_argument('--encoder', choices=['auto', 'cpu', 'nvenc'], default='auto')
    parser.add_argument('--speed', choices=['balanced', 'fast'], default='balanced')
    parser.add_argument('--video-set', choices=['all', 'dashboard'], default='all')


def encoder_options(requested, speed):
    if requested != 'cpu':
        probe = ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-f', 'lavfi', '-i',
                 'color=c=black:s=320x240:r=10', '-frames:v', '1', '-an',
                 '-c:v', 'h264_nvenc', '-pix_fmt', 'yuv420p', '-f', 'null', '-']
        try:
            result = subprocess.run(probe, capture_output=True, timeout=15, **process_options())
            if result.returncode == 0:
                return 'h264_nvenc', ['-c:v', 'h264_nvenc', '-preset', 'p1' if speed == 'fast' else 'p4',
                                      '-rc', 'vbr', '-cq', '23', '-b:v', '0']
            reason = result.stderr.decode('utf-8', errors='replace').strip()[:1000]
        except (OSError, subprocess.TimeoutExpired) as exc:
            reason = str(exc)
        if requested == 'nvenc':
            raise RuntimeError('NVIDIA hardware encoder unavailable: ' + reason)
        print('NVENC unavailable; using CPU encoder. ' + reason, flush=True)
    return 'libx264', ['-c:v', 'libx264', '-preset', 'ultrafast' if speed == 'fast' else 'veryfast', '-crf', '23']

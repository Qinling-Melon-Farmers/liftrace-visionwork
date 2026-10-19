"""Find FFmpeg without requiring an already-open app to refresh its PATH."""
import os
import shutil
from pathlib import Path


def find_ffmpeg_bin():
    suffix = '.exe' if os.name == 'nt' else ''
    candidates = []
    current = shutil.which('ffmpeg')
    if current:
        candidates.append(Path(current).parent)
    candidates.append(Path(__file__).resolve().parent / 'tools' / 'ffmpeg' / 'bin')
    if os.name == 'nt':
        local = os.environ.get('LOCALAPPDATA')
        if local:
            candidates.append(Path(local) / 'Microsoft' / 'WinGet' / 'Links')
            packages = Path(local) / 'Microsoft' / 'WinGet' / 'Packages'
            candidates.extend(path.parent for path in packages.glob('Gyan.FFmpeg*/*/bin/ffmpeg.exe'))
        # Read updated user PATH even when the parent desktop app has an older environment.
        import winreg
        try:
            with winreg.OpenKey(winreg.HKEY_CURRENT_USER, 'Environment') as key:
                user_path = winreg.QueryValueEx(key, 'Path')[0]
            candidates.extend(Path(os.path.expandvars(item.strip('"'))) for item in user_path.split(os.pathsep) if item)
        except OSError:
            pass
    for directory in candidates:
        if all((directory / (name + suffix)).is_file() for name in ('ffmpeg', 'ffprobe')):
            return directory
    return None


def configure_ffmpeg():
    if shutil.which('ffmpeg') and shutil.which('ffprobe'):
        return
    directory = find_ffmpeg_bin()
    if directory is not None:
        os.environ['PATH'] = str(directory) + os.pathsep + os.environ.get('PATH', '')

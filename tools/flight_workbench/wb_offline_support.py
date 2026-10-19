"""Native offline-worker dependencies, locks and local path dialogs."""
import json
import os
from pathlib import Path
import subprocess
import sys

TOOL = Path(__file__).resolve().parent / 'offline_replay'


def python_command():
    value = os.environ.get('REPLAY_PYTHON') or sys.executable
    path = Path(value).expanduser()
    if path.name.lower() == 'pythonw.exe' and path.with_name('python.exe').exists():
        return str(path.with_name('python.exe'))
    return str(path)


def process_options():
    return {'creationflags': subprocess.CREATE_NO_WINDOW | subprocess.CREATE_NEW_PROCESS_GROUP} if os.name == 'nt' else {'start_new_session': True}


def dependencies(env):
    code = '''import importlib.util,json,shutil,sys
from ffmpeg_support import configure_ffmpeg
configure_ffmpeg()
print(json.dumps({'python':sys.executable,'version':list(sys.version_info[:2]),'modules':{n:importlib.util.find_spec(n) is not None for n in ['numpy','scipy','cv2','rosbags','rosbag','matplotlib']},'ffmpeg':shutil.which('ffmpeg'),'ffprobe':shutil.which('ffprobe')}))'''
    try:
        result = subprocess.run([python_command(), '-c', code], cwd=TOOL, env=env, capture_output=True,
                                encoding='utf-8', timeout=20, **process_options())
        if result.returncode:
            raise ValueError(result.stderr.strip() or 'Python 依赖检查失败')
        info = json.loads(result.stdout.strip())
        common = []
        bridge = False
        if not (info['modules']['rosbags'] or info['modules']['rosbag']):
            try:
                subprocess.run([python_command(), '-c', 'import ros1_bridge; ros1_bridge.probe()'],
                               cwd=TOOL, env=env, capture_output=True, check=True, timeout=25,
                               **process_options())
                bridge = True
            except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
                common.append('缺少 rosbags 读取库或已有 ROS1 读取环境')
        minimum = (3, 8) if bridge or info['modules']['rosbag'] else (3, 10)
        if tuple(info['version']) < minimum:
            common.append('需要 Python %s.%s 或以上' % minimum)
        if not info['modules']['numpy']:
            common.append('缺少 NumPy')
        video = common + ['缺少 '+n for n in ('scipy', 'cv2') if not info['modules'][n]]
        video += ['缺少 '+n for n in ('ffmpeg', 'ffprobe') if not info[n]]
        analysis = common + ([] if info['modules']['matplotlib'] else ['缺少 Matplotlib'])
        for name in ('run.py', 'bag_replay.py', 'flight_analysis.py', 'ffmpeg_support.py', 'render_options.py', 'target_timeline.py', 'ros1_bridge.py'):
            if not (TOOL/name).is_file():
                video.append('缺少 '+name)
                analysis.append('缺少 '+name)
        return {'generate': not video, 'analysis': not analysis, 'view': True,
                'detail': '；'.join(video) if video else 'Windows / Linux 原生离线视频生成可用',
                'analysis_detail': '；'.join(analysis) if analysis else '投递与降落分析可用，曲线只导出 PNG',
                'python': info['python'], 'reader': 'ros1-bridge' if bridge else 'native', 'native_picker': os.name == 'nt'}
    except (ValueError, OSError, subprocess.TimeoutExpired) as error:
        return {'generate': False, 'analysis': False, 'view': True, 'detail': str(error),
                'analysis_detail': str(error), 'native_picker': os.name == 'nt'}


def acquire(path):
    path = Path(path)
    if path.is_symlink():
        raise ValueError('锁文件不能是符号链接')
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = path.open('a+b')
    try:
        if os.name == 'nt':
            import msvcrt
            if path.stat().st_size == 0:
                handle.write(b'0')
                handle.flush()
            handle.seek(0)
            msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
        else:
            import fcntl
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return handle
    except OSError:
        handle.close()
        raise ValueError('已有离线导出任务运行，请等待完成')


def pick(kind, env):
    if os.name != 'nt':
        raise ValueError('本机目录选择适用于 Windows；可导入 bag 并使用默认输出目录')
    if kind not in ('bag', 'folder'):
        raise ValueError('请选择 bag 或 folder')
    code = '''import json,sys,tkinter as tk
from tkinter import filedialog
root=tk.Tk();root.withdraw();root.attributes('-topmost',True)
try:
 value=filedialog.askopenfilename(parent=root,title='选择本地 ROS1 bag',filetypes=[('ROS1 bag','*.bag')]) if sys.argv[1]=='bag' else filedialog.askdirectory(parent=root,title='选择导出文件夹')
 print(json.dumps(value,ensure_ascii=False))
finally: root.destroy()'''
    result = subprocess.run([sys.executable, '-c', code, kind], env=env, capture_output=True,
                            encoding='utf-8', timeout=180, **process_options())
    if result.returncode:
        raise ValueError('无法打开本机文件对话框，请使用浏览器导入或默认输出目录')
    return json.loads(result.stdout.strip())

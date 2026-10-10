"""Local bag replay library; no SSH, ROS nodes, devices or model inference."""
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import threading
import time
import uuid
from urllib.parse import quote

TOOL = Path(__file__).resolve().parent
REPOSITORY = TOOL.parents[1]
PACKAGED = TOOL / 'bag_replay'
REPLAY_TOOL = PACKAGED if PACKAGED.is_dir() else TOOL.parent / 'bag_replay'
ASSETS = {'.html', '.mp4', '.webm', '.png', '.jpg', '.jpeg', '.svg', '.json',
          '.csv', '.md', '.txt', '.css', '.js', '.vtt'}
ACTIVE = {'queued', 'running'}
IS_WINDOWS = os.name == 'nt'


def worker_environment():
    env = dict(os.environ, PYTHONUNBUFFERED='1')
    # start_workbench.sh adds this solely to borrow pexpect. ROS_PYTHON finds
    # its own system site-packages; conda must not load Ubuntu's Python 3.8 numpy.
    if 'PYTHONPATH' in env:
        paths = [p for p in env['PYTHONPATH'].split(os.pathsep)
                 if p and os.path.normpath(p) != '/usr/lib/python3/dist-packages']
        if paths:
            env['PYTHONPATH'] = os.pathsep.join(paths)
        else:
            env.pop('PYTHONPATH')
    return env


def relative(value, empty=False):
    if empty and value == '':
        return ''
    if (not isinstance(value, str) or not value or len(value) > 1024
            or '\\' in value or ':' in value or any(ord(c) < 32 for c in value)
            or value.startswith('/') or any(p in ('', '.', '..') or p.startswith('.')
                                            for p in value.split('/'))):
        raise ValueError('非法本地相对路径')
    return value


def inside(root, value, empty=False):
    relative(value, empty)
    path = root
    if root.is_symlink():
        raise ValueError('数据根不能是符号链接')
    for part in PurePosixPath(value).parts:
        path = path / part
        if path.is_symlink():
            raise ValueError('不允许符号链接资源')
    path.resolve().relative_to(root.resolve())
    return path


class ReplayLibrary:
    def __init__(self, profile_dir, settings=None):
        settings = settings or {}
        self.output = Path(profile_dir).resolve() / 'replay_library'
        if self.output.is_symlink():
            raise ValueError('回放输出根不能是符号链接')
        defaults = {}
        if (REPOSITORY / 'AGENTS.md').is_file():
            defaults['flights'] = REPOSITORY / '试飞产物'
        defaults['data'] = TOOL / 'data'
        roots = settings.get('data_roots', defaults)
        if not isinstance(roots, dict):
            raise ValueError('replay.data_roots 必须为名称到本地路径的映射')
        self.roots = {}
        for key, value in roots.items():
            if not re.fullmatch(r'[a-zA-Z0-9_-]+', key) or key == 'generated':
                raise ValueError('非法 replay 数据根名称')
            path = Path(value).expanduser()
            if not path.is_absolute():
                path = TOOL / path
            if path.is_symlink():
                raise ValueError('数据根不能是符号链接')
            self.roots[key] = path.resolve()
        self.roots['generated'] = self.output
        self.lock = threading.RLock()
        self.thread = None
        self.child = None
        self.cached_capabilities = None

    def root(self, key):
        if not isinstance(key, str) or key not in self.roots:
            raise ValueError('请选择预定本地数据根')
        return self.roots[key]

    def capabilities(self):
        if self.cached_capabilities is not None:
            return self.cached_capabilities
        missing = []
        if IS_WINDOWS:
            missing.append('Windows 原生后端仅查看已生成回放；生成请在已有 ROS Noetic 的 WSL/Linux 内启动工作台，不自动桥接 WSL')
        else:
            for command in ('bash', 'ffmpeg', 'ffprobe'):
                if not shutil.which(command):
                    missing.append('缺少 ' + command)
            setup = Path(os.environ.get('ROS_SETUP', '/opt/ros/noetic/setup.bash'))
            if not setup.is_file():
                missing.append('缺少 ROS Noetic setup（可用 ROS_SETUP 指向已有环境）')
            elif shutil.which('bash'):
                # Same interpreters as the existing run.sh; imports only, no ROS master.
                probe = '''set +u
source "$ROS_SETUP"
"${ROS_PYTHON:-/usr/bin/python3}" -c 'import rosbag, numpy'
if [[ -z "${REPLAY_PYTHON:-}" ]]; then
  if [[ -f /home/xhj/miniconda3/etc/profile.d/conda.sh ]]; then
    source /home/xhj/miniconda3/etc/profile.d/conda.sh
    conda activate rl_drone
  fi
  REPLAY_PYTHON=$(command -v python3)
fi
"$REPLAY_PYTHON" -c 'import numpy, scipy, cv2'
'''
                try:
                    result = subprocess.run(['bash', '-e', '-c', probe], capture_output=True,
                                            timeout=20, env=dict(worker_environment(), ROS_SETUP=str(setup)))
                    if result.returncode:
                        missing.append('ROS 读取环境需 rosbag/NumPy；REPLAY_PYTHON 渲染环境需 NumPy/SciPy/OpenCV（沿用已有环境，不自动安装）')
                except (OSError, subprocess.TimeoutExpired):
                    missing.append('回放依赖检查失败或超时')
        if not all((REPLAY_TOOL / p).is_file() for p in ('run.sh', 'bag_replay.py')):
            missing.append('缺少随包 bag_replay/run.sh 或 bag_replay.py')
        self.cached_capabilities = {'generate': not missing, 'view': True,
                                    'detail': '；'.join(missing) or '本机离线生成可用：只读 bag，不启动 ROS 节点或重新推理'}
        return self.cached_capabilities

    def player_url(self, key, directory):
        return '/replay/result/' + key + '/' + (quote(directory, safe='/') + '/' if directory else '')

    def validated(self, directory):
        try:
            summary = json.loads(inside(directory, 'summary.json').read_text(encoding='utf-8'))
            validation = json.loads(inside(directory, 'validation.json').read_text(encoding='utf-8'))
            videos = summary.get('video_files')
            return (isinstance(videos, list) and bool(videos) and isinstance(validation, dict)
                    and all(isinstance(name, str) and name.endswith('.mp4')
                            and name in validation and inside(directory, name).is_file()
                            and inside(directory, name).stat().st_size > 0
                            and int(validation[name]['size']) == inside(directory, name).stat().st_size
                            for name in videos))
        except (OSError, ValueError, TypeError, AttributeError, KeyError):
            return False

    def catalog(self):
        bags, results = [], []
        truncated = False
        for key, root in self.roots.items():
            if not root.is_dir():
                continue
            visited = 0
            for directory, dirs, names in os.walk(root, followlinks=False):
                visited += 1
                base = Path(directory)
                dirs[:] = sorted(d for d in dirs if not d.startswith('.')
                                 and not (base / d).is_symlink() and d != 'frames')
                if len(base.relative_to(root).parts) >= 8:
                    truncated = truncated or bool(dirs)
                    dirs[:] = []
                if visited > 2000 or len(bags) + len(results) >= 2000:
                    truncated = True
                    break
                for name in sorted(names):
                    path = base / name
                    if path.is_symlink() or not path.is_file():
                        continue
                    if path.suffix.lower() == '.bag':
                        stat = path.stat()
                        bags.append({'root': key, 'path': path.relative_to(root).as_posix(),
                                     'size': stat.st_size, 'mtime': stat.st_mtime})
                    elif name == 'index.html':
                        if not self.validated(base):
                            continue
                        directory = base.relative_to(root).as_posix()
                        if directory == '.':
                            directory = ''
                        # Keep in-progress/failed generated results out of the player list.
                        if key == 'generated':
                            job = self._read(base.parent)
                            if job.get('state') != 'ready':
                                continue
                        results.append({'root': key, 'path': directory,
                                        'mtime': path.stat().st_mtime,
                                        'url': self.player_url(key, directory)})
        return {'ok': True, 'bags': sorted(bags, key=lambda r: r['mtime'], reverse=True),
                'results': sorted(results, key=lambda r: r['mtime'], reverse=True),
                'roots': {k: str(v) for k, v in self.roots.items()}, 'truncated': truncated,
                'capabilities': self.capabilities(), 'jobs': self.jobs()}

    def _read(self, directory):
        try:
            return json.loads((directory / 'job.json').read_text(encoding='utf-8'))
        except (OSError, ValueError):
            return {}

    def _save(self, directory, job):
        with self.lock:
            temporary = directory / '.job.tmp'
            temporary.write_text(json.dumps(job, ensure_ascii=False), encoding='utf-8')
            temporary.replace(directory / 'job.json')

    def jobs(self):
        records = []
        with self.lock:
            if self.output.is_dir():
                for directory in self.output.iterdir():
                    if directory.is_symlink() or not re.fullmatch(r'[0-9a-f]{32}', directory.name):
                        continue
                    job = self._read(directory)
                    if not job:
                        continue
                    if job.get('state') in ACTIVE and not self._occupied():
                        job.update(state='failed', error='上次服务中断，回放未完成；可重新生成到新目录')
                        self._save(directory, job)
                    try:
                        with (directory / 'operation.log').open('rb') as log:
                            log.seek(max(0, (directory / 'operation.log').stat().st_size - 12000))
                            job['tail'] = log.read().decode('utf-8', 'replace')
                    except OSError:
                        job['tail'] = ''
                    records.append(job)
        return sorted(records, key=lambda j: j['created_at'], reverse=True)[:100]

    def _occupied(self):
        if IS_WINDOWS:
            return False  # Native Windows never launches a replay worker.
        import fcntl
        if self.output.is_symlink() or (self.output / '.lock').is_symlink():
            raise ValueError('回放输出根或锁不能是符号链接')
        try:
            with (self.output / '.lock').open('a') as handle:
                try:
                    fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    return True
        except OSError:
            return False
        return False

    def _claim(self):
        # OS-owned lock survives page refresh and is released on process exit.
        # Shared output root also excludes a second workbench using the same profile.
        import fcntl
        if self.output.is_symlink() or (self.output / '.lock').is_symlink():
            raise ValueError('回放输出根或锁不能是符号链接')
        self.output.mkdir(parents=True, exist_ok=True)
        handle = (self.output / '.lock').open('a')
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            handle.close()
            raise ValueError('已有离线回放任务运行，请等待完成')
        for directory in self.output.iterdir():
            if re.fullmatch(r'[0-9a-f]{32}', directory.name) and not directory.is_symlink():
                job = self._read(directory)
                if job.get('state') in ACTIVE:
                    job.update(state='failed', error='上次服务中断，回放未完成；可重新生成到新目录')
                    self._save(directory, job)
        return handle

    def start(self, body):
        if not isinstance(body, dict):
            raise ValueError('请求体必须是 JSON 对象')
        fps, frame = body.get('fps', 10), body.get('frame', 'map')
        encoder = body.get('encoder', 'cpu')
        if not isinstance(encoder, str) or encoder not in ('auto', 'cpu', 'nvenc'):
            raise ValueError('encoder 必须为 auto、cpu 或 nvenc')
        if type(fps) is not int or not 1 <= fps <= 30:
            raise ValueError('fps 必须为 1–30 的整数')
        if not isinstance(frame, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_/]{0,127}', frame):
            raise ValueError('非法显示坐标系')
        bag = inside(self.root(body.get('root')), body.get('path'))
        if not bag.is_file() or bag.suffix.lower() != '.bag':
            raise ValueError('请选择预定数据根内已封闭的 .bag 文件')
        with bag.open('rb') as stream:
            if stream.read(13) != b'#ROSBAG V2.0\n':
                raise ValueError('不是有效的 ROS1 bag 文件头')
        if not self.capabilities()['generate']:
            raise ValueError(self.capabilities()['detail'])
        with self.lock:
            if self.thread is not None and self.thread.is_alive():
                raise ValueError('已有离线回放任务运行，请等待完成')
            claim = self._claim()
            try:
                identifier = uuid.uuid4().hex
                directory = inside(self.output, identifier)
                directory.mkdir()
                job = {'id': identifier, 'root': body['root'], 'path': body['path'],
                       'fps': fps, 'frame': frame, 'state': 'queued', 'phase': '等待启动',
                       'encoder': encoder,
                       'progress': None, 'created_at': time.time(),
                       'url': self.player_url('generated', identifier + '/result')}
                self._save(directory, job)
                self.thread = threading.Thread(target=self._run,
                                               args=(directory, bag, job, claim), daemon=True)
                self.thread.start()
            except Exception:
                claim.close()
                raise
            return {'ok': True, 'id': identifier}

    def _run(self, directory, bag, job, claim):
        child = None
        try:
            args = ['bash', str(REPLAY_TOOL / 'run.sh'), str(bag), str(directory / 'result'),
                    '--fps', str(job['fps']), '--frame', job['frame'], '--encoder', job['encoder']]
            job.update(state='running', phase='导出 bag（此阶段不提供百分比）')
            self._save(directory, job)
            child = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, start_new_session=True,
                                     env=worker_environment(),
                                     pass_fds=(claim.fileno(),))
            with self.lock:
                self.child = child
            with (directory / 'operation.log').open('wb') as log:
                for raw in iter(child.stdout.readline, b''):
                    log.write(raw)
                    log.flush()
                    line = raw.decode('utf-8', 'replace')
                    if line.startswith('Exported '):
                        job.update(phase='渲染视频', progress=None)
                    match = re.search(r'Render (\d+)/(\d+)', line)
                    if match:
                        done, total = map(int, match.groups())
                        job.update(phase='渲染视频', progress=round(100 * done / max(1, total), 1))
                    if line.startswith('All videos decoded;'):
                        job.update(phase='视频校验完成', progress=100)
                    self._save(directory, job)
            code = child.wait()
            if code or not self.validated(directory / 'result'):
                raise ValueError('回放失败（exit=%s），请查看 operation.log；不提供未验证播放器' % code)
            job.update(state='ready', phase='完成', progress=100, exit_code=code)
        except Exception as error:
            job.update(state='failed', phase='失败', error=str(error))
        finally:
            if child is not None:
                if child.stdout:
                    child.stdout.close()
                # Keep the exclusion lock until the owned process has exited.
                child.wait()
            with self.lock:
                self.child = None
            job['finished_at'] = time.time()
            try:
                self._save(directory, job)
            finally:
                claim.close()

    def close(self):
        # Stop only the offline job tree; never any ROS/device sessions.
        with self.lock:
            child = self.child
        if child is not None and child.poll() is None and not IS_WINDOWS:
            import signal
            try:
                os.killpg(child.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        if self.thread is not None:
            self.thread.join(timeout=5)

    def artifact(self, key, value):
        path = inside(self.root(key), value, empty=True)
        if path.is_dir():
            path = inside(self.root(key), (value + '/' if value else '') + 'index.html')
        if not path.is_file() or path.suffix.lower() not in ASSETS:
            raise ValueError('回放资源不存在或类型不允许')
        base = path.parent
        while not self.validated(base):
            if base == self.root(key):
                raise ValueError('资源不属于已验证的回放目录')
            base = base.parent
        if key == 'generated':
            identifier = value.split('/')[0]
            job = self._read(inside(self.output, identifier))
            if job.get('state') != 'ready' or not (value == identifier + '/result'
                                                  or value.startswith(identifier + '/result/')):
                raise ValueError('回放结果尚未验证完成')
        return path

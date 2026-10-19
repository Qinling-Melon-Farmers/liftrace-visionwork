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
import wb_offline_support as native

TOOL = Path(__file__).resolve().parent
REPOSITORY = TOOL.parents[1]
PACKAGED = TOOL / 'bag_replay'
REPLAY_TOOL = TOOL / 'offline_replay'
ASSETS = {'.html', '.mp4', '.webm', '.png', '.jpg', '.jpeg', '.svg', '.json',
          '.csv', '.md', '.txt', '.css', '.js', '.vtt'}
ACTIVE = {'queued', 'running'}
IS_WINDOWS = os.name == 'nt'


def worker_environment():
    env = dict(os.environ, PYTHONUNBUFFERED='1', PYTHONIOENCODING='utf-8')
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
    for part in value.split('/'):
        if (part.endswith((' ', '.')) or any(c in part for c in '<>"|?*')
                or re.fullmatch(r'(CON|PRN|AUX|NUL|COM[1-9]|LPT[1-9])', part.split('.')[0], re.I)):
            raise ValueError('本地路径不兼容 Windows 文件名')
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
            if not re.fullmatch(r'[a-zA-Z0-9_-]+', key) or key in ('generated', 'imports') or key.startswith(('picked_', 'export_')):
                raise ValueError('非法 replay 数据根名称')
            path = Path(value).expanduser()
            if not path.is_absolute():
                path = TOOL / path
            if path.is_symlink():
                raise ValueError('数据根不能是符号链接')
            self.roots[key] = path.resolve()
        self.roots['generated'] = self.output
        self.roots['imports'] = self.output / 'imports'
        self.output_locations = {}
        self._restore_locations()
        self.lock = threading.RLock()
        self.thread = None
        self.child = None
        self.stopping = threading.Event()
        self.cached_capabilities = None

    def root(self, key):
        if not isinstance(key, str) or key not in self.roots:
            raise ValueError('请选择预定本地数据根')
        return self.roots[key]

    def capabilities(self, refresh=False):
        if refresh or self.cached_capabilities is None:
            self.cached_capabilities = native.dependencies(worker_environment())
        return self.cached_capabilities

    def _restore_locations(self):
        try:
            locations = self.output.parent / 'offline_locations.json'
            if locations.is_symlink():
                return
            saved = json.loads(locations.read_text(encoding='utf-8'))
            if not isinstance(saved, dict) or not all(isinstance(saved.get(k, {}), dict) for k in ('sources', 'outputs')):
                return
            for key, value in saved.get('sources', {}).items():
                if isinstance(key, str) and isinstance(value, str) and re.fullmatch(r'picked_[0-9a-f]{32}', key):
                    path = Path(value)
                    if path.is_absolute() and not path.is_symlink():
                        self.roots[key] = path
            for key, value in saved.get('outputs', {}).items():
                if isinstance(key, str) and isinstance(value, str) and re.fullmatch(r'export_[0-9a-f]{32}', key):
                    path = Path(value)
                    if path.is_absolute() and not path.is_symlink():
                        self.roots[key] = path
                        self.output_locations[key] = str(path)
        except (OSError, ValueError, TypeError):
            pass

    def select_local(self, body):
        if not isinstance(body, dict) or body.get('kind') not in ('bag', 'folder'):
            raise ValueError('请选择本地 bag 或导出文件夹')
        value = native.pick(body['kind'], worker_environment())
        if not value:
            return {'ok': True, 'cancelled': True}
        selected = Path(value)
        if selected.is_symlink():
            raise ValueError('不允许选择符号链接')
        selected = selected.resolve()
        if body['kind'] == 'bag':
            relative(selected.name)
            if selected.suffix.lower() != '.bag' or not selected.is_file():
                raise ValueError('请选择已封闭的 ROS1 .bag 文件')
            with selected.open('rb') as source:
                if source.read(13) != b'#ROSBAG V2.0\n':
                    raise ValueError('不是 ROS1 bag 文件')
        with self.lock:
            key = ('picked_' if body['kind'] == 'bag' else 'export_') + uuid.uuid4().hex
            root = selected.parent if body['kind'] == 'bag' else selected
            if root.is_symlink() or not root.is_dir():
                raise ValueError('选择的本地目录无效')
            existing = self.output_locations.items() if body['kind'] == 'folder' else ((k, v) for k, v in self.roots.items() if k not in ('generated', 'imports') and not k.startswith('export_'))
            key = next((k for k, value in existing if Path(value) == root), key)
            self.roots[key] = root
            if body['kind'] == 'folder':
                self.output_locations[key] = str(root)
            self.output.parent.mkdir(parents=True, exist_ok=True)
            payload = {'sources': {k: str(v) for k, v in self.roots.items() if k.startswith('picked_')}, 'outputs': self.output_locations}
            temp = self.output.parent / '.offline_locations.tmp'
            temp.write_text(json.dumps(payload, ensure_ascii=False), encoding='utf-8')
            temp.replace(self.output.parent / 'offline_locations.json')
        return {'ok': True, 'root': key, 'path': selected.name if body['kind'] == 'bag' else '', 'directory': str(root)}

    def ingest(self, stream, length, name):
        if not isinstance(name, str) or name.lower().endswith('.bag') is False:
            raise ValueError('请选择已封闭的 ROS1 .bag 文件')
        relative(name)
        if '/' in name or type(length) is not int or not 13 <= length <= 4 * 1024**3:
            raise ValueError('bag 文件大小必须在 13 字节至 4 GiB 之间')
        identifier = uuid.uuid4().hex
        directory = inside(self.roots['imports'], identifier)
        directory.mkdir(parents=True)
        partial = directory / (name + '.part')
        try:
            with partial.open('xb') as output:
                remaining = length
                while remaining:
                    chunk = stream.read(min(1024*1024, remaining))
                    if not chunk:
                        raise ValueError('bag 导入未完成，请重新导入')
                    output.write(chunk)
                    remaining -= len(chunk)
            with partial.open('rb') as source:
                if source.read(13) != b'#ROSBAG V2.0\n':
                    raise ValueError('不是 ROS1 bag 文件')
            target = directory / name
            partial.rename(target)
            return {'ok': True, 'root': 'imports', 'path': identifier + '/' + name}
        except Exception:
            if partial.exists():
                partial.unlink()
            raise

    def validated_analysis(self, directory):
        try:
            summary = json.loads(inside(directory, 'analysis_summary.json').read_text(encoding='utf-8'))
            images = summary['images']
            return (isinstance(images, list) and bool(images) and inside(directory, 'analysis_report.html').is_file()
                    and all(isinstance(name, str) and name.endswith('.png') and inside(directory, name).is_file()
                            and inside(directory, name).stat().st_size > 0 for name in images))
        except (OSError, ValueError, TypeError, AttributeError, KeyError):
            return False

    def _ready_result(self, key, directory):
        if key != 'generated' and not key.startswith('export_'):
            return True
        candidates = [inside(self.output, directory.split('/')[0])] if key == 'generated' else list(self.output.iterdir()) if self.output.is_dir() else []
        for folder in candidates:
            if folder.is_symlink() or not re.fullmatch(r'[0-9a-f]{32}', folder.name):
                continue
            job = self._read(folder)
            if (job.get('result_root', 'generated') == key and job.get('result_path', folder.name+'/result') == directory
                    and job.get('state') == 'ready'):
                return True
        return False

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
        with self.lock:
            roots = list(self.roots.items())
        for key, root in roots:
            if not root.is_dir():
                continue
            visited = 0
            for directory, dirs, names in os.walk(root, followlinks=False):
                visited += 1
                base = Path(directory)
                dirs[:] = sorted(d for d in dirs if not d.startswith('.')
                                 and not (base / d).is_symlink() and d != 'frames'
                                 and not (key == 'generated' and base == root and d == 'imports'))
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
                    elif name in ('index.html', 'analysis_report.html'):
                        kind = 'analysis' if name == 'analysis_report.html' else 'video'
                        if not (self.validated_analysis(base) if kind == 'analysis' else self.validated(base)):
                            continue
                        directory = base.relative_to(root).as_posix()
                        if directory == '.':
                            directory = ''
                        if not self._ready_result(key, directory):
                            continue
                        results.append({'root': key, 'path': directory,
                                        'mtime': path.stat().st_mtime,
                                        'kind': kind, 'url': self.player_url(key, directory)+(name if kind == 'analysis' else '')})
        return {'ok': True, 'bags': sorted(bags, key=lambda r: r['mtime'], reverse=True),
                'results': sorted(results, key=lambda r: r['mtime'], reverse=True),
                'roots': {k: str(v) for k, v in self.roots.items()}, 'truncated': truncated,
                'capabilities': self.capabilities(refresh=True), 'jobs': self.jobs(), 'output_locations': self.output_locations}

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
        if self.output.is_symlink():
            raise ValueError('回放输出根不能是符号链接')
        try:
            handle = native.acquire(self.output / '.lock')
        except ValueError:
            return True
        handle.close()
        return False

    def _claim(self):
        if self.output.is_symlink():
            raise ValueError('回放输出根不能是符号链接')
        handle = native.acquire(self.output / '.lock')
        for directory in self.output.iterdir():
            if re.fullmatch(r'[0-9a-f]{32}', directory.name) and not directory.is_symlink():
                job = self._read(directory)
                if job.get('state') in ACTIVE:
                    job.update(state='failed', error='上次服务中断，可重新导出到新目录')
                    self._save(directory, job)
        return handle

    def start(self, body):
        if not isinstance(body, dict):
            raise ValueError('请求体必须是 JSON 对象')
        kind = body.get('kind', 'video')
        if kind not in ('video', 'analysis'):
            raise ValueError('请选择 video 或 analysis')
        fps, frame = body.get('fps', 10), body.get('frame', 'map')
        speed, video_set = body.get('speed', 'balanced'), body.get('video_set', 'all')
        if speed not in ('balanced', 'fast') or video_set not in ('all', 'dashboard'):
            raise ValueError('非法视频导出参数')
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
        capabilities = self.capabilities(refresh=True)
        if not capabilities['analysis' if kind == 'analysis' else 'generate']:
            raise ValueError(capabilities['analysis_detail' if kind == 'analysis' else 'detail'])
        output_key = body.get('output_key', 'generated')
        if output_key != 'generated' and output_key not in self.output_locations:
            raise ValueError('请选择默认目录或通过本机对话框选择的导出文件夹')
        with self.lock:
            if self.thread is not None and self.thread.is_alive():
                raise ValueError('已有离线回放任务运行，请等待完成')
            claim = self._claim()
            try:
                identifier = uuid.uuid4().hex
                directory = inside(self.output, identifier)
                directory.mkdir()
                result_path = identifier+'/result' if output_key == 'generated' else identifier+'_'+kind
                result = inside(self.root(output_key), result_path)
                job = {'id': identifier, 'root': body['root'], 'path': body['path'],
                       'fps': fps, 'frame': frame, 'state': 'queued', 'phase': '等待启动',
                       'encoder': encoder, 'kind': kind, 'speed': speed, 'video_set': video_set,
                       'result_root': output_key, 'result_path': result_path, 'output_directory': str(result),
                       'progress': None, 'created_at': time.time(),
                       'url': self.player_url(output_key, result_path)+('analysis_report.html' if kind == 'analysis' else '')}
                self._save(directory, job)
                self.stopping.clear()
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
            result = inside(self.root(job['result_root']), job['result_path'])
            if job['kind'] == 'analysis':
                args = [native.python_command(), str(REPLAY_TOOL/'flight_analysis.py'), str(bag), str(result)]
            else:
                args = [native.python_command(), str(REPLAY_TOOL/'run.py'), str(bag), str(result),
                        '--fps', str(job['fps']), '--frame', job['frame'], '--encoder', job['encoder'],
                        '--speed', job['speed'], '--video-set', job['video_set']]
            job.update(state='running', phase='读取本地 bag')
            self._save(directory, job)
            options = native.process_options()
            if not IS_WINDOWS:
                options['pass_fds'] = (claim.fileno(),)
            child = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, env=worker_environment(), **options)
            with self.lock:
                self.child = child
            if self.stopping.is_set():
                self._stop_child(child)
            with (directory / 'operation.log').open('wb') as log:
                for raw in iter(child.stdout.readline, b''):
                    log.write(raw)
                    log.flush()
                    line = raw.decode('utf-8', 'replace')
                    if line.startswith('Stage: analyze'):
                        job.update(phase='分析投递与降落模式', progress=None)
                    if line.startswith('Stage: plot'):
                        job.update(phase='导出曲线与报告', progress=None)
                    if line.startswith('Stage: verify'):
                        job.update(phase='完整解码与时长校验', progress=None)
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
            if self.stopping.is_set():
                raise ValueError('用户已停止当前离线导出')
            verified = self.validated_analysis(result) if job['kind'] == 'analysis' else self.validated(result)
            if code or not verified:
                raise ValueError('回放失败（exit=%s），请查看 operation.log；不提供未验证播放器' % code)
            job.update(state='ready', phase='完成', progress=100, exit_code=code)
        except Exception as error:
            job.update(state='failed', phase='已停止' if self.stopping.is_set() else '失败', error=str(error))
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
        self.stopping.set()
        with self.lock:
            child = self.child
        self._stop_child(child)
        if self.thread is not None:
            self.thread.join(timeout=5)

    def _stop_child(self, child):
        if child is not None and child.poll() is None and IS_WINDOWS:
            subprocess.run(['taskkill', '/PID', str(child.pid), '/T', '/F'], capture_output=True, timeout=15,
                           creationflags=subprocess.CREATE_NO_WINDOW)
        elif child is not None and child.poll() is None:
            import signal
            try:
                os.killpg(child.pid, signal.SIGTERM)
                try:
                    child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait(timeout=3)
            except ProcessLookupError:
                pass

    def artifact(self, key, value):
        path = inside(self.root(key), value, empty=True)
        if path.is_dir():
            name = 'analysis_report.html' if self.validated_analysis(path) else 'index.html'
            path = inside(self.root(key), (value + '/' if value else '') + name)
        if not path.is_file() or path.suffix.lower() not in ASSETS:
            raise ValueError('回放资源不存在或类型不允许')
        base = path.parent
        while not (self.validated(base) or self.validated_analysis(base)):
            if base == self.root(key):
                raise ValueError('资源不属于已验证的回放目录')
            base = base.parent
        directory = base.relative_to(self.root(key)).as_posix()
        if directory == '.':
            directory = ''
        if not self._ready_result(key, directory):
            raise ValueError('导出结果尚未验证完成')
        return path

"""Local-only ULog library and isolated offline analysis; no SSH or ROS imports."""
import json
import re
import subprocess
import sys
import threading
import time
import uuid
import zipfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

MAX_BYTES = 128 * 1024 * 1024
MAGIC = b'ULog\x01\x12\x35'
TOOL = Path(__file__).resolve().parent
VIEWER = TOOL / 'ulog_viewer'
if not VIEWER.is_dir():
    VIEWER = TOOL.parent / 'flight_logs' / 'ulg_motor_viewer'


class ULogLibrary:
    def __init__(self, profile_dir):
        self.root = Path(profile_dir) / 'ulog_library'
        self.lock = threading.RLock()
        self.pool = None  # Reading the library does not start workers.

    def directory(self, identifier):
        if not isinstance(identifier, str) or not re.fullmatch(r'[0-9a-f]{32}', identifier):
            raise ValueError('非法本地 ULog 编号')
        directory = self.root / identifier
        if directory.is_symlink() or not (directory / 'record.json').is_file():
            raise ValueError('本地 ULog 不存在')
        return directory

    def record(self, identifier):
        directory = self.directory(identifier)
        data = json.loads((directory / 'record.json').read_text(encoding='utf-8'))
        if data['state'] in ('queued', 'analyzing') and data.get('owner_pid') != __import__('os').getpid():
            data.update(state='failed', error='上次分析随本地服务退出而中断，请重新导入。')
        if data['state'] == 'ready':
            data['files'] = sorted(p.name for p in (directory / 'result').iterdir()
                                   if p.is_file() and p.suffix in ('.png', '.json', '.txt', '.csv'))
            data['summary'] = json.loads((directory / 'result' / 'summary.json').read_text(encoding='utf-8'))
        data.pop('owner_pid', None)
        return data

    def catalog(self):
        with self.lock:
            records = []
            if self.root.is_dir():
                for directory in self.root.iterdir():
                    if re.fullmatch(r'[0-9a-f]{32}', directory.name) and not directory.is_symlink():
                        try:
                            records.append(self.record(directory.name))
                        except (OSError, ValueError):
                            continue
            return {'ok': True, 'records': sorted(records, key=lambda r: r['created_at'], reverse=True)}

    def ingest(self, name, payload):
        name = str(name).replace('\\', '/').rsplit('/', 1)[-1]
        if not name.lower().endswith('.ulg') or len(name) > 240 or any(ord(c) < 32 for c in name):
            raise ValueError('请选择已封闭的 .ulg 文件')
        if not 16 <= len(payload) <= MAX_BYTES or not payload.startswith(MAGIC):
            raise ValueError('ULog 文件头或大小无效（上限128 MiB）；不接受 .part')
        with self.lock:
            if self.pool is None:
                self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='ulog-offline')
            identifier = uuid.uuid4().hex
            directory = self.root / identifier
            directory.mkdir(parents=True)
            (directory / 'input.ulg').write_bytes(payload)
            data = {'id': identifier, 'name': name, 'size': len(payload), 'created_at': time.time(),
                    'state': 'queued', 'owner_pid': __import__('os').getpid()}
            self._save(directory, data)
            self.pool.submit(self._analyze, directory, data)
            return {'ok': True, 'id': identifier}

    def _save(self, directory, data):
        # All reads/writes use this lock, including on Windows.
        with self.lock:
            (directory / 'record.json').write_text(json.dumps(data, ensure_ascii=False), encoding='utf-8')

    def _analyze(self, directory, data):
        data['state'] = 'analyzing'
        self._save(directory, data)
        try:
            result = subprocess.run([sys.executable, str(Path(__file__).resolve()), '--analyze',
                                     str(directory), str(VIEWER)], capture_output=True, timeout=180)
            if result.returncode:
                reason = result.stderr.decode('utf-8', 'replace')
                missing = 'ModuleNotFoundError' in reason or 'ImportError' in reason
                raise ValueError('分析依赖缺失：请使用已有含 numpy、matplotlib 的环境，并保留随包 pyulog。' if missing
                                 else '日志解析失败：请确认文件已完整下载且含可用数据。')
            with zipfile.ZipFile(directory / 'analysis.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
                for path in sorted((directory / 'result').iterdir()):
                    if path.is_file():
                        archive.write(path, path.name)
            data['state'] = 'ready'
        except (OSError, ValueError, subprocess.TimeoutExpired):
            error = sys.exc_info()[1]
            data.update(state='failed', error=str(error) if isinstance(error, ValueError)
                        else '离线分析失败或超时，请重新导入；未连接飞控。')
        self._save(directory, data)

    def artifact(self, identifier, name):
        with self.lock:
            directory = self.directory(identifier)
            record = self.record(identifier)
            if name == 'input.ulg':
                path = directory / name
            elif record['state'] == 'ready' and name == 'analysis.zip':
                path = directory / name
            elif record['state'] == 'ready' and name in record.get('files', []):
                path = directory / 'result' / name
            else:
                raise ValueError('分析文件尚未就绪或不存在')
            if path.is_symlink() or not path.is_file():
                raise ValueError('分析文件不存在')
            return path


if __name__ == '__main__':
    if len(sys.argv) != 4 or sys.argv[1] != '--analyze':
        raise SystemExit('Offline worker only')
    directory = Path(sys.argv[2])
    sys.path.insert(0, sys.argv[3])
    from analysis import read_log, export
    from trajectory import export_trajectory
    analysis = read_log(directory / 'input.ulg')
    record = json.loads((directory / 'record.json').read_text(encoding='utf-8'))
    analysis.path = Path(record['name'])  # Reports contain the display name, no local profile path.
    report = export(analysis, directory / 'result')
    report['source_file'] = record['name']
    (directory / 'result' / 'summary.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    export_trajectory(analysis, directory / 'result')

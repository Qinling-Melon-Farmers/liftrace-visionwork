"""Download names from the recorded run, never from current UI selections."""
import re
from datetime import datetime, timezone, timedelta
from pathlib import PurePosixPath
from urllib.parse import quote


REMOTE_INFO = r'''
def download_info(root, path):
    import re
    info = {"original": path.name, "settings": {}}
    # Search only this file's ancestor directories, never neighboring runs.
    for parent in path.parents:
        if parent == root:
            break
        meta_path = inside(root, parent / "run_metadata.json")
        if meta_path.is_file() and meta_path.stat().st_size <= 2*1024*1024:
            meta = read_json(meta_path)
            settings = meta.get("settings")
            if isinstance(settings, dict):
                info["settings"] = settings
                info["source"] = "run_metadata.json"
                break
    # A separate recorder includes the actual run metadata as a latched topic.
    if path.suffix == ".bag" and not info["settings"]:
        try:
            import rosbag
            with rosbag.Bag(str(path), "r") as bag:
                info["started_at"] = bag.get_start_time()
                for topic, message, stamp in bag.read_messages(topics=["/board_trials/run_metadata"]):
                    meta = json.loads(message.data)
                    if isinstance(meta.get("settings"), dict):
                        info["settings"] = meta["settings"]
                        info["source"] = "bag run_metadata topic"
                        break
        except Exception:
            pass  # Naming must never prevent an otherwise valid download.
    for part in reversed(path.relative_to(root).parts):
        match = re.search(r"(?<!\d)(20\d{6})[_-](\d{6})(?!\d)", part)
        if match:
            info["timestamp"] = match.group(1) + match.group(2)
            break
    if not info.get("timestamp") and not info.get("started_at"):
        for parent in path.parents:
            if parent == root:
                break
            job_path = inside(root, parent / "job.json")
            if job_path.is_file() and job_path.stat().st_size <= 2*1024*1024:
                info["started_at"] = read_json(job_path).get("started_at")
                if info["started_at"]:
                    break
    return info
'''


def download_name(path, info):
    original = PurePosixPath(path).name
    suffix = '.bag.active' if original.endswith('.bag.active') else '.bag' if original.endswith('.bag') else None
    if not suffix:
        return original
    try:
        if info.get('timestamp'):
            started = datetime.strptime(info['timestamp'], '%Y%m%d%H%M%S')
        else:
            started = datetime.fromtimestamp(float(info['started_at']), timezone(timedelta(hours=8)))
    except (KeyError, ValueError, TypeError, OverflowError, OSError):
        return original
    settings = info.get('settings') or {}
    if not isinstance(settings, dict):
        settings = {}
    def number(key, allowed):
        value = settings.get(key)
        return str(value) if type(value) is int and value in allowed else '未知'
    alignment = settings.get('landing_alignment_mode')
    alignment = {'low': 'non_vh', 'high': 'vh'}.get(alignment, alignment) if isinstance(alignment, str) else None
    if alignment not in ('non_vh', 'vh'):
        alignment = '未知'
    return ('%s-投递H%sD%s-H降落%sD%s%s' % (
        started.strftime('%m%d-%H%M'), number('drop_horizontal_compensation_mode', (1, 2)),
        number('drop_descent_mode', (1, 2, 3, 4)), alignment,
        number('landing_descent_mode', (1, 2, 3, 4)), suffix))


def attachment_header(name):
    # ASCII fallback for old clients, RFC 5987 UTF-8 for Windows/browser Chinese names.
    fallback = ''.join(c if c.isascii() and (c.isalnum() or c in '._-') else '_' for c in name)
    return 'attachment; filename="%s"; filename*=UTF-8\'\'%s' % (fallback, quote(name, safe=''))

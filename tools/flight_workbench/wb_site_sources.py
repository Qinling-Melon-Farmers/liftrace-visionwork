"""Exact source selection for command generation and read-only map references."""
from pathlib import Path
import yaml


def selected_site(group, pattern=None, shiftback=False):
    if type(shiftback) is not bool:raise ValueError("coverage_shiftback must be boolean")
    pattern = 'lawnmower' if pattern is None else pattern
    if pattern not in ('lawnmower', 'nested_rectangles'):
        raise ValueError('Unsupported coverage pattern')
    choices = group.get('site_configs_shiftback_by_pattern' if shiftback else 'site_configs_by_pattern')
    if shiftback and not isinstance(choices,dict):raise ValueError('Missing optional shiftback site configuration')
    source = choices.get(pattern) if isinstance(choices, dict) else group.get('site_config')
    if not isinstance(source, str) or not source.strip() or any(c in source for c in '\r\n\x00'):
        raise ValueError('Missing site_config for selected pattern')
    return source


def reference(source):
    base = (Path(__file__).parent/'waypoint_templates/sites').resolve()
    path = (base/source).resolve()
    if base not in path.parents or not path.is_file():
        raise ValueError('所选新场地文件未部署或无离线副本：'+source+'；不回退旧地图')
    return yaml.safe_load(path.read_text(encoding='utf-8-sig'))

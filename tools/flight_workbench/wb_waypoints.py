"""Flight-before-edit drafts; reuse the existing immutable geometry overlay lane."""
import base64
import copy
import hashlib
import json
from pathlib import Path
import threading
import uuid
import yaml
import wb_board
import wb_geometry
import wb_competition_geometry as competition
from wb_waypoint_validation import validate

LOCK = threading.RLock()
FOLDERS = {'04_corridor_landing': 'module04.yaml', '08_full_mission': 'module08.yaml'}
FIELDS = ('flight_area', 'corridor_waypoints', 'landing_xy', 'corridor_geometry')
READ_SOURCE = '''import json,pathlib,sys,yaml
root=pathlib.Path(sys.argv[1]).resolve(); path=pathlib.Path(sys.argv[2])
if not path.is_absolute():path=root/path
path=path.resolve()
if root not in path.parents:raise ValueError("site outside project")
if path.stat().st_size>2*1024*1024:raise ValueError("site too large")
print("WB_WAYPOINT_SITE="+json.dumps(yaml.safe_load(path.read_text()),ensure_ascii=True))
'''


def describe(workbench, body):
    if not isinstance(body, dict): raise ValueError('请求体必须是对象')
    group = next((g for g in workbench.config.get('groups', []) if g['id'] == body.get('group_id')), None)
    if not group or not (group.get('channel') == 'competition' or group.get('channel') == 'module' and group.get('folder') in FOLDERS):
        raise ValueError('地图调整支持模块04、08与独立正赛')
    source = body.get('competition_config') if group.get('channel') == 'competition' else None
    if source is None: source = group.get('site_config') or '{site_dir}/test_area.yaml'
    if not isinstance(source,str) or not source.strip() or any(c in source for c in '\r\n\x00'):
        raise ValueError('现场配置路径无效')
    source = wb_board.substitute(source.strip(), wb_board.group_variables(workbench.config, group))
    key = [workbench.target.host, workbench.board.root, group['id'], source]
    scope = hashlib.sha256(json.dumps(key, ensure_ascii=True).encode()).hexdigest()[:24]
    path = Path(wb_board.profile_path('waypoint_drafts'))/(scope+'.json')
    return group, source, scope, path


def draft(workbench, body):
    group, source, scope, path = describe(workbench, body)
    with LOCK:
        try:
            data = json.loads(path.read_text(encoding='utf-8'))
            if data.get('scope') != scope: raise ValueError('草稿来源不匹配')
            data['patch'] = (competition.validate_patch if group.get('channel')=='competition' else wb_geometry.validate_geometry)(data['patch'])
        except (OSError, ValueError, KeyError): data = None
    return {'ok': True, 'scope': scope, 'source': source, 'draft': data}


def merge(site, patch):
    result = copy.deepcopy(site)
    for key in (*FIELDS, 'site_confirmed'):
        if key in patch:
            result[key] = dict(result.get(key, {}), **patch[key]) if key == 'flight_area' else copy.deepcopy(patch[key])
    return result


def context(workbench, body):
    group, source, scope, _ = describe(workbench, body)
    loaded = draft(workbench, body)
    if workbench.connection.get('state') == 'ok' and workbench.target.transport != 'local':
        encoded = base64.b64encode(READ_SOURCE.encode()).decode()
        snippet = 'import base64;exec(base64.b64decode(%r))' % encoded
        command = '%s -c %s %s %s' % (wb_board.quote(workbench.config['connection'].get('board_python', '/usr/bin/python3')),
                    wb_board.quote(snippet), wb_board.quote(workbench.board.root), wb_board.quote(source))
        code, output = workbench.board.run(command, timeout=20)
        lines = [line for line in output.splitlines() if line.startswith('WB_WAYPOINT_SITE=')]
        if code or not lines: raise ValueError('无法读取所选板端现场配置；请检查路径与连接，未使用示例代替板端文件')
        site = json.loads(lines[-1].split('=', 1)[1]); origin = '板端所选现场配置'
    else:
        if group.get('channel') == 'competition':
            prefix = 'deployment/competition/'
            if not source.startswith(prefix): raise ValueError('离线未保存这份正赛配置；连接板端读取实际文件，或选择随包正赛模板')
            base = (Path(__file__).parent/'waypoint_templates/competition').resolve()
            template = (base/source[len(prefix):]).resolve()
            if base not in template.parents or not template.is_file(): raise ValueError('离线未保存所选正赛配置；请连接板端读取，未用其他模板代替')
        else: template = Path(__file__).parent/'waypoint_templates'/FOLDERS[group['folder']]
        site = yaml.safe_load(template.read_text(encoding='utf-8-sig'))
        origin = '离线参考模板；实际生效值需通过板端配置检查确认'
    if not isinstance(site, dict): raise ValueError('现场配置必须是对象')
    if group.get('channel') == 'competition': site = competition.editor_site(site)
    # Fill only missing display fields, with clear offline context; no flight values.
    if 'flight_area' not in site: raise ValueError('现场配置缺少flight_area，无法建立米制地图')
    if loaded['draft'] and loaded['draft'].get('enabled', True): site = merge(site, loaded['draft']['patch'])
    if body.get('site_geometry'): site = merge(site, (competition.validate_patch if group.get('channel')=='competition' else wb_geometry.validate_geometry)(body['site_geometry']))
    return dict(ok=True, site=site, source=source, scope=scope, origin=origin,
                name=group['name'], editor_kind='competition' if group.get('channel')=='competition' else 'module', draft=loaded['draft'])


def patch_from_site(site, competition_mode=False):
    if competition_mode:
        if not isinstance(site,dict): raise ValueError('地图必须是对象')
        return competition.validate_patch({key: copy.deepcopy(site[key]) for key in (*FIELDS,'site_confirmed') if key in site})
    errors = validate(site)
    if errors: raise ValueError('；'.join(errors))
    patch = {key: copy.deepcopy(site[key]) for key in FIELDS if key in site}
    return wb_geometry.validate_geometry(patch)


def save(workbench, body):
    group, source, scope, path = describe(workbench, body)
    if body.get('scope') != scope: raise ValueError('连接或现场配置已改变，请重新打开地图')
    trial = workbench.sessions.get('trial')
    if trial and trial.state in ('starting', 'running'): raise ValueError('任务运行期间不能应用航点；先结束当前任务')
    patch = patch_from_site(body.get('site'), group.get('channel')=='competition')
    data = dict(scope=scope, source=source, patch=patch, revision=uuid.uuid4().hex, enabled=True)
    with LOCK:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
        temporary.replace(path)
    return dict(ok=True, **data)


def disable(workbench, body):
    _, _, scope, path = describe(workbench, body)
    trial = workbench.sessions.get('trial')
    if trial and trial.state in ('starting', 'running'): raise ValueError('任务运行期间不能切换航点来源')
    if body.get('scope') != scope: raise ValueError('现场配置来源已变化，请刷新任务组')
    with LOCK:
        data = draft(workbench, body)['draft']
        if data:
            data['enabled'] = False
            temporary = path.with_suffix('.tmp')
            temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
            temporary.replace(path)
    return {'ok': True}


def preview(workbench, body):
    from wb_waypoint_validation import dump_site
    group, _, _, _ = describe(workbench, body)
    try:
        patch_from_site(body.get('site'), group.get('channel')=='competition')
        errors = []
    except ValueError as error: errors = [str(error)]
    return {'ok':True,'errors':errors,'yaml':dump_site(body.get('site'))}


def import_site(workbench, body):
    group, _, _, _ = describe(workbench, body)
    text = body.get('text', '')
    if not isinstance(text,str) or len(text.encode('utf-8'))>2*1024*1024: raise ValueError('配置文件须小于2MiB')
    site = yaml.safe_load(text)
    if group.get('channel')=='competition' and isinstance(site,dict) and 'flight_bounds' in site:
        site = competition.editor_site(site)
    patch_from_site(site, group.get('channel')=='competition')
    return site


def route(workbench, body):
    import wb_survey
    describe(workbench, body)
    site=body.get('site')
    if not isinstance(site,dict) or not isinstance(site.get('flight_area'),dict):
        raise ValueError('请先加载现场地图')
    points=wb_survey.route_points(site['flight_area'].get('search_bounds'),body.get('inset'),body.get('pattern'))
    return {'ok':True,'route':points,'pattern':body['pattern'],
            'note':'只生成搜索坐标；起飞原点、飞行高度、速度不变。搜索范围不含走廊，未评估实际视觉覆盖或避障后路径。'}

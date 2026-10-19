"""Map editor adapter for the independent competition entry; no ROS or devices."""
import base64
import copy
import hashlib
import inspect
import json
import math
import shlex

AREA_KEYS = {'center_bounds':'flight_bounds', 'target_bounds':'target_bounds',
             'search_bounds':'search_center_bounds', 'coverage_bounds':'coverage_bounds',
             'staging_xy':'staging_xy', 'survey_xy':'survey_xy', 'map_size':'map_size'}

def editor_site(profile):
    if not isinstance(profile, dict):
        raise ValueError('正赛场地配置必须是对象')
    area = {key: copy.deepcopy(profile[value]) for key, value in AREA_KEYS.items()}
    schedule = profile.get('corridor_speed_schedule') or {}
    following = profile.get('following_speed_profile') or {}
    result = dict(flight_area=area, corridor_waypoints=copy.deepcopy(profile.get('corridor_waypoints', [])),
                  landing_xy=copy.deepcopy(profile.get('landing_xy')), high_agl=profile['high_agl'],
                  max_agl=profile['max_agl'], site_confirmed=profile.get('site_confirmed', False))
    if schedule:
        result['corridor_geometry'] = dict(wall_axis=schedule['axis'],
            wall_coordinates=copy.deepcopy(schedule['wall_coordinates']),
            entry_waypoints=following.get('corridor_after_waypoints', 1))
    return result

def validate_patch(value):
    allowed = {'flight_area','corridor_waypoints','landing_xy','corridor_geometry','site_confirmed'}
    if not isinstance(value, dict) or set(value) - allowed:
        raise ValueError('正赛地图仅修改场地范围、搜索/走廊/H坐标、墙面和现场确认')
    def finite(v): return type(v) in (int, float) and math.isfinite(v)
    def vec(v, n): return isinstance(v, list) and len(v) == n and all(finite(t) for t in v)
    def inside(p, b): return vec(p, 2) and b[0] <= p[0] <= b[1] and b[2] <= p[1] <= b[3]
    area = value.get('flight_area')
    if not isinstance(area, dict) or set(area) != set(AREA_KEYS):
        raise ValueError('正赛地图需完整的飞行/目标/搜索/低空覆盖范围、准备点、搜索点及地图尺寸')
    for key in ('center_bounds','target_bounds','search_bounds','coverage_bounds'):
        b = area[key]
        if not vec(b,4) or b[0]>=b[1] or b[2]>=b[3]: raise ValueError(key+' 范围格式错误')
    center, search = area['center_bounds'], area['search_bounds']
    for key, outer in [('target_bounds',center),('search_bounds',center),('coverage_bounds',search)]:
        b = area[key]
        if not inside([b[0],b[2]],outer) or not inside([b[1],b[3]],outer):
            raise ValueError(key+' 超出外层范围')
    if not inside([0,0],center) or not inside(area['staging_xy'],search):
        raise ValueError('原点须在飞行边界内；准备点须在搜索区内')
    route = area['survey_xy']
    if not isinstance(route,list) or not 1<=len(route)<=8 or any(not inside(p,search) for p in route):
        raise ValueError('正赛搜索点须为搜索区内1～8个有序XY点')
    size = area['map_size']
    if not vec(size,3) or min(size)<=0 or max(abs(center[0]),abs(center[1]))+.3>=size[0]/2 or max(abs(center[2]),abs(center[3]))+.3>=size[1]/2:
        raise ValueError('地图尺寸须覆盖飞行边界并预留0.30m参考余量')
    points = value.get('corridor_waypoints')
    if not isinstance(points,list) or not 2<=len(points)<=100:
        raise ValueError('请填写2～100个实测走廊点')
    for p in points:
        if not isinstance(p,dict) or set(p)!={'x','y','agl'} or not inside([p['x'],p['y']],center) or not finite(p['agl']) or not .5<=p['agl']<=1.8:
            raise ValueError('走廊点须在飞行边界内，FC AGL为0.5～1.8m')
    if not inside(value.get('landing_xy'),center): raise ValueError('请填写飞行边界内的H中心')
    if type(value.get('site_confirmed')) is not bool: raise ValueError('现场测量确认须明确勾选或取消')
    geometry = value.get('corridor_geometry')
    if geometry is not None:
        if not isinstance(geometry,dict) or set(geometry)!={'wall_axis','wall_coordinates','entry_waypoints'}:
            raise ValueError('墙面配置格式错误')
        if type(geometry['wall_axis']) is not int or geometry['wall_axis'] not in (0,1): raise ValueError('墙面轴须为X或Y')
        walls = geometry['wall_coordinates']
        if not isinstance(walls,list) or not 1<=len(walls)<=20 or not all(finite(v) for v in walls): raise ValueError('墙面坐标须为有限数值')
        entry = geometry['entry_waypoints']
        if type(entry) is not int or not 1<=entry<=len(points): raise ValueError('入口序号须在走廊点列表内')
    return copy.deepcopy(value)

def merge_profile(profile, patch):
    patch = validate_patch(patch)
    result = copy.deepcopy(profile)
    for key, target in AREA_KEYS.items(): result[target] = patch['flight_area'][key]
    for key in ('corridor_waypoints','landing_xy','site_confirmed'): result[key] = patch[key]
    geometry = patch.get('corridor_geometry')
    if geometry is not None:
        result.setdefault('corridor_speed_schedule',{}).update(axis=geometry['wall_axis'],wall_coordinates=geometry['wall_coordinates'])
        result.setdefault('following_speed_profile',{})['corridor_after_waypoints'] = geometry['entry_waypoints']
    return result


def overlay_command(source, geometry, revision):
    if not isinstance(revision,str) or not revision.isascii() or not revision.isalnum() or not 1<=len(revision)<=32:
        raise ValueError('坐标草稿版本无效，请重新生成命令')
    clean = validate_patch(geometry)
    data = json.dumps(clean, sort_keys=True, separators=(',',':'), allow_nan=False)
    token = hashlib.sha256((source+'\ncompetition\n'+revision+'\n'+data).encode()).hexdigest()[:20]
    relative = 'logs/flight_workbench/site_overlays/competition_'+token+'.yaml'
    # Embed only the pure adapter: an independent board installation need not
    # contain workbench or board_trials. Production validation stays on board.
    shared = 'import copy,math\nAREA_KEYS='+repr(AREA_KEYS)+'\n'+inspect.getsource(validate_patch)+'\n'+inspect.getsource(merge_profile)
    writer = '''import base64,json,sys,pathlib,yaml
root=pathlib.Path.cwd().resolve()
source=pathlib.Path(sys.argv[1]);source=(root/source).resolve() if not source.is_absolute() else source.resolve()
dest=(root/sys.argv[2]).resolve()
if root not in source.parents or root not in dest.parents:raise ValueError("field/overlay outside project")
if source.stat().st_size>2*1024*1024:raise ValueError("field too large")
patch=json.loads(base64.b64decode(sys.argv[3]))
'''+shared+'''
profile=yaml.safe_load(source.read_text(encoding="utf-8-sig"))
profile=merge_profile(profile,patch)
sys.path[:0]=[str(root/"patrol_uav_ws-patrol_planner/src/uav_mission/src"),str(root/"vision_ws/src/uav_high_view/src")]
from uav_mission.competition_config import validate
validate(profile,flight=False)
content=yaml.safe_dump(profile,sort_keys=False,allow_unicode=True)
dest.parent.mkdir(parents=True,exist_ok=True)
if dest.exists() and dest.read_text(encoding="utf-8")!=content:raise ValueError("site source changed; regenerate command with a new geometry revision")
if not dest.exists():
 with dest.open("x",encoding="utf-8") as f:f.write(content)
print("MEASURED_SITE_OVERLAY="+str(dest))
'''
    payload = base64.b64encode(data.encode()).decode()
    # ASCII payload survives Windows -> SSH command encodings.
    wrapped = 'import base64;exec(base64.b64decode('+repr(base64.b64encode(writer.encode()).decode())+'))'
    command = 'python3 -c '+shlex.quote(wrapped)+' '+shlex.quote(source)+' '+shlex.quote(relative)+' '+shlex.quote(payload)
    return relative, command, clean
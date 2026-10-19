"""Measured site overlays for existing 04/08 entries; no hardware operations."""
import base64
import hashlib
import json
import math
import shlex
import wb_survey

CORRIDOR_FOLDERS = ('04_corridor_landing', '08_full_mission')
SURVEY_FOLDERS = ('02_high_view_revisit','06_high_priority','07_memory_only','08_full_mission','09_high_speed_capture')
FOLDERS = tuple(set(CORRIDOR_FOLDERS + SURVEY_FOLDERS))


def validate_geometry(value):
    if not isinstance(value, dict) or set(value) - {'corridor_waypoints', 'landing_xy', 'corridor_geometry', 'flight_area','survey_plan'}:
        raise ValueError('只允许走廊航点、H坐标、墙面几何与flight_area；不修改速度或飞行模式')
    def number(v):
        return type(v) in (int, float) and math.isfinite(v)
    def vector(v, n):
        return isinstance(v, list) and len(v) == n and all(number(x) for x in v)
    if not value:raise ValueError('场地坐标不能为空')
    points = value.get('corridor_waypoints')
    has_corridor = 'corridor_waypoints' in value or 'landing_xy' in value
    if has_corridor and (not isinstance(points, list) or not 2 <= len(points) <= 100):
        raise ValueError('走廊至少两个、最多100个有序航点')
    for p in (points or []):
        if not isinstance(p, dict) or set(p) != {'x', 'y', 'agl'} or not all(number(x) for x in p.values()) or not 0 < p['agl'] <= 4:
            raise ValueError('每点必须为有限数值x/y/agl；agl为FC离地高度，0<agl≤4m')
    if has_corridor and not vector(value.get('landing_xy'), 2):
        raise ValueError('必须填写两个有限数值的H中心坐标')
    geometry = value.get('corridor_geometry')
    if geometry is not None:
        if not has_corridor:raise ValueError('墙面几何须配合走廊航点')
        if not isinstance(geometry, dict) or set(geometry) != {'wall_axis', 'wall_coordinates', 'entry_waypoints'}:
            raise ValueError('墙面几何需wall_axis/wall_coordinates/entry_waypoints')
        walls = geometry['wall_coordinates']
        if (type(geometry['wall_axis']) is not int or geometry['wall_axis'] not in (0, 1)
                or not isinstance(walls, list) or not 1 <= len(walls) <= 20 or not all(number(v) for v in walls)
                or type(geometry['entry_waypoints']) is not int or not 1 <= geometry['entry_waypoints'] <= len(points)):
            raise ValueError('墙面轴0=X/1=Y，墙位置需实测；入口序号从1起且不超过航点数')
    area = value.get('flight_area', {})
    if not isinstance(area, dict) or set(area) - {'center_bounds','target_bounds','search_bounds','staging_xy','survey_xy','map_size'}:
        raise ValueError('未知flight_area字段')
    for key, v in area.items():
        if key.endswith('_bounds'):
            if not vector(v,4) or v[0]>=v[1] or v[2]>=v[3]:raise ValueError('范围格式为[xmin,xmax,ymin,ymax]')
        elif key == 'survey_xy':
            if not isinstance(v,list) or not 3<=len(v)<=100 or not all(vector(p,2) for p in v):raise ValueError('survey_xy需至少三个XY点')
        elif not vector(v,3 if key=='map_size' else 2):raise ValueError('无效'+key)
    if 'survey_plan' in value:wb_survey.plan(value['survey_plan'])
    return json.loads(json.dumps(value, allow_nan=False))


# Runs only when an operator submits preview/check/flight. The command plan itself
# is pure/local. Keep the field file intact; merge an immutable run overlay.
REMOTE_WRITER = '''import base64,json,sys,pathlib,yaml,tempfile
root=pathlib.Path.cwd().resolve()
source=pathlib.Path(sys.argv[1]);dest=root/sys.argv[2]
patch=json.loads(base64.b64decode(sys.argv[3]))
sys.path.insert(0,str(root/"tools/flight_workbench"))
from wb_geometry import validate_geometry
patch=validate_geometry(patch)
survey=patch.pop("survey_plan",None)
rig=yaml.safe_load((root/"deployment/board_trials_4x4/common/uav_board_trials/config/known_rig.yaml").read_text())
if survey is not None:
 from wb_survey import plan
 result=plan(survey)
 offset=float(rig["body_to_imu_xyz"][2])+float(rig["imu_to_camera_xyz"][2])
 if abs(offset-result["fc_to_camera_z"])>1e-6:raise ValueError("Camera vertical offset differs from deployed known_rig; review height conversion")
 patch.setdefault("flight_area",{}).update(search_bounds=result["search_bounds"],survey_xy=result["route"])
 patch["high_agl"]=result["high_agl"]
 print("SURVEY_COVERAGE="+json.dumps(result))
profile=yaml.safe_load(source.read_text()) or {}
if not isinstance(profile,dict):raise ValueError("site profile must be an object")
for key,value in patch.items():
 if key=="flight_area":profile[key]=dict(profile.get(key,{}) or {},**value)
 else:profile[key]=value
# Use production validation before creating a deployable overlay, and retain the
# module's own task/release/default settings. No ROS nodes are started here.
sys.path[:0]=[str(root/"deployment/board_trials_4x4/common/uav_board_trials/scripts"),str(root/"patrol_uav_ws-patrol_planner/src/uav_mission/src"),str(root/"vision_ws/src/uav_high_view/src")]
from trial_config import apply_site_profile,validate_settings,generate
settings=yaml.safe_load((root/"deployment/board_trials_4x4"/sys.argv[4]/"settings.yaml").read_text())
apply_site_profile(settings,profile);validate_settings(settings)
rig=yaml.safe_load((root/"deployment/board_trials_4x4/common/uav_board_trials/config/known_rig.yaml").read_text())
with tempfile.TemporaryDirectory(prefix="workbench_geometry_") as tmp:
 generate(root,tmp,settings,(0.,0.,0.),rig)
if root not in dest.resolve().parents:raise ValueError("overlay path outside workspace")
dest.parent.mkdir(parents=True,exist_ok=True)
content=yaml.safe_dump(profile,sort_keys=False)
if dest.exists() and dest.read_text()!=content:raise ValueError("site source changed; regenerate command with a new geometry revision")
if not dest.exists():
 with dest.open("x") as f:f.write(content)
print("MEASURED_SITE_OVERLAY="+str(dest))
'''


def overlay_command(source, folder, geometry, revision):
    if folder not in FOLDERS:raise ValueError('该组不支持手动搜索/走廊坐标')
    if not isinstance(revision,str) or not revision.isascii() or not revision.isalnum() or not 1<=len(revision)<=32:
        raise ValueError('坐标草稿版本无效，请重新生成命令')
    clean=validate_geometry(geometry)
    if 'survey_plan' in clean and folder not in SURVEY_FOLDERS:raise ValueError('该组没有高位搜索阶段')
    if any(k in clean for k in ('landing_xy','corridor_waypoints','corridor_geometry')) and folder not in CORRIDOR_FOLDERS:raise ValueError('该组不使用走廊/H航点')
    data=json.dumps(clean,sort_keys=True,separators=(',',':'),allow_nan=False)
    token=hashlib.sha256((source+'\n'+folder+'\n'+revision+'\n'+data).encode()).hexdigest()[:20]
    relative='logs/flight_workbench/site_overlays/'+folder+'_'+token+'.yaml'
    payload=base64.b64encode(data.encode()).decode()
    command='python3 -c '+shlex.quote(REMOTE_WRITER)+' '+shlex.quote(source)+' '+shlex.quote(relative)+' '+shlex.quote(payload)+' '+shlex.quote(folder)
    return relative, command, clean
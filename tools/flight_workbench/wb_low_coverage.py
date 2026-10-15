"""Workbench adapter for the explicit, unflown low-coverage backup. No ROS/SSH actions."""
import base64
import copy
import hashlib
import json
import math
from pathlib import Path
import shlex
import uuid
import yaml

FOLDER = '10_low_coverage_full'
ENTRY = 'deployment/board_trials_4x4/'+FOLDER+'/start.sh'
FIELDS = ('flight_area','corridor_waypoints','landing_xy','corridor_geometry',
          'landing_transit_agl','landing_capture_agl','max_agl')

def height(value):
    if type(value) not in (int,float) or value not in (1.8,1.9):
        raise ValueError('备用遍历FC高度只允许1.8或1.9m')
    return float(value)

def is_request(workbench, body):
    return any(g.get('id')==body.get('group_id') and g.get('folder')==FOLDER
               for g in workbench.config.get('groups',[]))

def normalize_site(raw, base=None):
    if not isinstance(raw,dict):raise ValueError('地图顶层必须是对象')
    if 'flight_bounds' in raw:
        required=('flight_bounds','target_bounds','coverage_bounds','staging_xy','survey_xy','map_size','corridor_waypoints','landing_xy')
        if any(k not in raw for k in required):raise ValueError('原实测地图缺少范围、走廊或H字段')
        site=dict(flight_area=dict(center_bounds=raw['flight_bounds'],target_bounds=raw['target_bounds'],
            search_bounds=raw['coverage_bounds'],staging_xy=raw['staging_xy'],survey_xy=raw['survey_xy'],map_size=raw['map_size']),
            corridor_waypoints=raw['corridor_waypoints'],landing_xy=raw['landing_xy'])
        if raw.get('corridor_speed_schedule'):
            g=raw['corridor_speed_schedule'];site['corridor_geometry']=dict(wall_axis=g['axis'],wall_coordinates=g['wall_coordinates'],entry_waypoints=g['entry_waypoints'])
        for key in ('landing_transit_agl','landing_capture_agl','max_agl'):
            if key in raw:site[key]=raw[key]
        # 原高位搜索中心范围和四边形只归档展示，不用作牛耕路线。
        site['_low_coverage_original']=copy.deepcopy(raw)
    elif 'flight_area' in raw:
        site=copy.deepcopy(raw)
    else:raise ValueError('请导入含flight_bounds/coverage_bounds或flight_area的完整地图')
    site=copy.deepcopy(site)
    site.setdefault('high_agl',2.0);site.setdefault('low_agl',1.8);site.setdefault('max_agl',2.3)
    if base:
        for key in ('landing_transit_agl','landing_capture_agl','max_agl'):
            if key not in raw and key not in site and key in base:site[key]=base[key]
    return site

def patch_from_site(site):
    site=normalize_site(site)
    finite=lambda v:type(v) in (int,float) and math.isfinite(v)
    area=site.get('flight_area')
    if not isinstance(area,dict) or set(area)!={'center_bounds','target_bounds','search_bounds','staging_xy','survey_xy','map_size'}:raise ValueError('需完整flight_area范围、S点、兼容航点和地图尺寸')
    for key in ('center_bounds','target_bounds','search_bounds'):
        b=area[key]
        if not isinstance(b,list) or len(b)!=4 or not all(finite(v) for v in b) or b[0]>=b[1] or b[2]>=b[3]:raise ValueError(key+'须为xmin,xmax,ymin,ymax')
    def inside(p,b):return isinstance(p,list) and len(p)==2 and all(finite(v) for v in p) and b[0]<=p[0]<=b[1] and b[2]<=p[1]<=b[3]
    outer=area['center_bounds'];search=area['search_bounds']
    if not inside([0,0],outer) or not inside(area['staging_xy'],outer):raise ValueError('起飞原点和准备点须在飞行边界内')
    for key in ('target_bounds','search_bounds'):
        b=area[key]
        if not inside([b[0],b[2]],outer) or not inside([b[1],b[3]],outer):raise ValueError(key+'须在飞行边界内')
    if not inside(site.get('landing_xy'),outer):raise ValueError('H点超出飞行边界')
    points=site.get('corridor_waypoints')
    if not isinstance(points,list) or not 2<=len(points)<=100:raise ValueError('至少填写两个走廊点')
    for p in points:
        if not isinstance(p,dict) or set(p)!={'x','y','agl'} or not inside([p['x'],p['y']],outer) or not finite(p['agl']) or not .5<=p['agl']<=1.8:raise ValueError('走廊点需边界内有限x/y/agl，AGL为0.5～1.8m')
    if not isinstance(area['survey_xy'],list) or not 1<=len(area['survey_xy'])<=8 or any(not inside(p,outer) for p in area['survey_xy']):raise ValueError('兼容高位点无效；本备份不执行该路线')
    size=area['map_size'];cap=site['max_agl']
    if not isinstance(size,list) or len(size)!=3 or not all(finite(v) and v>0 for v in size):raise ValueError('地图尺寸需三个正数')
    if not finite(cap) or not 2.15<=cap<=3.5 or size[2]<cap+.1:raise ValueError('最大高度/地图Z需保留1.9m遍历和恢复余量')
    if max(abs(outer[0]),abs(outer[1]))+.3>=size[0]/2 or max(abs(outer[2]),abs(outer[3]))+.3>=size[1]/2:raise ValueError('地图XY不足以容纳实测边界')
    g=site.get('corridor_geometry')
    if g is not None:
        if not isinstance(g,dict) or set(g)!={'wall_axis','wall_coordinates','entry_waypoints'} or type(g['wall_axis']) is not int or g['wall_axis'] not in (0,1) or not isinstance(g['wall_coordinates'],list) or not g['wall_coordinates'] or not all(finite(v) for v in g['wall_coordinates']) or type(g['entry_waypoints']) is not int or not 1<=g['entry_waypoints']<=len(points):raise ValueError('门墙轴、坐标或入口序号无效')
    return {k:copy.deepcopy(site[k]) for k in FIELDS if k in site}

def overlay_command(site, revision, agl):
    agl=height(agl);patch=patch_from_site(site)
    if not isinstance(revision,str) or not revision.isascii() or not revision.isalnum() or not 1<=len(revision)<=32:raise ValueError('请先生成并确认本组地图命令')
    data=json.dumps(patch,sort_keys=True,separators=(',',':'),allow_nan=False)
    token=hashlib.sha256((revision+'\n'+str(agl)+'\n'+data).encode()).hexdigest()[:20]
    dest='logs/flight_workbench/site_overlays/'+FOLDER+'_fc_'+str(agl).replace('.','p')+'_'+token+'.yaml'
    # 由新模块本身做配置/路线检查；不借用08的高位生成器。
    script="""import base64,json,pathlib,sys,yaml
root=pathlib.Path.cwd().resolve();dest=(root/sys.argv[1]).resolve()
if root not in dest.parents:raise ValueError('overlay outside project')
profile=json.loads(base64.b64decode(sys.argv[2]))
profile['obstacle_columns_enabled']=False
content=yaml.safe_dump(profile,sort_keys=False,allow_unicode=True)
dest.parent.mkdir(parents=True,exist_ok=True)
if dest.exists() and dest.read_text(encoding='utf-8')!=content:raise ValueError('immutable coverage overlay conflict')
if not dest.exists():
 with dest.open('x',encoding='utf-8') as stream:stream.write(content)
print('LOW_COVERAGE_SITE_OVERLAY='+str(dest))
"""
    return dest,'python3 -c '+shlex.quote(script)+' '+shlex.quote(dest)+' '+shlex.quote(base64.b64encode(data.encode()).decode())

def build_command(config,group,mode,*,route=None,real_release=None,check_config=False,
                  site_geometry=None,geometry_revision=None,coverage_agl=None,coverage_pattern=None,motion_optimized=False,speed_profile=None,**options):
    if mode not in ('preview','flight') or (route or group.get('channel'))!='module':raise ValueError('备用遍历只支持模块preview/flight入口')
    if config.get('simulation',{}).get('enabled'):raise ValueError('备用遍历候选尚未接入工作台SITL入口')
    if type(check_config) is not bool or type(motion_optimized) is not bool or real_release is not None and type(real_release) is not bool:raise ValueError('入口开关需布尔值')
    if any(v is not None and v is not False for v in options.values()):raise ValueError('备用遍历继承现有投递/H链；不接受高位路线、续扫或其他专项参数')
    pattern="lawnmower" if coverage_pattern is None else coverage_pattern
    if not isinstance(pattern,str) or pattern not in ("lawnmower","nested_rectangles"):
        raise ValueError("备用搜索形状仅支持lawnmower或nested_rectangles")
    speed_profile='competition' if speed_profile is None else speed_profile
    if speed_profile not in ('competition','limited'):raise ValueError('速度档仅支持competition或limited')
    agl=height(1.8 if coverage_agl is None else coverage_agl)
    writer=None
    if site_geometry is None:
        if geometry_revision is not None:raise ValueError('手动地图版本缺少地图内容')
        dest=group.get('site_config')
        if not isinstance(dest,str) or not dest.strip() or any(c in dest for c in '\r\n\x00'):raise ValueError('缺少有效板端默认地图路径')
    else:
        dest,writer=overlay_command(site_geometry,geometry_revision,agl)
    common=' --site-config '+shlex.quote(dest)+' --coverage-agl %.1f'%agl+' --coverage-pattern '+pattern+' --speed-profile '+speed_profile
    if motion_optimized:common+=' --motion-optimized'
    check='bash '+ENTRY+' preview'+common+' --check-config'
    real=group.get('release')=='real' if real_release is None else real_release
    if check_config:body=check
    elif mode=='flight':body=check+' && bash '+ENTRY+' flight'+common+(' --real-release' if real else '')
    else:body='bash '+ENTRY+' preview'+common
    return (writer+' && '+body if writer else body),'候选未实飞：%s，FC %.1fm；使用原地图测量bounds，沿Y长线/横向内收0.6m/牛耕间距2.6m/回字形圈内收1.8m由新模块动态生成；虚拟柱/虚拟天花板关闭，实体避障保留；飞行前检查同一地图，投递/H继承08。'%('牛耕' if pattern=='lawnmower' else '回字形',agl)


def _identity(workbench,body):
    import wb_board
    g=next((g for g in workbench.config.get('groups',[]) if g['id']==body.get('group_id') and g.get('folder')==FOLDER),None)
    if g is None:raise ValueError('未知备用遍历组')
    source=g.get('site_config','')
    scope=hashlib.sha256(json.dumps([workbench.target.host,workbench.board.root,g['id'],source]).encode()).hexdigest()[:24]
    return g,source,scope,Path(wb_board.profile_path('waypoint_drafts'))/(scope+'.json')

def map_action(workbench,action,body):
    g,source,scope,path=_identity(workbench,body)
    if action=='draft':
        try:
            data=json.loads(path.read_text(encoding='utf-8'));patch_from_site(data['patch'])
        except (OSError,ValueError,KeyError):data=None
        return dict(ok=True,scope=scope,source=source,draft=data)
    if action=='context':
        loaded=map_action(workbench,'draft',body)
        site=yaml.safe_load((Path(__file__).parent/'waypoint_templates/module10.yaml').read_text(encoding='utf-8'))
        origin='随包实测waypoint_map参考；须导入现场最新地图并执行配置检查'
        if workbench.connection.get('state')=='ok' and workbench.target.transport!='local':
            import wb_board
            code,output=workbench.board.run('cat -- '+wb_board.quote(workbench.board.abs_path(source)),timeout=20)
            if code:raise ValueError('无法读取所选板端实测地图；请核对路径，不使用参考文件代替')
            site=normalize_site(yaml.safe_load(output));origin='板端所选实测地图'
        if loaded['draft'] and loaded['draft'].get('enabled',True):site=copy.deepcopy(loaded['draft']['patch'])
        if body.get('site_geometry'):site=copy.deepcopy(body['site_geometry'])
        site=normalize_site(site);patch_from_site(site)
        return dict(ok=True,folder=FOLDER,editor_kind='module',group_id=g['id'],site=site,source=source,scope=scope,origin=origin,name=g['name'],draft=loaded['draft'])
    if action=='preview':
        try:patch=patch_from_site(body.get('site'));errors=[]
        except (ValueError,KeyError,TypeError) as error:patch={};errors=[str(error)]
        return dict(ok=True,errors=errors,yaml=yaml.safe_dump(patch,allow_unicode=True,sort_keys=False))
    if action=='import':
        text=body.get('text')
        if not isinstance(text,str) or not 0<len(text.encode())<=2*1024*1024:raise ValueError('地图YAML最大2MiB')
        site=normalize_site(yaml.safe_load(text),body.get('base_site'));patch_from_site(site)
        return dict(ok=True,site=site,imported_fields=list(site),ignored_fields=[])
    if action=='export':
        return dict(ok=True,yaml='# 备用遍历实测地图；范围单位米，FC高度由卡片1.8/1.9m选项指定。\n'+yaml.safe_dump(patch_from_site(body.get('site')),allow_unicode=True,sort_keys=False))
    trial=workbench.sessions.get('trial')
    if trial and trial.state in ('starting','running'):raise ValueError('任务运行期间不能应用或切换地图')
    if body.get('scope')!=scope:raise ValueError('连接或地图来源变化，请重新打开地图')
    if action=='save':
        patch=patch_from_site(body.get('site'))
        data=dict(scope=scope,source=source,patch=patch,revision=uuid.uuid4().hex,enabled=True,import_name=body.get('import_name'))
    elif action=='disable':
        data=map_action(workbench,'draft',body)['draft']
        if not data:return dict(ok=True)
        data['enabled']=False
    else:raise ValueError('备用遍历不使用高位路线编辑；请修改实测搜索范围')
    path.parent.mkdir(parents=True,exist_ok=True);temp=path.with_suffix('.tmp')
    temp.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8');temp.replace(path)
    return dict(ok=True,**data)

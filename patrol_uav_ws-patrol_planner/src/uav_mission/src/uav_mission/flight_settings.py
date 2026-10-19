"""Flight-before-start overrides. Pure validation; no ROS or hardware actions."""
import copy
import math

DROP_FOLDERS = ('01_visual_interrupt','02_high_view_revisit','05_low_multi','06_high_priority','08_full_mission','10_low_coverage_full')
H_FOLDERS = ('03_h_landing','04_corridor_landing','05_low_multi','08_full_mission','10_low_coverage_full')
DROP_FIELDS = {
 'drop_horizontal_compensation_mode': (1,2), 'drop_descent_mode': (1,2,3,4),
 'drop_agl': (.35,1.), 'release_min_agl': (.001,1.1), 'release_max_agl': (.001,1.1),
 'drop_descent_speed_mps': (.001,1.), 'drop_descent_mode4_speed_mps': (.001,1.),
 'drop_descent_acceleration_mps2': (.001,2.), 'drop_descent_braking_mps2': (.001,2.),
 'drop_descent_hold_tolerance_m': (.001,.03),
}
H_FIELDS = {
 'landing_alignment_mode': ('non_vh','vh'), 'landing_descent_mode': (1,2,3,4),
 'landing_handoff_mode': ('POSCTL','AUTO.LAND','OFFBOARD'),
 'landing_descent_speed_mps': (.001,1.),
 'landing_offboard': {'descent_speed':(.001,1.),'touchdown_overtravel':(.001,.1)},
}

TASK_FIELDS = {
 'cruise_speed': (.1,1.2), 'cruise_acceleration': (.1,1.),
 'initialization_timeout': (30.,300.), 'motion_action_timeout': (.001,600.),
 'survey_policy': {
  'coarse_min_confidence': (0.,1.), 'recheck_observe_seconds': (3.,15.),
  'recheck_shift_after_seconds': (.5,1.), 'recheck_shift_radius_m': (.4,.6),
  'max_uncertainty_m': (.25,.5), 'descent_radius_m': (.001,2.),
  'descent_max_candidates': (1,25), 'survey_stall_seconds': (3.,20.),
  'survey_progress_m': (.05,.3),
 },
}

def validate_patch(patch, folder):
    if not isinstance(patch,dict) or not patch:raise ValueError('飞行参数不能为空')
    fields={}
    if folder in DROP_FOLDERS or folder=='competition':fields.update(DROP_FIELDS)
    if folder in ('05_low_multi','10_low_coverage_full'):fields['drop_agl']=(.15,1.)
    if folder=='08_full_mission':fields['drop_agl']=(.25,1.)
    if folder in H_FOLDERS or folder=='competition':fields.update(H_FIELDS)
    if folder in ('08_full_mission','10_low_coverage_full','competition'):fields.update(copy.deepcopy(TASK_FIELDS))
    if folder in ('08_full_mission','10_low_coverage_full'):fields['ordinary_waypoint_dwell_enabled']=bool
    if folder=='competition':
        fields.update(lane_spacing=(.2,1.2),target_action_timeout=(.001,600.))
        fields['survey_policy']['resume_budget_seconds']=(10.,180.)
    if set(patch)-set(fields):raise ValueError('该组不支持参数：'+', '.join(sorted(set(patch)-set(fields))))
    def check(value,rule,key):
        if isinstance(rule,dict):
            if not isinstance(value,dict) or not value or set(value)-set(rule):raise ValueError('未知字段 '+key)
            for k,v in value.items():check(v,rule[k],key+'.'+k)
        elif rule is bool:
            if type(value) is not bool:raise ValueError('参数必须是布尔值 '+key)
        elif key=='survey_policy.descent_max_candidates':
            if type(value) is not int or not rule[0]<=value<=rule[1]:raise ValueError('候选数量必须为1～25的整数')
        elif rule is None:
            if not isinstance(value,list) or len(value)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in value):raise ValueError('H中心必须为两个有限XY坐标')
        elif key.endswith('_mode'):
            if type(value) is not type(rule[0]) or value not in rule:raise ValueError('无效模式 '+key)
        elif type(value) not in (int,float) or not math.isfinite(value) or not rule[0]<=value<=rule[1]:
            raise ValueError(key+' 超出范围 '+str(rule))
    for k,v in patch.items():check(v,fields[k],k)
    if ('release_min_agl' in patch)!=('release_max_agl' in patch):raise ValueError('释放高度上下限必须一起填写')
    if 'release_min_agl' in patch:
        lo,hi=patch['release_min_agl'],patch['release_max_agl']
        if lo>hi or ('drop_agl' in patch and not lo<=patch['drop_agl']<=hi):raise ValueError('要求释放下限 ≤ 投递目标高度 ≤ 释放上限')
    if 'landing_descent_mode' in patch and 'landing_handoff_mode' in patch:
        if (patch['landing_descent_mode']==4)!=(patch['landing_handoff_mode']=='OFFBOARD'):raise ValueError('H模式4必须配合OFFBOARD交接')
    return copy.deepcopy(patch)

def apply_patch(settings, patch, folder):
    patch=validate_patch(patch,folder)
    result=copy.deepcopy(settings)
    for key,value in patch.items():
        result[key]=dict(result.get(key,{}) or {},**value) if isinstance(value,dict) else value
    # Validate coupled values against inherited settings as well.
    from uav_mission.drop_descent_config import drop_descent_parameters,drop_horizontal_compensation_mode
    from uav_mission.drop_height_config import release_heights
    from uav_mission.landing_posctl_config import validate_landing_posctl
    if folder in DROP_FOLDERS or folder=='competition':
        drop_descent_parameters(result);drop_horizontal_compensation_mode(result);release_heights(result)
    if folder in H_FOLDERS or folder=='competition':validate_landing_posctl(result)
    return result

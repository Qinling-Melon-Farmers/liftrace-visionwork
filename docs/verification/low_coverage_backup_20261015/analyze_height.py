from pathlib import Path
import collections,json,numpy as np
root=Path('/home/xhj/liftrace-worktrees/r2026-board-vision-tests')
a=root/'试飞产物/flight_20261015_1501_analysis'
out=root/'docs/verification/low_coverage_backup_20261015'
meta=json.loads((a/'metadata.json').read_text())[-1]['data'];ground=meta['ground_reference']['ground_z']
d=collections.defaultdict(list)
for line in (a/'records.jsonl').open():
    r=json.loads(line)
    if r['topic'] in ('/navigation/local_pose','/mavros/local_position/pose','/mavros/setpoint_position/local','/navigation/setpoint_mission','/mavros/local_position/velocity_local','/mavros/setpoint_raw/target_local','/Odometry'):d[r['topic']].append(r)
def pos(r):
    m=r['msg'];p=m.get('pose',{});p=p.get('pose',p)
    return p['position']
res=dict(ground_z=ground,virtual_ceiling_enabled=meta['settings']['virtual_ceiling_enabled'],max_agl=meta['settings']['max_agl'],windows=[])
for lo,hi,target in [(62,68,2.),(70,78,2.),(90,100,1.4),(112,117,1.4)]:
    pp=[r for r in d['/navigation/local_pose'] if lo<=r['t']<=hi]
    cc=[r for r in d['/mavros/setpoint_position/local'] if lo<=r['t']<=hi]
    vv=[r['msg']['twist']['linear']['z'] for r in d['/mavros/local_position/velocity_local'] if lo<=r['t']<=hi]
    ll=[r for r in d['/Odometry'] if lo<=r['t']<=hi]
    agl=np.array([pos(r)['z']-ground for r in pp]);cagl=np.array([pos(r)['z']-ground for r in cc])
    res['windows'].append(dict(seconds=[lo,hi],target_agl=target,fc_agl_min_med_max=[float(x) for x in (agl.min(),np.median(agl),agl.max())],
        fc_minus_goal_min_med_max=[float(x) for x in ((agl-target).min(),np.median(agl-target),(agl-target).max())],
        sent_command_agl_min_med_max=[float(x) for x in (cagl.min(),np.median(cagl),cagl.max())],
        fc_position_average_vz=(pos(pp[-1])['z']-pos(pp[0])['z'])/(pp[-1]['t']-pp[0]['t']),
        reported_fc_mean_vz=float(np.mean(vv)),lio_position_average_vz=(pos(ll[-1])['z']-pos(ll[0])['z'])/(ll[-1]['t']-ll[0]['t'])))
(out/'height_review.json').write_text(json.dumps(res,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(res,ensure_ascii=False,indent=2))
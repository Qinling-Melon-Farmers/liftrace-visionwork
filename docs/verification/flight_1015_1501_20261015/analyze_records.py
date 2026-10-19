from pathlib import Path
import json, collections, math, statistics, argparse
p=argparse.ArgumentParser();p.add_argument('analysis_dir',type=Path);p.add_argument('output_dir',type=Path);args=p.parse_args();a=args.analysis_dir;r=args.output_dir;r.mkdir(parents=True,exist_ok=True)
rows=[json.loads(x) for x in (a/'records.jsonl').open()];d=collections.defaultdict(list)
for row in rows:d[row['topic']].append(row)
meta=json.loads((a/'metadata.json').read_text())[-1]['data'];ground=meta['ground_reference']['ground_z'];start=json.loads((a/'bag_info.json').read_text())['start']
summary={'bag':json.loads((a/'bag_info.json').read_text())['bag'],'bag_start_epoch':start,'ground_z':ground,'actual_settings':meta['settings'],'ground_reference':meta['ground_reference'],'pilot_posctl_confirmed':True}
windows=[]
for lo,hi in [(62,68),(70,78),(90,100),(112,117)]:
 p=[x for x in d['/navigation/local_pose'] if lo<=x['t']<=hi];v=[x['msg']['twist']['linear']['z'] for x in d['/mavros/local_position/velocity_local'] if lo<=x['t']<=hi];target=[x['msg']['velocity']['z'] for x in d['/mavros/setpoint_raw/target_local'] if lo<=x['t']<=hi]
 delta=p[-1]['msg']['pose']['position']['z']-p[0]['msg']['pose']['position']['z']
 windows.append(dict(window=[lo,hi],delta_z=delta,pose_average_vz=delta/(p[-1]['t']-p[0]['t']),reported_mean_vz=statistics.mean(v),reported_target_mean_vz=statistics.mean(target)))
summary['vertical_consistency']=windows
summary['mission_events']=[]
seen=set()
for x in d['/uav_high_view/probe_status']:
 m=json.loads(x['msg']['data'])
 for event in m.get('events',[]):
  key=json.dumps(event,sort_keys=True)
  if key not in seen:seen.add(key);summary['mission_events'].append(event)
summary['mode_changes']=[];last=None
for x in d['/mavros/state']:
 value=(x['msg']['mode'],x['msg']['armed'])
 if value!=last:summary['mode_changes'].append(dict(t=x['t'],mode=value[0],armed=value[1]));last=value
summary['pillbox_hint_gates']=[]
for line in (a/'detections.jsonl').open():
 x=json.loads(line)
 if x['topic']!='/uav_vision/navigation_hints' or not 60<x['t']<71:continue
 for det in x['msg']['detections']:
  if det['class_name'] not in ('pillbox','panzer'):continue
  pose=min(d['/navigation/local_pose'],key=lambda p:abs(p['t']-x['t']))
  summary['pillbox_hint_gates'].append(dict(t=x['t'],image_t=det['header']['stamp']-start,cls=det['class_name'],confidence=det['class_confidence'],map_xy=[det['map_point'][k] for k in 'xy'],nearby_agl=pose['msg']['pose']['position']['z']-ground))
(r/'numeric_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
(a/'analysis_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))

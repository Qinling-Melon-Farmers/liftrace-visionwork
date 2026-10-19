#!/usr/bin/env python3
"""Recorded evidence analysis, no ROS processes or model inference. Run from B."""
from pathlib import Path
import argparse,bisect,csv,json,math,re,sys
from collections import Counter
import cv2,numpy as np,yaml
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools/bag_replay'))
from bag_replay import Timeline,Frames,TransformTree,xyz
p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();run=a.run.resolve();out=run/'analysis';out.mkdir(exist_ok=True)
d=json.loads((run/'replay/data.json').read_text());extra=json.loads((out/'extra_rows.json').read_text());ctl=yaml.safe_load((run/'control.yaml').read_text());meta=json.loads((run/'run_metadata.json').read_text());tree=TransformTree(d['rows'],d['start'])
pose=Timeline(extra['/navigation/local_pose']);sp=Timeline(extra['/navigation/setpoint_mission'])
def write_json(name,value): (out/name).write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')
def write_csv(name,rows):
 if not rows:return
 with (out/name).open('w',encoding='utf-8',newline='') as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def rotation(m): return Rotation.from_quat([m['orientation'][k] for k in ('x','y','z','w')])
def quant(v):
 v=np.asarray(v,dtype=float);return dict(n=len(v),p50=float(np.median(v)),p95=float(np.percentile(v,95)),max=float(np.max(v))) if len(v) else dict(n=0)
def detections(q,key,cls):return [(r,z) for r in q['rows'][key] for z in r['m']['detections'] if z['class_name']==cls]
reference_frames=json.loads((ROOT/'logs/h_detection_quality_20261006/frames.json').read_text())
def image_file(base,f):
 path=base/f['file']
 if path.exists():return path
 idx=int(Path(f['file']).stem)
 ref=reference_frames[idx]
 if abs(ref['stamp']-f['stamp'])>1e-6:raise ValueError('Reference frame timestamp mismatch')
 return Path(ref['file'])
def rgb_frame(q,base,t):
 f=min(q['frames'],key=lambda f:abs(f['stamp']-t));im=cv2.imread(str(image_file(base,f)));return f,im
# Full delivery event sequence, retain receipt and source times.
events=[]
for k in ('command','result','release'):
 for r in d['rows'][k]:
  m=r['m'];events.append(dict(receipt_s=r['t'],source_s=r['stamp'],topic=k,decision_seq=m.get('decision_seq',0),slot=m.get('payload_slot',0),target=m.get('target_class',''),reason=m.get('reason',''),success=m.get('success',''),terminal=m.get('terminal','')))
events.sort(key=lambda r:r['receipt_s']);write_csv('mission_events.csv',events)
# Control log excerpts with original line numbers, sorted ROS timestamp.
logs=[]
for i,line in enumerate((run/'application.log').read_text(errors='replace').splitlines(),1):
 if not any(s in line for s in ('[ExternalLanding]','[PatrolControl] External LAND','[DropSystem]','[ReleaseArbiter] commitment','[AlignMode] ->','waypoint temp ','waypoint_temp:')):continue
 m=re.search(r'\[(179\d+\.\d+)\]',line)
 if m:logs.append(dict(t=float(m.group(1))-d['start'],line=i,text=re.sub(r'\\u001b\[[0-9;]*m|\x1b\[[0-9;]*m','',line)))
logs.sort(key=lambda x:x['t']);write_csv('control_log_events.csv',logs)
slots=[];trajectory=[];fig,axes=plt.subplots(3,3,figsize=(16,11))
for slot in (1,2,3):
 releases=[r for r in d['rows']['release'] if r['m']['payload_slot']==slot];start,ack=releases
 cls=start['m']['target_class'];seq=start['m']['decision_seq'];t=start['t'];align=next(r for r in d['rows']['result'] if r['m']['decision_seq']==seq and r['m']['reason']=='strict_alignment_context_valid');accepted=next(r for r in d['rows']['result'] if r['m']['decision_seq']==seq and r['m']['reason']=='patrol_control_alignment_accepted');approach=next(r for r in d['rows']['command'] if r['m']['decision_seq']==seq)
 offset=np.array(ctl['drop_system']['slot_offsets'][slot-1]);setp=np.array(xyz(sp.msg(t)['pose']['position']));frozen=setp[:2]-offset
 maps=[(r,z) for r,z in detections(d,'mapped',cls) if z['map_valid'] and align['stamp']-.7<=r['stamp']<=align['stamp']+.3]
 target=np.median([xyz(z['map_point'])[:2] for _,z in maps],axis=0) if maps else np.array([np.nan,np.nan])
 snapshots=[]
 for label,rr in [('call',start),('ack',ack)]:
  tt=rr['t'];pr=pose.row(tt);sr=sp.row(tt);pp=pr['m']['pose'];pv=np.array(xyz(pp['position']));sv=np.array(xyz(sr['m']['pose']['position']));rot=rotation(pp);rpy=rot.as_euler('xyz',degrees=True);yaw=math.radians(rpy[2]);R2=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
  # CONDITIONAL measured installation interpretation: lateral/horizontal lever arm only.
  assumed_outlet=pv[:2]+R2@offset;ideal=-R2@offset
  f,_=rgb_frame(d,run/'replay',tt);auth=Timeline(extra['/mission/release_authorization']).row(tt)
  snapshots.append(dict(event=label,t=tt,pose=pv.tolist(),setpoint=sv.tolist(),tracking_xy_m=(pv[:2]-sv[:2]).tolist(),pose_age_s=tt-pr['t'],setpoint_age_s=tt-sr['t'],roll_pitch_yaw_deg=rpy.tolist(),target_estimate_xy=target.tolist(),body_minus_target_estimate_xy_m=(pv[:2]-target).tolist(),configured_offset_in_body_xy_m=(R2.T@offset).tolist(),ideal_fc_offset_if_table_is_installation_xy_m=ideal.tolist(),conditional_outlet_xy=assumed_outlet.tolist(),conditional_outlet_minus_target_xy_m=(assumed_outlet-target).tolist(),source_image=f,authorization_reason=(auth or {}).get('m',{}).get('reason',''),authorization_age_s=tt-auth['t'] if auth else None))
 rows=[]
 for pr in extra['/navigation/local_pose']:
  tt=pr['t']
  if not accepted['t']<=tt<=ack['t']+2.4:continue
  pp=pr['m']['pose'];pv=np.array(xyz(pp['position']));sv=np.array(xyz(sp.msg(tt)['pose']['position']));yaw=rotation(pp).as_euler('xyz')[2]
  row=dict(slot=slot,t=tt,fc_x=pv[0],fc_y=pv[1],fc_z=pv[2],sp_x=sv[0],sp_y=sv[1],sp_z=sv[2],yaw_deg=math.degrees(yaw),frozen_x=frozen[0],frozen_y=frozen[1],mapped_target_x=target[0],mapped_target_y=target[1]);rows.append(row);trajectory.append(row)
 ar=np.array([[r[k] for k in ('t','fc_x','fc_y','fc_z','sp_x','sp_y','sp_z')] for r in rows]);at=ar[:,0]-accepted['t']
 for j in (0,1):
  ax=axes[slot-1,j];ax.plot(at,(ar[:,1+j]-frozen[j])*100,label='FC vs frozen');ax.plot(at,(ar[:,4+j]-frozen[j])*100,label='setpoint vs frozen');ax.axhline((target[j]-frozen[j])*100,color='g',ls='--',label='mapped target estimate');ax.axvline(t-accepted['t'],color='red',label='raw call');ax.axvline(ack['t']-accepted['t'],color='orange',ls=':',label='ACK');ax.set_title(f'Slot {slot} {cls}: {"XY"[j]} offset [cm]');ax.grid();ax.legend(fontsize=7)
 ax=axes[slot-1,2];ax.plot(at,ar[:,3]-meta['ground_reference']['ground_z'],label='FC nominal AGL');ax.plot(at,ar[:,6]-meta['ground_reference']['ground_z'],label='setpoint nominal AGL');ax.axvline(t-accepted['t'],color='red');ax.axvline(ack['t']-accepted['t'],color='orange',ls=':');ax.legend(fontsize=7);ax.grid();ax.set_title(f'Slot {slot}: height [m]')
 slots.append(dict(slot=slot,physical_label={1:'rear',2:'right',3:'left'}[slot],target=cls,decision_seq=seq,approach_t=approach['t'],alignment_accepted_t=accepted['t'],alignment_valid_t=align['t'],call_t=t,ack_t=ack['t'],offset_camera_init_xy=offset.tolist(),frozen_alignment_xy=frozen.tolist(),mapped_at_alignment_xy=target.tolist(),mapped_sample_count=len(maps),snapshots=snapshots))
fig.suptitle('Recorded FC/setpoint vs frozen pixel alignment; target is recorded projection, NOT ground truth');fig.tight_layout();fig.savefig(out/'delivery_alignment.png',dpi=150);plt.close(fig);write_csv('delivery_trajectory.csv',trajectory);write_json('slots.json',slots)
# Record FC mode, landed state, handoff facts. Sampled ROS states are not PX4 causal events.
trans=[]
for key,rows,fields in [('fc',d['rows']['fc'],('mode','armed')),('landed',extra['/mavros/extended_state'],('landed_state',))]:
 last=None
 for r in rows:
  val=tuple(r['m'][k] for k in fields)
  if val!=last:trans.append(dict(t=r['t'],topic=key,value=dict(zip(fields,val))));last=val
handoff=[dict(t=r['t'],**json.loads(r['m']['data'])) for r in extra['/patrol_control/external_landing_handoff']];write_json('flight_state_transitions.json',dict(transitions=trans,handoff=handoff))
# Current and historical H observations; use same code and comparable stage windows.
references=[('221730',d,run/'replay',ctl),('1006_160434',json.loads((ROOT/'logs/h_flight_review_20261006/replay_160434/data.json').read_text()),ROOT/'logs/h_flight_review_20261006/replay_160434',yaml.safe_load((ROOT/'logs/h_flight_review_20261006/board_landing_20261006_160434/control.yaml').read_text()))]
hstats=[];hrows=[];sheets=[];fig,axes=plt.subplots(2,2,figsize=(13,9))
for name,q,base,c in references:
 land=next(r for r in q['rows']['result'] if r['m']['reason']=='patrol_control_landing_accepted')['t'];raw=detections(q,'raw','landing_pad');maps=detections(q,'mapped','landing_pad');valid=[(r,z) for r,z in maps if z['map_valid']];mp=np.array([xyz(z['map_point'])[:2] for r,z in valid]);median=np.median(mp,axis=0);delay=[r['t']-r['stamp'] for r,z in raw];gaps=np.diff(sorted(set(r['stamp'] for r,z in raw)));frame_matches=[]
 rawframes=Frames([r for r,z in raw]);photos=[]
 for f in q['frames']:
  if not land<=f['stamp']<=raw[-1][0]['stamp']:continue
  matched=rawframes.match(f['stamp']);frame_matches.append(bool(matched))
  if matched is None:continue
  z=matched['m']['detections'][0];im=cv2.imread(str(image_file(base,f)))
  if im is None:continue
  roi=z['roi'];crop=im[max(0,roi['y_offset']):min(im.shape[0],roi['y_offset']+roi['height']),max(0,roi['x_offset']):min(im.shape[1],roi['x_offset']+roi['width'])]
  gray=cv2.cvtColor(crop,cv2.COLOR_BGR2GRAY);hsv=cv2.cvtColor(crop,cv2.COLOR_BGR2HSV);dark=gray<90;bright=gray>170
  photos.append(dict(t=f['stamp'],gray_p50=float(np.median(gray)),dark_v_p50=float(np.median(hsv[:,:,2][dark])) if dark.any() else None,dark_s_p50=float(np.median(hsv[:,:,1][dark])) if dark.any() else None,bright_gray_p50=float(np.median(gray[bright])) if bright.any() else None))
 firstfull=next(((r,z) for r,z in raw if 100<z['center_px']['z']<200),raw[0]);ft=firstfull[0]['stamp'];f,im=rgb_frame(q,base,ft);im=cv2.resize(im,(640,360));cv2.putText(im,f'{name} H source {f["stamp"]:.3f}s',(12,30),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,255),2);sheets.append(im)
 stat=dict(name=name,reference_scope='H recognition/alignment, not autonomous success' if name!='221730' else 'test field chain acceptance',land_accepted_t=land,raw_count=len(raw),mapped_count=len(maps),mapped_valid_count=len(valid),first_raw_receipt_t=raw[0][0]['t'],last_raw_receipt_t=raw[-1][0]['t'],first_observation_after_land_s=raw[0][0]['stamp']-land,source_gap_s=quant(gaps),receipt_minus_image_stamp_s=quant(delay),map_median_xy=median.tolist(),map_radial_deviation_m=quant(np.linalg.norm(mp-median,axis=1)),tf_age_s=quant([z['transform_age_sec'] for r,z in valid]),sameframe_matched=len([v for v in frame_matches if v]),sampled_frames_in_raw_span=len(frame_matches),photometry=photos,comparability='ROI includes board/background; no exposure/lux telemetry, no human truth')
 hstats.append(stat)
 for r,z in maps:hrows.append(dict(run=name,t=r['t'],source_t=r['stamp'],map_valid=z['map_valid'],x=z['map_point']['x'],y=z['map_point']['y'],center_x=z['center_px']['x'],center_y=z['center_px']['y'],radius=z['center_px']['z'],tf_age=z['transform_age_sec']))
 axes[0,0].plot([r['stamp']-land for r,z in valid],(mp[:,0]-median[0])*100,label=name);axes[0,1].plot([r['stamp']-land for r,z in valid],(mp[:,1]-median[1])*100,label=name)
 axes[1,0].scatter([r['stamp']-land for r,z in raw],[z['center_px']['y'] for r,z in raw],s=5,label=name)
 qt=TransformTree(q['rows'],q['start']);positions=[]
 for r in q['rows']['odom']:
  if not land-2<r['t']<q['duration']:continue
  m=r['m'];mat=qt.matrix(m['header']['frame_id'],'camera_init',r['t'])
  if mat is not None:positions.append((r['t']-land,*(mat@np.array([*xyz(m['pose']['pose']['position']),1]))[:3]))
 ar=np.array(positions);ground=c['uav_vision']['drop_ground_z'];axes[1,1].plot(ar[:,0],ar[:,3]-ground,label=name)
for ax,title in zip(axes.flat,['mapped H X deviation [cm]','mapped H Y deviation [cm]','H image center y [px]','FC nominal AGL [m]']):ax.set_title(title);ax.set_xlabel('seconds from LAND acceptance');ax.grid();ax.legend()
fig.suptitle('Recorded H comparison; per-run median is not truth; 1006 = old HSV / AUTO.LAND');fig.tight_layout();fig.savefig(out/'h_comparison.png',dpi=150);plt.close(fig);cv2.imwrite(str(out/'h_capture_comparison.jpg'),np.hstack(sheets));write_csv('h_observations.csv',hrows)
# Same-frame geometry A/B from existing production extraction tool, no ROS stages changed.
frames=json.loads((out/'h_frames.json').read_text());geo={}
for variant,folder in [('otsu','h_otsu'),('hsv','h_hsv')]:
 with (out/folder/'geometry.csv').open() as f: vals=list(csv.DictReader(f))
 geo[variant]=[dict(frame_t=fr['stamp'],found=int(v['found']),x=float(v['x']),y=float(v['y']),quality=float(v['quality']),radius=float(v['radius'])) for fr,v in zip(frames,vals)]
intervals=[('before_land',275,288.576),('land_to_latch',288.576,291.594),('latch_to_handoff',291.594,294.577),('pilot_tail',294.577,307)]
for stat in hstats:
 if stat['name']=='221730':
  stat['sameframe_geometry']={key:{label:dict(frames=sum(lo<=r['frame_t']<hi for r in vals),found=sum(r['found'] for r in vals if lo<=r['frame_t']<hi)) for label,lo,hi in intervals} for key,vals in geo.items()}
write_json('h_comparison.json',hstats);write_json('sameframe_h_geometry.json',geo)
# Key current control samples. Constant times reference identified recorded events above.
hmed=np.array(hstats[0]['map_median_xy']);hsamples=[]
for label,tt in [('land_accepted',288.596),('alignment_latched',291.594),('handoff_requested',294.498),('handoff_observed',294.578),('on_ground',296.776),('disarm_and_cancel',299.577),('record_end',305.903)]:
 pr=pose.row(tt);sr=sp.row(tt);pv=np.array(xyz(pr['m']['pose']['position']));sv=np.array(xyz(sr['m']['pose']['position']));hsamples.append(dict(event=label,t=tt,pose_xyz=pv.tolist(),setpoint_xyz=sv.tolist(),fc_minus_h_median_xy_m=(pv[:2]-hmed).tolist(),fc_distance_h_median_xy_m=float(np.linalg.norm(pv[:2]-hmed)),roll_pitch_yaw_deg=rotation(pr['m']['pose']).as_euler('xyz',degrees=True).tolist(),pose_age_s=tt-pr['t'],setpoint_age_s=tt-sr['t']))
write_json('h_control_samples.json',hsamples)
# Conditional slot3 geometry illustrates signs only, not an observed physical outlet/impact.
s=slots[2];fig,ax=plt.subplots(figsize=(8,7));target=np.array(s['mapped_at_alignment_xy']);frozen=np.array(s['frozen_alignment_xy']);base=s['snapshots'][0];body=np.array(base['pose'][:2]);goal=np.array(base['setpoint'][:2]);outlet=np.array(base['conditional_outlet_xy']);ideal=target+np.array(base['ideal_fc_offset_if_table_is_installation_xy_m'])
for label,point,color in [('mapped center estimate',target,'green'),('frozen pixel alignment',frozen,'gray'),('configured FC goal',goal,'orange'),('FC at raw call',body,'blue'),('CONDITIONAL left outlet',outlet,'red'),('CONDITIONAL ideal FC',ideal,'purple')]:
 xy=(point-target)*100;ax.scatter(*xy,color=color,s=75);ax.annotate(label,xy,xytext=(8,8),textcoords='offset points',color=color,fontsize=9)
for a0,a1,color in [(frozen,goal,'orange'),(body,outlet,'red'),(target,ideal,'purple')]:
 ax.annotate('',xy=(a1-target)*100,xytext=(a0-target)*100,arrowprops=dict(arrowstyle='->',color=color,lw=2))
ax.set_aspect('equal',adjustable='datalim');ax.set_xlabel('camera_init X relative to projected target [cm]');ax.set_ylabel('camera_init Y relative to projected target [cm]');ax.grid();ax.set_title('Slot3: observed FC/goal + CONDITIONAL r_left=(0,+12cm) FLU\nNo physical outlet tracking or package impact truth');fig.tight_layout();fig.savefig(out/'slot3_geometry.png',dpi=150);plt.close(fig)
# Annotation sheet anchored to original camera timestamps; mark principal point and recorded centers.
tiles=[]
for t in (219.2,220.8,222,224,227,228.8,229.8,231,288.8,291.6,294.5,296.8):
 f,im=rgb_frame(d,run/'replay',t);k=np.array(json.loads((run/'camera_info.json').read_text())['K']).reshape(3,3);cv2.drawMarker(im,(round(k[0,2]),round(k[1,2])),(0,0,255),cv2.MARKER_CROSS,30,2)
 for key,color in [('mapped',(255,0,255)),('raw',(0,255,0))]:
  line=Frames(d['rows'][key]).match(f['stamp'])
  if line:
   for z in line['m']['detections']:
    if z['class_name'] in ('panzer','landing_pad'):
     cv2.drawMarker(im,(round(z['center_px']['x']),round(z['center_px']['y'])),color,cv2.MARKER_CROSS,25,2)
 im=cv2.resize(im,(640,360));cv2.putText(im,f'source t={f["stamp"]:.3f}s',(12,32),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,255),2);tiles.append(im)
cv2.imwrite(str(out/'sameframe_contact_sheet.jpg'),np.vstack([np.hstack(tiles[i:i+3]) for i in range(0,len(tiles),3)]))
write_json('analysis_scope.json',dict(run=str(run),bag_start_epoch=d['start'],duration_s=d['duration'],video_fps=5,ground_z=meta['ground_reference']['ground_z'],user_acceptance='test-field chain acceptance; not all automatic gates PASS',no_ulog=True,no_ground_truth=True,controls_or_extrinsics_changed=False,reference_bag='20261006_160434, H detected / descent / manual takeover; not a known full autonomous success',geometry_runtime='local extracted production C++ OpenCV; not measured board inference',installation_interpretation='12cm measured magnitude preserved; lever-arm signs/frame shown conditionally, not calibrated from FC positions'))
print(json.dumps(dict(slots=slots,h=[{k:v for k,v in s.items() if k!='photometry'} for s in hstats]),ensure_ascii=False,indent=2))
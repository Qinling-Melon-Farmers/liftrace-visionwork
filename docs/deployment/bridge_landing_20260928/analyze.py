"""Analyze recorded evidence; never publish ROS messages or execute inference."""
import argparse,bisect,collections,csv,json
from pathlib import Path
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def xyz(p): return [p[k] for k in ('x','y','z')]
def changes(rows,fn):
 out=[];last=object()
 for row in rows:
  v=fn(row['m'])
  if v!=last:out.append(dict(t=row['t'],value=v));last=v
 return out
def at(rows,t):
 return rows[max(0,bisect.bisect_right([v['t'] for v in rows],t)-1)]
def write_csv(path,rows):
 if rows:
  with path.open('w',newline='') as f:
   w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)

def main():
 p=argparse.ArgumentParser();p.add_argument('--base',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
 a.out.mkdir(parents=True,exist_ok=True);summary={}
 cv2.setNumThreads(2)
 for tag in ['17-16-41','17-34-47']:
  run=a.base/tag;out=a.out/tag;out.mkdir(exist_ok=True)
  d=json.loads((run/'replay/data.json').read_text());r=d['rows']
  sup=json.loads((run/'supplement.json').read_text());extra=json.loads((run/'extra.json').read_text())
  motion=np.array([[v['t'],*xyz(v['m']['pose']['pose']['position'])] for v in r['odom']]);mt=motion[:,0]
  vel=np.column_stack([(np.interp(mt+.25,mt,motion[:,i])-np.interp(mt-.25,mt,motion[:,i]))/.5 for i in (1,2,3)])
  speed=np.linalg.norm(vel[:,:2],axis=1)
  sp=np.array([[v['t'],*xyz(v['m']['pose']['position'])] for v in r['setpoint']])
  fc=changes(r['fc'],lambda m:[m['mode'],m['armed']]);ls=changes(sup['/mavros/extended_state'],lambda m:m['landed_state'])
  exits=[x['t'] for x in fc if x['value'][0]!='OFFBOARD' and any(y['value'][0]=='OFFBOARD' and y['t']<x['t'] for y in fc)]
  mode_exit=exits[0];takeoff=next(x['t'] for x in ls if x['value']==2);land=next(x['t'] for x in ls if x['value']==1 and x['t']>takeoff)
  commands=[dict(t=x['t'],source_t=x['stamp'],seq=x['m']['decision_seq'],command=x['m']['command'],target=x['m']['target_class'],slot=x['m']['payload_slot'],goal=xyz(x['m']['goal']['pose']['position']),reason=x['m']['reason']) for x in r['command']]
  events=[]
  for key in ('command','result','release'):
   for x in r[key]:
    m=x['m'];events.append(dict(t=x['t'],kind=key,seq=m.get('decision_seq',''),target=m.get('target_class',''),slot=m.get('payload_slot',''),reason=m.get('reason',''),success=m.get('success','')))
  events.sort(key=lambda v:v['t']);write_csv(out/'events.csv',events)
  phases=[];names={0:'SEARCH',1:'APPROACH',2:'DROP',3:'RESUME',4:'RETURN',5:'LAND'}
  for i,c in enumerate(commands):
   end=commands[i+1]['t'] if i+1<len(commands) else mode_exit;mask=(mt>=c['t'])&(mt<end)
   if mask.any():phases.append(dict(seq=c['seq'],phase=names[c['command']],target=c['target'],start=c['t'],end=end,seconds=end-c['t'],local_z_p50=float(np.median(motion[mask,3])),speed_xy_p50=float(np.median(speed[mask])),speed_xy_p95=float(np.percentile(speed[mask],95))))
  write_csv(out/'phase_stats.csv',phases)
  detector=[x for x in r['raw'] if x['m']['source']=='target_detector']
  targets={}
  for x in r['targets']:
   for q in x['m']['targets']:
    key=str(q['id'])+':'+q['class_name'];v=targets.setdefault(key,dict(first=x['t'],first_confirmed=None,first_confirmed_valid=None,max_streak=0,max_count=0))
    if q['state']==2:
     if v['first_confirmed'] is None:v['first_confirmed']=x['t']
     if q['map_valid'] and v['first_confirmed_valid'] is None:v['first_confirmed_valid']=x['t']
    v['max_streak']=max(v['max_streak'],q['consecutive_observe_count']);v['max_count']=max(v['max_count'],q['observe_count'])
  intervals=[(60.874,112.073,'panzer')] if tag=='17-16-41' else [(69.276,95.386,'panzer'),(121.446,129.613,'bridge')]
  geometry=[]
  for lo,hi,cls in intervals:
   circ=[q for x in r['mapped'] if lo<=x['t']<=hi for q in x['m']['detections'] if q['class_name']=='circle' and q['map_valid']]
   scores=np.array([q['geometry_confidence'] for q in circ])
   sem=[q for x in r['mapped'] if lo<=x['t']<=hi for q in x['m']['detections'] if q['class_name']==cls and q['map_valid']]
   raw=[q for x in detector if lo<=x['t']<=hi for q in x['m']['detections'] if q['class_name']==cls]
   ctx=[x for x in extra['/uav_vision/release_evidence_context'] if lo<=x['t']<=hi]
   geometry.append(dict(target=cls,start=lo,end=hi,raw_category_count=len(raw),valid_refined_category_count=len(sem),valid_circle_count=len(circ),circle_score_min=float(scores.min()),circle_score_p50=float(np.median(scores)),circle_score_max=float(scores.max()),circle_score_ge_080=int((scores>=.8).sum()),circle_score_ge_070=int((scores>=.7).sum()),context_reasons=dict(collections.Counter(x['m']['context_reason'] for x in ctx))))
  gaps=[]
  for key,rows in [('setpoint',r['setpoint']),('cloud',r['cloud']),('mission',r['mission']),('targets',r['targets']),('detector',detector)]:
   ts=np.array([x['t'] for x in rows])
   if len(ts)<2:continue
   gd=np.diff(ts);idx=np.argsort(gd)[-5:]
   gaps.extend(dict(topic=key,start=float(ts[i]),end=float(ts[i+1]),gap=float(gd[i])) for i in idx if gd[i]>.3)
  write_csv(out/'message_gaps.csv',gaps)
  tf={}
  for x in r['tf']:
   for tr in x['m']['transforms']:
    key=tr['header']['frame_id']+' -> '+tr['child_frame_id'];v=tf.setdefault(key,dict(first=x['t'],last=x['t'],n=0));v['last']=x['t'];v['n']+=1
  firstarm=next(x['t'] for x in fc if x['value'][1]);ground=motion[mt<firstarm,3]
  releases=[dict(t=x['t'],**{k:x['m'][k] for k in ['payload_slot','target_class','success','reason']},fc_local_z=float(np.interp(x['t'],mt,motion[:,3]))) for x in r['release']]
  control_land=[x for x in extra['/mission/command'] if x['m']['command'] in (4,5)]
  permit3=[x for x in extra['/mission/release_permission'] if x['m']['payload_slot']==3 and x['m']['permitted']]
  stats=dict(bag=Path(d['bag']).name,duration=d['duration'],frames=len(d['frames']),missing=d['missing'],commands=commands,fc_changes=fc,landed_changes=ls,recorded_airborne_seconds=land-takeoff,prearm_local_z_median=float(np.median(ground)),max_local_z=float(motion[:,3].max()),release_results=releases,targets=targets,geometry=geometry,phases=phases,message_gaps=gaps,tf_streams=tf,last_topic_times={k:v[-1]['t'] for k,v in r.items() if v},detector_class_counts=dict(collections.Counter(q['class_name'] for x in detector for q in x['m']['detections'])),third_slot_permitted_count=len(permit3),final_mission=json.loads(r['mission'][-1]['m']['data']),recorded_control_land_or_return_count=len(control_land),offboard_exit=mode_exit,setpoint_last=sp[-1,0])
  assert not any(c['command'] in (4,5) for c in commands)
  assert not control_land
  if tag=='17-34-47':assert sp[-1,0]<mode_exit and len(permit3)==0
  else:assert sp[-1,0]>mode_exit and geometry[0]['context_reasons']=={'alignment_context_geometry_missing':435}
  summary[tag]=stats;(out/'metrics.json').write_text(json.dumps(stats,ensure_ascii=False,indent=2)+'\n')
  fig,axes=plt.subplots(3,1,figsize=(13,10),sharex=True)
  axes[0].plot(mt,motion[:,3],label='FC local Z (not AGL)');axes[0].plot(sp[:,0],sp[:,3],label='recorded setpoint Z',alpha=.8)
  axes[1].plot(mt,speed,label='XY speed (0.5s pose difference)')
  for cls,color in [('red_cross','red'),('panzer','darkorange'),('bridge','royalblue'),('tank','purple')]:
   vals=[(x['stamp'],q['class_confidence']) for x in detector for q in x['m']['detections'] if q['class_name']==cls]
   if vals:arr=np.array(vals);axes[2].scatter(arr[:,0],arr[:,1],label=cls,c=color,s=8)
  for ax in axes:
   for rel in releases:ax.axvline(rel['t'],color='green',ls=':',alpha=.7)
   ax.axvline(mode_exit,color='red',ls='--',label='OFFBOARD exit');ax.legend(loc='upper right');ax.grid(alpha=.2)
  axes[0].set_ylabel('Z / m');axes[1].set_ylabel('XY / m/s');axes[2].set_ylabel('YOLO confidence');axes[2].set_xlabel('Bag seconds (detections: image stamp)')
  fig.suptitle(tag+' | recorded flight: no task LAND command');fig.tight_layout();fig.savefig(out/'height_speed_detection.png',dpi=150);plt.close(fig)
  fig,ax=plt.subplots(figsize=(11,6));flight=motion[mt<=land];ax.plot(flight[:,1],flight[:,2],label='FC estimated trajectory until ON_GROUND')
  search=np.array([c['goal'] for c in commands if c['command'] in (0,3)]);ax.plot(search[:,0],search[:,1],'--o',label='issued SEARCH / RESUME')
  for c in commands:
   if c['command']==1:ax.scatter(*c['goal'][:2],marker='x',s=90);ax.text(c['goal'][0]+.06,c['goal'][1],c['target'])
  p0=np.array(xyz(at(r['odom'],land)['m']['pose']['pose']['position']));ax.scatter(*p0[:2],c='red',s=90,marker='s',label='FC on-ground')
  ax.annotate('+X initial head',xy=(1,0),xytext=(.1,.25),arrowprops=dict(arrowstyle='->',color='red'),color='red')
  ax.set_xlabel('MAVROS map X / m');ax.set_ylabel('MAVROS map Y / m');ax.set_aspect('equal');ax.legend();ax.grid(alpha=.25);fig.tight_layout();fig.savefig(out/'trajectory.png',dpi=150);plt.close(fig)
  # Local failure window with unconnected stale setpoint (do not extrapolate it).
  lo=58 if tag=='17-16-41' else 118;hi=118 if tag=='17-16-41' else 136
  fig,axes=plt.subplots(3,1,figsize=(13,9),sharex=True)
  axes[0].plot(mt,motion[:,3],label='FC local Z');axes[0].plot(sp[:,0],sp[:,3],label='setpoint Z')
  grid=np.arange(lo,hi,.05);st=sp[:,0];ages=grid-st[np.maximum(0,np.searchsorted(st,grid,side='right')-1)]
  axes[1].plot(grid,ages,label='age of last recorded setpoint');axes[1].axhline(.5,color='gray',ls=':')
  tt=[];gg=[]
  for x in r['mapped']:
   for q in x['m']['detections']:
    if q['class_name']=='circle' and lo<=x['t']<=hi:tt.append(x['t']);gg.append(q['geometry_confidence'])
  axes[2].scatter(tt,gg,s=5,label='projected circle score');axes[2].axhline(.8,c='red',ls='--',label='remote aux threshold .80');axes[2].axhline(.7,c='green',ls=':',label='remote standard threshold .70')
  for ax in axes:
   ax.axvline(mode_exit,color='red',ls='--');ax.set_xlim(lo,hi);ax.grid(alpha=.25);ax.legend(loc='upper left')
  axes[0].set_ylabel('Z / m');axes[1].set_ylabel('setpoint age / s');axes[2].set_ylabel('geometry score');axes[2].set_xlabel('Bag seconds')
  fig.tight_layout();fig.savefig(out/'failure_window.png',dpi=150);plt.close(fig)
  times=[24.8,57.8,65.,85.,110.,114.] if tag=='17-16-41' else [59.643,65.6,90.9,118.2,127.5,130.]
  frames=sorted(d['frames'],key=lambda x:x['stamp']);ft=np.array([x['stamp'] for x in frames]);cells=[]
  for i,t in enumerate(times):
   nearby=[x for x in detector if abs(x['stamp']-t)<.16 and any(q['class_name'] in ('panzer','bridge') for q in x['m']['detections'])]
   if nearby:t=min(nearby,key=lambda x:abs(x['stamp']-t))['stamp']
   fr=frames[int(np.argmin(abs(ft-t)))];im=cv2.imread(str(run/'replay'/fr['file']))
   if im is None:raise ValueError('frames required for contact sheet')
   matches=[q for x in r['raw'] if x['m']['source']=='target_detector' and abs(x['stamp']-fr['stamp'])<=.03 for q in x['m']['detections']]
   circs=[q for x in r['mapped'] if abs(x['stamp']-fr['stamp'])<=.03 for q in x['m']['detections'] if q['class_name']=='circle']
   for q in matches:
    roi=q['roi'];x,y,w,h=[roi[k] for k in ('x_offset','y_offset','width','height')];cv2.rectangle(im,(x,y),(x+w,y+h),(0,235,0),2);cv2.putText(im,q['class_name']+f" {q['class_confidence']:.3f}",(x,max(24,y-8)),0,.8,(0,235,0),2)
   for q in circs[:1]:
    x,y,rad=xyz(q['center_px']);cv2.circle(im,(int(x),int(y)),int(rad),(255,220,0),2);cv2.putText(im,f"circle {q['geometry_confidence']:.3f}",(int(x),int(y)),0,.7,(255,220,0),2)
   cv2.arrowedLine(im,(110,60),(30,60),(0,0,255),3);cv2.putText(im,'head +X / image LEFT',(120,66),0,.65,(0,0,255),2)
   cv2.arrowedLine(im,(70,90),(70,160),(0,220,220),3);cv2.putText(im,'body +Y',(85,155),0,.65,(0,220,220),2)
   pane=np.zeros((90,1280,3),np.uint8);po=at(r['odom'],t)['m']['pose']['pose']['position'];fcm=at(r['fc'],t)['m']['mode']
   cv2.putText(pane,f"{tag} image={fr['stamp']:.3f}s | FC Z={po['z']:.3f}m | {fcm}",(15,30),0,.8,(245,245,245),1)
   cv2.putText(pane,'RECORDED detections only; no inference rerun; source match <=30ms',(15,65),0,.65,(210,210,210),1)
   shot=np.vstack([im,pane]);cv2.imwrite(str(out/f'frame_{i+1}.jpg'),shot);cells.append(cv2.resize(shot,(640,405)))
  cv2.imwrite(str(out/'contact_sheet.jpg'),np.vstack([np.hstack(cells[i:i+2]) for i in range(0,6,2)]))
 (a.out/'metrics.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
 print(json.dumps({k:{f:v[f] for f in ['recorded_airborne_seconds','prearm_local_z_median','max_local_z','geometry','third_slot_permitted_count','offboard_exit','setpoint_last']} for k,v in summary.items()},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
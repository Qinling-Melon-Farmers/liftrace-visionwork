from pathlib import Path
import argparse,json,csv,collections
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ap=argparse.ArgumentParser();ap.add_argument('--run-dir',type=Path,required=True);ap.add_argument('--out-dir',type=Path,required=True);args=ap.parse_args();run=args.run_dir;n=run/'numeric';out=args.out_dir;out.mkdir(parents=True,exist_ok=True)
start=json.loads((run/'bag_info.json').read_text())['start']
def read(k):return json.loads((n/(k+'.json')).read_text())
ground=-.20592633428052062
r=list(csv.DictReader((n/'h_gate_reconstruction.csv').open()));a={k:np.array([float(x[k]) for x in r]) for k in r[0] if k!='failures'}
sp=[x for x in read('navigation__setpoint_mission') if 270<=x['t']<297];ps=[x for x in read('navigation__local_pose') if 270<=x['t']<297]
ts=np.array([x['t'] for x in sp]);zs=np.array([x['m']['pose']['position']['z']-ground for x in sp]);tp=np.array([x['t'] for x in ps]);zp=np.array([x['m']['pose']['position']['z']-ground for x in ps])
fig,ax=plt.subplots(4,1,figsize=(13,10),sharex=True)
ax[0].plot(tp,zp,label='FC AGL');ax[0].plot(ts,zs,label='Final position command AGL');ax[0].axhline(.35,color='gray',ls=':',label='Configured low target (never commanded)');ax[0].set_ylabel('FC height (m)');ax[0].legend(loc='lower left',fontsize=9)
ax[1].plot(a['t'],100*a['xy'],label='Max pose/odom distance to current H');ax[1].axhline(6,color='red',ls='--',label='Capture threshold: 6 cm');ax[1].set_ylim(0,20);ax[1].set_ylabel('XY error (cm)');ax[1].legend(loc='upper right',fontsize=9)
ax[2].plot(a['t'],100*a['speed']);ax[2].axhline(10,color='red',ls='--');ax[2].set_ylim(0,35);ax[2].set_ylabel('Horizontal speed (cm/s)')
ax[3].plot(a['t'],1000*a['image_age'],label='H image age at control tick');ax[3].axhline(200,color='red',ls='--',label='Inner high capture limit: 200 ms');ax[3].axhline(500,color='gray',ls=':',label='Outer H validity limit: 500 ms');ax[3].set_ylabel('Image age (ms)');ax[3].legend(loc='upper right',fontsize=9)
for x in ax:
 x.axvspan(280.7899,283.8895,color='green',alpha=.10);x.axvline(296.178,color='black',ls=':');x.grid(alpha=.25)
ax[3].set_xlabel('Seconds since bag start (2026-10-14 04:25:22.167 CST)');ax[0].set_title('H landing never entered descent: 200 ms inner image-age gate repeatedly resets the 1 s capture')
fig.tight_layout();fig.savefig(out/'h_handoff_gates.png',dpi=150);plt.close(fig)
marks=[]
for x in read('uav_vision__detections_mapped'):
 if 271.2<=x['t']<296.17:
  for d in x['m']['detections']:
   if d['class_name']=='landing_pad':marks.append((x,d))
mt=np.array([x['t'] for x,d in marks]);mp=np.array([[d['map_point'][k] for k in 'xy'] for x,d in marks]);ages=np.array([x['stamp']-d['header']['stamp'] for x,d in marks]);q=mp[mt>276];med=np.median(q,axis=0);rad=np.linalg.norm(q-med,axis=1)
fig,ax=plt.subplots(figsize=(8,7));ix=(tp>=271.2)&(tp<296.17);pp=np.array([[x['m']['pose']['position'][k] for k in 'xy'] for x in ps]);ax.plot(pp[ix,0],pp[ix,1],label='FC path');ax.scatter(mp[:,0],mp[:,1],c=mt,s=9,cmap='viridis',label='New visual H centers');ax.scatter([8.1354505265],[-3.456116797],marker='x',s=100,color='red',label='Configured H approach coordinate');ax.scatter(*med,s=100,marker='+',color='black',label='Median visual H center after 276s');ax.set_aspect('equal');ax.set_xlabel('Mission X (m)');ax.set_ylabel('Mission Y (m)');ax.grid(alpha=.25);ax.legend(fontsize=9);ax.set_title('H is continually observed; initial approach coordinate differs from visual center');fig.tight_layout();fig.savefig(out/'h_target_path.png',dpi=150);plt.close(fig)
metrics={'bag':'1014-0425-投递H2D2-H降落vhD2.bag','start_epoch':start,'landing_command_t':271.205,'posctl_first_record_t':296.178,'landing_wait_sec':296.178-271.205,'landing_mode':'vh / descent 2 / POSCTL','capture_agl_m':1.2,'low_target_agl_m':.35,'posctl_trigger_agl_m':.4,'setpoint_agl_range_m':[float(zs[(ts>=271.2)&(ts<296.17)].min()),float(zs[(ts>=271.2)&(ts<296.17)].max())],'new_H_observations':len(marks),'mapped_valid_H':sum(d['map_valid'] for x,d in marks),'H_receive_age_sec_median_p95_max':np.percentile(ages,[50,95,100]).tolist(),'H_after276_xy_median':med.tolist(),'H_after276_radial_scatter_m_p95_max':np.percentile(rad,[95,100]).tolist(),'reconstructed_ticks':len(r),'image_older_than_200ms_ticks':int(sum(a['image_age']>.2)),'geometry_speed_qualified_ticks':int(sum((a['xy']<=.06)&(a['speed']<.1))),'image_age_failed_among_geometry_speed_qualified':int(sum((a['xy']<=.06)&(a['speed']<.1)&(a['image_age']>.2))),'source_revision':'2adc69bf9ae7cc3b62cbd88f06ba61034c20f427','previous_local_revision':'e340976740b131b401460431e88123ba407fc158','remote_new_reachable_commits':29}
def spans(mask):
 groups=[];first=None;last=None
 for t,ok in zip(a['t'],mask):
  if ok:
   if first is None:first=t
   last=t
  elif first is not None:groups.append([float(last-first),float(first),float(last)]);first=None
 if first is not None:groups.append([float(last-first),float(first),float(last)])
 return sorted(groups,reverse=True)
metrics['geometry_speed_longest_spans_sec']=spans((a['xy']<=.06)&(a['speed']<.1))[:4];metrics['all_basic_gates_longest_span_sec']=spans([not x['failures'] for x in r])[0]
for file,label in [('h_gate_cpp_result.csv','production_gate'),('h_gate_cpp_sensitivity.csv','H_age_and_gap_500ms_sensitivity')]:
 rr=list(csv.DictReader((n/file).open()));ok=[float(x['now'])-start for x in rr if x['ready']=='1'];metrics[label]={'qualified_evaluations':len(ok),'first_qualification_t':ok[0] if ok else None}
(out/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2)+'\n');print(json.dumps(metrics,ensure_ascii=False,indent=2))
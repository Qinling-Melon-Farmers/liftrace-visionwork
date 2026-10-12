import json,re,math
from pathlib import Path
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
out=Path('/home/xhj/liftrace-deliverables/flight_review_20261012');fig,axes=plt.subplots(3,2,figsize=(15,10),sharex='col')
for col,key in enumerate(['bag6','full']):
 j=json.load(open(out/key/'analysis.json'));tim=json.load(open(out/key/'timing.json'));e=tim['events'][0];g=tim['ground_z'];a=e['align'];end=e['ack']+1
 for topic,label in [('/navigation/local_pose','FC feedback AGL'),('/navigation/setpoint_mission','FC command AGL')]:
  p=[p for p in j['poses'] if p[1]==topic and a<=p[0]<=end]
  axes[0,col].plot([p[0]-a for p in p],[p[6]-g for p in p],label=label,lw=1.2)
 axes[0,col].axhspan(.35,.45,color='green',alpha=.12);axes[0,col].axhline(.4,color='green',ls=':',lw=1)
 fr=next(r for r in j['logs'] if 'frozen slot=1' in r['text'])
 m=re.search(r'center=\(([-\d.]+),([-\d.]+)\) FC=\(([-\d.]+),([-\d.]+)\)',fr['text']);cx,cy,fx,fy=map(float,m.groups())
 pp=[p for p in j['poses'] if p[1]=='/navigation/local_pose' and e['freeze']<=p[0]<=end]
 ts=[];errors=[]
 for p in pp:
  qx,qy,qz,qw=p[7:11];yaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
  fc=math.hypot(p[4]-fx,p[5]-fy); outlet=math.hypot(p[4]-.12*math.cos(yaw)-cx,p[5]-.12*math.sin(yaw)-cy)
  ts.append(p[0]-a);errors.append(max(fc,outlet)*100)
 axes[1,col].plot(ts,errors,label='max FC/outlet error (rounded frozen XY)',lw=1.2);axes[1,col].axhline(4,color='green',ls='--')
 rr=[r for r in json.load(open(out/key/'permissions.json')) if r['topic']=='/mission/release_authorization' and a<=r['t']<=end]
 print(key,'permission_samples',len(rr))
 if rr:
  axes[2,col].step([r['t']-a for r in rr],[int(r['allowed']) for r in rr],where='post',label='authorization allowed')
 for name,color in [('freeze','orange'),('descent','purple'),('request','red'),('ack','black')]:
  for ax in axes[:,col]:ax.axvline(e[name]-a,color=color,ls=':',alpha=.7)
 axes[0,col].set_title(key+' red_cross: ALIGN to ACK %.2fs'%(e['ack']-a));axes[2,col].set_xlabel('Seconds since ALIGN accepted');axes[0,col].legend(fontsize=8);axes[1,col].legend(fontsize=7);axes[2,col].legend(fontsize=8)
 for ax in axes[:,col]:ax.grid(alpha=.2)
axes[0,0].set_ylabel('FC AGL (m)');axes[1,0].set_ylabel('Horizontal error (cm)');axes[2,0].set_ylabel('Permission');fig.tight_layout();fig.savefig(out/'red_cross_comparison.png',dpi=140)
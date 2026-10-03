from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parents[3];D=R/'logs/flight_delivery_20260921';S=D/'cloud_sequence'
pose=np.array(json.loads(next(D.glob('corridor_diag_20260920_23*.json')).read_text())['pose'])
fig,axs=plt.subplots(2,4,figsize=(18,9),sharex=True,sharey=True)
for ax,s in zip(axs.flat,[40,45,48,51,54,56,60,65]):
 for label,col,size in [('inflate','orange',9),('raw','navy',3)]:
  a=np.load(S/(label+str(s)+'.npy'));a=a[(a[:,2]>.15)&(a[:,2]<.65)]
  ax.scatter(a[:,0],a[:,1],s=size,c=col,alpha=.5,label=label)
 p=pose[pose[:,0]<=s];ax.plot(p[:,1],p[:,2],'g-',lw=1);ax.plot(p[-1,1],p[-1,2],'ro',ms=6)
 ax.set_title(f't={s}s; z slice 0.15..0.65m');ax.set_xlim(1.5,4.6);ax.set_ylim(-1.2,1.2);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)')
axs.flat[0].legend();fig.suptitle('23h flight: cloud evolution in camera_init; POSCTL begins at 55.335s')
fig.tight_layout();fig.savefig(R/'docs/deployment/flight_review_20260921/cloud_evolution.png',dpi=150)
c=json.loads((S/'counts.json').read_text())
for label in ['raw','inflate']:
 a=[v for v in c if v[1]==label and v[2]>10];print(label,'first >10 points in rear ROI:',a[0] if a else None)

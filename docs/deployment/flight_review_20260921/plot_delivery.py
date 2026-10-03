from pathlib import Path
import json,numpy as np
from scipy.interpolate import BSpline
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parents[3];D=R/'logs/flight_delivery_20260921';O=R/'docs/deployment/flight_review_20260921';O.mkdir(exist_ok=True)
stats={}
fig,axes=plt.subplots(2,3,figsize=(16,9))
for row,hour in enumerate(('22','23')):
 d=json.loads(next(D.glob('corridor_diag_20260920_'+hour+'*.json')).read_text());e=json.loads((D/('extra'+hour+'.json')).read_text())
 p=np.array(d['pose']);c=np.array(e['cmd']);raw=np.load(D/(hour+'_raw.npy'));occ=np.load(D/(hour+'_inflate.npy'))
 s=e['splines'][-1];curve=BSpline(s['knots'],s['pts'],3);ts=np.linspace(0,s['knots'][-4],1500);xyz=curve(ts)
 cut=occ[(occ[:,2]>.20)&(occ[:,2]<.40)];rawcut=raw[(raw[:,2]>.1)&(raw[:,2]<.7)]
 ax=axes[row,0];ax.scatter(rawcut[:,0],rawcut[:,1],s=2,c='gray',label='raw z=.1-.7');ax.scatter(cut[:,0],cut[:,1],s=3,c='orange',label='inflated z=.2-.4');ax.plot(p[:,1],p[:,2],label='flight');ax.plot(xyz[:,0],xyz[:,1],'r--',label='last spline');ax.set(xlim=(-.5,5.5),ylim=(-2,2),title=hour+':00 map / route',xlabel='X (m)',ylabel='Y (m)');ax.legend(fontsize=7);ax.set_aspect('equal')
 ax=axes[row,1];ax.plot(p[:,0],p[:,1],label='X actual');ax.plot(c[:,0],c[:,1],label='X command');ax.plot(p[:,0],p[:,3],label='Z actual');ax.plot(c[:,0],c[:,3],label='Z command');ax.set(title='Position vs bag elapsed time',xlabel='seconds');ax.legend()
 nearest=cKDTree(xyz).query(p[:,1:])[0];speed=np.linalg.norm(c[:,4:7],axis=1)
 ax=axes[row,2];ax.plot(p[:,0],nearest,label='distance to last spline');ax.plot(c[:,0],speed,label='command speed');ax.axhline(.45,color='r',linestyle=':',label='FSM .45m');ax.set(xlim=(s['t'],min(60,p[-1,0])),title='Last leg diagnostics',xlabel='seconds');ax.legend(fontsize=8)
 frozen=np.linalg.norm(c[1:,1:4]-c[:-1,1:4],axis=1)<1e-10
 starts=np.r_[0,np.where(~frozen)[0]+1];ends=np.r_[starts[1:]-1,len(c)-1];blocks=[(float(c[b,0]-c[a,0]),int(a),int(b)) for a,b in zip(starts,ends) if c[a,0]>=s['t']]
 length,a,b=max(blocks);t=c[a,0];near=p[np.argmin(abs(p[:,0]-t))];dist=float(cKDTree(xyz).query(near[1:])[0]);goal=np.array(s['pts'][-1])
 stats[hour]=dict(last_spline_t=s['t'],freeze_start=t,freeze_duration=length,freeze_xyz=c[a,1:4].tolist(),pose_at_freeze=near.tolist(),distance_to_curve_at_freeze=dist,goal=goal.tolist(),states=e['states'],inflated_points=len(occ),raw_points=len(raw),nearest_inflated_to_goal=float(cKDTree(occ).query(goal)[0]),nearest_inflated_to_curve=float(cKDTree(occ).query(xyz)[0].min()))
fig.tight_layout();fig.savefig(O/'flight_comparison.png',dpi=150);(O/'metrics.json').write_text(json.dumps(stats,indent=2));print(json.dumps(stats,indent=2))

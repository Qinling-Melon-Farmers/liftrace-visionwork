from pathlib import Path
import csv,json,math,bisect,numpy as np
h=Path(__file__).resolve().parents[4];d=h/'docs/verification/latest_six_20261005';b=json.loads((h/'logs/latest_six_20261005_batch/matrix.json').read_text());run=Path(next(x['run'] for x in b['results'] if x['seed']==38 and x['variant']=='snake2'))
def arr(name):return np.array([[float(r[k]) for k in ('t','x','y','z')] for r in csv.DictReader((run/(name+'.csv')).open())])
a=arr('truth_pose');fc=arr('mavros_pose');li=arr('lio_pose');p=arr('planner_setpoint');ct=136.178
interp=lambda ar,t:np.array([np.interp(t,ar[:,0],ar[:,i]) for i in range(1,4)])
truth=interp(a,ct);fcv=interp(fc,ct);liv=interp(li,ct)
path=p[(p[:,0]>=129.5)&(p[:,0]<=ct)][:,1:];start=path[:-1];v=path[1:]-start;coeff=np.clip(np.sum((truth-start)*v,axis=1)/np.maximum(1e-12,np.sum(v*v,axis=1)),0,1);points=start+v*coeff[:,None];dist=np.linalg.norm(points-truth,axis=1);j=int(np.argmin(dist))
last=a[(a[:,0]>=ct-2)&(a[:,0]<=ct-.1)];errors={}
for n,ar in [('mavros',fc),('lio',li)]:
 ds=[interp(ar,t)-xyz for t,*xyz in last];errors[n]=dict(max_xy_error_m=max(np.linalg.norm(x[:2]) for x in ds),max_abs_z_error_m=max(abs(x[2]) for x in ds))
out=dict(contact_ros_s=ct,truth_xyz=truth.tolist(),fc_xyz=fcv.tolist(),lio_xyz=liv.tolist(),fc_minus_truth_m=(fcv-truth).tolist(),lio_minus_truth_m=(liv-truth).tolist(),nearest_emitted_setpoint_polyline_xyz=points[j].tolist(),distance_to_sampled_setpoint_polyline_m=float(dist[j]),precontact_2s_estimation_errors=errors,note='Setpoint stream includes deliberate lookahead. Spatial polyline distance is diagnostic, not a full time-aligned B-spline collision proof. Interpolated LIO may include a post-contact sample; precontact error table ends 0.1 seconds before contact to avoid interpolating post-contact observations.')
(d/'collision_snake2_38_tracking.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
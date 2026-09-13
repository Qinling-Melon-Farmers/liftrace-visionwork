"""Offline landing-failure traces, using recorded poses and contact events."""
import argparse,json
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle,Rectangle

p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
params=yaml.safe_load((a.run/'rosparams.yaml').read_text())
contacts=json.loads((a.run/'gazebo_contact_status.json').read_text())
contact=contacts['events'][0]['ros_stamp']
ground=params['target_map_projector']['ground_z'];offset=params['competition_key_recorder']['truth_world_offset']
series={name:np.loadtxt(a.run/(name+'.csv'),delimiter=',',skiprows=1,ndmin=2)[:,:4] for name in ('truth_pose','mavros_pose','lio_pose','mavros_setpoint')}
series['truth_pose'][:,1:4]+=offset
for name in ('mavros_pose','lio_pose','mavros_setpoint'):series[name][:,3]-=ground
def at(name,t):
    rows=series[name]
    return np.array([np.interp(t,rows[:,0],rows[:,i]) for i in (1,2,3)])
t=np.linspace(contact-8,contact,401)
truth=np.array([at('truth_pose',v) for v in t]);mav=np.array([at('mavros_pose',v) for v in t]);lio=np.array([at('lio_pose',v) for v in t]);cmd=np.array([at('mavros_setpoint',v) for v in t])
fig,axes=plt.subplots(2,2,figsize=(12,9));xy,zax,err,control=axes.flat
xy.add_patch(Rectangle((4.8,8.),.2,1.3,color='.5',alpha=.4));xy.add_patch(Rectangle((3.7,9.1),1.3,.2,color='.5',alpha=.4))
xy.add_patch(Circle((4.2,8.5),.5,fill=False,color='black'));xy.text(4.2,8.5,'H',ha='center')
for points,label,style in [(truth,'Truth FC','-'),(mav,'MAVROS estimate','--'),(lio,'LIO',':')]:
    xy.plot(points[:,0],points[:,1],style,label=label,lw=1.4)
    zax.plot(t-contact,points[:,2],style,label=label,lw=1.2)
xy.scatter(*truth[-1,:2],marker='*',s=130,color='red',label='First wall contact')
xy.set(xlim=(3.7,5.),ylim=(8.,9.3),xlabel='X (m)',ylabel='Y (m)',title='H landing area; east wall contact');xy.set_aspect('equal');xy.legend(fontsize=8)
zax.plot(t-contact,cmd[:,2],label='Position command',lw=.8,color='gray');zax.set(xlabel='Seconds relative to contact',ylabel='FC AGL (m)',title='Near-ground estimate separation and rebound');zax.legend(fontsize=8)
err.plot(t-contact,np.linalg.norm(mav[:,:2]-truth[:,:2],axis=1),label='MAVROS XY error')
err.plot(t-contact,np.abs(mav[:,2]-truth[:,2]),label='MAVROS Z error')
err.plot(t-contact,np.linalg.norm(lio[:,:2]-truth[:,:2],axis=1),label='LIO XY error')
err.plot(t-contact,np.abs(lio[:,2]-truth[:,2]),label='LIO Z error')
err.set(xlabel='Seconds relative to contact',ylabel='Absolute difference (m)',title='Interpolated pose differences');err.legend(fontsize=8)
for i,name in enumerate(('X','Y','Z')):control.plot(t-contact,cmd[:,i]-mav[:,i],label=name)
control.set(xlabel='Seconds relative to contact',ylabel='Command minus estimate (m)',title='Position input error; not a measured thrust command');control.legend(fontsize=8)
for ax in axes.flat:ax.grid(alpha=.2)
for ax in (zax,err,control):ax.axvline(0,color='red',ls=':',lw=1)
fig.suptitle('Seed34 baseline | recorded LAND failure | no flight-code modification')
fig.tight_layout();fig.savefig(a.out/'seed34_landing_failure.png',dpi=170);plt.close(fig)
result=dict(contact_ros_s=contact,contact_pairs=contacts['events'][0]['pairs'],
            truth_fc_xyz_at_contact=truth[-1].tolist(),mavros_fc_xyz_at_contact=mav[-1].tolist(),lio_fc_xyz_at_contact=lio[-1].tolist(),
            mavros_xy_error_m=float(np.linalg.norm(mav[-1,:2]-truth[-1,:2])),mavros_z_error_m=float(abs(mav[-1,2]-truth[-1,2])),
            lio_xy_error_m=float(np.linalg.norm(lio[-1,:2]-truth[-1,:2])),lio_z_error_m=float(abs(lio[-1,2]-truth[-1,2])),
            limitation='Interpolated telemetry shows separation and rebound near ground; does not establish estimator/dynamics root cause or strategy causality.')
(a.out/'landing_failure_metrics.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))

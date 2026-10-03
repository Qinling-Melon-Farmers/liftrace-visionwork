"""Recorded landing geometry and PX4 descent commands; no new simulation."""
import csv,json,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Rectangle,Circle
D=Path(__file__).resolve().parent
items=json.loads((D/'runs.json').read_text());seq=json.loads((D/'landing_sequence.json').read_text())
fig,axes=plt.subplots(2,3,figsize=(15,8));xyfig,xyaxes=plt.subplots(1,3,figsize=(15,5));stats=[]
corners=np.array([[x,y,z] for x in (-.5,.5) for y in (-.5,.5) for z in (-.5,.5)])*[.55,.55,.4]+[0,0,-.02]
for item,row,ax,va,xa in zip(items,seq,axes[0],axes[1],xyaxes):
    run=Path(item['run'])
    with (run/'truth_pose.csv').open() as f:a=np.array([[float(v[k]) for k in ('t','x','y','z','qx','qy','qz','qw')] for v in csv.DictReader(f)])
    land=row['land_command_ros'];contact=row['contact_events'][0]['ros_stamp'] if row['contact_events'] else a[-1,0]
    select=(a[:,0]>=land-1)&(a[:,0]<=contact+.3);rt=a[:,0]-land
    rot=Rotation.from_quat(a[:,4:]).as_matrix();vertices=np.einsum('tij,kj->tki',rot,corners)+a[:,None,1:4]+np.array([0,0,.22]);bottom=vertices[:,:,2].min(axis=1)
    ax.plot(rt[select],a[select,3]+.22,label='True FC AGL');ax.plot(rt[select],bottom[select],label='55 cm guard bottom',lw=.8)
    ax.axhline(.005,color='gray',ls=':',label='H support top ~5 mm');ax.set(title=item['label'],ylabel='World height (m)')
    dt=np.diff(a[:,0]);ok=(dt>0)&(dt<.5)&select[1:];vz=np.diff(a[:,3])/np.maximum(dt,1e-6)
    va.plot(rt[1:][ok],vz[ok],label='True vertical speed')
    sp=row['setpoints'].get('trajectory_setpoint',[])
    va.plot([v['t']-land for v in sp],[-v['velocity[2]'] if v.get('velocity[2]') is not None else np.nan for v in sp],label='PX4 trajectory vz (up positive)',lw=.8)
    for sub in (ax,va):
        if row['first_near_support_ros']:sub.axvline(row['first_near_support_ros']-land,color='black',ls=':',label='Near support')
        if row['contact_events']:sub.axvline(contact-land,color='red',ls='--',label='Guard-wall event')
        sub.grid(alpha=.2);sub.legend(fontsize=7);sub.set_xlim(-1,float(a[-1,0]-land))
    va.set(xlabel='Seconds after LAND decision',ylabel='Vertical speed (m/s)')
    field=next(v for v in ET.parse(item['world']).iter('model') if v.get('name')=='toudi2')
    for wall in field.findall('link'):
        p=list(map(float,wall.findtext('pose').split()));s=list(map(float,wall.findtext('collision/geometry/box/size').split()))
        xa.add_patch(Rectangle((p[0]-s[0]/2,p[1]-s[1]/2),s[0],s[1],color='.55',alpha=.5))
    xa.add_patch(Circle((8.5,-4.2),.5,fill=False,color='black'));xa.text(8.5,-4.2,'H',ha='center')
    xa.plot(a[select,1],a[select,2],lw=1,label='True FC track')
    for t,color,label in [(row['first_near_support_ros'],'#1971ae','Near support'),(contact,'#ce4423','Contact / end')]:
        if t is None:continue
        i=min(len(a)-1,int(np.argmin(abs(a[:,0]-t))))
        poly=vertices[i,ConvexHull(vertices[i,:,:2]).vertices,:2]
        xa.add_patch(Polygon(poly,fill=False,edgecolor=color,lw=1.5,label=label+' guard'))
        xa.scatter(a[i,1],a[i,2],s=15,color=color)
    xa.set(xlim=(7.7,9.4),ylim=(-5.05,-3.4),xlabel='Start X (m)',ylabel='Start Y (m)',title=item['label']);xa.set_aspect('equal');xa.legend(fontsize=7);xa.grid(alpha=.2)
    near=row['first_near_support_ros'];before=(a[1:,0]>=near-.7)&(a[1:,0]<=near) if near else ok;after=(a[1:,0]>=near)&(a[1:,0]<=contact) if near else ok
    stats.append(dict(label=item['label'],first_near_support_ros=near,min_guard_bottom_m=float(bottom[select].min()),descent_min_vz_mps=float(vz[before&ok].min()) if (before&ok).any() else None,rebound_max_vz_mps=float(vz[after&ok].max()) if (after&ok).any() else None))
fig.suptitle('Approach support, descent command, then rebound | ground contact impulse was not archived',fontsize=13);fig.tight_layout();fig.savefig(D/'landing_ground_sequence.png',dpi=160);plt.close(fig)
xyfig.suptitle('Invisible robustness guard versus wall geometry | nearest 10 Hz truth samples, not visible airframe mesh',fontsize=12);xyfig.tight_layout();xyfig.savefig(D/'landing_contact_xy.png',dpi=160);plt.close(xyfig)
(D/'landing_ground_stats.json').write_text(json.dumps(stats,indent=2));print(json.dumps(stats,indent=2))

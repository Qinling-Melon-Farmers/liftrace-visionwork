"""Offline, nominal full high-route coverage; never changes flight configuration."""
from pathlib import Path
import json, math
import numpy as np
import cv2, yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.path import Path as PolygonPath
from scipy.spatial import ConvexHull
R=Path(__file__).resolve().parents[3];D=Path(__file__).resolve().parent
param=json.loads((R/'docs/verification/panzer_location_repair_20260927/preflight_seed38.json').read_text())
base='/navigation/mission_manager/high_view_probe/config/'
route=np.array([param[base+'staging_xy']]+param[base+'survey_xy'],float)
bounds=np.array(param['/navigation/mission_manager/high_view_full/boundary_policy/bounds'])
old=json.loads((R/'docs/deployment/flight_fov_20260926/result.json').read_text())['calibration']
w,h=old['w'],old['h'];fx,fy,cx,cy=old['K'][0],old['K'][4],old['K'][2],old['K'][5]
step=.01;x=np.arange(bounds[0]+step/2,bounds[1],step);y=np.arange(bounds[2]+step/2,bounds[3],step)
xx,yy=np.meshgrid(x,y);points=np.column_stack((xx.ravel(),yy.ravel()))
area=(bounds[1]-bounds[0])*(bounds[3]-bounds[2]);results=[];masks={}

def footprint(agl,new=False,centered=False,erode=0):
    height=agl-.16
    length,width=(3.6*height/1.84,1.8*height/1.84) if new else (w*height/fx,h*height/fy)
    cu,cv=(.5,.5) if centered else (cx/w,cy/h)
    xmin,xmax=-(1-cu)*length+erode,cu*length-erode
    ymin,ymax=-cv*width+erode,(1-cv)*width-erode
    return np.array([[xmin,ymin],[xmax,ymin],[xmax,ymax],[xmin,ymax]]),[length,width]

def coverage(fp,segments=None):
    mask=np.zeros(len(points),bool);cumulative=[]
    path=route if segments is None else route[:segments+1]
    for a,b in zip(path[:-1],path[1:]):
        q=np.vstack((fp+a,fp+b));q=q[ConvexHull(q).vertices]
        mask|=PolygonPath(q).contains_points(points)
        cumulative.append(float(mask.mean()*100))
    return mask.reshape(xx.shape),cumulative

for name,agl,new,centered in [('old_2_6',2.6,False,False),('measured_2_6',2.6,True,False),('measured_3_0',3.,True,False),('measured_2_6_centered',2.6,True,True),('measured_3_0_centered',3.,True,True)]:
    fp,size=footprint(agl,new,centered);mask,cum=coverage(fp)
    ring,_=coverage(footprint(agl,new,centered,erode=.5)[0]);masks[name]=(mask,ring,fp)
    eligible=(xx>=bounds[0]+.5)&(xx<=bounds[1]-.5)&(yy>=bounds[2]+.5)&(yy<=bounds[3]-.5)
    row=dict(name=name,fc_agl_m=agl,lens_agl_m=agl-.16,footprint_m=size,footprint_offsets_m=fp.tolist(),covered_percent=float(mask.mean()*100),uncovered_area_m2=float((~mask).mean()*area),cumulative_percent=cum,one_m_ring_center_coverage_percent=float(ring[eligible].mean()*100),assumed_centered=centered)
    row['targets']={}
    for seed in (31,38):
        scene=yaml.safe_load((R/f'docs/verification/history_31_40_20260920/seed_{seed}/field_config.yaml').read_text())
        targets=[]
        for target in scene['spawn']['frozen_layout']:
            pt=np.array([target['x'],target['y']]);hit=False;complete=False
            margin=.175 if target['class']=='red_cross' else .5
            small=footprint(agl,new,centered,erode=margin)[0]
            for a,b in zip(route[:-1],route[1:]):
                for poly,kind in ((np.vstack((fp+a,fp+b)),'point'),(np.vstack((small+a,small+b)),'full')):
                    inside=PolygonPath(poly[ConvexHull(poly).vertices]).contains_point(pt)
                    if kind=='point':hit|=inside
                    else:complete|=inside
            targets.append(dict(class_name=target['class'],xy=pt.tolist(),center_ever_visible=bool(hit),one_m_circle_or_035m_cross_ever_contained=bool(complete)))
        row['targets'][str(seed)]=targets
    results.append(row)

fig,axes=plt.subplots(2,3,figsize=(15,10))
for col,name in enumerate(('old_2_6','measured_2_6','measured_3_0')):
    row=next(r for r in results if r['name']==name);mask,ring,fp=masks[name]
    for idx,display in enumerate((mask,ring)):
        ax=axes[idx,col];ax.imshow(display,origin='lower',extent=bounds,cmap=ListedColormap(['#ffd5c8','#d7ecdc']),interpolation='nearest')
        ax.plot(route[:,0],route[:,1],'o--',color='#245c90',lw=1,ms=3)
        ax.plot(0,0,'k>');ax.annotate('Nose +X',xy=(.7,0),xytext=(-.4,.45),arrowprops={'arrowstyle':'->'},fontsize=8)
        foot=fp+route[2];ax.plot(*np.vstack((foot,foot[0])).T,color='#8c5a00',lw=1)
        for seed,marker,color in ((31,'x','#9d228c'),(38,'+','#111111')):
            pan=next(t for t in row['targets'][str(seed)] if t['class_name']=='panzer')
            ax.scatter(*pan['xy'],marker=marker,color=color,s=45,label=f'seed{seed} panzer')
        metric=row['covered_percent'] if idx==0 else row['one_m_ring_center_coverage_percent']
        ax.set(title=f'{name}: {metric:.2f}%'+(' ground points' if idx==0 else ' eligible 1m-ring centers'),xlabel='Fixed-start X / forward (m)',ylabel='Y / left (m)',aspect='equal');ax.legend(fontsize=7,loc='lower right');ax.set_xlim(bounds[:2]);ax.set_ylim(bounds[2:]);ax.grid(alpha=.12)
fig.suptitle('Complete nominal high route; no obstacle occlusion, no roll/pitch, no recognition guarantee\nTop: any ground point visible; bottom: 1m circle fits in one frame (target-center area shrunk 0.5m for metric)')
fig.tight_layout();fig.savefig(D/'coverage_comparison.png',dpi=150);plt.close(fig)
fig,axes=plt.subplots(1,3,figsize=(15,5))
fp,_=footprint(2.6,True)
for ax,n in zip(axes,(2,3,5)):
    mask,cum=coverage(fp,n)
    ax.imshow(mask,origin='lower',extent=bounds,cmap=ListedColormap(['#ffd5c8','#d7ecdc']))
    ax.plot(route[:,0],route[:,1],'k--',alpha=.25);ax.plot(route[:n+1,0],route[:n+1,1],'o-',color='#245c90')
    ax.set(title=f'After survey waypoint {n}: {cum[-1]:.2f}%',xlabel='X / forward (m)',ylabel='Y / left (m)',aspect='equal');ax.set_xlim(bounds[:2]);ax.set_ylim(bounds[2:])
fig.suptitle('Measured footprint, FC 2.6m: planned route prefixes, not actual early-interruption timestamps')
fig.tight_layout();fig.savefig(D/'route_prefixes.png',dpi=150);plt.close(fig)
output=dict(scope='offline ideal ground visibility only',step_m=step,area_m2=float(area),bounds=bounds.tolist(),route=route.tolist(),measured=dict(fc_agl_m=2.,lens_offset_m=.16,long_m=3.6,short_m=1.8),calibration=old,cases=results)
(D/'results.json').write_text(json.dumps(output,indent=2)+'\n')
print(json.dumps([dict(name=r['name'],coverage=r['covered_percent'],ring=r['one_m_ring_center_coverage_percent'],targets=r['targets']) for r in results],indent=2))

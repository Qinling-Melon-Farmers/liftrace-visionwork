"""Offline geometric review of the actual formal field template; starts no nodes."""
from pathlib import Path
import json, math, sys
import numpy as np
import yaml
from shapely.geometry import box, MultiPoint
from shapely.ops import unary_union
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as Patch

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/src'))
from uav_mission.search_policy import SearchPolicy
field = yaml.safe_load((ROOT/'deployment/competition/field.example.yaml').read_text())
rig = yaml.safe_load((ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/config/competition/known_rig.yaml').read_text())
cal = yaml.safe_load((ROOT/'vision_ws/src/camera_sdk/param/calibration_1280x720.yaml').read_text())
w,h = cal['image_width'],cal['image_height']
k=cal['camera_matrix']['data'];fx,fy,cx,cy=k[0],k[4],k[2],k[5]
offset = -(rig['body_to_imu_xyz'][2]+rig['imu_to_camera_xyz'][2])
route = np.array([field['staging_xy']]+field['survey_xy'],float)
def rectangle(bounds):
    a,b,c,d=bounds
    return box(a,c,b,d)
regions={key:rectangle(field[key]) for key in ('flight_bounds','target_bounds','coverage_bounds')}
def footprint(agl,model='measured',centered=False,margin=0):
    lens=agl-offset
    length,width=(3.6*lens/(2-offset),1.8*lens/(2-offset)) if model=='measured' else (w*lens/fx,h*lens/fy)
    u,v=(.5,.5) if centered else (cx/w,cy/h)
    a,b=-(1-u)*length+margin,u*length-margin
    c,d=-v*width+margin,(1-v)*width-margin
    assert a<b and c<d
    return np.array([[a,c],[b,c],[b,d],[a,d]]),[length,width]
def sweep(fp,path):
    return unary_union([MultiPoint(np.vstack((fp+a,fp+b))).convex_hull for a,b in zip(path[:-1],path[1:])])
def area_stats(shape):
    return {key:dict(area_m2=region.area,covered_m2=shape.intersection(region).area,
                    coverage_percent=100*shape.intersection(region).area/region.area,
                    uncovered_m2=region.difference(shape).area)
            for key,region in regions.items()}
cases=[];shapes={}
for model,agl,centered in [('calibration_K',2.6,False),('measured',2.6,False),('measured',2.6,True),('measured',2.,False),('measured',3.,False)]:
    fp,size=footprint(agl,model,centered)
    shape=sweep(fp,route)
    full=sweep(footprint(agl,model,centered,margin=.5)[0],route)
    eligible=regions['target_bounds'].buffer(-.5,join_style=2)
    name=f'{model}_{agl:g}'+('_centered' if centered else '')
    row=dict(name=name,fc_agl_m=agl,lens_agl_m=agl-offset,footprint_m=size,
             single_frame_area_m2=size[0]*size[1],unclipped_sweep_m2=shape.area,
             footprint_offsets_m=fp.tolist(),regions=area_stats(shape),
             full_1m_circle_eligible_center_area_m2=eligible.area,
             full_1m_circle_covered_center_area_m2=full.intersection(eligible).area,
             full_1m_circle_center_percent=100*full.intersection(eligible).area/eligible.area,
             prefixes=[dict(survey_waypoints=n,**area_stats(sweep(fp,route[:n+1]))['target_bounds']) for n in range(1,len(route))])
    cases.append(row);shapes[name]=shape
measure_fp,measure_size=footprint(2.)
_,cal_size=footprint(2.,'calibration_K')
bx0,bx1,by0,by1=field['coverage_bounds']
low_policy=SearchPolicy(bx0,bx1,by0,by1,field['lane_spacing'],field['low_agl'])
low_route=np.array([[p.x,p.y] for p in low_policy.waypoints])
low_shape=sweep(footprint(field['low_agl'])[0],low_route)
result=dict(source='formal field.example.yaml, cb19c20f; horizontal straight complete route, fixed yaw=0',
    assumption='flat ground; no occlusion, tracking error, tilt or distortion; measured span keeps calibrated principal-point proportions',
    no_ros_nodes=True,fc_to_lens_vertical_m=offset,route=route.tolist(),
    route_length_m=float(np.linalg.norm(np.diff(route,axis=0),axis=1).sum()),
    bounds={key:field[key] for key in regions},
    measured_at_2m=dict(footprint_m=measure_size,single_frame_area_m2=6.48,
        symmetric_fov_degrees=[2*math.degrees(math.atan(x/(2*(2-offset)))) for x in measure_size]),
    calibration_at_2m=dict(footprint_m=cal_size,measured_relative_span_change_percent=[100*(m/c-1) for m,c in zip(measure_size,cal_size)],
        fov_degrees=[math.degrees(math.atan(cx/fx)+math.atan((w-cx)/fx)),math.degrees(math.atan(cy/fy)+math.atan((h-cy)/fy))]),
    cases=cases,
    full_low_fallback=dict(waypoints=len(low_route),route_length_m=float(np.linalg.norm(np.diff(low_route,axis=0),axis=1).sum()),
        footprint_m=footprint(field['low_agl'])[1],regions=area_stats(low_shape),
        high_and_full_low_union=area_stats(shapes['measured_2.6'].union(low_shape)),
        note='Hypothetical full fallback only; excludes transition to first low point; runtime may stop or reorder earlier'))
(OUT/'coverage.json').write_text(json.dumps(result,indent=2)+'\n')
def draw_geometry(ax,g,**kwargs):
    if g.is_empty:return
    for p in ([g] if g.geom_type=='Polygon' else g.geoms):
        ax.add_patch(Patch(np.array(p.exterior.coords),**kwargs))
        for interior in p.interiors:
            ax.add_patch(Patch(np.array(interior.coords),facecolor='#ffe2d9',edgecolor='none'))
fig,axs=plt.subplots(1,3,figsize=(15,6),constrained_layout=True)
for ax,name in zip(axs,('calibration_K_2.6','measured_2.6','measured_2')):
    row=next(x for x in cases if x['name']==name)
    target=regions['target_bounds']
    draw_geometry(ax,target,facecolor='#ffe2d9',edgecolor='black',linewidth=1.)
    draw_geometry(ax,shapes[name].intersection(target),facecolor='#bde2c8',edgecolor='none')
    draw_geometry(ax,regions['coverage_bounds'],facecolor='none',edgecolor='#887700',linestyle=':')
    ax.plot(route[:,0],route[:,1],'o-',color='#245f98',markersize=3,linewidth=1.3)
    for i,p in enumerate(route):ax.annotate(str(i),p,xytext=(5,5),textcoords='offset points',fontsize=8)
    ax.annotate('nose +X',xy=(1.,0.),xytext=(-.1,0.),arrowprops={'arrowstyle':'->'},fontsize=8)
    stats=row['regions']['target_bounds']
    ax.set(title=f'{name}: {stats["covered_m2"]:.3f} / {target.area:.2f} m2\n{stats["coverage_percent"]:.2f}% ground points',xlabel='X forward (m)',ylabel='Y left (m)',aspect='equal',xlim=(-.7,7.6),ylim=(-5.,5.))
    ax.grid(alpha=.15)
fig.suptitle('Formal full high route: ideal flat-ground visibility, NOT detection probability\nGreen visible / peach missed; dotted box is low-search waypoint bounds, not its camera footprint')
fig.savefig(OUT/'coverage.png',dpi=150);plt.close(fig)
print(json.dumps(result,indent=2))

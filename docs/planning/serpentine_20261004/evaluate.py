"""Offline route design only. No ROS, Gazebo, hardware or production edits."""
from pathlib import Path
import json
import numpy as np
import yaml
from shapely.geometry import box, MultiPoint
from shapely.ops import unary_union
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[2]
cal=yaml.safe_load((ROOT/'vision_ws/src/camera_sdk/param/calibration_1280x720.yaml').read_text())
K=cal['camera_matrix']['data']; w=cal['image_width'];h=cal['image_height']
rig=yaml.safe_load((ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/config/competition/known_rig.yaml').read_text())
offset=-(rig['body_to_imu_xyz'][2]+rig['imu_to_camera_xyz'][2])
profiles=[
 dict(name='legacy_outer10_wall20',inner_bounds=[-.5,9.1,-4.8,4.8],outer_wall_thickness=.2,partition_thickness=.2),
 dict(name='inner10_wall20',inner_bounds=[-.5,9.5,-5,5],outer_wall_thickness=.2,partition_thickness=.2),
 dict(name='inner10_wall05',inner_bounds=[-.5,9.5,-5,5],outer_wall_thickness=.05,partition_thickness=.05),
 dict(name='outer10_wall05',inner_bounds=[-.65,9.25,-4.95,4.95],outer_wall_thickness=.05,partition_thickness=.05)]
for g in profiles:
 x0,x1,y0,y1=g['inner_bounds'];c=x1-1.5;p=c-g['partition_thickness']
 g.update(search_bounds=[x0,p,y0,y1],corridor_bounds=[c,x1,y0,y1],corridor_clear_width=1.5,
          outer_wall_height=4.,inner_wall_height=1.5,door_clear_width=.8,
          doorway_planes_y=[1.6,-1.6],entrance_y=[y1-1.5,y1],
          neutral_guide_x=(c+x1)/2,
          landing_h_xy=[(c+x1)/2,y0+.8],
          geometry_only=True)
 g['search_area']=(p-x0)*(y1-y0)
routes=[
 dict(name='old_rectangle_fc2p6',camera_agl=2.6-offset,xy=[[1,-3.5],[5.5,-3.5],[5.5,3.5],[1,3.5],[1,-3.5]]),
 dict(name='snake2_camera2p6',camera_agl=2.6,xy=[[1.8,-3.7],[1.8,3.9],[5.6,3.9],[5.6,-3.7]]),
 dict(name='snake3_camera2p0',camera_agl=2.,xy=[[1.,-3.95],[1.,4.1],[3.7,4.1],[3.7,-3.95],[6.4,-3.95],[6.4,4.1]])]
routes.append(dict(name='old_rectangle_camera2p6',camera_agl=2.6,xy=routes[0]['xy']))
def rect(v):a,b,c,d=v;return box(a,c,b,d)
def fp(lens,model,shrink=0):
 L,W=(3.6*lens/1.84,1.8*lens/1.84) if model=='measured' else (w*lens/K[0],h*lens/K[4])
 u,v=K[2]/w,K[5]/h
 a,b=-(1-u)*L+shrink,u*L-shrink;c,d=-v*W+shrink,(1-v)*W-shrink
 assert a<b and c<d
 return np.array([[a,c],[b,c],[b,d],[a,d]])
def sweep(f,path):
 return unary_union([MultiPoint(np.vstack((f+a,f+b))).convex_hull for a,b in zip(path[:-1],path[1:])])
def percent(s,region):return 100*s.intersection(region).area/region.area
results=[]
for route in routes:
 path=np.array(route['xy']);route.update(fc_agl=route['camera_agl']+offset,yaw_rad=0.,
    survey_length=float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()),
    staging_xy=[.6,.05],ingress_straight_length=float(np.linalg.norm(path[0]-[.6,.05])),
    flight_ready=False)
 for g in profiles:
  area=rect(g['search_bounds']);interior=area.buffer(-.25,join_style=2)
  for model in ('measured','calibration_K'):
   for margin in (0.,.15):
    footprint=fp(route['camera_agl'],model,margin);shape=sweep(footprint,path)
    full=sweep(fp(route['camera_agl'],model,margin+.5),path)
    fullcore=area.buffer(-.65,join_style=2)
    centers=area.buffer(-.5,join_style=2)
    redcore=area.buffer(-.175,join_style=2)
    missing_core=interior.difference(shape).area
    results.append(dict(route=route['name'],geometry=g['name'],model=model,footprint_erosion_m=margin,
        search_area=area.area,visible_area=shape.intersection(area).area,point_percent=percent(shape,area),
        interior_point_percent=percent(shape,interior),interior_missing_m2=missing_core,
        full_1m_circle_percent=percent(full,centers),full_1m_circle_with_15cm_wall_gap_percent=percent(full,fullcore),
        red_cross_center_percent=percent(shape,redcore),outside_search_m2=shape.difference(area).area))
# Independent sanity checks; these are geometric checks, not flight gates.
assert abs(profiles[1]['search_area']-83.)<1e-8
assert abs(profiles[2]['search_area']-84.5)<1e-8
for route in routes[1:3]:
 assert route['fc_agl']<3.0
 assert all(rect(profiles[2]['search_bounds']).buffer(-.275).covers(MultiPoint([p])) for p in route['xy'])
for row in results:
 assert 0<=row['point_percent']<=100.000001
 assert row['full_1m_circle_with_15cm_wall_gap_percent']<=100.000001
 for r in routes[1:3]:
  if row['route']==r['name'] and row['geometry']=='inner10_wall05':
   assert row['interior_point_percent']>99.99, row
bundle=dict(design_only=True,frame='takeoff +X forward +Y left',camera_offset_below_fc=offset,
    geometry_profiles=profiles,routes=routes,results=results,
    assumptions=['flat ground; fixed yaw=0; straight motion between points; no occlusion',
                 'measured span retains principal point fractions of existing calibration',
                 '15cm rectangular erosion is sensitivity analysis, not a proven error bound',
                 'full 1m circle / axis-aligned square, not arbitrary rotated paper corners',
                 'all lane calculations exclude ingress, low revisit, corridor and return'])
(OUT/'design.json').write_text(json.dumps(bundle,indent=2)+'\n')
(OUT/'route_candidates.yaml').write_text(yaml.safe_dump(dict(design_only=True,not_a_field_override=True,geometry_profiles=profiles,route_candidates=routes[1:3]),sort_keys=False))
fig,axs=plt.subplots(1,3,figsize=(15,6),constrained_layout=True)
g=profiles[2];area=rect(g['search_bounds'])
for ax,route in zip(axs,routes):
 path=np.array(route['xy']);shape=sweep(fp(route['camera_agl'],'calibration_K',.15),path).intersection(area)
 ax.add_patch(Rectangle((-.5,-5),8.45,10,facecolor='#f9d6cc',edgecolor='black'))
 for poly in ([shape] if shape.geom_type=='Polygon' else shape.geoms):
  ax.add_patch(Polygon(np.array(poly.exterior.coords),facecolor='#bcdcc1',edgecolor='none'))
  for hole in poly.interiors:ax.add_patch(Polygon(np.array(hole.coords),facecolor='#f9d6cc'))
 ax.add_patch(Rectangle((8.,-5),1.5,10,facecolor='#dce8f7',edgecolor='gray'))
 ax.add_patch(Rectangle((7.95,-5),.05,8.5,facecolor='#444444'))
 ax.plot(path[:,0],path[:,1],'o-',color='#135991')
 for i,p in enumerate(path):ax.annotate(str(i+1),p,xytext=(4,4),textcoords='offset points')
 ax.annotate('nose +X',xy=(2.,0),xytext=(.2,0),arrowprops={'arrowstyle':'->'})
 row=next(r for r in results if r['route']==route['name'] and r['geometry']==g['name'] and r['model']=='calibration_K' and r['footprint_erosion_m']==.15)
 ax.set(title=f"{route['name']}\n{route['survey_length']:.2f}m; point coverage {row['point_percent']:.2f}%",
        aspect='equal',xlim=(-1,10),ylim=(-5.4,5.4),xlabel='X forward (m)',ylabel='Y left (m)')
 ax.grid(alpha=.2)
fig.suptitle('10 x 10 m CLEAR interior, 5 cm partition, 1.5 m CLEAR corridor\nNarrower K footprint, eroded 15 cm; no trees/occlusion included')
fig.savefig(OUT/'route_comparison.png',dpi=145);plt.close(fig)
for route in routes:
 print(route['name'],route['survey_length'],route['fc_agl'])
 for r in results:
  if r['route']==route['name'] and r['geometry']=='inner10_wall05':
   print(r['model'],r['footprint_erosion_m'],*[round(r[k],3) for k in ('point_percent','interior_point_percent','full_1m_circle_with_15cm_wall_gap_percent','red_cross_center_percent')])

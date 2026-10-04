"""Read existing flight measurements and scene fixtures; no ROS or flight."""
from pathlib import Path
import csv, json, math, xml.etree.ElementTree as ET
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

ROOT=Path(__file__).resolve().parents[3]
B=ROOT.parent/'r2026-board-vision-tests'
OUT=Path(__file__).resolve().parent
fov=json.loads((B/'docs/deployment/flight_fov_20260926/result.json').read_text())
K=fov['calibration']['K'];width=fov['calibration']['w'];height=fov['calibration']['h']
table=[]
for agl in (.6,.9,1.,1.4,1.8,2.,2.6,3.,4.):
    lens=agl-.16
    size=[3.6*lens/1.84,1.8*lens/1.84]
    replay=np.array([[row['footprint_from_square']['mean_width'],row['footprint_from_square']['mean_height'],row['footprint_from_square']['area']] for row in fov['rows']])
    for i,row in enumerate(fov['rows']):
        ratio=lens/row['camera_agl'];replay[i,:2]*=ratio;replay[i,2]*=ratio**2
    table.append(dict(fc_agl=agl,lens_agl=lens,measured_span=size,measured_area=size[0]*size[1],
        calibration_span=[width*lens/K[0],height*lens/K[4]],
        recorded_five_frame_extrapolation_min=replay.min(axis=0).tolist(),
        recorded_five_frame_extrapolation_max=replay.max(axis=0).tolist()))

runs=[]
for run in ('173929','175604','181136'):
    directory=B/f'logs/site_download_20261003_{run}'
    ref=json.loads((directory/'ground_reference.json').read_text())
    poses=list(csv.DictReader((directory/'navigation_pose.csv').open()))
    ground=ref['ground_z'];high=ref['high_z']
    high_poses=[p for p in poses if abs(float(p['z'])-high)<.3]
    yaw=[]
    for p in high_poses:
        x,y,z,w=(float(p[k]) for k in ('qx','qy','qz','qw'))
        yaw.append(math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))))
    runs.append(dict(run=run,ground_z=ground,high_local_z=high,
        configured_high_agl=ref['settings']['high_agl'],
        high_band_yaw_deg_percentiles=np.percentile(yaw,[0,5,50,95,100]).tolist(),
        note='yaw samples within +/-0.3m of high local Z; may include ascent/descent; not isolated cruise intervals'))

scene=ROOT/'docs/verification/integrated_two_20261004/seed_38'
world=ET.parse(scene/'field.world').getroot().find('world')
walls=[]
for link in world.findall('model/link'):
    if not link.get('name','').startswith('Wall_'):continue
    p=list(map(float,link.findtext('pose').split()));s=list(map(float,link.findtext('collision/geometry/box/size').split()))
    walls.append(dict(name=link.get('name'),pose=p,size=s))
field=yaml.safe_load((ROOT/'deployment/competition/field.example.yaml').read_text())
rt=yaml.safe_load((scene/'fast_runtime.yaml').read_text())
gate=yaml.safe_load((scene/'fast_gate.yaml').read_text())
result=dict(measurements_source='flight_fov_20260926/result.json: five manually marked 1m boards; not independent altitude truth',
    height_table=table,flight_height_and_yaw=runs,walls=walls,
    outer_field_area=100.,inner_field_area=92.16,search_area=75.84,corridor_area=14.4,partition_strip_area=1.92,
    formal_flight_bounds=field['flight_bounds'],formal_high_route=field['survey_xy'],
    validated_post_route=rt['mission']['post_delivery_route'],gate=gate)
(OUT/'field_followup.json').write_text(json.dumps(result,indent=2)+'\n')
fig,ax=plt.subplots(figsize=(8,7),constrained_layout=True)
for xy,w,h,color,label in [((-.7,-5),10,10,'#dddddd','Outer footprint: 100 m2'),((-.5,-4.8),9.6,9.6,'white','Inner area: 92.16 m2'),((-.5,-4.8),7.9,9.6,'#d5edd5','Search: 75.84 m2'),((7.6,-4.8),1.5,9.6,'#d1e6fa','Corridor: 14.40 m2')]:
    ax.add_patch(Rectangle(xy,w,h,color=color,label=label))
for wall in walls:
    p,s=wall['pose'],wall['size'];ax.add_patch(Rectangle((p[0]-s[0]/2,p[1]-s[1]/2),s[0],s[1],color='#555555'))
route=np.array(rt['mission']['post_delivery_route']);ax.plot(route[:,0],route[:,1],'o--',color='#d07a16',label='Neutral before/after-wall goals')
survey=np.array(field['survey_xy']);ax.plot(survey[:,0],survey[:,1],'o-',color='#228539',label='Formal high survey goals')
ax.scatter([0,8.5],[0,-4.2],marker='s',color='red');ax.text(.15,.15,'Start');ax.text(8.6,-4.1,'H')
ax.axvline(9.5,color='red',linestyle=':',label='Formal example max X=9.5')
ax.set(xlabel='X forward (m)',ylabel='Y left (m)',title='Current seed38 geometry and formal-template goals\nKnown wall planes; 0.8m gaps are NOT task goals',aspect='equal',xlim=(-1,10),ylim=(-5.3,5.3))
ax.legend(fontsize=8,loc='upper left');fig.savefig(OUT/'field_layout.png',dpi=140);plt.close(fig)
print(json.dumps(dict(height_table=table,flight_height_and_yaw=runs),indent=2))

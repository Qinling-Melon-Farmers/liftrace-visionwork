"""Geometry and existing-log analysis only: no ROS master, renderer or flight."""
from pathlib import Path
import importlib.util,json,sys,math
import numpy as np,cv2,yaml
from scipy.spatial import ConvexHull
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.path import Path as PolyPath
from matplotlib.patches import Rectangle

D=Path(__file__).resolve().parent;R=D.parents[2]
sys.path.insert(0,str(R/'simulation_tools'))
from r2026_scene import scene_layout
spec=importlib.util.spec_from_file_location('exporter',R/'simulation_tools/tools/export_r2026_scene.py')
exporter=importlib.util.module_from_spec(spec);spec.loader.exec_module(exporter)

def main():
    mission=R/'patrol_uav_ws-patrol_planner/src/uav_mission/config'
    layout=scene_layout(2672,door_seed=3672,obstacle_seed=4672,door_mode='continuous',outer_wall_height=4.)
    exporter.export_scene(R/'vision_ws/src/uav_vision_eval/models/r2026_horizontal_field/field.world',
        mission/'coverage_r2026_horizontal.yaml',mission/'vcl06_full_low_corridor_runtime.yaml',D/'example_scene',layout)
    # A separate, not-yet-flown height candidate. Frozen prior input is not edited.
    rt=yaml.safe_load((D/'example_scene/experimental_runtime.yaml').read_text())
    for point in rt['mission']['post_delivery_route'][1:8]:point[2]=.9-.22
    for stage in rt['mission']['post_delivery_parameter_stages']:
        if stage['after_completed_waypoints']==2:
            stage['parameters']['/external_planner_max_command_z']=1.0-.22
            stage['parameters']['/fast_planner_node/sdf_map/virtual_ceil_height']=1.1-.22
    rt['mission']['post_delivery_route_revision']='continuous-080-height-candidate-STATIC-ONLY'
    rt['mission']['delivery_reserve_per_slot']=70.
    (D/'example_scene/corridor_090_candidate.yaml').write_text(yaml.safe_dump(rt,sort_keys=False))
    gate=yaml.safe_load((D/'example_scene/gate_geometry.yaml').read_text())
    gate['post_delivery_gate']['low_height_region']['max_height']=1.2
    for door in gate['post_delivery_gate']['doors']:door['z_max']=1.2
    (D/'example_scene/gate_120_candidate.yaml').write_text(yaml.safe_dump(gate,sort_keys=False))
    source=R/'logs/reliability_seed2672_20260919_175901/actual_camera_info.json'
    cam=json.loads(source.read_text());width,height=cam['width'],cam['height'];K=np.array(cam['K']).reshape(3,3)
    u=np.linspace(0,width-1,100);v=np.linspace(0,height-1,100)
    pixels=np.vstack((np.c_[u,np.zeros_like(u)],np.c_[np.full_like(v,width-1),v],np.c_[u[::-1],np.full_like(u,height-1)],np.c_[np.zeros_like(v),v[::-1]]))
    rays=cv2.undistortPoints(pixels.reshape(-1,1,2),K,np.array(cam['D'])).reshape(-1,2)
    route=np.array([[0,0],[-3.5,1],[3.5,1],[3.5,5.5],[-3.5,5.5],[-3.5,1]])
    step=.025;x=np.arange(-4.8+step/2,4.8,step);y=np.arange(-.5+step/2,7.6,step);xx,yy=np.meshgrid(x,y)
    points=np.c_[xx.ravel(),yy.ravel()];results=[];fig,axes=plt.subplots(1,2,figsize=(13,7))
    for agl,ax in zip((2.6,3.),axes):
        foot=-(agl-.16)*rays[:,::-1];foot=foot[ConvexHull(foot).vertices]
        covered=np.zeros(len(points),bool)
        for a,b in zip(route,route[1:]):
            poly=np.vstack((foot+a,foot+b));poly=poly[ConvexHull(poly).vertices]
            covered|=PolyPath(poly).contains_points(points)
        fraction=float(covered.mean());extent=np.ptp(foot,axis=0)
        results.append(dict(fc_agl_m=agl,camera_agl_m=agl-.16,footprint_extent_xy_m=extent.tolist(),
                            footprint_area_m2=float(ConvexHull(foot).volume),ideal_route_coverage=fraction,
                            covered_ground_m2=float(covered.sum()*step**2),red_cross_center_pixels=float(K[0,0]*.35/(agl-.16))))
        ax.imshow(covered.reshape(xx.shape),origin='lower',extent=(-4.8,4.8,-.5,7.6),cmap=matplotlib.colors.ListedColormap(['#ffd6cf','#d5eaf2']))
        ax.plot(route[:,0],route[:,1],'o--',color='#225c92',lw=1)
        fp=foot+route[1];ax.plot(*np.vstack((fp,fp[0])).T,color='#008b65',label='One camera footprint')
        ax.set(xlabel='X (m)',ylabel='Y (m)',title=f'FC {agl:.1f}m / camera {agl-.16:.2f}m\nIdeal ground coverage {100*fraction:.2f}%')
        ax.set_aspect('equal');ax.grid(alpha=.25);ax.legend(fontsize=8)
    fig.suptitle('Identical nominal route, zero tilt, calibrated distortion | no occlusion/detection/flight model')
    fig.tight_layout();fig.savefig(D/'height_fov_comparison.png',dpi=160);plt.close(fig)
    ratio=2.84/2.44
    vertical15=.18*math.cos(math.radians(15))+.275*math.sqrt(2)*math.sin(math.radians(15))
    both15=.275*(math.sin(math.radians(15))+math.cos(math.radians(15))*math.sin(math.radians(15)))+.18*math.cos(math.radians(15))**2
    data=dict(scope='OFFLINE_GEOMETRY_ONLY_NOT_NEW_SIMULATION',camera_source=str(source),heights=results,
              linear_footprint_ratio=ratio,area_ratio=ratio**2,pixel_scale_ratio=1/ratio,
              max_body_above_fc_total_tilt_15_m=vertical15,max_body_above_fc_roll_pitch_each_15_m=both15,
              source_scene=layout,mechanism_reserve_per_slot_s=10.,map_z_buffer_scale_3_to_36=1.2)
    (D/'analysis.json').write_text(json.dumps(data,indent=2))
    fig,axes=plt.subplots(1,2,figsize=(13,5))
    ax=axes[0]
    for i,center in enumerate([8.,8.12,8.35,8.58,8.7]):
        for lo,hi in ((7.6,center-.4),(center+.4,9.1)):
            if hi>lo:ax.add_patch(Rectangle((lo,i-.22),hi-lo,.44,color='#667680'))
        ax.plot([center-.4,center+.4],[i,i],color='#12a179',lw=6)
    ax.set(yticks=range(5),yticklabels=['edge right','offset 1','center','offset 2','edge left'],xlim=(7.5,9.2),ylim=(-.6,4.6),xlabel='Corridor Y (m)',title='Same fixed wall X, continuous 0.80m opening')
    ax.axvline(8.35,color='black',ls=':',label='Neutral waypoint Y (not door truth)');ax.legend(fontsize=8)
    ax=axes[1];ax.axhline(1.5,color='#697a85',lw=3,label='Internal wall top 1.50m')
    ax.axhline(1.2,color='darkorange',ls='--',label='Proposed FC observed ceiling 1.20m')
    ax.axhline(.9,color='#225c92',ls='--',label='Nominal FC cruise 0.90m')
    ax.add_patch(Rectangle((.2,1.2-.3),.55,.3+vertical15,color='#83bfd8',alpha=.7))
    ax.text(.8,1.2+vertical15,f'Upper guard <= {1.2+vertical15:.3f}m\n(total tilt <=15 deg)',fontsize=9)
    ax.set(xlim=(0,2.2),ylim=(0,1.7),xticks=[],ylabel='AGL (m)',title='Already inflated 55cm envelope, no duplicate padding');ax.legend(fontsize=8,loc='lower right');ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(D/'door_height_design.png',dpi=160);plt.close(fig)
    print(json.dumps({k:v for k,v in data.items() if k not in ('source_scene','camera_source')},indent=2))

if __name__=='__main__':main()

from pathlib import Path
import sys,json,math,collections,xml.etree.ElementTree as ET,shutil,datetime
import numpy as np,yaml
from scipy.spatial.transform import Rotation
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle
from matplotlib.transforms import Affine2D
R=next(p for p in Path(__file__).resolve().parents if (p/'AGENTS.md').exists() and (p/'patrol_uav_ws-patrol_planner').is_dir())
O=R/'logs/_artifacts/full_random_five_20260910';D=R/'docs/verification/full_random_five_20260910'
sys.path[:0]=[str(R/'top_level_scripts'),str(R/'simulation_tools'),str(R/'patrol_uav_ws-patrol_planner/src/uav_mission/src')]
from collect_run_summary import collect
from r2026_scene import rectangle_vertices,polygons_overlap
from uav_mission.search_policy import SearchPolicy
COLORS={'tent':'#ad7939','pillbox':'#2579aa','bridge':'#9161ac','panzer':'#50793e','red_cross':'#d94c44'}
plt.rcParams.update({'font.size':9,'axes.titlesize':11,'axes.spines.top':False,'axes.spines.right':False,'savefig.facecolor':'white'})
def read_json(p,default=None):return json.loads(p.read_text()) if p.exists() else ({} if default is None else default)
def csv(p):
    if not p.exists() or p.stat().st_size<40:return np.empty((0,8))
    a=np.loadtxt(p,delimiter=',',skiprows=1,ndmin=2)
    return a if a.size else np.empty((0,8))
def val(e,p,d='0 0 0 0 0 0'):return [float(x) for x in e.findtext(p,d).split()]
def walls(run):
    root=ET.parse(run/'scenario_inputs/field.world').getroot()
    field=next(x for x in root.iter('model') if x.get('name')=='toudi2')
    result=[]
    for link in field.findall('link'):
        if link.get('name','').startswith('Wall'):
            p=val(link,'pose');s=val(link,'collision/geometry/box/size','0 0 0')
            result.append({'name':link.get('name'),'pose':p,'size':s,'polygon':rectangle_vertices(p[0],p[1],s[0],s[1],p[5])})
    return result
def scene(ax,run,labels=False):
    for w in walls(run):
        p=w['pose'];s=w['size'];patch=Rectangle((p[0]-s[0]/2,p[1]-s[1]/2),s[0],s[1],fc='#858b92',ec='#535961',lw=.6)
        patch.set_transform(Affine2D().rotate_around(p[0],p[1],p[5])+ax.transData);ax.add_patch(patch)
    layout=read_json(run/'scenario_inputs/scene.json')
    for t in layout['trees']:
        ax.add_patch(Circle((t['x'],t['y']),.43,fc='#5c9466',alpha=.35))
        patch=Rectangle((t['x']-.3,t['y']-.3),.6,.6,fc='#a58b61',ec='#816a45',alpha=.55)
        patch.set_transform(Affine2D().rotate_around(t['x'],t['y'],t['yaw'])+ax.transData);ax.add_patch(patch)
    for x,y in [(0,0),(4.2,8.5)]:
        ax.add_patch(Circle((x,y),.5,fill=False,color='#212936',lw=1.2));ax.text(x,y,'H',ha='center',va='center',clip_on=True)
    truth=yaml.safe_load((run/'random_field_truth.yaml').read_text())
    for t in truth['targets']:
        x,y=t['world_x'],t['world_y'];s=.35 if t['class']=='red_cross' else 1.;c=COLORS[t['class']]
        patch=Rectangle((x-s/2,y-s/2),s,s,fc=c,ec=c,alpha=.3)
        patch.set_transform(Affine2D().rotate_around(x,y,t['yaw'])+ax.transData);ax.add_patch(patch)
        ax.scatter(x,y,s=12,color=c)
        if labels:ax.annotate(t['class'],(x,y),xytext=(3,5),textcoords='offset points',fontsize=7,color=c)
    ax.set(xlim=(-5.05,5.05),ylim=(-.8,9.5),xlabel='X (m)',ylabel='Y (m)');ax.set_aspect('equal');ax.grid(alpha=.14)
    return truth,layout
def match(a,t):return a[np.argmin(abs(a[:,0]-t))] if len(a) else None
def percentile(v):
    return {'n':len(v),'min':float(np.min(v)),'p50':float(np.median(v)),'p95':float(np.percentile(v,95)),'max':float(np.max(v))} if len(v) else {'n':0}
def events(run):
    result=[]
    for line in (run/'key_events.jsonl').read_text().splitlines():
        try:result.append(json.loads(line))
        except ValueError:pass
    return result
def analyze_one(result):
    seed=result['seed'];run=Path(result['run_dir']);out=D/('seed_%02d'%seed);out.mkdir(exist_ok=True)
    s=collect(run);g=read_json(run/'gate_status.json');m=g['metrics'];ev=events(run)
    truth=csv(run/'truth_pose.csv');mav=csv(run/'mavros_pose.csv');cmd=csv(run/'mavros_setpoint.csv');planner=csv(run/'planner_setpoint.csv')
    rt=yaml.safe_load((run/'scenario_inputs/runtime.yaml').read_text());sc=rt['search'];nom=np.array([x.as_tuple() for x in SearchPolicy(**{k:sc[k] for k in ['min_x','max_x','min_y','max_y','lane_spacing','altitude']}).waypoints]);post=np.array(rt['mission']['post_delivery_route'])
    decisions=[x for x in ev if x['kind']=='decision'];release=[x for x in ev if x['kind']=='release' and x['data'].get('success')];recover=[x for x in ev if x['kind']=='result' and x['data'].get('reason')=='release_recovery_confirmed']
    armed=next((x['ros_sec'] for x in ev if x['kind']=='state' and x['data'].get('armed')),None)
    disarm=next((x['ros_sec'] for x in ev if x['kind']=='state' and x['data'].get('armed') is False and armed is not None and x['ros_sec']>armed),None)
    auto=next((x['ros_sec'] for x in ev if x['kind']=='state' and x['data'].get('mode')=='AUTO.LAND'),None)
    route_start=next((x['ros_sec'] for x in decisions if x['data'].get('reason','').startswith('post_delivery_route:1/')),None)
    land_start=next((x['ros_sec'] for x in decisions if x['data'].get('command')==5),None)
    last=truth[-1];end=float(last[0]);start=min([x['issued_ns']/1e9 for x in g.get('decision_fences',[])] or [end])
    contact=read_json(run/'gazebo_contact_status.json');audit=read_json(run/'visual_delivery_audit.json')
    phases=[]
    for x in ev:
        if x['kind']=='mission' and (not phases or phases[-1]['phase']!=x['data'].get('phase')):phases.append({'t':x['ros_sec'],'phase':x['data'].get('phase','UNKNOWN'),'reason':x['data'].get('last_reason','')})
    terminal=[x for x in ev if x['kind']=='result' and x['data'].get('terminal')]
    timeline=[{'t':x['ros_sec'],'kind':x['kind'],**{k:x['data'].get(k) for k in ['command','decision_seq','status','stage','reason','target_class','payload_slot']},'goal':x['data'].get('goal',{}).get('pose',{}).get('position')} for x in ev if x['kind']=='decision' or x in terminal]
    layout=yaml.safe_load((run/'random_field_truth.yaml').read_text());scene_data=read_json(run/'scenario_inputs/scene.json');ww=walls(run)
    legality=[];polys=[]
    for t in layout['targets']:
        size=.35 if t['class']=='red_cross' else 1.;p=rectangle_vertices(t['world_x'],t['world_y'],size,size,t['yaw']);polys.append((t['class'],p))
        overlaps=[w['name'] for w in ww if polygons_overlap(p,w['polygon'])]
        tree_overlap=[i for i,v in enumerate(scene_data['trees']) if polygons_overlap(p,rectangle_vertices(v['x'],v['y'],.6,.6,v['yaw']))]
        legality.append({'class':t['class'],'wall_overlaps':overlaps,'tree_box_overlaps':tree_overlap,'corners':p})
    pair_overlaps=[(a[0],b[0]) for i,a in enumerate(polys) for b in polys[i+1:] if polygons_overlap(a[1],b[1])]
    release_geometry=[]
    for e in release:
        pp=match(truth,e['ros_sec']);target=next((t for t in layout['targets'] if t['class']==e['data'].get('target_class')),None)
        if target is not None:
            release_geometry.append({'ros_sec':e['ros_sec'],'class':target['class'],'slot':e['data'].get('payload_slot'),'actual_fc_xyz_local':pp[1:4].tolist(),'actual_fc_agl_m':float(pp[3]+.22),'body_center_to_target_m':float(np.linalg.norm(pp[1:3]-[target['world_x'],target['world_y']])),'scope':'Body XY at mock ACK, not a physical parcel impact point'})
    aligned=np.column_stack([np.interp(truth[:,0],mav[:,0],mav[:,i]) for i in range(1,4)]) if len(mav)>1 else None
    pos_error=truth[:,1:4]-aligned if aligned is not None else np.empty((0,3))
    corners=np.array([[x,y,z] for x in [-.275,.275] for y in [-.275,.275] for z in [-.22,.18]])
    xy=Rotation.from_quat(last[4:8]).apply(corners)[:,:2]+last[1:3]
    fin={name:{'center_error_m':float(np.linalg.norm(last[1:3]-np.array(center))),'envelope_radius_m':float(np.max(np.linalg.norm(xy-np.array(center),axis=1)))} for name,center in [('start_H',[0,0]),('finish_H',[4.2,8.5])]}
    phase_durations=collections.defaultdict(float)
    for i,p in enumerate(phases):phase_durations[p['phase']]+=max(0,(phases[i+1]['t'] if i+1<len(phases) else end)-p['t'])
    door_names=[x['name'] for x in m.get('door_crossings',[]) if x['name']!='corridor_entry']
    item={'seed':seed,'door_pattern':scene_data['door_pattern'],'status':g['status'],'gate_reason':g['reason'],'gate_errors':g.get('errors',[]),'checks_passed':sum(g['checks'].values()),'checks_total':len(g['checks']),'failed_checks':[k for k,v in g['checks'].items() if not v],'run_dir':str(run),'source':s['source'],'release_count':m['release_commit_count'],'recovery_count':m['recovery_success_count'],'route_points':m['post_delivery_return_success_count'] if m.get('post_delivery_decision_indices') else 0,'doors_crossed':door_names,'door_crossings':m.get('door_crossings',[]),'mission_start_ros':start,'record_end_ros':end,'observed_mission_span_s':max(0,end-start),'complete_mission_s':m.get('mission_ros_sec') if g['status']=='PASS' else None,'release_events':release,'recovery_events':recover,'collision_count':contact.get('actual_collision_count'),'contact_events':contact.get('events',[]),'route_start_ros':route_start,'land_start_ros':land_start,'auto_land_ros':auto,'disarm_ros':disarm,'route_to_disarm_s':disarm-route_start if disarm is not None and route_start is not None else None,'land_to_disarm_s':disarm-land_start if disarm is not None and land_start is not None else None,'final_xyz_local':last[1:4].tolist(),'final_h_geometry':fin,'landed_on_ground':m.get('final_landed_state')==1 or bool(g['checks'].get('final_landed_on_ground')),'disarmed':bool(g['checks'].get('final_vehicle_disarmed')),'max_fc_agl_m':float(truth[:,3].max()+.22),'xy_path_length_m':float(np.linalg.norm(np.diff(truth[:,1:3],axis=0),axis=1).sum()),'truth_mavros_xy_error_m':percentile(np.linalg.norm(pos_error[:,:2],axis=1)),'truth_mavros_abs_z_error_m':percentile(abs(pos_error[:,2])),'planner_ready_latency_ros_s':s['planner_ready_latency_ros_s'],'planner_reasons':dict(collections.Counter(x['data'].get('reason','') for x in ev if x['kind']=='planner')),'phase_durations_s':dict(phase_durations),'phases':phases,'last_terminal_events':timeline[-14:],'scene_legality':{'targets':legality,'target_pair_overlaps':pair_overlaps,'valid':not pair_overlaps and not any(x['wall_overlaps'] or x['tree_box_overlaps'] for x in legality)},'audit_status':audit.get('status'),'audit_reason':audit.get('failure_reason'),'raw_calls':audit.get('raw_calls'),'cleanup_pass':s['cleanup_pass'],'recording_files':[str(x.relative_to(run)) for x in run.rglob('*') if x.suffix in ('.mp4','.bag','.active')],'csv_rows':{'truth':len(truth),'mavros':len(mav),'command':len(cmd),'planner':len(planner)}}
    item['release_geometry']=release_geometry
    route_ids={x['data']['decision_seq'] for x in decisions if x['data'].get('reason','').startswith('post_delivery_route:')}
    item['route_points']=len({x['data']['decision_seq'] for x in terminal if x['data'].get('status')==3 and x['data'].get('decision_seq') in route_ids})
    manifest=yaml.safe_load((run/'manifest.yaml').read_text())
    a=manifest.get('start_time');b=manifest.get('end_time')
    if a and b:
        def dt(v):return datetime.datetime.fromisoformat(v) if isinstance(v,str) else v
        item['wall_duration_s']=(dt(b)-dt(a)).total_seconds()
        item['mean_observed_rtf']=end/item['wall_duration_s'] if item['wall_duration_s'] else None
    item['physics_safety']={k:m.get(k) for k in ['boundary_violations','height_violations','max_observed_height','latest_landed_state','latest_armed']}
    fig,ax=plt.subplots(2,2,figsize=(14,11));scene(ax[0,0],run,True);ax[0,0].plot(nom[:,0],nom[:,1],'--',c='#828999',lw=.8,label='Nominal search route');ax[0,0].plot(post[:,0],post[:,1],'o--',c='#926732',lw=1,ms=3,label='Fixed neutral post route');ax[0,0].set_title('Recorded layout and nominal routes');None
    scene(ax[0,1],run);ax[0,1].plot(truth[:,1],truth[:,2],c='#136daf',lw=1.2,label='Actual aircraft')
    if len(planner):ax[0,1].plot(planner[:,1],planner[:,2],c='#8466aa',alpha=.45,lw=.6,label='Planner setpoint')
    if len(cmd):ax[0,1].plot(cmd[:,1],cmd[:,2],c='#ec8a3e',alpha=.65,lw=.65,label='MAVROS command')
    for i,e in enumerate(release):
        pp=match(truth,e['ros_sec']);ax[0,1].scatter(pp[1],pp[2],marker='*',s=100,color=COLORS.get(e['data'].get('target_class'),'#1a9563'),zorder=6);ax[0,1].annotate('D%d'%(i+1),pp[1:3],xytext=(4,-10),textcoords='offset points',fontsize=8)
    ax[0,1].scatter(last[1],last[2],c='#bf3547',s=35,marker='X',zorder=6,label='Final sample');None;ax[0,1].set_title('Actual path, commands and releases')
    ax[1,0].plot(truth[:,0],truth[:,3]+.22,c='#136daf',lw=1.1,label='Actual FC AGL')
    if len(mav):ax[1,0].plot(mav[:,0],mav[:,3]+.22,c='#78965f',lw=.65,alpha=.7,label='Estimated z + 0.22')
    if len(cmd):ax[1,0].plot(cmd[:,0],cmd[:,3]+.22,c='#ec8a3e',lw=.8,alpha=.8,label='Command z + 0.22')
    for i,e in enumerate(release):ax[1,0].axvline(e['ros_sec'],c='#8a3f6f',ls=':',lw=.8);ax[1,0].annotate('D%d'%(i+1),(e['ros_sec'],.97),xycoords=('data','axes fraction'),ha='center',fontsize=8)
    for t,label in [(route_start,'Route'),(auto,'AUTO.LAND')]:
        if t is not None:ax[1,0].axvline(t,c='#555b6b',ls='--',lw=.7);ax[1,0].annotate(label,(t,.86),xycoords=('data','axes fraction'),rotation=90,ha='right',va='top',fontsize=7)
    ax[1,0].set(xlabel='ROS simulation time (s)',ylabel='Height (m)',title='Height, estimation and commands');ax[1,0].grid(alpha=.2);ax[1,0].legend(fontsize=7,loc='lower left')
    names=list(dict.fromkeys(p['phase'] for p in phases));palette=plt.get_cmap('tab10')
    for i,p in enumerate(phases):
        stop=phases[i+1]['t'] if i+1<len(phases) else end;n=names.index(p['phase']);ax[1,1].broken_barh([(p['t'],max(0,stop-p['t']))],(n-.32,.64),facecolors=palette(n%10))
    for e in release:ax[1,1].axvline(e['ros_sec'],c='#8a3f6f',ls=':',lw=.6)
    for e in recover:ax[1,1].axvline(e['ros_sec'],c='#24784e',ls='--',lw=.6)
    ax[1,1].set(yticks=range(len(names)),yticklabels=names,xlabel='ROS simulation time (s)',title='Mission phases (dotted: release; dashed: recovery)');ax[1,1].grid(axis='x',alpha=.2)
    fig.suptitle('Seed %d | doors %s | %s | drops %d/3 | route %d/9 | doors %d/2'%(seed,item['door_pattern'],g['status'],item['release_count'],item['route_points'],len(door_names)),fontsize=14);handles0,labels0=ax[0,0].get_legend_handles_labels();handles1,labels1=ax[0,1].get_legend_handles_labels();fig.legend(handles0+handles1,labels0+labels1,loc='upper center',bbox_to_anchor=(.5,.955),ncol=3,fontsize=8,frameon=False);fig.tight_layout(rect=(0,0,1,.90));fig.savefig(D/('seed_%02d.png'%seed),dpi=150);plt.close(fig)
    # Local diagnostic panel is recorded after every run, including a successful landing.
    low=max(truth[0,0],end-25);mask=truth[:,0]>=low;tt=truth[mask]
    fig,axs=plt.subplots(2,2,figsize=(13,9));scene(axs[0,0],run);axs[0,0].plot(tt[:,1],tt[:,2],c='#136daf',label='Actual')
    if len(cmd):cc=cmd[cmd[:,0]>=low];axs[0,0].plot(cc[:,1],cc[:,2],c='#ec8a3e',lw=.8,label='Command')
    axs[0,0].set(xlim=(max(-5.05,tt[:,1].min()-.5),min(5.05,tt[:,1].max()+.5)),ylim=(max(-.8,tt[:,2].min()-.5),min(9.5,tt[:,2].max()+.5)),title='Last 25 s: local XY');axs[0,0].legend(fontsize=8)
    for i,label in enumerate(['X','Y']):axs[0,1].plot(tt[:,0],tt[:,i+1],label='Actual '+label)
    if len(mav):
        mm=mav[mav[:,0]>=low]
        for i,label in enumerate(['X','Y']):axs[0,1].plot(mm[:,0],mm[:,i+1],'--',lw=.8,label='Estimated '+label)
    axs[0,1].set(title='Local position vs estimate',xlabel='ROS time (s)',ylabel='Position (m)');axs[0,1].legend(fontsize=8)
    axs[1,0].plot(tt[:,0],tt[:,3]+.22,label='Actual FC AGL')
    if len(mav):axs[1,0].plot(mm[:,0],mm[:,3]+.22,'--',label='Estimated z + 0.22')
    if len(cmd):axs[1,0].plot(cc[:,0],cc[:,3]+.22,lw=.8,label='Command z + 0.22')
    axs[1,0].set(title='Final height',xlabel='ROS time (s)',ylabel='Height (m)');axs[1,0].legend(fontsize=8)
    angles=Rotation.from_quat(tt[:,4:8]).as_euler('xyz',degrees=True)
    for i,label in enumerate(['Roll','Pitch','Yaw']):axs[1,1].plot(tt[:,0],angles[:,i],label=label,lw=.9)
    axs[1,1].set(title='Actual attitude',xlabel='ROS time (s)',ylabel='Angle (deg)');axs[1,1].legend(fontsize=8)
    for a in axs.flat:a.grid(alpha=.15)
    fig.suptitle('Seed %d: terminal diagnostics | %s'%(seed,g['reason']),fontsize=12);fig.tight_layout(rect=(0,0,1,.96));fig.savefig(out/'terminal_diagnostics.png',dpi=150);plt.close(fig)
    (out/'timeline.json').write_text(json.dumps(timeline,ensure_ascii=False,indent=2));(out/'metrics.json').write_text(json.dumps(item,ensure_ascii=False,indent=2))
    for name in ['gate_status.json','gazebo_contact_status.json','manifest.yaml','random_field_status.json','random_field_truth.yaml','ros_system_state.json']:
        if (run/name).exists():shutil.copy2(run/name,out/name)
    shutil.copytree(run/'scenario_inputs',out/'scenario_inputs',dirs_exist_ok=True)
    return item
def main():
    state=read_json(O/'matrix/matrix_status.json');assert state['status']=='COMPLETE' and len(state['results'])==5
    D.mkdir(exist_ok=True);metrics=[analyze_one(x) for x in state['results']]
    (D/'flight_metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2));shutil.copy2(O/'matrix/matrix_status.json',D/'matrix_status.json')
    fig,axs=plt.subplots(2,3,figsize=(15,10))
    for row,ax in zip(metrics,axs.flat):
        run=Path(row['run_dir']);scene(ax,run);a=csv(run/'truth_pose.csv');ax.plot(a[:,1],a[:,2],color='#136daf',lw=.8);ax.scatter(a[-1,1],a[-1,2],color='#bf3547',marker='X',s=24)
        ax.set_title('Seed %d / %s / %s / %d drops'%(row['seed'],row['door_pattern'],row['status'],row['release_count']))
    axs.flat[-1].axis('off');axs.flat[-1].text(.06,.9,'Full-random five-seed matrix\n\nBlue line: actual aircraft path\nBrown / green: rotated box / tree\nColored squares: recorded target boards\nGray: physical walls\nX: last position\n\nDoors: LL, RR, LR, LL, RR\nRL was not sampled.\nNo videos or full-flight bags.\nTruth is evaluation-only.',va='top',fontsize=12)
    fig.tight_layout();fig.savefig(D/'all_seed_layouts_paths.png',dpi=160);plt.close(fig)
    fig,axs=plt.subplots(2,2,figsize=(13,9));labels=[str(x['seed'])+' '+x['door_pattern'] for x in metrics];xx=np.arange(5)
    for j,(key,label,total) in enumerate([('release_count','Drops',3),('recovery_count','Recoveries',3),('route_points','Route points',9)]):axs[0,0].bar(xx+(j-1)*.23,[x[key]/total for x in metrics],width=.22,label=label)
    axs[0,0].set(xticks=xx,xticklabels=labels,ylim=(0,1.1),ylabel='Fraction completed',title='Stage completion');axs[0,0].legend(fontsize=8)
    names=sorted(set(k for row in metrics for k in row['phase_durations_s']));bottom=np.zeros(5)
    for name in names:
        v=np.array([x['phase_durations_s'].get(name,0) for x in metrics]);axs[0,1].bar(xx,v,bottom=bottom,label=name);bottom+=v
    axs[0,1].set(xticks=xx,xticklabels=labels,ylabel='Recorded ROS seconds',title='Observed phase time (failures are censored)');axs[0,1].legend(fontsize=6,ncol=2)
    for row in metrics:
        a=csv(Path(row['run_dir'])/'truth_pose.csv');axs[1,0].plot(a[:,0],a[:,3]+.22,lw=.9,label='Seed '+str(row['seed']))
    axs[1,0].set(xlabel='ROS simulation time (s)',ylabel='Actual FC AGL (m)',title='Height comparison');axs[1,0].legend(fontsize=8)
    for j,(key,label) in enumerate([('truth_mavros_xy_error_m','XY'),('truth_mavros_abs_z_error_m','Absolute Z')]):axs[1,1].bar(xx+(j-.5)*.32,[x[key].get('p95',0) for x in metrics],width=.31,label=label)
    axs[1,1].set(xticks=xx,xticklabels=labels,ylabel='P95 estimate / truth discrepancy (m)',title='Whole-run localization discrepancy');axs[1,1].legend(fontsize=8)
    for a in axs.flat:a.grid(axis='y',alpha=.15)
    fig.tight_layout();fig.savefig(D/'matrix_comparison.png',dpi=160);plt.close(fig)
    stats={'n':5,'full_pass':sum(x['status']=='PASS' for x in metrics),'three_drops':sum(x['release_count']==3 for x in metrics),'three_recoveries':sum(x['recovery_count']==3 for x in metrics),'nine_route_points':sum(x['route_points']==9 for x in metrics),'two_doors':sum(len(x['doors_crossed'])==2 for x in metrics),'collision_runs':sum(bool(x['collision_count']) for x in metrics),'legal_scenes':sum(x['scene_legality']['valid'] for x in metrics),'cleanup_pass':all(x['cleanup_pass'] for x in metrics),'media_file_count':sum(len(x['recording_files']) for x in metrics),'complete_mission_s':percentile([x['complete_mission_s'] for x in metrics if x['complete_mission_s'] is not None]),'route_to_disarm_s':percentile([x['route_to_disarm_s'] for x in metrics if x['status']=='PASS' and x['route_to_disarm_s'] is not None]),'raw_gate_error_counts':dict(collections.Counter(e for x in metrics for e in x['gate_errors']))}
    (D/'summary_stats.json').write_text(json.dumps(stats,indent=2));print(json.dumps(stats,indent=2));print(json.dumps([{k:x[k] for k in ['seed','status','release_count','route_points','doors_crossed','gate_errors','last_terminal_events']} for x in metrics],indent=2))
    cards=''.join('<article><h2>Seed %d — %s — %s</h2><a href="seed_%02d.png"><img src="seed_%02d.png"></a><p><a href="seed_%02d/terminal_diagnostics.png">Terminal diagnostics</a> · <a href="seed_%02d/timeline.json">Timeline</a></p></article>'%(x['seed'],x['door_pattern'],x['status'],x['seed'],x['seed'],x['seed'],x['seed']) for x in metrics)
    (D/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Full-random five-seed matrix</title><style>body{font:16px sans-serif;max-width:1450px;margin:28px auto;background:#f1f3f5;color:#243344}article{background:white;padding:20px;margin:24px 0}img{width:100%}a{color:#1267a4}</style><h1>Full-random five-seed matrix</h1><p>Recorded world, targets, actual flight paths and heights. No video recording. Each original failure is retained. Truth only enters evaluation.</p><a href="all_seed_layouts_paths.png"><img src="all_seed_layouts_paths.png"></a><a href="matrix_comparison.png"><img src="matrix_comparison.png"></a>'+cards)
if __name__=='__main__':main()

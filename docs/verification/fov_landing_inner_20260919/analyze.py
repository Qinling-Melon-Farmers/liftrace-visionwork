"""Matched fixed-start-frame flight analysis; offline truth only."""
import argparse
import csv
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle,Rectangle

D=Path(__file__).resolve().parent
R=D.parents[2]
spec=importlib.util.spec_from_file_location('prior_speed',D.parent/'fast_full_random_20260914/analyze_fast.py')
prior=importlib.util.module_from_spec(spec);spec.loader.exec_module(prior)


def scene(ax,world,truth):
    root=ET.parse(world).getroot().find('world')
    field=next(m for m in root.findall('model') if m.get('name')=='toudi2')
    for link in field.findall('link'):
        if not link.get('name','').startswith('Wall'):continue
        p=list(map(float,link.findtext('pose').split()));s=list(map(float,link.findtext('collision/geometry/box/size').split()))
        ax.add_patch(Rectangle((p[0]-s[0]/2,p[1]-s[1]/2),s[0],s[1],color='.5',alpha=.6))
    for tree in field.findall('model'):
        p=list(map(float,tree.findtext('pose').split()));ax.add_patch(Circle(p[:2],.43,color='forestgreen',alpha=.35))
    for t in truth['targets']:
        ax.scatter(t['world_x'],t['world_y'],s=30,marker='s',color='darkred')
        ax.annotate(t['class'],(t['world_x'],t['world_y']),xytext=(3,4),textcoords='offset points',fontsize=7)
    for obj in root.findall('include'):
        if obj.findtext('uri')=='model://landing_h':
            p=list(map(float,obj.findtext('pose').split()));ax.add_patch(Circle(p[:2],.5,fill=False,color='black'));ax.text(p[0],p[1],'H',ha='center',va='center')
    ax.set(xlim=(-.85,9.45),ylim=(-5.15,5.15),xlabel='Start-frame X inward (m)',ylabel='Start-frame Y left (m)')
    ax.set_aspect('equal');ax.grid(alpha=.2)


prior.base.scene=scene


def one(item):
    run=Path(item['run']);m=prior.analyze(item,D);out=D/(str(item['seed'])+'_'+item['label'])
    a=prior.base.csv4(run/'truth_pose.csv');start=m['start_ros_s'];end=m['observed_end_ros_s']
    ev=prior.base.events(run/'key_events.jsonl')
    dec=[e for e in ev if e['kind']=='decision'];res=[e for e in ev if e['kind']=='result']
    motion=[]
    for i,e in enumerate(dec):
        d=e['data'];seq=d['decision_seq'];issued=d['header']['stamp']['stamp_ns']/1e9
        results=[v['data'] for v in res if v['data']['decision_seq']==seq]
        terminal=next((v for v in reversed(results) if v['status'] in (3,4,5,6,7)),None)
        stop=terminal['header']['stamp']['stamp_ns']/1e9 if terminal else (dec[i+1]['ros_sec'] if i+1<len(dec) else end)
        motion.append(dict(seq=seq,command=d['command'],reason=d['reason'],target=d.get('target_class'),start=issued-start,end=stop-start,duration=max(0.,stop-issued),terminal=terminal.get('reason') if terminal else None))
    m['actions']=motion
    m['land_command_mission_s']=next((v['start'] for v in motion if v['command']==5),None)
    import re
    match=re.search(r'\[(\d+\.\d+),\s*(\d+\.\d+)\].*Gate terminal status=',(run/'run.log').read_text(errors='replace'))
    m['gate_terminal_ros_s']=float(match[2]) if match else None
    contacts=json.loads((run/'gazebo_contact_status.json').read_text()).get('events',[])
    m['contact_events']=contacts
    m['first_contact_ros_s']=contacts[0]['ros_stamp'] if contacts else None
    high=m.get('high_view_final',{})
    top=next((e['time'] for e in high.get('events',[]) if e['stage']=='SURVEY_INTERRUPTED_TOP3'),None)
    m['top3_interrupt_mission_s']=top-start if top is not None else None
    truth_targets=yaml.safe_load((run/'random_field_truth.yaml').read_text())['targets']
    true_xy={t['class']:np.array([t['world_x'],t['world_y']]) for t in truth_targets}
    m['low_reacquisition_truth_errors']=[dict(class_name=v['class_name'],error_m=float(np.linalg.norm(np.array(v['xy'])-true_xy[v['class_name']])),time=v['time']-start) for v in high.get('reacquisitions',[])]
    m['conflict_hypothesis_evaluation']={c:[dict(xy=h['xy'],intended_class_error_m=float(np.linalg.norm(np.array(h['xy'])-true_xy[c])),nearest_truth_class=min(true_xy,key=lambda k:np.linalg.norm(np.array(h['xy'])-true_xy[k]))) for h in hs] for c,hs in high.get('conflict_locations',{}).items()}
    m['coverage_fallback_started']=high.get('fallback_started')
    m['conflict_checked']=high.get('conflict_checked',[])
    # Comparable milestones use actual route decisions, including recovery after third ACK.
    post=[e for e in dec if e['data']['command']==4 and e['data']['reason'].startswith('post_delivery_route:')]
    land=next((e['ros_sec'] for e in dec if e['data']['command']==5),None)
    third=start+m['third_commit_s'] if m['third_commit_s'] is not None else None
    transit=post[0]['ros_sec'] if post else None
    staging=post[1]['ros_sec'] if len(post)>1 else None
    corridor=post[2]['ros_sec'] if len(post)>2 else None
    moments=[start,third,transit,staging,corridor,land,end]
    labels=['start_to_third_ack','last_release_recovery','fast_transfer_to_staging','staging_descent','corridor_to_H','H_landing']
    m['milestone_durations_s']={k:(b-a if a is not None and b is not None else None) for k,a,b in zip(labels,moments,moments[1:])}
    # Initial planning delays, distinct from stalls on an accepted trajectory.
    planner=[e['data'] for e in ev if e['kind']=='planner'];planning=[]
    for goal in sorted({p['goal_seq'] for p in planner}):
        rows=[p for p in planner if p['goal_seq']==goal]
        first=min(p['header']['stamp']['stamp_ns']/1e9 for p in rows)
        ready=next((p['header']['stamp']['stamp_ns']/1e9 for p in rows if p['status']==2),None)
        planning.append(dict(goal_seq=goal,first_ros=first,ready_delay_s=ready-first if ready else None,max_attempt=max(p['planning_attempt'] for p in rows),reasons=sorted({p['reason'] for p in rows})))
    m['initial_planning']=planning
    records=prior.base.events(run/'trajectory_progress.jsonl');recoveries={}
    for row in records:
        p=row['data']
        if p['source']==0:recoveries[p['goal_seq']]=max(recoveries.get(p['goal_seq'],0),p['recoveries'])
    m['liveness_recovery_requests']=sum(recoveries.values())
    m['liveness_budget_exhausted']=any(p['data']['reason']=='liveness_budget_exhausted' for p in records)
    # FC attitude is truth, combined tilt angle from the body vertical.
    with (run/'truth_pose.csv').open() as f:rows=list(csv.DictReader(f))
    q=np.array([[float(row[k]) for k in ('qx','qy','qz','qw')] for row in rows]);tilt=np.degrees(np.arccos(np.clip(1-2*(q[:,0]**2+q[:,1]**2),-1,1)))
    active=(a[:,0]>=start)&(a[:,0]<=end)
    region=active&(a[:,1]>=7.6)&(a[:,1]<=9.1)&(abs(a[:,2])<=4.8)
    m['corridor_geometry']=dict(max_agl_m=float((a[region,3]+.22).max()) if region.any() else None,max_combined_tilt_deg=float(tilt[region].max()) if region.any() else None,samples=int(region.sum()),tilt_above15_samples=int((tilt[region]>15).sum()))
    before_land=region & (a[:,0]<land) if land is not None else region
    m['corridor_geometry']['cruise_max_combined_tilt_deg']=float(tilt[before_land].max()) if before_land.any() else None
    # World-Z support of the already-inflated 55x55x40cm guard (offset -2cm).
    from scipy.spatial.transform import Rotation
    zrow=Rotation.from_quat(q).as_matrix()[:,2,:]
    half=np.full(len(a),.275)
    if land is not None:half[a[:,0]>=land]=.25
    guard_top=a[:,3]+.22+(np.abs(zrow[:,:2]).sum(axis=1)*half+np.abs(zrow[:,2])*.2)-.02*zrow[:,2]
    m['corridor_geometry']['max_guard_top_agl_m']=float(guard_top[region].max()) if region.any() else None
    if land is not None:
        window=(a[:,0]>=land-3)&(a[:,0]<=end)
        contact=m['first_contact_ros_s'] or end
        pre_contact=(a[:,0]>=land)&(a[:,0]<contact)
        estimates={};fig,axs=plt.subplots(3,1,figsize=(12,8),sharex=True)
        axs[0].plot(a[window,0]-land,a[window,3]+.22,label='Truth FC AGL',lw=1)
        for name in ('lio_pose','mavros_pose'):
            data=prior.base.csv4(run/(name+'.csv'))
            interp=np.column_stack([np.interp(a[:,0],data[:,0],data[:,i]) for i in (1,2,3)])
            zerr=interp[:,2]-a[:,3];xyerr=np.linalg.norm(interp[:,:2]-a[:,1:3],axis=1)
            estimates[name]=dict(pre_contact_xy_error_max_m=float(xyerr[pre_contact].max()) if pre_contact.any() else None,pre_contact_z_error_max_m=float(np.abs(zerr[pre_contact]).max()) if pre_contact.any() else None)
            axs[0].plot(a[window,0]-land,interp[window,2]+.22,label=name,lw=.8)
            axs[1].plot(a[window,0]-land,zerr[window],label=name,lw=.8)
            axs[2].plot(a[window,0]-land,xyerr[window],label=name,lw=.8)
        for ax in axs:
            ax.grid(alpha=.2);ax.legend(fontsize=8)
            if m['first_contact_ros_s']:ax.axvline(contact-land,color='red',ls='--',label='Contact')
            if m['gate_terminal_ros_s']:ax.axvline(m['gate_terminal_ros_s']-land,color='purple',ls=':')
        axs[0].set_ylabel('FC AGL (m)');axs[1].set_ylabel('Estimate - truth Z (m)');axs[2].set_ylabel('XY error (m)');axs[2].set_xlabel('Seconds since LAND decision')
        fig.suptitle('Near-ground estimates | dashed red: first contact; dotted purple: Gate terminal')
        fig.tight_layout();fig.savefig(out/'landing_diagnostic.png',dpi=160);plt.close(fig)
        m['landing_estimation']=estimates
    fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True)
    for name in ('truth_pose','lio_pose','mavros_pose','mavros_setpoint'):
        data=prior.base.csv4(run/(name+'.csv'));axes[0].plot(data[:,0]-start,data[:,3]+.22,lw=.7,label=name)
    limit=1.2 if item['label']!='A_baseline' else .7
    limits=np.full(len(a),limit)
    h_start=post[-1]['ros_sec'] if post and post[-1]['data']['reason'].startswith('post_delivery_route:9/9:') else np.inf
    hmask=region&(a[:,0]>=h_start)&(a[:,2]<=-2.1)
    limits[hmask]=1.2
    axes[0].plot(a[:,0]-start,np.where(region,limits,np.nan),'r--',label='Phase-aware corridor/H gate limit')
    axes[1].plot(a[:,0]-start,tilt,lw=.7);axes[1].plot(a[:,0]-start,np.where(region,15,np.nan),'r--',label='15 deg corridor geometry assumption')
    axes[0].set_ylabel('FC AGL (m)');axes[1].set_ylabel('Combined tilt (deg)');axes[1].set_xlabel('Mission seconds')
    for ax in axes:ax.grid(alpha=.2);ax.legend(fontsize=8);ax.set_xlim(0,end-start)
    fig.tight_layout();fig.savefig(out/'height_tilt.png',dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(12,5))
    for source,label in [(0,'FSM'),(1,'Server')]:
        rows=[r['data'] for r in records if r['data']['source']==source]
        ax.plot([p['header']['stamp']/1e9-start for p in rows],[p['stagnant_seconds'] for p in rows],lw=.6,label=label)
    ax.axhline(4,ls=':',color='red');ax.set(xlabel='Mission seconds',ylabel='No physical progress (s)',title='Progress watchdog; initial planning delays reported separately');ax.legend();ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(out/'progress.png',dpi=160);plt.close(fig)
    pilot=D.parent/'fast_full_random_20260914/pilot'
    for script,name in [('check_tree_overflight.py','tree_projection.json'),('check_body_projection.py','body_projection.json')]:
        subprocess.run([sys.executable,str(pilot/script),str(run),item['world'],'--out',str(out/name)],check=True,stdout=subprocess.DEVNULL)
    (out/'metrics.json').write_text(json.dumps(m,indent=2));return m


def main():
    p=argparse.ArgumentParser();p.add_argument('runs',type=Path);a=p.parse_args();items=json.loads(a.runs.read_text());metrics=[one(i) for i in items]
    (D/'summary.json').write_text(json.dumps(metrics,indent=2))
    fig,ax=plt.subplots(figsize=(9,9));scene(ax,Path(items[0]['world']),yaml.safe_load((Path(items[0]['run'])/'random_field_truth.yaml').read_text()))
    for item,m in zip(items,metrics):
        data=prior.base.csv4(Path(item['run'])/'truth_pose.csv');ax.plot(data[:,1],data[:,2],lw=.8,label=item['label']+' '+m['status'])
    ax.legend();ax.set_title('Same frozen layout | complete recorded trajectories');fig.tight_layout();fig.savefig(D/'paired_paths.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(13,4))
    for ax,key,title in zip(axes,['top3_interrupt_mission_s','third_commit_s','completed_mission_s'],['Top3 interruption (s)','Third delivery ACK (s)','Completed mission (s)']):
        for i,m in enumerate(metrics):
            if m[key] is not None:ax.bar(i,m[key]);ax.text(i,m[key],f'{m[key]:.1f}',ha='center',va='bottom')
            else:ax.text(i,0,'No top3 early stop' if key=='top3_interrupt_mission_s' else 'FAIL: not completed',rotation=90,ha='center')
        ax.set_xticks(range(len(metrics)),[m['label'] for m in metrics],rotation=20);ax.set_title(title);ax.grid(axis='y',alpha=.2)
    fig.tight_layout();fig.savefig(D/'comparison.png',dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(11,5));left=np.zeros(len(metrics))
    for phase in ['start_to_third_ack','last_release_recovery','fast_transfer_to_staging','staging_descent','corridor_to_H','H_landing']:
        values=[m['milestone_durations_s'][phase] or 0 for m in metrics];bars=ax.barh(range(len(metrics)),values,left=left,label=phase)
        if phase=='H_landing':
            for bar,m in zip(bars,metrics):
                if m['status']!='PASS':bar.set_hatch('///')
        left+=values
    ax.set_yticks(range(len(metrics)),[m['label'] for m in metrics]);ax.set_xlabel('Simulation seconds (incomplete sections omitted)');ax.legend(fontsize=8,bbox_to_anchor=(1.02,1));fig.tight_layout();fig.savefig(D/'stage_comparison.png',dpi=160);plt.close(fig)
    print(json.dumps([{k:m.get(k) for k in ('label','status','top3_interrupt_mission_s','third_commit_s','completed_mission_s','milestone_durations_s','corridor_geometry','liveness_recovery_requests')} for m in metrics],indent=2))

if __name__=='__main__':main()

"""Offline paired full-mission report; truth never enters navigation."""
import argparse
import csv
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle


def csv4(path):
    with path.open() as f:
        return np.array([[float(r[k]) for k in ('t','x','y','z')] for r in csv.DictReader(f)])


def events(path):
    return [json.loads(s) for s in path.read_text().splitlines() if s.strip()]


def scene(ax, world, truth):
    field=next(m for m in ET.parse(world).getroot().find('world').findall('model') if m.get('name')=='toudi2')
    for link in field.findall('link'):
        if not link.get('name','').startswith('Wall'): continue
        pose=[float(v) for v in link.findtext('pose','0 0 0 0 0 0').split()]
        size=[float(v) for v in link.findtext('collision/geometry/box/size','0 0 0').split()]
        ax.add_patch(Rectangle((pose[0]-size[0]/2,pose[1]-size[1]/2),size[0],size[1],color='.5',alpha=.6))
    for model in field.findall('model'):
        if 'Tree' in model.get('name',''):
            p=[float(v) for v in model.findtext('pose','0 0 0 0 0 0').split()]
            ax.add_patch(Circle(p[:2],.43,color='forestgreen',alpha=.35))
    for target in truth['targets']:
        x,y=target['world_x'],target['world_y']
        ax.scatter(x,y,s=30,marker='s',color='darkred')
        ax.annotate(target['class'],(x,y),xytext=(3,4),textcoords='offset points',fontsize=7)
    for x,y in [(0,0),(4.2,8.5)]:
        ax.add_patch(Circle((x,y),.5,fill=False,color='black'))
        ax.text(x,y,'H',ha='center',va='center')
    ax.set(xlim=(-5.15,5.15),ylim=(-.85,9.45),xlabel='X (m)',ylabel='Y (m)')
    ax.set_aspect('equal');ax.grid(alpha=.2)


def analyze(item,out):
    run=Path(item['run']);label=item['label'];seed=item['seed']
    gate=json.loads((run/'gate_status.json').read_text())
    params=yaml.safe_load((run/'rosparams.yaml').read_text())
    truth=yaml.safe_load((run/'random_field_truth.yaml').read_text())
    ev=events(run/'key_events.jsonl');dec=[e for e in ev if e['kind']=='decision']
    start=min(e['data']['header']['stamp']['stamp_ns']/1e9 for e in dec)
    a=csv4(run/'truth_pose.csv');est=csv4(run/'mavros_pose.csv');cmd=csv4(run/'mavros_setpoint.csv')
    offset=params['competition_key_recorder']['truth_world_offset'][2]
    ground=params['target_map_projector']['ground_z']
    releases=[e for e in ev if e['kind']=='release']
    end=start+gate['metrics']['mission_ros_sec'] if gate['metrics'].get('mission_ros_sec') is not None else a[-1,0]
    active=a[(a[:,0]>=start)&(a[:,0]<=end)]
    dt=np.diff(active[:,0]);delta=np.diff(active[:,1:4],axis=0)
    valid=(dt>0)&(dt<=.5)
    speed=np.linalg.norm(delta[:,:2],axis=1)[valid]/dt[valid]
    metrics=dict(label=label,seed=seed,run=str(run),status=gate['status'],reason=gate['reason'],
        failed_checks=gate.get('failed_checks',[]),start_ros_s=start,observed_end_ros_s=end,
        completed_mission_s=gate['metrics'].get('mission_ros_sec'),
        observed_duration_s=end-start,xy_distance_m=float(np.linalg.norm(delta[:,:2],axis=1)[valid].sum()),
        xyz_distance_m=float(np.linalg.norm(delta,axis=1)[valid].sum()),
        speed_xy_p95_mps=float(np.percentile(speed,95)) if len(speed) else None,
        max_fc_agl_m=float((active[:,3]+offset).max()),
        releases=releases,gate_metrics=gate['metrics'],
        collisions=json.loads((run/'gazebo_contact_status.json').read_text())['actual_collision_count'],
        sample_gap_count=int((dt>.5).sum()))
    research=run/'high_view_full_events.jsonl'
    statuses=events(research) if research.exists() else []
    if statuses: metrics['high_view_final']=statuses[-1]['status']
    # Release commit timing comes from terminal payload-committed results,
    # deduplicated by slot; raw release diagnostic messages are kept separately.
    commits={}
    for e in ev:
        d=e['data']
        if e['kind']=='result' and d.get('payload_committed'):
            commits.setdefault(d['payload_slot'],dict(t=d['header']['stamp']['stamp_ns']/1e9,target=d['target_class']))
    metrics['commit_times']=list(commits.values())
    metrics['third_commit_s']=max(c['t'] for c in commits.values())-start if len(commits)==3 else None
    stages=[]
    for e in statuses:
        name=e['status']['stage']
        if name=='SURVEY' and not e['status'].get('ascent_verified'): name='ASCEND'
        if not stages or name!=stages[-1]['name']: stages.append(dict(name=name,t=e['t']))
    metrics['high_view_stage_changes']=stages
    timeline=[];active_seq=None;ordinary='SEARCH';research_phase=None
    combined=[(e['ros_sec'],'event',e) for e in ev if e['kind'] in ('decision','result')]
    combined += [(s['t'],'research',s) for s in stages]
    for t,kind,e in sorted(combined,key=lambda r:r[0]):
        if t<start or t>end:continue
        if kind=='research':research_phase=e['name']
        elif e['kind']=='decision':
            d=e['data'];active_seq=d['decision_seq']
            ordinary={0:'SEARCH',1:'APPROACH',2:'ALIGN',3:'SEARCH',4:'CORRIDOR',5:'LAND',6:'HOLD',7:'ABORT'}[d['command']]
        elif e['data']['decision_seq']==active_seq:
            d=e['data']
            if d['command'] in (1,2):ordinary={1:'APPROACH',2:'CAPTURE',3:'ALIGN',4:'RELEASE',5:'RECOVERY'}.get(d['stage'],ordinary)
        phase=research_phase if research_phase in ('ASCEND','SURVEY','RETURN_COLUMN','DESCEND','REVISIT','REACQUIRE') else ordinary
        if not timeline or timeline[-1]['phase']!=phase:timeline.append(dict(t=t,phase=phase))
    durations={}
    for i,entry in enumerate(timeline):
        duration=(timeline[i+1]['t'] if i+1<len(timeline) else end)-entry['t']
        durations[entry['phase']]=durations.get(entry['phase'],0)+duration
    metrics['sampled_phase_durations_s']=durations
    metrics['sampled_phase_changes']=timeline
    dest=out/(str(seed)+'_'+label);dest.mkdir(exist_ok=True)
    (dest/'metrics.json').write_text(json.dumps(metrics,indent=2))
    (dest/'gate_status.json').write_text(json.dumps(gate,indent=2))
    fig,axes=plt.subplots(2,2,figsize=(14,11));xy,zax,vax,tax=axes.flat
    scene(xy,Path(item['world']),truth)
    xy.plot(a[:,1],a[:,2],lw=1,label='Actual flight',color='#185b9a')
    xy.plot(cmd[:,1],cmd[:,2],lw=.5,alpha=.5,label='Command trajectory',color='#ef8425')
    goals=[e['data']['goal']['pose']['position'] for e in dec if e['data'].get('has_goal')]
    if goals:
        xy.scatter([g['x'] for g in goals],[g['y'] for g in goals],s=18,marker='x',color='black',label='Issued navigation goals')
    xy.legend(fontsize=7);xy.set_title('Complete flight and navigation goals')
    for values,shift,name,style in [(a,offset,'True FC AGL','-'),(est,-ground,'Estimated FC AGL','--'),(cmd,-ground,'Command FC AGL',':')]:
        zax.plot(values[:,0]-start,values[:,3]+shift,style,lw=.8,label=name)
    zax.set(xlabel='Simulation seconds after first mission command',ylabel='Height (m)',title='Height through delivery, doors and landing');zax.legend(fontsize=8)
    vax.plot(active[1:,0][valid]-start,speed,lw=.8,label='True horizontal speed')
    vax.plot(active[1:,0][valid]-start,delta[:,2][valid]/dt[valid],lw=.6,label='True vertical speed')
    vax.set(xlabel='Simulation seconds after first mission command',ylabel='Speed (m/s)',title='Speed from truth poses (gaps excluded)');vax.legend(fontsize=8)
    names={0:'SEARCH',1:'APPROACH',2:'ALIGN',3:'RESUME',4:'RETURN',5:'LAND',6:'HOLD',7:'ABORT'}
    tax.step([e['ros_sec']-start for e in dec],[e['data']['command'] for e in dec],where='post',lw=1)
    for c in commits.values():
        for ax in (zax,vax,tax): ax.axvline(c['t']-start,color='darkred',ls=':',alpha=.7)
        tax.annotate(c['target'],(c['t']-start,0),rotation=45,fontsize=7)
    for st in stages: tax.axvline(st['t']-start,color='gray',alpha=.3)
    tax.set(xlabel='Simulation seconds after first mission command',ylabel='MissionDecision command enum',title='Issued commands; red dotted lines = committed slots')
    tax.set_yticks(list(names),list(names.values()))
    for ax in (zax,vax,tax):ax.grid(alpha=.2)
    fig.suptitle(f'Seed {seed} | {label} | {gate["status"]} | {len(commits)}/3 committed | collisions {metrics["collisions"]}')
    fig.tight_layout();fig.savefig(dest/'flight_charts.png',dpi=170);plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(12,6))
    phase_names=list(durations)
    axes[0].barh(phase_names,list(durations.values()),color='#3677a6')
    axes[0].set(xlabel='Simulation seconds',title='Sampled phase duration (event/status boundaries)')
    target_classes=sorted(set(e['data'].get('target_class','') for e in ev if e['kind'] in ('decision','result','release'))-{''})
    for i,cls in enumerate(target_classes):
        for kind,marker in [('decision','>'),('result','.'),('release','*')]:
            xs=[e['ros_sec']-start for e in ev if e['kind']==kind and e['data'].get('target_class')==cls]
            axes[1].scatter(xs,[i]*len(xs),s=35 if kind=='release' else 12,marker=marker,label=kind if i==0 else None)
    axes[1].set_yticks(range(len(target_classes)),target_classes)
    axes[1].set(xlabel='Simulation seconds after first mission command',title='Target transactions (release message is diagnostic; commits checked separately)')
    axes[1].legend(fontsize=8)
    for ax in axes:ax.grid(axis='x',alpha=.2)
    fig.tight_layout();fig.savefig(dest/'phase_target_timeline.png',dpi=170);plt.close(fig)
    return metrics


def main():
    p=argparse.ArgumentParser();p.add_argument('--runs',type=Path,required=True);p.add_argument('--out',type=Path,required=True);args=p.parse_args()
    args.out.mkdir(exist_ok=True)
    items=json.loads(args.runs.read_text());metrics=[analyze(item,args.out) for item in items]
    (args.out/'summary.json').write_text(json.dumps(metrics,indent=2))
    fig,axes=plt.subplots(1,3,figsize=(13,4))
    for ax,key,title in zip(axes,['third_commit_s','completed_mission_s','xy_distance_m'],['Third committed delivery (s)','Completed mission (s)','Observed XY distance (m)']):
        for i,m in enumerate(metrics):
            value=m[key]
            if value is not None:ax.bar(i,value,color='#3b75a9' if m['label']=='baseline' else '#dd913a')
            else:ax.text(i,0,'Incomplete',ha='center',rotation=90)
        ax.set_xticks(range(len(metrics)),[str(m['seed'])+' '+m['label'] for m in metrics],rotation=25,ha='right');ax.set_title(title);ax.grid(axis='y',alpha=.2)
    fig.tight_layout();fig.savefig(args.out/'comparison.png',dpi=170);plt.close(fig)
    seeds=sorted(set(i['seed'] for i in items))
    fig,axes=plt.subplots(1,len(seeds),figsize=(7*len(seeds),7),squeeze=False)
    paired=[]
    for seed,ax in zip(seeds,axes.flat):
        pair=[i for i in items if i['seed']==seed]
        scene(ax,Path(pair[0]['world']),yaml.safe_load((Path(pair[0]['run'])/'random_field_truth.yaml').read_text()))
        for item in pair:
            data=csv4(Path(item['run'])/'truth_pose.csv')
            ax.plot(data[:,1],data[:,2],lw=.85,alpha=.8,label=item['label'])
        ax.legend();ax.set_title(f'Seed {seed}: complete actual trajectories')
        mm={m['label']:m for m in metrics if m['seed']==seed}
        if set(mm)=={'baseline','strategy'}:
            result={'seed':seed,'both_gate_pass':all(m['status']=='PASS' for m in mm.values())}
            for key in ('third_commit_s','completed_mission_s'):
                a,b=mm['baseline'][key],mm['strategy'][key]
                result[key+'_gain_s']=a-b if a is not None and b is not None else None
                result[key+'_gain_percent']=100*(a-b)/a if a is not None and b is not None and a>0 else None
            paired.append(result)
    fig.tight_layout();fig.savefig(args.out/'paired_paths.png',dpi=170);plt.close(fig)
    (args.out/'paired_differences.json').write_text(json.dumps(paired,indent=2))


if __name__=='__main__':main()

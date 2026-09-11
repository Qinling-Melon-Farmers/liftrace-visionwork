"""Summarize authorized research runs without launching ROS or simulation."""
from pathlib import Path
import json
import collections
import xml.etree.ElementTree as ET
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle
from matplotlib.transforms import Affine2D

ROOT=Path(__file__).resolve().parents[3]
OUT=Path(__file__).resolve().parent
plt.rcParams.update({'font.size':9,'axes.spines.top':False,'axes.spines.right':False})


def read(path,default=None):
    return json.loads(path.read_text()) if path.exists() else default


def csv(path):
    return np.loadtxt(path,delimiter=',',skiprows=1,ndmin=2)


def command_motion_summary(run):
    """Same read-only timing method for current and older runs without an observer."""
    pose=csv(run/'truth_pose.csv')
    events=[json.loads(x) for x in (run/'key_events.jsonl').read_text().splitlines() if x]
    unique={}
    for e in events:
        if e['kind']=='decision':unique.setdefault(e['data']['decision_seq'],e)
    decisions=sorted(unique.values(),key=lambda e:e['ros_sec'])
    terminal={}
    for e in events:
        if e['kind']=='result' and e['data'].get('terminal'):terminal.setdefault(e['data']['decision_seq'],e)
    mid=(pose[1:,0]+pose[:-1,0])/2;dt=np.diff(pose[:,0]);ds=np.linalg.norm(np.diff(pose[:,1:3],axis=0),axis=1)
    mask=np.zeros(len(mid),dtype=bool);intervals=[];waits=[]
    for i,e in enumerate(decisions):
        data=e['data'];seq=data['decision_seq']
        if data['command'] not in [0,3]:continue
        end=min(decisions[i+1]['ros_sec'] if i+1<len(decisions) else pose[-1,0],terminal.get(seq,{}).get('ros_sec',pose[-1,0]))
        mask|=(mid>=e['ros_sec'])&(mid<end)&(dt>0)&(dt<=.5)
        intervals.append({'seq':seq,'command':data['command'],'start':e['ros_sec'],'end':end})
        term=terminal.get(seq)
        if term and term['data'].get('reason')=='search_initial_plan_timeout':
            waits.append({'seq':seq,'seconds':term['ros_sec']-e['ros_sec'],'goal':data['goal']['pose']['position']})
    return {'search_resume_s':float(dt[mask].sum()),'search_resume_xy_m':float(ds[mask].sum()),
            'initial_wait_events':waits,'initial_wait_s':sum(x['seconds'] for x in waits)}


def field(ax,run):
    xml=ET.parse(run/'scenario_inputs/field.world').getroot()
    for model in xml.iter('model'):
        if 'tree' in model.get('name','').lower():
            p=[float(x) for x in model.findtext('pose').split()]
            ax.add_patch(Circle(p[:2],.43,color='#568361',alpha=.35))
        if model.get('name')!='toudi2':continue
        for link in model.findall('link'):
            if not link.get('name','').startswith('Wall'):continue
            p=[float(x) for x in link.findtext('pose','0 0 0 0 0 0').split()]
            s=[float(x) for x in link.findtext('collision/geometry/box/size','0 0 0').split()]
            patch=Rectangle((p[0]-s[0]/2,p[1]-s[1]/2),s[0],s[1],color='#969ca5')
            patch.set_transform(Affine2D().rotate_around(p[0],p[1],p[5])+ax.transData);ax.add_patch(patch)
    for target in yaml.safe_load((run/'random_field_truth.yaml').read_text())['targets']:
        ax.scatter(target['world_x'],target['world_y'],marker='s',s=25,
                   color='#c33e4a' if target['class']=='red_cross' else '#af9145')
    ax.set(xlim=(-5,5),ylim=(-.8,9.5),xlabel='X (m)',ylabel='Y (m)');ax.set_aspect('equal');ax.grid(alpha=.15)


def analyze(run):
    gate=read(run/'gate_status.json',{})
    diagnostic=read(run/'research_diagnostic_status.json',{})
    status=gate.get('status',diagnostic.get('status','INCOMPLETE'))
    pose=csv(run/'truth_pose.csv');mav=csv(run/'mavros_pose.csv');cmd=csv(run/'mavros_setpoint.csv')
    events=[json.loads(x) for x in (run/'key_events.jsonl').read_text().splitlines() if x]
    decisions=[e for e in events if e['kind']=='decision']
    releases=[e for e in events if e['kind']=='release' and e['data'].get('success')]
    t0=min([e['ros_sec'] for e in decisions] or [pose[0,0]])
    phases=[]
    for e in events:
        if e['kind']=='mission' and (not phases or phases[-1]['name']!=e['data']['phase']):
            phases.append({'t':e['ros_sec'],'name':e['data']['phase']})
    phase_t=np.array([p['t'] for p in phases]);phase_n=np.array([p['name'] for p in phases])
    dt=np.diff(pose[:,0]);mid=(pose[1:,0]+pose[:-1,0])/2
    ids=np.searchsorted(phase_t,mid,side='right')-1
    search=(ids>=0)&(phase_n[np.maximum(ids,0)]=='SEARCH')&(dt>0)&(dt<=.5)
    ds=np.linalg.norm(np.diff(pose[:,1:3],axis=0),axis=1)
    obs=[json.loads(x) for x in (run/'coverage/status.jsonl').read_text().splitlines() if x]
    accepted=[o for o in obs if o.get('accepted')]
    fresh=[o for o in accepted if o.get('map_fresh')]
    rt=yaml.safe_load((run/'scenario_inputs/runtime.yaml').read_text())
    target_seed=yaml.safe_load((run/'random_field_truth.yaml').read_text()).get('seed')
    metrics=gate.get('metrics',{})
    contact=read(run/'gazebo_contact_status.json',{})
    row={'run_dir':str(run),'status':status,'gate_reason':gate.get('reason',diagnostic.get('reason')),
         'checks_passed':sum(gate.get('checks',{}).values()),'checks_total':len(gate.get('checks',{})),
         'mission_start_basis':'first recorded decision, not referee clock','mission_start_ros':t0,
         'last_ros':float(pose[-1,0]),'drops':len(releases),
         'third_release_s':releases[2]['ros_sec']-t0 if len(releases)>=3 else None,
         'release_classes':[e['data'].get('target_class') for e in releases],
         'gate_mission_s':metrics.get('mission_ros_sec'),
         'search_seconds':float(dt[search].sum()),'search_xy_m':float(ds[search].sum()),
         'search_slow_seconds':float(dt[search & (ds/np.maximum(dt,1e-9)<.1)].sum()),
         'full_xy_m':float(ds.sum()),'max_fc_agl_m':float(pose[:,3].max()+.22),
         'collisions':contact.get('actual_collision_count'),'landing_ground':metrics.get('final_landed_state')==1 or gate.get('checks',{}).get('final_landed_on_ground',False),
         'post_route_points':metrics.get('post_delivery_return_success_count'),
         'lane_spacing':rt['search']['lane_spacing'],'observer_records':len(obs),'observer_accepted':len(accepted),
         'observer_fresh_maps':len(fresh),'fresh_fraction_of_accepted':len(fresh)/len(accepted) if accepted else None,
         'observer_reason_counts':dict(collections.Counter(o['reason'] for o in obs)),
         'map_reason_counts':dict(collections.Counter(o.get('map_reason','unrecorded') for o in accepted)),
         'max_active_seen_estimate_m2':max(o['estimated_seen_area_m2'] for o in obs),
         'observer_processing_ms':{k:float(np.percentile([o['processing_ms'] for o in obs if 'processing_ms' in o],v)) for k,v in [('p50',50),('p95',95),('p99',99)]},
         'sync_samples':len(list((run/'coverage').glob('*.png'))),
         'media_files':[str(p.relative_to(run)) for p in run.rglob('*') if p.suffix in ['.mp4','.bag']]}
    row['command_motion']=command_motion_summary(run)
    label=run.name
    fig,axes=plt.subplots(2,2,figsize=(13,10))
    field(axes[0,0],run)
    axes[0,0].plot(pose[:,1],pose[:,2],lw=.9,color='#196ba0',label='Actual path')
    sc=rt['search'];ys=[sc['min_y']]
    while ys[-1]<sc['max_y']:ys.append(min(ys[-1]+sc['lane_spacing'],sc['max_y']))
    for y in ys:axes[0,0].plot([sc['min_x'],sc['max_x']],[y,y],'--',color='#77918c',lw=.4,alpha=.6)
    for i,e in enumerate(releases):
        p=pose[np.argmin(abs(pose[:,0]-e['ros_sec']))]
        axes[0,0].scatter(p[1],p[2],marker='*',color='#d73d50',s=80)
        axes[0,0].annotate('D%d'%(i+1),p[1:3],xytext=(4,5),textcoords='offset points')
    axes[0,0].set_title('Frozen layout, nominal lanes and actual path')
    axes[0,1].plot(pose[:,0]-t0,pose[:,3]+.22,label='Actual FC AGL')
    axes[0,1].plot(mav[:,0]-t0,mav[:,3]+.22,lw=.6,label='Estimated z +0.22')
    axes[0,1].plot(cmd[:,0]-t0,cmd[:,3]+.22,lw=.6,label='Command z +0.22')
    for e in releases:axes[0,1].axvline(e['ros_sec']-t0,color='#b64e64',ls=':',lw=.7)
    axes[0,1].set(title='Height and releases',xlabel='Time since first decision (s)',ylabel='Height (m)');axes[0,1].legend(fontsize=8)
    tt=np.array([o['receipt_ros'] for o in obs])-t0
    axes[1,0].plot(tt,[o['estimated_seen_area_m2'] for o in obs],color='#287a4c',label='Active visibility estimate (TTL60s)')
    axes[1,0].plot(tt,[o.get('footprint_area_m2',0) for o in obs],color='#4886ad',lw=.6,label='Current-frame footprint')
    axes[1,0].set(title='Observation estimate, not target recall or free area',xlabel='Time since first decision (s)',ylabel='Area (m²)');axes[1,0].legend(fontsize=8)
    names=list(dict.fromkeys(p['name'] for p in phases));colors=plt.get_cmap('tab10')
    for i,p in enumerate(phases):
        end=phases[i+1]['t'] if i+1<len(phases) else pose[-1,0]
        index=names.index(p['name'])
        axes[1,1].broken_barh([(p['t']-t0,max(0,end-p['t']))],(index-.3,.6),facecolors=colors(index%10))
    axes[1,1].set(yticks=range(len(names)),yticklabels=names,xlabel='Time since first decision (s)',title='Recorded mission phases')
    for ax in axes.flat:ax.grid(alpha=.15)
    fig.suptitle(f"{label}\n{status} | drops {len(releases)}/3 | lanes {sc['lane_spacing']:.2f}m | maps {len(fresh)}/{len(accepted)}",fontsize=12)
    fig.tight_layout(rect=(0,0,1,.94));fig.savefig(OUT/(label+'.png'),dpi=145);plt.close(fig)
    return row

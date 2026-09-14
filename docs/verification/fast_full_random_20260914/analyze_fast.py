"""Offline speed/flight analysis. No truth data is consumed by navigation."""
import argparse
import importlib.util
import json
from pathlib import Path
import re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
spec=importlib.util.spec_from_file_location('full_report',HERE.parent/'high_view_full_20260914/analyze.py')
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)

def analyze(item,out):
    m=base.analyze(item,out)
    run=Path(item['run']);dest=out/(str(item['seed'])+'_'+item['label'])
    a=base.csv4(run/'truth_pose.csv')
    dt=np.diff(a[:,0]);valid=(dt>0)&(dt<=.5)
    t=a[1:,0][valid];v=np.linalg.norm(np.diff(a[:,1:3],axis=0)[valid],axis=1)/dt[valid]
    text=(run/'run.log').read_text(errors='replace')
    transitions=[]
    for line in text.splitlines():
        if 'Following speed phase=' not in line:continue
        match=re.search(r'\[\d+\.\d+,\s*(\d+\.\d+)\].*Following speed phase=(\w+) lead=([\d.]+)',line)
        if match:transitions.append(dict(t=float(match[1]),phase=match[2],lead_m=float(match[3])))
    if not transitions:
        progress=json.loads((run/'live_progress.json').read_text())
        transitions=progress.get('mission',{}).get('speed_transitions',[])
    stats={}
    for i,e in enumerate(transitions):
        stop=transitions[i+1]['t'] if i+1<len(transitions) else m['observed_end_ros_s']
        select=(t>=max(e['t'],m['start_ros_s']))&(t<stop)
        data=v[select&(v>.03)]
        row=stats.setdefault(e['phase'],dict(samples=[],duration_s=0.))
        row['samples'].extend(data.tolist());row['duration_s']+=max(0.,stop-max(e['t'],m['start_ros_s']))
    for row in stats.values():
        data=row.pop('samples');row.update(moving_samples=len(data),median_mps=float(np.median(data)) if data else None,p95_mps=float(np.percentile(data,95)) if data else None)
    m['following_speed_transitions']=transitions;m['following_speed_stats']=stats
    transit=next((e for e in transitions if e['phase']=='TRANSIT_TO_CORRIDOR'),None)
    corridor=next((e for e in transitions if e['phase']=='CORRIDOR' and (transit is None or e['t']>=transit['t'])),None)
    m['third_commit_to_corridor_entry_s']=(corridor['t']-m['start_ros_s']-m['third_commit_s'] if corridor and m['third_commit_s'] is not None else None)
    m['fast_corridor_transit_s']=corridor['t']-transit['t'] if corridor and transit else None
    fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True)
    axes[0].plot(t-m['start_ros_s'],v,lw=.6,label='Actual horizontal speed')
    axes[0].axhline(1.2,ls=':',color='gray',label='Planner upper limit (not commanded cruise speed)')
    axes[0].set(ylabel='Speed (m/s)');axes[0].legend(fontsize=8)
    if transitions:
        xs=[e['t']-m['start_ros_s'] for e in transitions]+[m['observed_end_ros_s']-m['start_ros_s']]
        ys=[e['lead_m'] for e in transitions]+[transitions[-1]['lead_m']]
        axes[1].step(xs,ys,where='post')
        for e in (transit,corridor):
            if e:
                for ax in axes:ax.axvline(e['t']-m['start_ros_s'],ls='--',alpha=.7)
                axes[1].annotate(e['phase'],(e['t']-m['start_ros_s'],e['lead_m']),rotation=30,fontsize=8)
    axes[1].set(xlabel='Simulation seconds after first mission command',ylabel='Following distance (m)')
    for ax in axes:ax.grid(alpha=.2)
    fig.suptitle(f"Seed {item['seed']} {item['label']} | actual speed and phase limits")
    fig.tight_layout();fig.savefig(dest/'speed_profile.png',dpi=160);plt.close(fig)
    params=base.yaml.safe_load((run/'rosparams.yaml').read_text())
    offset=params['competition_key_recorder']['truth_world_offset'][2]
    fig=plt.figure(figsize=(10,7));ax=fig.add_subplot(111,projection='3d')
    ax.plot(a[:,1],a[:,2],a[:,3]+offset,lw=.8)
    ax.set(xlabel='X (m)',ylabel='Y (m)',zlabel='True FC AGL (m)',title=f"Seed {item['seed']} {item['label']} full 3D flight")
    fig.tight_layout();fig.savefig(dest/'route_3d.png',dpi=160);plt.close(fig)
    (dest/'metrics.json').write_text(json.dumps(m,indent=2))
    return m

def main():
    p=argparse.ArgumentParser();p.add_argument('runs',type=Path);p.add_argument('--out',type=Path,default=HERE);args=p.parse_args()
    args.out.mkdir(parents=True,exist_ok=True)
    items=json.loads(args.runs.read_text())
    metrics=[analyze(item,args.out) for item in items]
    (args.out/'summary.json').write_text(json.dumps(metrics,indent=2))
    print(json.dumps([{k:m.get(k) for k in ('seed','label','status','third_commit_s','completed_mission_s','collisions','third_commit_to_corridor_entry_s','following_speed_stats')} for m in metrics],indent=2))

if __name__=='__main__':main()

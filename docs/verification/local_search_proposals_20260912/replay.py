#!/usr/bin/env python3
"""Offline native-probe fixtures + historical proposal generation; no ROS startup."""
from collections import Counter
import json
from pathlib import Path
import subprocess
import sys
import time
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,BoundaryNorm
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'vision_ws/src/uav_coverage_memory/src'))
from uav_coverage_memory.memory import Config
from uav_coverage_memory.queries import Snapshot,MissionLedger
from uav_coverage_memory.local_proposals import ProposalConfig,Context,LocalProposer
RUNS=['coverage_seed32_reference070_20260911_174716',
      'coverage_seed32_reference080_20260911_182732',
      'coverage_seed34_reference080_wait20_20260911_190504']
COLORS=['#eeeeee','#d5bcdf','#efbb69','#565b66','#92d5e4','#2d927c']


def draw(ax,snapshot,batch,selected=None,points=None):
    c=snapshot.config
    ax.imshow(snapshot.states,origin='lower',extent=[c.min_x,c.max_x,c.min_y,c.max_y],
              cmap=ListedColormap(COLORS),norm=BoundaryNorm([-1,10,30,50,70,90,101],6),alpha=.8)
    if points is not None:
        # Display thinning only; these raw points do NOT decide clearance.
        p=points[points[:,2]>.18][::8]
        ax.scatter(p[:,0],p[:,1],s=.5,c='#404040',alpha=.3)
    for candidate in batch.candidates:
        p=np.asarray([batch.context.pose,candidate.entry,candidate.exit])
        ax.plot(p[:,0],p[:,1],color='#946f2d',lw=.9,alpha=.7)
    if selected:
        p=np.asarray([batch.context.pose,selected['entry'],selected['exit']])
        ax.plot(p[:,0],p[:,1],color='#ce3a35',lw=2.5,label='Selected fixture hint')
    ax.scatter(*batch.context.pose[:2],s=65,c='#1564a1',marker='o',label='Estimated pose')
    ax.scatter(*batch.context.nominal_goal[:2],s=80,c='#b52323',marker='x',label='Retained nominal goal')
    ax.set_xlim(c.min_x,c.max_x);ax.set_ylim(c.min_y,c.max_y);ax.set_aspect('equal')
    ax.set_xlabel('Mission x (m)');ax.set_ylabel('Mission y (m)')


def fixture_probe(batch,blocked_south=False):
    binary=ROOT/'patrol_uav_ws-patrol_planner/devel/lib/plan_env/local_segment_probe_cli'
    origin=np.array([-5.,-1.,0.]);resolution=.1
    occupied=[]
    for x in range(100):
        for y in range(90):
            p=origin[:2]+(np.array([x,y])+.5)*resolution
            if np.linalg.norm(p-[-1.,2.])<.55 or (blocked_south and p[1]<1.65):
                for z in range(3,20):occupied.append((x,y,z))
    tokens=[resolution,*origin,-4.8,-.5,.25,4.8,7.4,2.,10.,10.2,len(occupied)]
    for p in occupied:tokens.extend(p)
    tokens.append(len(batch.segments))
    for a,b in batch.segments:tokens.extend((*a,*b))
    output=subprocess.check_output([str(binary)],input=' '.join(map(str,tokens)),text=True).splitlines()
    accepted,reason,checked=output[0].split()
    return dict(accepted=bool(int(accepted)),reason=reason,checked_voxels=int(checked),
        request_id=batch.request_id,frame_id='map',stamp=10.2,map_stamp=10.,map_revision=1,
        planner_session='OFFLINE_SYNTHETIC_GRID',known_free_proven=False,requires_trajectory_validation=True,
        status=[int(x) for x in output[1].split()])


def fixtures():
    config=Config(-5,5,-1,8,.1,0,'map');states=np.zeros((90,100),np.int8)
    states[25:]=100  # y >= 1.5 already observed, southern band remains unseen
    snapshot=Snapshot(config,states,10.,'fixture',2,states==100)
    context=Context('fixture',1,'SEARCH','map',(-3.,2.,1.2),10.,(4.,2.,1.2),50.)
    results={}
    fig,axes=plt.subplots(1,2,figsize=(11,6))
    for ax,blocked in zip(axes,[False,True]):
        proposer=LocalProposer(ProposalConfig((-4,4,0,7)))
        batch=proposer.prepare(snapshot,context,10.1)
        reply=fixture_probe(batch,blocked)
        result=proposer.finish(batch,reply,context,2,10.2)
        results['south_blocked' if blocked else 'south_clear']=dict(probe=reply,result=result)
        draw(ax,snapshot,batch,result['selected'])
        circle=plt.Circle((-1,2),.55,color='#333333',alpha=.7);ax.add_patch(circle)
        if blocked:ax.axhspan(-.5,1.65,color='#333333',alpha=.35)
        ax.set_title('South blocked: no usable new-view hint' if blocked else 'North seen: prefer clear southern band')
        ax.legend(loc='upper right',fontsize=8)
    assert results['south_clear']['result']['accepted']
    assert results['south_clear']['result']['selected']['entry'][1]<2.
    assert not results['south_blocked']['result']['accepted']
    fig.suptitle('Synthetic inflated-grid fixtures | same native precheck as the planner service')
    fig.text(.5,.02,'Green: historical visibility estimate. These are offline fixtures, not flight trajectories or measured time savings.',ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.05,1,.95));fig.savefig(HERE/'native_fixtures.png',dpi=150);plt.close(fig)
    (HERE/'fixtures.json').write_text(json.dumps(results,indent=2)+'\n')


def history():
    report={};examples=[];all_rows=[]
    for run in RUNS:
        directory=ROOT/'logs'/run
        runtime=yaml.safe_load((directory/'scenario_inputs/runtime.yaml').read_text())['search']
        bounds=tuple(runtime[k] for k in ['min_x','max_x','min_y','max_y'])
        proposer=LocalProposer(ProposalConfig(bounds));ledger=MissionLedger()
        events=[json.loads(line) for line in (directory/'key_events.jsonl').open()]
        decisions=[r for r in events if r['kind']=='decision']
        pose=np.genfromtxt(directory/'mavros_pose.csv',delimiter=',',names=True)
        reasons=Counter();timings=[];maximum=0;chosen=None
        for path in sorted((directory/'coverage').glob('[0-9]*.json')):
            j=json.loads(path.read_text());t=j['result']['image_stamp']
            applicable=[r for r in decisions if r['ros_sec']<=t]
            if not applicable:reasons['no_recorded_decision']+=1;continue
            decision=applicable[-1]['data']
            with np.load(path.with_suffix('.npz')) as arrays:states=arrays['state_grid']
            raw=Snapshot(Config(**j['config']),states,t,j['result']['epoch'],j['result']['generation'])
            if not raw.epoch:continue
            snapshot=ledger.update(raw,t)
            if decision['command'] not in [0,3]:reasons['not_search_or_resume']+=1;continue
            index=int(np.searchsorted(pose['t'],t,side='right'))-1
            if index<0:reasons['pose_missing']+=1;continue
            row=pose[index];goal=decision['goal']['pose']['position']
            context=Context(decision['mission_id'],decision['decision_seq'],'SEARCH' if decision['command']==0 else 'RESUME',
                decision['goal']['header']['frame_id'],tuple(float(row[k]) for k in ['x','y','z']),float(row['t']),
                tuple(goal[k] for k in ['x','y','z']),decision['deadline']['stamp_ns']/1e9)
            start=time.perf_counter()
            try:batch=proposer.prepare(snapshot,context,t)
            except ValueError as e:reasons[str(e)]+=1;continue
            timings.append((time.perf_counter()-start)*1000);maximum=max(maximum,len(batch.candidates))
            reasons['proposal_generated_clearance_unavailable']+=1
            best=max(batch.candidates,key=lambda c:c.observation['observation_score'])
            all_rows.append(dict(run=run,sample=path.stem,stamp=t,decision_seq=context.decision_seq,
                candidate_count=len(batch.candidates),highest_observation_score_name=best.name,
                geometry_precheck='NOT_RECORDED',selected_for_flight=None))
            if chosen is None or best.observation['potential_observation_area_m2']>chosen[0]:
                chosen=(best.observation['potential_observation_area_m2'],snapshot,batch,path)
        report[run]=dict(reasons=dict(reasons),max_candidates=maximum,
            generation_ms_p50=float(np.median(timings)) if timings else None,
            generation_ms_p95=float(np.percentile(timings,95)) if timings else None,
            geometry_precheck='NOT_RECORDED_IN_HISTORICAL_RUNS',flight_savings_measured=False)
        if chosen:examples.append((run,*chosen[1:]))
    fig,axes=plt.subplots(1,len(examples),figsize=(15,6))
    for ax,(run,snapshot,batch,path) in zip(np.atleast_1d(axes),examples):
        with np.load(path.with_suffix('.npz')) as arrays:points=arrays['map_points']
        draw(ax,snapshot,batch,points=points)
        ax.set_title(run.replace('coverage_','').split('_20260911')[0]+'\nsample '+path.stem,fontsize=10)
    fig.suptitle('Historical observed cells and bounded local candidates | no recorded planner SDF clearance')
    fig.text(.5,.02,'Brown lines are proposed geometry only. Grey dots are recorded occupied points, not the planner inflated grid.',ha='center',fontsize=10)
    fig.tight_layout(rect=(0,.05,1,.95));fig.savefig(HERE/'historical_candidates.png',dpi=150);plt.close(fig)
    (HERE/'history.json').write_text(json.dumps(report,indent=2)+'\n')
    (HERE/'candidate_index.json').write_text(json.dumps(all_rows,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    fixtures();history()

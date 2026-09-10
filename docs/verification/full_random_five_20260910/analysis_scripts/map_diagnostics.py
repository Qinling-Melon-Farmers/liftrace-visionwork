from pathlib import Path
import json,sys,shutil,collections
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=next(p for p in Path(__file__).resolve().parents if (p/'AGENTS.md').exists() and (p/'patrol_uav_ws-patrol_planner').is_dir())
O=R/'logs/_artifacts/full_random_five_20260910';D=R/'docs/verification/full_random_five_20260910'
sys.path.insert(0,str(Path(__file__).resolve().parent));from analyze import scene,walls
out=[]
for seed in [33,34]:
    row=json.loads((D/('seed_%02d'%seed)/'metrics.json').read_text());run=Path(row['run_dir']);snapshots=[]
    for p in sorted(run.glob('local_map_failure_*.json')):
        data=json.loads(p.read_text());goal=np.array(data['goal']);summary={'file':p.name,'ros_sec':data['ros_sec'],'goal':data['goal'],'clouds':{}}
        fig,ax=plt.subplots(1,2,figsize=(13,5.5));scene(ax[0],run);ax[0].scatter(goal[0],goal[1],marker='X',s=90,c='#a50026',zorder=8,label='Requested goal')
        for name,color in [('inflated_map','#e78069'),('static_map','#367da6')]:
            cloud=data['clouds'].get(name,{});points=np.array(cloud.get('points',[])).reshape(-1,3)
            summary['clouds'][name]={'n':len(points),'stamp':cloud.get('stamp'),'error':cloud.get('error'),'nearest_goal_3d_m':float(np.linalg.norm(points-goal,axis=1).min()) if len(points) else None}
            if len(points):
                band=points[abs(points[:,2]-goal[2])<.026]
                ax[0].scatter(band[:,0],band[:,1],s=4,c=color,alpha=.45,label=name+' at goal-height slice')
                slab=points[abs(points[:,1]-goal[1])<.20];ax[1].scatter(slab[:,0],slab[:,2],s=5,c=color,alpha=.45,label=name)
        ax[1].scatter(goal[0],goal[2],c='#a50026',s=90,marker='X',zorder=8,label='Requested goal');ax[1].axhline(-.22,c='#555',ls='--',lw=.8,label='Nominal floor z')
        ax[1].set(xlabel='X (m)',ylabel='Local z (m)',title='Local XZ slice: |Y - goal Y| < 0.20m');ax[0].set_title('Recorded map around requested goal');ax[0].legend(fontsize=7);ax[1].legend(fontsize=7);ax[1].grid(alpha=.15)
        fig.suptitle('Seed %d | %s | ROS %.3fs | goal %s'%(seed,p.stem,data['ros_sec'],data['goal']),fontsize=11);fig.tight_layout(rect=(0,0,1,.95));name=p.stem+'.png';fig.savefig(D/('seed_%02d'%seed)/name,dpi=140);plt.close(fig)
        copied=D/('seed_%02d'%seed)/p.name
        if copied.exists():
            assert copied.resolve().is_relative_to(D.resolve())
            copied.unlink()  # Only the report copy; full source cloud stays in the run directory.
        summary['raw_local_path']=str(p)
        (D/('seed_%02d'%seed)/(p.stem+'_summary.json')).write_text(json.dumps(summary,indent=2))
        snapshots.append(summary)
    ev=[json.loads(l) for l in (run/'key_events.jsonl').read_text().splitlines()]
    goals=collections.defaultdict(list)
    for e in ev:
        if e['kind']=='planner':goals[e['data']['goal_seq']].append(e)
    plan=[]
    for k,items in goals.items():
        fails=[x for x in items if x['data'].get('reason')=='new_trajectory_attempt_failed']
        if not fails:continue
        first=items[0];plan.append({'goal_seq':k,'start':first['ros_sec'],'last':items[-1]['ros_sec'],'requested_goal':first['data'].get('requested_goal',{}).get('pose',{}).get('position'),'failed_attempts':len(fails),'first_failure':fails[0]['ros_sec']})
    result={'seed':seed,'snapshots':snapshots,'failed_planner_goals':plan};out.append(result);print(json.dumps(result,indent=2))
(D/'map_diagnostics.json').write_text(json.dumps(out,indent=2))

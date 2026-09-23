"""Read-only paired analysis of the authorized 34/38/40 SITL batch."""
from pathlib import Path
import importlib.util,json,collections
import numpy as np,yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

D=Path(__file__).resolve().parent
R=D.parents[2]
OLD=D.parent/'failed_three_20260923'
HISTORY=D.parent/'history_31_40_20260920'

def events(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line]

def pose_near(path,at):
    import csv
    with path.open() as handle:
        rows=list(csv.DictReader(handle))
    row=min(rows,key=lambda r:abs(float(r['t'])-at))
    return {k:float(row[k]) for k in ('t','x','y','z','qx','qy','qz','qw')}

def main():
    batch=json.loads((R/'logs/near_wall_degrade_20260923_batch/matrix.json').read_text())
    assert batch['status']=='COMPLETE' and [r['seed'] for r in batch['results']]==[34,38,40]
    spec=importlib.util.spec_from_file_location('history_analysis',HISTORY/'analyze_current.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.D=D
    old=json.loads((OLD/'selected_matrix.json').read_text())
    old_by_seed={r['seed']:r for r in old['results']}
    rows=[];metrics=[]
    fig,axes=plt.subplots(1,3,figsize=(18,6))
    for ax,row in zip(axes,batch['results']):
        seed=row['seed'];run=Path(row['run']);prev=Path(old_by_seed[seed]['run'])
        manifest=yaml.safe_load((run/'manifest.yaml').read_text())
        assert manifest['git_head']==batch['source']
        assert manifest['resolved_uav_mission']==str(R/'patrol_uav_ws-patrol_planner/src/uav_mission')
        assert manifest['resolved_uav_vision']==str(R/'vision_ws/src/uav_vision')
        truth=yaml.safe_load((run/'random_field_truth.yaml').read_text())
        prior_truth=yaml.safe_load((prev/'random_field_truth.yaml').read_text())
        target_error=max(float(np.hypot(t['world_x']-next(p['world_x'] for p in prior_truth['targets'] if p['class']==t['class']),t['world_y']-next(p['world_y'] for p in prior_truth['targets'] if p['class']==t['class']))) for t in truth['targets'])
        assert target_error<1e-4,(seed,target_error)
        scene=HISTORY/f'seed_{seed}/field.world'
        item=dict(seed=seed,label='rerun',run=str(run),world=str(scene),source=batch['source'])
        m=mod.one(item);metrics.append(m)
        high=events(run/'high_view_full_events.jsonl')[-1]['status']
        stages=collections.Counter(e.get('stage') for e in high.get('events',[]))
        contact=json.loads((run/'gazebo_contact_status.json').read_text())
        first=(contact['events'][0] if contact.get('events') else None)
        contact_pose=({name:pose_near(run/(name+'.csv'),first['ros_stamp'])
                       for name in ('truth_pose','mavros_pose','mavros_setpoint')}
                      if first else None)
        rows.append(dict(seed=seed,old_status=old_by_seed[seed]['status'],old_drops=old_by_seed[seed]['drops'],status=row['status'],drops=row['drops'],reason=row['reason'],third_commit_s=m['third_commit_s'],completed_s=m['completed_mission_s'],first_contact=first,contact_pose=contact_pose,stage_counts=dict(stages),degraded_from=high.get('degraded_from'),unreachable_classes=high.get('unreachable_classes'),revisit_events=[e for e in high.get('events',[]) if e.get('stage') in ('NEAR_WALL_BOUNDED_APPROACH','DELIVERY_POINT_UNREACHABLE','NEAR_WALL_TARGET_UNREACHABLE','LOWER_WEIGHT_HINT_SELECTED','LOW_COVERAGE_HANDOFF','LOCAL_WALL_VERIFY')],target_error_m=target_error,run=str(run)))
        mod.scene(ax,scene,truth)
        for path,label,style,color in [(prev,'previous','--','gray'),(run,'current','-','tab:blue')]:
            pose=mod.prior.base.csv4(path/'truth_pose.csv')
            ax.plot(pose[:,1],pose[:,2],style,color=color,lw=.9,label=label)
        ax.set_title(f'Seed {seed}: {row["drops"]} mock drops / {row["reason"]}')
        ax.legend(fontsize=8)
        print(f'Seed {seed}: metrics and plots ready',flush=True)
    fig.tight_layout();fig.savefig(D/'paired_paths.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(12,4))
    xs=np.arange(3)
    for ax,key,title in [(axes[0],'drops','Committed mock drops'),(axes[1],'third_commit_s','Third drop time if reached')]:
        for i,r in enumerate(rows):
            value=r[key]
            if value is not None:ax.bar(i,value,color='tab:blue')
            else:ax.text(i,.02,'not reached',rotation=90,ha='center')
        ax.set_xticks(xs,[str(r['seed']) for r in rows]);ax.set_title(title);ax.grid(axis='y',alpha=.2)
    fig.tight_layout();fig.savefig(D/'outcomes.png',dpi=150);plt.close(fig)
    (D/'comparison.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n')
    (D/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps([{k:r[k] for k in ('seed','old_drops','drops','reason','third_commit_s','degraded_from','first_contact','contact_pose','revisit_events')} for r in rows],ensure_ascii=False,indent=2))

if __name__=='__main__':main()

"""Explicitly authorized frozen three-seed regression. No retries or tuning."""
from pathlib import Path
import argparse,collections,fcntl,json,os,signal,subprocess,time
D=Path(__file__).resolve().parent;R=D.parents[2]
SEEDS=(40,)

def main():
    p=argparse.ArgumentParser();p.add_argument('--execute-authorized-three',action='store_true');args=p.parse_args()
    if not args.execute_authorized_three:raise SystemExit('Explicit authorization flag required')
    lock=open('/tmp/liftrace_failed_three_20260923.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    batch=R/'logs/failed_three_20260923_seed40_repeat_batch';batch.mkdir(exist_ok=True);statefile=batch/'matrix.json'
    if statefile.exists():raise SystemExit('Existing state: inspect, do not automatically restart')
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=R,text=True).strip()
    state=dict(status='RUNNING',source=head,seeds=list(SEEDS),pid=os.getpid(),results=[],active=None)
    def save():
        tmp=statefile.with_suffix('.tmp');tmp.write_text(json.dumps(state,indent=2));tmp.replace(statefile)
    def interrupted(signum,frame):raise KeyboardInterrupt('signal '+str(signum))
    signal.signal(signal.SIGTERM,interrupted);save()
    for seed in SEEDS:
        if subprocess.check_output(['git','rev-parse','HEAD'],cwd=R,text=True).strip()!=head:
            state['status']='SOURCE_CHANGED';save();raise SystemExit('Source changed during matrix')
        subprocess.run(['bash','top_level_scripts/check_sim_processes.sh'],cwd=R,check=True)
        scene=R/f'docs/verification/history_31_40_20260920/seed_{seed}'
        prefix=f'failedthree_repeat_seed{seed}';before=set((R/'logs').glob(prefix+'_*'))
        cmd=['env','SIM_RUN_AUTHORIZED=1','SIM_NO_RECORD=1','SIM_REQUIRE_GATE=1','SIM_STORAGE_GUARD_PATH=/mnt/f',
             'timeout','--signal=TERM','--kill-after=60s','7200s','bash','top_level_scripts/sim_run.sh',prefix,
             'bash','top_level_scripts/roslaunch_rl_drone.sh',str(D/'replay.launch'),f'scene_dir:={scene}',f'field_seed:={seed}',
             'target_model_path:=/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt']
        env=os.environ.copy();env.pop('SIM_RUN_AUTHORIZED',None)
        env.update(UAV_WS=str(R/'patrol_uav_ws-patrol_planner'),VISION_WS=str(R/'vision_ws'),ASTRA_MODEL_ROOT=str(R/'simulation_assets/models'),GAZEBO_MODEL_PATH=str(R/'vision_ws/src/uav_vision_eval/models')+':'+str(R/'simulation_assets/models'))
        row=dict(seed=seed,status='RUNNING',run=None,command=cmd);state['active']=row;save()
        with (batch/f'seed{seed}.log').open('w') as out:
            child=subprocess.Popen(cmd,cwd=R,env=env,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
            row['pid']=child.pid;save()
            try:
                while child.poll() is None:
                    found=set((R/'logs').glob(prefix+'_*'))-before
                    if found and row['run'] is None:
                        if len(found)!=1:raise RuntimeError('Ambiguous run directory')
                        row['run']=str(found.pop());save()
                    time.sleep(3)
            except BaseException:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=60)
                finally:state['status']='INTERRUPTED';save()
                raise
        row['exit_code']=child.returncode
        found=set((R/'logs').glob(prefix+'_*'))-before
        if row['run'] is None and len(found)==1:row['run']=str(found.pop())
        run=Path(row['run']) if row['run'] else None
        row['cleanup_pass']=bool(run and 'SITL cleanup verification: PASS' in (run/'run.log').read_text(errors='replace'))
        if not run or not (run/'gate_status.json').exists() or not row['cleanup_pass']:
            row['status']='INFRA_STOP';state['status']='INFRA_STOP';save();raise SystemExit('Infrastructure/cleanup failure')
        gate=json.loads((run/'gate_status.json').read_text());metrics=gate.get('metrics',{})
        row.update(status=gate['status'],reason=gate['reason'],failed_checks=gate.get('failed_checks',[]),mission_s=metrics.get('mission_ros_sec'),drops=metrics.get('release_commit_count'))
        failures=[]
        for line in (run/'key_events.jsonl').open():
            try:e=json.loads(line)
            except json.JSONDecodeError:continue
            data=e.get('data',{})
            if e.get('kind')=='result' and data.get('terminal') and data.get('status') in (4,5,6,7):
                failures.append(dict(t=e.get('ros_sec'),decision=data.get('decision_seq'),reason=data.get('reason')))
        row['first_failure_analysis']=dict(terminal=gate['reason'],errors=gate.get('errors',[]),action_failures=failures,scope='Different preselected layout next; never replaces this outcome')
        state['results'].append(row);state['active']=None;save()
        print(f'seed{seed}: {row["status"]}; reason={row["reason"]}; cleanup PASS',flush=True)
        if gate['reason'] in ('startup_wall_timeout','field_status_fail'):
            state['status']='INFRA_STOP';save();raise SystemExit('Startup failure: analyze before another launch')
    state['status']='COMPLETE';save();print('Three finished; no retries',flush=True)

if __name__=='__main__':main()

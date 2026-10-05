"""Run exactly one authorized case; review its outcome before the next case."""
from pathlib import Path
import argparse,fcntl,json,os,signal,subprocess,time
R=Path(__file__).resolve().parents[3]
D=Path(__file__).resolve().parent
CASES=[(38,'resume_off'),(38,'resume_on')]
FROZEN_SOURCE='7a9502049b9448856152036632ef31857ada2582'
FIXED_CASES=[(38,'resume_on_fixed')]
THREADS=dict(OPENCV_FOR_THREADS_NUM='1',OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='2')

def launch_command(seed,variant,scene,model):
    prefix=f'seed38_resume_{variant}_seed{seed}'
    return ['env','SIM_RUN_AUTHORIZED=1','SIM_NO_RECORD=1','SIM_REQUIRE_GATE=1','SIM_STORAGE_GUARD_PATH=/mnt/f',
            'timeout','--signal=TERM','--kill-after=60s','7200s','bash','top_level_scripts/sim_run.sh',prefix,
            'bash','top_level_scripts/roslaunch_rl_drone.sh',str(D/'replay.launch'),f'scene_dir:={scene}',f'field_seed:={seed}',
            'target_model_path:='+str(model.resolve()),'high_agl:=2.16',
            'resume_survey_enabled:='+str(variant!='resume_off').lower(),
            'stop_on_collision:='+str(variant!='resume_on_fixed').lower()]

def main():
    p=argparse.ArgumentParser()
    mode=p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--case',type=int,choices=range(2))
    mode.add_argument('--rerun-fixed',action='store_true',help='One correction run, separate from frozen A/B')
    p.add_argument('--model',type=Path,required=True)
    p.add_argument('--execute-authorized',action='store_true')
    a=p.parse_args()
    if not a.execute_authorized:raise SystemExit('Explicit authorization required')
    if not a.model.is_file():raise SystemExit('Model file missing')
    lock=open('/tmp/liftrace_seed38_resume_20261005.lock','w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    cases=FIXED_CASES if a.rerun_fixed else CASES
    case_index=0 if a.rerun_fixed else a.case
    batch=R/('logs/seed38_resume_20261005_fixed_batch' if a.rerun_fixed else 'logs/seed38_resume_20261005_batch')
    path=batch/'matrix.json'
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=R,text=True).strip()
    if a.rerun_fixed:
        original=R/'logs/seed38_resume_20261005_batch/matrix.json'
        frozen=json.loads(original.read_text())
        if frozen['source']!=FROZEN_SOURCE or frozen['status']!='COMPLETE' or len(frozen['results'])!=2:
            raise SystemExit('Original frozen A/B must remain complete at 7a950204')
        if head==FROZEN_SOURCE or subprocess.run(['git','merge-base','--is-ancestor',FROZEN_SOURCE,head],cwd=R).returncode:
            raise SystemExit('Commit the main fix before running the correction')
        if path.exists():raise SystemExit('Correction matrix already exists; no overwrite or retry')
        from correction_preflight import check_offline
        check_offline(rerun_fixed=True)
    elif head!=FROZEN_SOURCE:
        raise SystemExit('Original A/B source is frozen at 7a950204; use --rerun-fixed')
    state=json.loads(path.read_text()) if path.exists() else dict(source=head,cases=cases,results=[],active=None,status='READY')
    if a.rerun_fixed:state.update(original_matrix=str(original),original_source=FROZEN_SOURCE)
    if state['source']!=head:raise SystemExit('Frozen source changed')
    if state.get('active') or len(state['results'])!=case_index:raise SystemExit('Review existing state; no implicit retries')
    if subprocess.check_output(['git','diff','--name-only'],cwd=R).strip():raise SystemExit('Tracked source has uncommitted edits')
    def save():
        temp=path.with_suffix('.tmp');temp.write_text(json.dumps(state,indent=2));temp.replace(path)
    def interrupt(signum,frame):raise KeyboardInterrupt('signal '+str(signum))
    signal.signal(signal.SIGTERM,interrupt)
    subprocess.run(['bash','top_level_scripts/check_sim_processes.sh'],cwd=R,check=True)
    seed,variant=cases[case_index]
    scene_variant='resume_on' if a.rerun_fixed else variant
    scene=next(x['scene'] for x in json.loads((D/'scenes.json').read_text()) if x['seed']==seed and x['variant']==scene_variant)
    scene=str((R/scene).resolve())
    if not (Path(scene)/'field.world').is_file():raise SystemExit('Scene missing')
    prefix=f'seed38_resume_{variant}_seed{seed}'
    before=set((R/'logs').glob(prefix+'_*'))
    cmd=launch_command(seed,variant,scene,a.model)
    env=os.environ.copy();env.pop('SIM_RUN_AUTHORIZED',None)
    env.update(THREADS)
    env.update(UAV_WS=str(R/'patrol_uav_ws-patrol_planner'),VISION_WS=str(R/'vision_ws'),
               ASTRA_MODEL_ROOT=str(R/'simulation_assets/models'),
               GAZEBO_MODEL_PATH=str(R/'vision_ws/src/uav_vision_eval/models')+':'+str(R/'simulation_assets/models'))
    row=dict(seed=seed,variant=variant,source=head,scene=scene,scene_variant=scene_variant,
             stop_on_collision=not a.rerun_fixed,threads=THREADS.copy(),
             status='RUNNING',run=None,command=cmd,started_wall=time.time())
    batch.mkdir(exist_ok=True)
    state.update(active=row,status='RUNNING');save()
    with (batch/f'{variant}_seed{seed}.log').open('w') as log:
        child=subprocess.Popen(cmd,cwd=R,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        row['pid']=child.pid;save()
        try:
            while child.poll() is None:
                found=set((R/'logs').glob(prefix+'_*'))-before
                if found and row['run'] is None:
                    if len(found)!=1:raise RuntimeError('Ambiguous run')
                    row['run']=str(found.pop());save()
                time.sleep(3)
        except BaseException:
            os.killpg(child.pid,signal.SIGTERM)
            try:child.wait(timeout=65)
            finally:state['status']='INTERRUPTED';save()
            raise
    row['exit_code']=child.returncode;row['finished_wall']=time.time()
    found=set((R/'logs').glob(prefix+'_*'))-before
    if row['run'] is None and len(found)==1:row['run']=str(found.pop())
    run=Path(row['run']) if row['run'] else None
    row['cleanup_pass']=bool(run and 'SITL cleanup verification: PASS' in (run/'run.log').read_text(errors='replace'))
    subprocess.run(['bash','top_level_scripts/check_sim_processes.sh'],cwd=R,check=True)
    if not run or not (run/'gate_status.json').exists() or not row['cleanup_pass']:
        state['status']='INFRA_STOP';save();raise SystemExit('Infrastructure/cleanup failure')
    gate=json.loads((run/'gate_status.json').read_text());metrics=gate.get('metrics',{})
    row.update(status=gate['status'],reason=gate['reason'],failed_checks=gate.get('failed_checks',[]),
               mission_s=metrics.get('mission_ros_sec'),drops=metrics.get('release_commit_count'))
    errors=[]
    for line in (run/'key_events.jsonl').open():
        try:e=json.loads(line)
        except json.JSONDecodeError:continue
        data=e.get('data',{})
        if e.get('kind')=='result' and data.get('terminal') and data.get('status') in (4,5,6,7):
            errors.append(dict(t=e.get('ros_sec'),decision=data.get('decision_seq'),reason=data.get('reason')))
    row['first_failure_analysis']=dict(terminal=gate['reason'],errors=gate.get('errors',[]),action_failures=errors)
    state['results'].append(row);state['active']=None
    state['status']='COMPLETE' if len(state['results'])==len(cases) else 'AWAITING_REVIEW'
    save();print(json.dumps(row,indent=2),flush=True)
if __name__=='__main__':main()

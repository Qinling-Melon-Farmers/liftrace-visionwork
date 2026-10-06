#!/usr/bin/env python3
"""Prepare or run ONE authorized reviewed seed. No implicit execution/retries.
Default is read-only dry-run. Actual execution additionally requires main's
review/build notification and --execute-authorized --reviewed-head <commit>.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import time
from projection_preflight import (ROOT,D,MODEL,SEEDS,scene_for,load_launch,
                                  check_parameters,flatten,check_build)
import yaml

BATCH = ROOT/'logs/drop_precision_20261006_batch'
THREADS = dict(OPENCV_FOR_THREADS_NUM='1',OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='2')


def command_for(seed,model):
    return ['env','SIM_RUN_AUTHORIZED=1','SIM_NO_RECORD=1','SIM_REQUIRE_GATE=1',
        'SIM_STORAGE_GUARD_PATH=/mnt/f','timeout','--signal=TERM','--kill-after=60s','7200s',
        'bash','top_level_scripts/sim_run.sh',f'drop_precision_seed{seed}',
        'bash','top_level_scripts/roslaunch_rl_drone.sh',str(D/'replay.launch'),
        'scene_dir:='+str(scene_for(seed)),'field_seed:='+str(seed),
        'target_model_path:='+str(model.resolve()),'high_agl:=2.16','resume_survey_enabled:=false']


def git(*args):
    return subprocess.check_output(['git',*args],cwd=ROOT,text=True).strip()


def summary_failure(run,gate):
    failures=[]
    for line in (run/'key_events.jsonl').read_text().splitlines():
        event=json.loads(line);data=event.get('data',{})
        if event.get('kind')=='result' and data.get('terminal') and data.get('status') in (4,5,6,7):
            failures.append(dict(ros_s=event.get('ros_sec'),decision=data.get('decision_seq'),reason=data.get('reason')))
    return dict(terminal=gate.get('reason'),errors=gate.get('errors'),action_failures=failures[:10])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',type=int,choices=(0,1),required=True)
    parser.add_argument('--model',type=Path,default=MODEL)
    parser.add_argument('--dry-run',action='store_true')
    parser.add_argument('--execute-authorized',action='store_true')
    parser.add_argument('--reviewed-head',help='Exact commit reviewed/built by main before THIS batch')
    parser.add_argument('--previous-run-reviewed',type=Path,help='Case 1: exact prior run inspected for first failure')
    args=parser.parse_args()
    if args.dry_run and args.execute_authorized:
        parser.error('Dry-run cannot also execute')
    seed=SEEDS[args.case]
    preflight,params=load_launch(seed,args.model)
    cmd=command_for(seed,args.model)
    if not args.execute_authorized:
        print(json.dumps(dict(mode='DRY_RUN',simulation_started=False,preflight=preflight,command=shlex.join(cmd)),indent=2))
        return
    if not args.reviewed_head:
        parser.error('Execution requires main review/build notification and --reviewed-head')
    head=git('rev-parse','HEAD')
    reviewed=git('rev-parse',args.reviewed_head)
    if head!=reviewed:
        raise SystemExit('HEAD is not the reviewed source')
    if git('diff','--name-only') or git('diff','--cached','--name-only'):
        raise SystemExit('Main must commit the reviewed source before execution; runner never commits')
    production_untracked=git('ls-files','--others','--exclude-standard','vision_ws/src','patrol_uav_ws-patrol_planner/src')
    if production_untracked:
        raise SystemExit('Untracked production source must be included in main review/commit')
    build=check_build()
    # Locks/output are created ONLY after the explicit execution guard.
    lock=open('/tmp/liftrace_drop_precision_20261006.lock','w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    subprocess.run(['bash','top_level_scripts/check_sim_processes.sh'],cwd=ROOT,check=True)
    path=BATCH/'matrix.json'
    state=json.loads(path.read_text()) if path.exists() else dict(source=head,cases=[[31,'precision'],[38,'precision']],results=[],active=None,status='READY')
    if state['source']!=head or state.get('active') or len(state['results'])!=args.case or state['status'] not in ('READY','AWAITING_REVIEW'):
        raise SystemExit('Existing batch/source/order prevents overwrite or retry')
    if args.case==1:
        prior=state['results'][0]
        if not prior.get('cleanup_pass') or not args.previous_run_reviewed or args.previous_run_reviewed.resolve()!=Path(prior['run']).resolve():
            raise SystemExit('Inspect the exact seed31 run/first failure before case 1')
    BATCH.mkdir(exist_ok=True)
    def save():
        tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(state,indent=2)+'\n');tmp.replace(path)
    prefix=f'drop_precision_seed{seed}'
    before=set((ROOT/'logs').glob(prefix+'_*'))
    env=os.environ.copy();env.pop('SIM_RUN_AUTHORIZED',None)
    env.update(THREADS)
    env.update(UAV_WS=str(ROOT/'patrol_uav_ws-patrol_planner'),VISION_WS=str(ROOT/'vision_ws'),
        ASTRA_MODEL_ROOT=str(ROOT/'simulation_assets/models'),
        GAZEBO_MODEL_PATH=str(ROOT/'vision_ws/src/uav_vision_eval/models')+':'+str(ROOT/'simulation_assets/models'))
    row=dict(seed=seed,variant='precision',source=head,scene=str(scene_for(seed)),
        status='RUNNING',run=None,command=cmd,preflight=preflight,build=build,threads=THREADS,
        circle_quality_ordered_nms=False,stop_on_collision=False,started_wall=time.time(),runtime_projection_verified=False)
    state.update(status='RUNNING',active=row);save()
    def interrupt(signum,frame):
        raise KeyboardInterrupt('signal '+str(signum))
    signal.signal(signal.SIGTERM,interrupt)
    signal.signal(signal.SIGHUP,interrupt)
    child=None
    try:
        with (BATCH/f'precision_seed{seed}.log').open('x') as output:
            child=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
            row['pid']=child.pid;save()
            runtime_signature=None
            while child.poll() is None:
                found=set((ROOT/'logs').glob(prefix+'_*'))-before
                if found and row['run'] is None:
                    if len(found)!=1:
                        raise RuntimeError('Ambiguous new run')
                    run=found.pop();row['run']=str(run)
                    (run/'projection_preflight.json').write_text(json.dumps(preflight,indent=2)+'\n')
                    (run/'effective_parameters_prelaunch.json').write_text(json.dumps(params,indent=2)+'\n')
                    save()
                if row['run'] and not row['runtime_projection_verified']:
                    runtime=Path(row['run'])/'rosparams.yaml'
                    if runtime.is_file():
                        signature=(runtime.stat().st_size,runtime.stat().st_mtime_ns)
                        if signature!=runtime_signature:
                            runtime_signature=signature
                            time.sleep(2)
                            continue
                        # Recorder writes a snapshot; only parse once its YAML is
                        # complete and contains the new required namespace.
                        try:
                            document=yaml.safe_load(runtime.read_text())
                            current=flatten(document) if isinstance(document,dict) else {}
                        except yaml.YAMLError:
                            current={}
                        if '/drop_aligner/exact_drop_projection' in current:
                            check_parameters(current)
                            row['runtime_projection_verified']=True;save()
                time.sleep(2)
            row['exit_code']=child.returncode
    except BaseException as error:
        if child is not None and child.poll() is None:
            os.killpg(child.pid,signal.SIGTERM)
            try:
                child.wait(timeout=60)
            except subprocess.TimeoutExpired:
                # The owned sim wrapper must clean up; use the required helper
                # if its graceful trap could not complete, never launch again.
                subprocess.run(['bash','top_level_scripts/stop_toudi3_sim.sh'],cwd=ROOT)
                child.wait(timeout=10)
        row.update(status='INTERRUPTED',error=type(error).__name__+': '+str(error),finished_wall=time.time())
        state.update(status='INTERRUPTED',active=row);save()
        subprocess.run(['bash','top_level_scripts/check_sim_processes.sh'],cwd=ROOT,check=True)
        raise
    row['finished_wall']=time.time()
    found=set((ROOT/'logs').glob(prefix+'_*'))-before
    if row['run'] is None and len(found)==1:
        row['run']=str(found.pop())
    run=Path(row['run']) if row['run'] else None
    process_check=subprocess.run(['bash','top_level_scripts/check_sim_processes.sh'],cwd=ROOT)
    row['cleanup_pass']=bool(run and process_check.returncode==0 and
        'SITL cleanup verification: PASS' in (run/'run.log').read_text(errors='replace'))
    if not run or not (run/'gate_status.json').is_file() or not row['cleanup_pass']:
        state.update(status='INFRA_STOP',active=row);save()
        raise SystemExit('Infrastructure/cleanup failed; no next run')
    (run/'projection_preflight.json').write_text(json.dumps(preflight,indent=2)+'\n')
    (run/'effective_parameters_prelaunch.json').write_text(json.dumps(params,indent=2)+'\n')
    runtime=check_parameters(flatten(yaml.safe_load((run/'rosparams.yaml').read_text())))
    row['runtime_projection_verified']=True
    gate=json.loads((run/'gate_status.json').read_text())
    row.update(status=gate['status'],reason=gate.get('reason'),failed_checks=gate.get('failed_checks'),
        runtime_projection=runtime,first_failure_analysis=summary_failure(run,gate))
    row['source_unchanged']=git('rev-parse','HEAD')==head and not git('diff','--name-only') and not git('diff','--cached','--name-only')
    state['results'].append(row);state['active']=None
    state['status']='COMPLETE' if args.case==1 else 'AWAITING_REVIEW'
    if not row['source_unchanged']:
        state['status']='SOURCE_CHANGED_STOP'
    save()
    print(json.dumps(dict(seed=seed,run=str(run),status=row['status'],reason=row['reason'],
        cleanup_pass=row['cleanup_pass'],runtime_projection_verified=True,batch=state['status']),indent=2))

if __name__=='__main__':
    main()
#!/usr/bin/env python3
"""One explicitly authorized exact-ON run; cases 0=seed31, 1=seed38 only.

Default dry-run never creates locks/logs or starts ROS. The directory's legacy
AB label does not authorize OFF cases: this is historical comparison only.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import time
from preflight import ROOT,D,MODEL,SEEDS,scene_for,load_launch,check_parameters,flatten,check_build
import yaml

THREADS=dict(OPENCV_FOR_THREADS_NUM='1',OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='2')


def git(*args):
    return subprocess.check_output(['git',*args],cwd=ROOT,text=True).strip()


def stop_owned(child,prefix):
    """TERM the owned sim_run wrapper, preserving its ordinary cleanup trap."""
    if child.poll() is not None:return
    processes={}
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            args=[x.decode(errors='replace') for x in (p/'cmdline').read_bytes().split(b'\0') if x]
            parent=int(next(x.split(':',1)[1] for x in (p/'status').read_text().splitlines() if x.startswith('PPid:')))
            processes[int(p.name)]=(parent,args)
        except (OSError,ValueError,StopIteration):pass
    owned={child.pid}
    for _ in range(20):owned.update(pid for pid,(parent,args) in processes.items() if parent in owned)
    wrappers=[pid for pid in owned for parent,args in [processes.get(pid,(0,[]))]
              if args and Path(args[0]).name=='bash' and any(x.endswith('/sim_run.sh') and i+1<len(args) and args[i+1]==prefix for i,x in enumerate(args))]
    if len(wrappers)>1:raise RuntimeError('Ambiguous owned wrapper')
    os.kill(wrappers[0] if wrappers else child.pid,signal.SIGTERM)
    try:child.wait(timeout=60)
    except subprocess.TimeoutExpired:
        subprocess.run(['bash','top_level_scripts/stop_toudi3_sim.sh'],cwd=ROOT)
        child.wait(timeout=15)


def cleanup(run):
    attempts=[]
    def call(script):
        value=subprocess.run(['bash','top_level_scripts/'+script],cwd=ROOT,text=True,capture_output=True)
        attempts.append(dict(script=script,exit_code=value.returncode,output=value.stdout+value.stderr,wall=time.time()))
        return value.returncode==0
    raw=(run/'run.log').read_text(errors='replace') if run and (run/'run.log').exists() else ''
    original='PASS' if 'SITL cleanup verification: PASS' in raw else ('FAIL' if 'SITL cleanup verification: FAIL' in raw else 'MISSING')
    raw_lines=[l for l in raw.splitlines() if 'cleanup verification:' in l or 'Local SITL cleanup failed:' in l]
    cleared=call('check_sim_processes.sh')
    if not cleared or original!='PASS':
        call('stop_toudi3_sim.sh')
        for _ in range(10):
            cleared=call('check_sim_processes.sh')
            if cleared:break
            time.sleep(1)
    record=dict(wrapper_original=original,wrapper_original_lines=raw_lines,final_zero_residual=cleared,attempts=attempts,
                note='Original wrapper FAIL is preserved; later zero check is supplementary, never relabeled as original PASS.')
    if run:(run/'cleanup_verification.json').write_text(json.dumps(record,indent=2)+'\n')
    return record


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case',type=int,choices=(0,1),required=True)
    p.add_argument('--camera-dir',type=Path,required=True)
    p.add_argument('--model',type=Path,default=MODEL)
    p.add_argument('--batch-name',default='drop_precision_20261007_history_batch')
    p.add_argument('--dry-run',action='store_true')
    p.add_argument('--execute-authorized',action='store_true')
    p.add_argument('--reviewed-head')
    p.add_argument('--previous-run-reviewed',type=Path)
    a=p.parse_args()
    if not re.fullmatch(r'drop_precision_20261007_[a-z0-9][a-z0-9_-]{0,47}',a.batch_name):p.error('Invalid task-scoped batch name')
    if a.dry_run and a.execute_authorized:p.error('Dry-run cannot execute')
    seed=SEEDS[a.case];prefix=f'drop_precision_new_seed{seed}'
    preflight,params=load_launch(seed,a.model,a.camera_dir)
    camera_dir=a.camera_dir.resolve();batch=ROOT/'logs'/a.batch_name
    cmd=['env','SIM_RUN_AUTHORIZED=1','SIM_NO_RECORD=1','SIM_REQUIRE_GATE=1','SIM_STORAGE_GUARD_PATH=/mnt/f',
         'timeout','--signal=TERM','--kill-after=60s','7200s','bash','top_level_scripts/sim_run.sh',prefix,
         'bash','top_level_scripts/roslaunch_rl_drone.sh',str(D/'replay.launch'),
         'scene_dir:='+str(scene_for(seed)),'field_seed:='+str(seed),'target_model_path:='+str(a.model.resolve()),
         'high_agl:=2.16','resume_survey_enabled:=false','vehicle_sdf:='+str(camera_dir/'model.sdf'),
         'camera_contract:='+str(camera_dir/'camera_contract.json')]
    if not a.execute_authorized:
        print(json.dumps(dict(mode='DRY_RUN',simulation_started=False,cases=[31,38],variant='EXACT_ON_ONLY',
            comparison='historical reference; source and camera both change',batch=str(batch),preflight=preflight,command=shlex.join(cmd)),indent=2));return
    if not a.reviewed_head:p.error('Main review/build notification and pinned reviewed HEAD are required')
    head=git('rev-parse','HEAD')
    if head!=git('rev-parse',a.reviewed_head):raise SystemExit('HEAD is not reviewed')
    if git('diff','--name-only') or git('diff','--cached','--name-only'):raise SystemExit('Reviewed tree must be committed first')
    if git('ls-files','--others','--exclude-standard','vision_ws/src','patrol_uav_ws-patrol_planner/src'):
        raise SystemExit('Untracked production input is not reviewed/committed')
    build=check_build()
    lock=open('/tmp/liftrace_drop_precision_20261007.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    subprocess.run(['bash','top_level_scripts/check_sim_processes.sh'],cwd=ROOT,check=True)
    path=batch/'matrix.json'
    state=json.loads(path.read_text()) if path.exists() else dict(source=head,camera_contract=preflight['camera_contract'],
        cases=[[31,'exact_on'],[38,'exact_on']],comparison='HISTORICAL_REFERENCE_ONLY',results=[],active=None,status='READY')
    if state['source']!=head or state['camera_contract']!=preflight['camera_contract'] or state.get('active') or len(state['results'])!=a.case or state['status'] not in ('READY','AWAITING_REVIEW'):
        raise SystemExit('Existing batch/source/camera/order prevents overwrite or retry')
    if a.case:
        prior=state['results'][a.case-1]
        if not prior.get('cleanup_final_pass') or not prior.get('camera_verified') or not prior.get('source_unchanged') or not a.previous_run_reviewed or a.previous_run_reviewed.resolve()!=Path(prior['run']).resolve():
            raise SystemExit('Review the exact prior run; source, camera and zero cleanup must hold')
    batch.mkdir(exist_ok=True)
    def save():
        tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(state,indent=2)+'\n');tmp.replace(path)
    row=dict(seed=seed,variant='exact_on',source=head,status='RUNNING',run=None,command=cmd,preflight=preflight,
             build=build,started_wall=time.time(),camera_verified=False,runtime_projection_verified=False)
    state.update(active=row,status='RUNNING');save()
    before=set((ROOT/'logs').glob(prefix+'_*'));child=None;error=None;run=None
    env=os.environ.copy();env.pop('SIM_RUN_AUTHORIZED',None);env.update(THREADS)
    env.update(UAV_WS=str(ROOT/'patrol_uav_ws-patrol_planner'),VISION_WS=str(ROOT/'vision_ws'),
        ASTRA_MODEL_ROOT=str(ROOT/'simulation_assets/models'),
        GAZEBO_MODEL_PATH=str(ROOT/'vision_ws/src/uav_vision_eval/models')+':'+str(ROOT/'simulation_assets/models'))
    def interrupted(signum,frame):raise KeyboardInterrupt('signal '+str(signum))
    for sig in (signal.SIGTERM,signal.SIGHUP,signal.SIGINT):signal.signal(sig,interrupted)
    try:
        with (batch/f'case{a.case}_seed{seed}_exact_on.log').open('x') as output:
            child=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
            row['pid']=child.pid;save()
            while child.poll() is None:
                found=set((ROOT/'logs').glob(prefix+'_*'))-before
                if found and run is None:
                    if len(found)!=1:raise RuntimeError('Ambiguous owned run directory')
                    run=found.pop();row['run']=str(run)
                    (run/'projection_preflight.json').write_text(json.dumps(preflight,indent=2)+'\n')
                    (run/'effective_parameters_prelaunch.json').write_text(json.dumps(params,indent=2)+'\n');save()
                if run and (run/'camera_check.json').exists():
                    camera=json.loads((run/'camera_check.json').read_text())
                    if camera['status']=='FAIL':raise RuntimeError('Camera mismatch: '+str(camera.get('reason')))
                    if camera['status']=='PASS' and not row['camera_verified']:
                        row['camera_verified']=True;row['runtime_projection_verified']=True;save()
                time.sleep(2)
    except BaseException as exc:
        error=type(exc).__name__+': '+str(exc)
        if child:stop_owned(child,prefix)
    finally:
        if child and child.poll() is None:stop_owned(child,prefix)
        if run is None:
            found=set((ROOT/'logs').glob(prefix+'_*'))-before
            if len(found)==1:run=found.pop();row['run']=str(run)
        record=cleanup(run)
        gate=json.loads((run/'gate_status.json').read_text()) if run and (run/'gate_status.json').exists() else None
        if run and (run/'camera_check.json').exists():
            row['camera_verified']=json.loads((run/'camera_check.json').read_text())['status']=='PASS'
        row.update(exit_code=child.returncode if child else None,finished_wall=time.time(),error=error,
            raw_gate=gate.get('status') if gate else None,gate_file_present=gate is not None,
            reason=gate.get('reason') if gate else 'no_final_gate',cleanup=record,cleanup_final_pass=record['final_zero_residual'],
            source_unchanged=git('rev-parse','HEAD')==head and not git('diff','--name-only') and not git('diff','--cached','--name-only'))
        row['status']=gate['status'] if gate else ('INTERRUPTED_DIAGNOSTIC' if error and error.startswith('KeyboardInterrupt') else 'INCOMPLETE_FAIL')
        state['results'].append(row);state['active']=None
        state['status']='COMPLETE' if a.case==1 else 'AWAITING_REVIEW'
        if not row['cleanup_final_pass'] or not row['source_unchanged'] or not row['camera_verified']:state['status']='BLOCKED'
        save()
    print(json.dumps({k:row[k] for k in ['seed','run','status','raw_gate','reason','camera_verified','cleanup_final_pass','source_unchanged']},indent=2))
    if row['status']!='PASS' or not row['cleanup_final_pass'] or not row['camera_verified'] or not row['source_unchanged']:raise SystemExit(1)


if __name__=='__main__':main()

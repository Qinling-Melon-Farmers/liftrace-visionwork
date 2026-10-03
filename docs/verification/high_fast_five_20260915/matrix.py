"""Explicitly authorized, serial five-run driver. No automatic retries."""
import argparse,json,os,subprocess,time,shutil,fcntl
from pathlib import Path
D=Path(__file__).resolve().parent;R=D.parents[2]
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--execute-authorized-five',action='store_true');args=parser.parse_args()
    if not args.execute_authorized_five:raise SystemExit('Execution requires explicit current-request authorization flag')
    lock=open('/tmp/liftrace_high_fast_five.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    batch=R/'logs/high_fast_five_20260915_batch';batch.mkdir(exist_ok=True)
    state_path=batch/'matrix.json'
    if state_path.exists():raise SystemExit('Existing batch state: inspect/poll it; do not restart automatically')
    source=subprocess.check_output(['git','rev-parse','HEAD'],cwd=R,text=True).strip()
    state=dict(status='RUNNING',pid=os.getpid(),source=source,results=[],active=None)
    def save():
        tmp=state_path.with_suffix('.tmp');tmp.write_text(json.dumps(state,indent=2));tmp.replace(state_path)
    save()
    model='/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt'
    for case in json.loads((D/'cases.json').read_text()):
        seed=case['seed'];prefix=f'high_fast_five_seed{seed}';before=set((R/'logs').glob(prefix+'_*'))
        subprocess.run(['bash','top_level_scripts/check_sim_processes.sh'],cwd=R,check=True,stdout=subprocess.DEVNULL)
        command=['env','SIM_RUN_AUTHORIZED=1','SIM_NO_RECORD=1','SIM_REQUIRE_GATE=1','SIM_STORAGE_GUARD_PATH=/mnt/f',
                 'timeout','--signal=TERM','--kill-after=30s','2850s','bash','top_level_scripts/sim_run.sh',prefix,
                 'bash','top_level_scripts/roslaunch_rl_drone.sh','uav_high_view','fast_comparison.launch',
                 'strategy:=true',f'field_seed:={seed}',f'target_model_path:={model}']
        command += [f'{k}:={case[k]}' for k in ('world','field_config','runtime_config','gate_geometry_config')]
        environment=os.environ.copy();environment.pop('SIM_RUN_AUTHORIZED',None)
        environment['ASTRA_MODEL_ROOT']=str(R/'simulation_assets/models')
        environment['GAZEBO_MODEL_PATH']=str(R/'vision_ws/src/uav_vision_eval/models')+':'+environment['ASTRA_MODEL_ROOT']
        row=dict(seed=seed,status='RUNNING',command=command,run=None);state['active']=row;save()
        with (batch/f'seed_{seed}_wrapper.log').open('w') as log:
            child=subprocess.Popen(command,cwd=R,env=environment,stdout=log,stderr=subprocess.STDOUT)
            row['pid']=child.pid;save()
            while child.poll() is None:
                found=set((R/'logs').glob(prefix+'_*'))-before
                if found and row['run'] is None:
                    assert len(found)==1
                    run=found.pop();row['run']=str(run);dest=run/'scenario_inputs';dest.mkdir(exist_ok=True)
                    for key,name in [('world','field.world'),('field_config','field_config.yaml'),('runtime_config','runtime.yaml'),('gate_geometry_config','gate_geometry.yaml')]:shutil.copyfile(case[key],dest/name)
                    save()
                time.sleep(3)
            row['exit_code']=child.returncode
        if row['run'] is None:
            found=set((R/'logs').glob(prefix+'_*'))-before
            if len(found)==1:row['run']=str(found.pop())
        run=Path(row['run']) if row['run'] else None
        gate_path=run/'gate_status.json' if run else None
        row['cleanup_pass']=bool(run and 'SITL cleanup verification: PASS' in (run/'run.log').read_text(errors='replace'))
        if not gate_path or not gate_path.is_file() or not row['cleanup_pass']:
            row['status']='INFRA_STOP';state['status']='INFRA_STOP';save();raise SystemExit('Infrastructure/cleanup stop; inspect before further runs')
        gate=json.loads(gate_path.read_text());row.update(status=gate['status'],reason=gate['reason'],failed_checks=gate.get('failed_checks',[]),mission_s=gate['metrics'].get('mission_ros_sec'))
        event=run/'high_view_full_events.jsonl'
        if event.is_file():
            last=json.loads(event.read_text().splitlines()[-1])['status'];row['terminal_stage']=last.get('stage');row['failure']=last.get('failure')
        # Each failure is retained with its first terminal reason; the next
        # preselected layout is a different trial, never a replacement retry.
        state['results'].append(row);state['active']=None;save()
        print(f'seed{seed}: {row["status"]}; cleanup PASS',flush=True)
    state['status']='COMPLETE';save();print('Five runs finished; no retries.',flush=True)
if __name__=='__main__':main()

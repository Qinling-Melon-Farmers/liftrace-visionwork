"""One explicitly authorized frozen-seed trial; no retry or seed replacement."""
import argparse,json,os,subprocess,shutil,time
from pathlib import Path
from preflight import D,R,arguments

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--execute-authorized-once',action='store_true')
    if not parser.parse_args().execute_authorized_once:raise SystemExit('Current user authorization required')
    case=json.loads((D/'case.json').read_text())
    assert json.loads((D/'preflight.json').read_text())['status']=='PASS'
    batch=R/'logs/reliability_trial_20260919_batch';batch.mkdir(exist_ok=True)
    statefile=batch/'state.json'
    if statefile.exists():raise SystemExit('Existing attempt found: inspect it, never silently retry')
    subprocess.run(['bash','top_level_scripts/check_sim_processes.sh'],cwd=R,check=True)
    prefix=f'reliability_seed{case["seed"]}'
    before=set((R/'logs').glob(prefix+'_*'))
    command=['env','SIM_RUN_AUTHORIZED=1','SIM_NO_RECORD=1','SIM_REQUIRE_GATE=1',
        'SIM_STORAGE_GUARD_PATH=/mnt/f','timeout','--signal=TERM','--kill-after=30s','2850s',
        'bash','top_level_scripts/sim_run.sh',prefix,'bash','top_level_scripts/roslaunch_rl_drone.sh',
        'uav_high_view','fast_comparison.launch']+arguments(case)
    env=os.environ.copy();env.pop('SIM_RUN_AUTHORIZED',None)
    env['UAV_WS']=str(R/'patrol_uav_ws-patrol_planner');env['VISION_WS']=str(R/'vision_ws')
    env['ASTRA_MODEL_ROOT']=str(R/'simulation_assets/models')
    env['GAZEBO_MODEL_PATH']=str(R/'vision_ws/src/uav_vision_eval/models')+':'+env['ASTRA_MODEL_ROOT']
    state=dict(status='RUNNING',seed=case['seed'],source=subprocess.check_output(['git','rev-parse','HEAD'],cwd=R,text=True).strip(),command=command,run=None)
    def save():statefile.write_text(json.dumps(state,indent=2))
    save()
    with (batch/'wrapper.log').open('w') as log:
        process=subprocess.Popen(command,cwd=R,env=env,stdout=log,stderr=subprocess.STDOUT)
        state['pid']=process.pid;save()
        while process.poll() is None:
            found=set((R/'logs').glob(prefix+'_*'))-before
            if found and state['run'] is None:
                assert len(found)==1
                run=found.pop();state['run']=str(run)
                shutil.copytree(D/f'seed_{case["seed"]}',run/'scenario_inputs')
                shutil.copyfile(D/'selection.json',run/'scenario_inputs/selection.json');save()
            time.sleep(3)
        state['exit_code']=process.returncode
    run=Path(state['run']) if state['run'] else None
    if run and (run/'gate_status.json').exists():
        gate=json.loads((run/'gate_status.json').read_text());state.update(status=gate['status'],reason=gate['reason'])
    else:state.update(status='INFRA_STOP',reason='missing_gate_or_run')
    state['cleanup_pass']=bool(run and 'SITL cleanup verification: PASS' in (run/'run.log').read_text(errors='replace'))
    save();print(json.dumps(state,indent=2))
    return 0 if state['status']=='PASS' and state['cleanup_pass'] else 1

if __name__=='__main__':raise SystemExit(main())

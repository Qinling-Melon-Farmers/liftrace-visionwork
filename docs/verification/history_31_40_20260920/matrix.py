"""One explicitly authorized serial pass over frozen 31-40; no retries."""
import argparse,fcntl,json,os,signal,subprocess,time
from pathlib import Path
D=Path(__file__).resolve().parent;R=D.parents[2]
def main():
    p=argparse.ArgumentParser();p.add_argument('--execute-authorized-ten',action='store_true');a=p.parse_args()
    if not a.execute_authorized_ten:raise SystemExit('Explicit ten-run authorization flag required')
    lock=open('/tmp/liftrace_history_ten_20260920.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    batch=R/'logs/history31_40_20260920_batch';batch.mkdir(exist_ok=True);state_path=batch/'matrix.json'
    if state_path.exists():raise SystemExit('Existing batch state; inspect it rather than relaunching')
    state=dict(status='RUNNING',pid=os.getpid(),source=subprocess.check_output(['git','rev-parse','HEAD'],cwd=R,text=True).strip(),results=[],active=None)
    def save():
        temp=state_path.with_suffix('.tmp');temp.write_text(json.dumps(state,indent=2));temp.replace(state_path)
    save()
    for case in json.loads((D/'cases.json').read_text()):
        seed=case['seed'];prefix=f'history20260920_seed{seed}';before=set((R/'logs').glob(prefix+'_*'))
        subprocess.run(['bash','top_level_scripts/check_sim_processes.sh'],cwd=R,check=True,stdout=subprocess.DEVNULL)
        cmd=['env','SIM_RUN_AUTHORIZED=1','SIM_NO_RECORD=1','SIM_REQUIRE_GATE=1','SIM_STORAGE_GUARD_PATH=/mnt/f','timeout','--signal=TERM','--kill-after=60s','7200s','bash','top_level_scripts/sim_run.sh',prefix,'bash','top_level_scripts/roslaunch_rl_drone.sh','uav_high_view','fov_inner_repair.launch',f'scene_dir:={case["scene_dir"]}',f'field_seed:={seed}','corridor_fast:=true','high_agl:=2.6','target_model_path:=/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt']
        env=os.environ.copy();env.pop('SIM_RUN_AUTHORIZED',None)
        env.update(UAV_WS=str(R/'patrol_uav_ws-patrol_planner'),VISION_WS=str(R/'vision_ws'),ASTRA_MODEL_ROOT=str(R/'simulation_assets/models'),GAZEBO_MODEL_PATH=str(R/'vision_ws/src/uav_vision_eval/models')+':'+str(R/'simulation_assets/models'))
        row=dict(seed=seed,status='RUNNING',run=None,command=cmd);state['active']=row;save()
        with (batch/f'seed_{seed}_wrapper.log').open('w') as out:
            child=subprocess.Popen(cmd,cwd=R,env=env,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
            row['pid']=child.pid;save()
            try:
                while child.poll() is None:
                    found=set((R/'logs').glob(prefix+'_*'))-before
                    if found and row['run'] is None:
                        assert len(found)==1;row['run']=str(found.pop());save()
                    time.sleep(3)
            except BaseException:
                os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=60);state['status']='INTERRUPTED';save();raise
        row['exit_code']=child.returncode
        if row['run'] is None:
            found=set((R/'logs').glob(prefix+'_*'))-before
            if len(found)==1:row['run']=str(found.pop())
        run=Path(row['run']) if row['run'] else None
        row['cleanup_pass']=bool(run and 'SITL cleanup verification: PASS' in (run/'run.log').read_text(errors='replace'))
        if not run or not (run/'gate_status.json').exists() or not row['cleanup_pass']:
            row['status']='INFRA_STOP';state['status']='INFRA_STOP';save();raise SystemExit('Infrastructure/cleanup failure; inspect before continuing')
        gate=json.loads((run/'gate_status.json').read_text());row.update(status=gate['status'],reason=gate['reason'],failed_checks=gate.get('failed_checks',[]),mission_s=gate.get('metrics',{}).get('mission_ros_sec'))
        row['first_failure_analysis']=dict(terminal_reason=gate['reason'],errors=gate.get('errors',[]),scope='Recorded terminal failure, retained; next seed is a different preselected layout, never a replacement retry')
        state['results'].append(row);state['active']=None;save()
        print(f'seed{seed}: {row["status"]}, {row["reason"]}; cleanup PASS',flush=True)
    state['status']='COMPLETE';save();print('Ten runs finished; no retries',flush=True)
if __name__=='__main__':main()

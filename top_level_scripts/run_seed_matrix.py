#!/usr/bin/env python3
"""Sequential explicitly authorized seed runs; each child uses sim_run.sh."""
from pathlib import Path
import argparse,json,os,subprocess,signal,sys,shutil,time
from collect_run_summary import collect
p=argparse.ArgumentParser();p.add_argument('--scene-prefix',default='r2026_matrix');p.add_argument('--project-root',required=True);p.add_argument('--output-dir',required=True);p.add_argument('--pilot-run',required=True);p.add_argument('--seeds',nargs='+',required=True,type=int);p.add_argument('--record-camera-video',action='store_true');p.add_argument('--observe-full-trial',action='store_true');p.add_argument('--scenario-index',type=Path,help='Offline exported scene files indexed by seed; no truth-derived flight goals');a=p.parse_args()
root=Path(a.project_root).resolve();out=Path(a.output_dir).resolve();out.mkdir(parents=True,exist_ok=True)
assert os.environ.get('SIM_RUN_AUTHORIZED')=='1','Explicit authorization is required for this listed batch'
assert len(a.seeds)==len(set(a.seeds)) and all(s>0 for s in a.seeds)
pilot=Path(a.pilot_run);assert json.loads((pilot/'gate_status.json').read_text())['status']=='PASS'
scenarios=json.loads(a.scenario_index.read_text()) if a.scenario_index else {}
scene_files={'world':'field.world','field_config':'field_config.yaml','runtime_config':'runtime.yaml','gate_geometry_config':'gate_geometry.yaml'}
for seed in a.seeds:
    if scenarios:
        item=scenarios[str(seed)]
        assert set(item['args'])==set(scene_files)
        for path in item['args'].values():assert Path(path).is_file(),path
        assert Path(item['scene_json']).is_file()
assert not subprocess.check_output(['git','status','--porcelain'],cwd=root), 'Worktree must be clean'
head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()
state={'status':'RUNNING','pid':os.getpid(),'frozen_head':head,'seeds':a.seeds,'results':[],'current':None}
state['scenario_index']=scenarios
def save():
    tmp=out/'matrix_status.json.tmp';tmp.write_text(json.dumps(state,indent=2));tmp.replace(out/'matrix_status.json')
def interrupt(_sig,_frame):raise KeyboardInterrupt
signal.signal(signal.SIGTERM,interrupt);signal.signal(signal.SIGINT,interrupt)
save()
try:
    for seed in a.seeds:
        assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()==head,'Source changed during the matrix'
        env=os.environ.copy();env['SIM_SCENE']=a.scene_prefix+'_seed%02d'%seed
        state['current']={'seed':seed,'run_dir':None};save();run=None
        with (out/('seed_%02d_console.txt'%seed)).open('w',buffering=1) as log:
            extra=[key+':='+value for key,value in scenarios.get(str(seed),{}).get('args',{}).items()]
            child=subprocess.Popen([str(root/'top_level_scripts/run_competition_sim.sh'),'field_seed:='+str(seed),'record_camera_video:='+str(a.record_camera_video).lower(),'record_overview_video:=false','record_debug:=false','gui:=false','rviz:=false','observe_full_trial:='+str(a.observe_full_trial).lower(),*extra],cwd=root,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
            try:
                for line in child.stdout:
                    log.write(line);print(line.rstrip(),flush=True)
                    if line.startswith('Run dir: ') and run is None:
                        run=Path(line.split(': ',1)[1].strip());state['current']['run_dir']=str(run);save()
                        dest=run/'scenario_inputs';shutil.copytree(pilot/'scenario_inputs',dest,dirs_exist_ok=True)
                        source=json.loads((dest/'source.json').read_text());source['head']=head
                        for name,relative in source['paths'].items():shutil.copy2(root/relative,dest/name)
                        if scenarios:
                            item=scenarios[str(seed)]
                            for arg,path in item['args'].items():
                                name=scene_files[arg];shutil.copy2(path,dest/name)
                                source['paths'][name]=os.path.relpath(path,root)
                            shutil.copy2(item['scene_json'],dest/'scene.json')
                            source['scope']='full_random_targets_trees_doors'
                            source['field_seed']=seed
                        (dest/'source.json').write_text(json.dumps(source,indent=2))
                code=child.wait()
            except BaseException:
                child.terminate();child.wait(timeout=60);raise
        for name in ['roscore','rosmaster','rosout','roslaunch','gzserver','gzclient','px4','mavros_node','rviz']:
            assert subprocess.run(['pgrep','-x',name],stdout=subprocess.DEVNULL).returncode==1,'Residual '+name
        if run is not None and (run/'gate_status.json').exists():
            result=collect(run)
            assert result['seed'] in (None,seed),'Requested seed was not applied'
            result={'seed':seed,'run_dir':str(run),'status':result['status'],'exit_code':code,'checks_passed':result['checks_passed'],'checks_total':result['checks_total'],'mission_ros_sec':result['metrics']['mission_ros_sec'],'releases':result['metrics']['release_commit_count'],'post_route':result['metrics']['post_delivery_return_success_count'],'passages':len(result['metrics']['door_crossings']),'bag_files':result['bag_files'],'cleanup_pass':result['cleanup_pass']}
        else:result={'seed':seed,'run_dir':str(run) if run else None,'status':'STARTUP_ERROR','exit_code':code}
        state['results'].append(result);state['current']=None;save();print('RESULT '+json.dumps(result),flush=True)
    signatures=[]
    for item in state['results']:
        path=Path(item['run_dir'])/'summary.json' if item['run_dir'] else None
        if path and path.exists():
            layout=json.loads(path.read_text())['layout'].get('targets',[])
            if layout:signatures.append(tuple((x['class'],x['x'],x['y'],x['yaw']) for x in layout))
    state['distinct_layouts']=len(set(signatures))
    state['status']='COMPLETE';save()
except KeyboardInterrupt:
    state['status']='INTERRUPTED';save();raise SystemExit(130)
except BaseException as exc:
    state['status']='ERROR';state['error']=str(exc);save();raise

"""Expand launch parameters only. No ROS nodes or simulator are started."""
import json,subprocess,sys
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[3];D=Path(__file__).resolve().parent
model='/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt'
cases=json.loads((D/'scenarios.json').read_text())
old=ROOT/'docs/verification/high_view_render_20260913/seed_32'
cases['pilot']=dict(seed=32,world=str(old/'field.world'),field_config=str(old/'field_config.yaml'),runtime_config=str(D/'pilot/runtime.yaml'),gate_geometry_config=str(ROOT/'docs/verification/high_view_full_20260914/seed_32/gate_geometry.yaml'))
result=[]
print('Validation interpreter:',sys.executable,sys.version.split()[0],flush=True)
for name,case in cases.items():
    pair=[]
    for strategy in ('false','true'):
        # Use the parent's selected environment, not roslaunch's system-Python shebang.
        cmd=[sys.executable,'-c','import roslaunch; roslaunch.main()','--dump-params','uav_high_view','fast_comparison.launch',f'strategy:={strategy}',f'target_model_path:={model}',f'field_seed:={case["seed"]}']
        cmd += [f'{key}:={case[key]}' for key in ('world','field_config','runtime_config','gate_geometry_config')]
        params=yaml.safe_load(subprocess.check_output(cmd,text=True,cwd=ROOT))
        if not isinstance(params,dict):
            raise RuntimeError(f'Parameter expansion returned no mapping: {name}, strategy={strategy}')
        prefix='/navigation/mission_manager/'
        assert params[prefix+'following_speed_profile/cruise_lead_m']==1.
        assert params['/external_planner_start_max_distance']==1.2
        column_key='/fast_planner_node/sdf_map/horizontal_avoidance/column_top_z'
        assert params.get(column_key,-1.)==-1.
        stages=params[prefix+'mission/post_delivery_parameter_stages']
        assert '/px4_max_distance' not in stages[0]['parameters']
        assert stages[1]['parameters']['/px4_max_distance']==.15
        assert params['/fast_planner_node/manager/max_vel']==1.2
        pair.append(params)
    for key in pair[0]:
        if key.startswith('/navigation_vcl06_assertion/') or key.startswith('/navigation/mission_manager/mission/'):
            assert pair[0][key]==pair[1][key],key
    result.append(dict(case=name,stage_zero_fast=True,stage_one_slow=True,shared_gate_and_mission=True,command_admission_m=1.2,python_executable=sys.executable,python_version=sys.version.split()[0]))
(D/'static_validation.json').write_text(json.dumps(result,indent=2))
print('PASS: 12 launch expansions; five paired random layouts plus pilot.')

"""Verify completed report inputs and artifacts; never starts simulation."""
import json
from pathlib import Path
import yaml
from PIL import Image

D=Path(__file__).resolve().parent
items=json.loads((D/'runs.json').read_text())
assert {(i['seed'],i['label']) for i in items}=={(s,m) for s in (32,34) for m in ('baseline','strategy')}
assert len(items)==4
results=[]
for item in items:
    run=Path(item['run']);folder=D/f"{item['seed']}_{item['label']}"
    gate=json.loads((run/'gate_status.json').read_text())
    metric=json.loads((folder/'metrics.json').read_text())
    manifest=yaml.safe_load((run/'manifest.yaml').read_text())
    assert manifest['git_head'].startswith('5667bed')
    assert gate['status']==metric['status']
    assert metric['gate_metrics']==gate['metrics']
    assert len(metric['commit_times'])==gate['metrics']['release_commit_count']
    if gate['status']=='PASS':
        assert all(gate['checks'].values()) and manifest['exit_code']==0
        assert len(metric['commit_times'])==3 and metric['collisions']==0
        assert metric['completed_mission_s']==gate['metrics']['mission_ros_sec']
    checks={}
    if item['label']=='strategy':
        high=metric['high_view_final'];required=set(high['required_classes'])
        checks['three_required_hints']=set(high['top_hints'])==required
        checks['no_return_to_home_column']=not any(e['stage']=='RETURN_COLUMN' for e in high['events'])
        checks['fresh_low_reacquisitions']=all(-1e-6<=r['time']-r['last_seen_ns']/1e9<=.500001 for r in high['reacquisitions'])
        checks['three_classes_reacquired']=set(r['class_name'] for r in high['reacquisitions'])==required
        checks['true_fc_below_3m']=metric['max_fc_agl_m']<=3.
        for r in high['reacquisitions']:
            arrival=max(e['time'] for e in high['events'] if e['stage']=='REACQUIRE' and e['time']<=r['time'])
            checks['new_observation_after_arrival:'+r['class_name']]=r['last_seen_ns']/1e9>arrival
        hints=[h['xy'] for h in high['top_hints'].values()]
        goals=[d for d in gate['decision_fences'] if d['reason']=='high_view_full:REVISIT']
        checks['low_revisit_goal_count_at_least_three']=len(goals)>=3
        checks['low_revisit_goals_are_hints']=all(any(abs(d['goal_x']-xy[0])<1e-8 and abs(d['goal_y']-xy[1])<1e-8 for xy in hints) for d in goals)
        if gate['status']=='PASS':assert all(checks.values()),checks
    for name in ('flight_charts.png','route_stages.png','phase_target_timeline.png'):
        with Image.open(folder/name) as img:img.verify()
    results.append(dict(seed=item['seed'],label=item['label'],gate=gate['status'],checks_passed=sum(gate['checks'].values()),checks_total=len(gate['checks']),high_strategy_checks=checks,run_exit_code=manifest['exit_code']))

pairs=[]
for seed in (32,34):
    paths=[Path(next(i['run'] for i in items if i['seed']==seed and i['label']==mode)) for mode in ('baseline','strategy')]
    truths=[yaml.safe_load((p/'random_field_truth.yaml').read_text()) for p in paths]
    signature=lambda t:sorted((v['class'],v['world_x'],v['world_y'],v['yaw']) for v in t['targets'])
    assert signature(truths[0])==signature(truths[1])
    params=[yaml.safe_load((p/'rosparams.yaml').read_text()) for p in paths]
    assert params[0]['navigation']['mission_manager']['mission']==params[1]['navigation']['mission_manager']['mission']
    gates=[dict(p['navigation_vcl06_assertion']) for p in params]
    for g in gates:g.pop('report_path')
    assert gates[0]==gates[1]
    cameras=[json.loads((p/'actual_camera_info.json').read_text()) for p in paths]
    for c in cameras:c.pop('header')
    assert cameras[0]==cameras[1]
    assert (paths[0]/'scenario_inputs/field.world').read_bytes()==(paths[1]/'scenario_inputs/field.world').read_bytes()
    pairs.append(dict(seed=seed,actual_targets_equal=True,world_equal=True,shared_mission_and_gate_equal=True,camera_info_equal=True))
for name in ('comparison.png','paired_paths.png','delivery_milestones.png'):
    with Image.open(D/name) as img:img.verify()
if (D/'seed34_landing_failure.png').exists():
    with Image.open(D/'seed34_landing_failure.png') as img:img.verify()
assert (D/'REPORT.md').exists() and (D/'index.html').exists()
result=dict(status='PASS',scope='Report consistency and additional high-strategy checks, not a new flight Gate',runs=results,pairs=pairs,individual_figures=12,comparison_figures=3,landing_diagnostic_figures=int((D/'seed34_landing_failure.png').exists()))
(D/'validation.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))

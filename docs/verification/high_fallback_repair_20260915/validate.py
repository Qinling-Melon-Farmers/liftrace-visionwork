import json,re,subprocess
from pathlib import Path
import yaml
D=Path(__file__).resolve().parent;R=D.parents[2]
batch=json.loads((D/'matrix.json').read_text())
assert batch['status']=='COMPLETE' and [r['seed'] for r in batch['results']]==[31,32,34]
assert len({r['run'] for r in batch['results']})==3
followup=json.loads((D/'followup32_matrix.json').read_text())
assert followup['status']=='COMPLETE' and len(followup['results'])==1 and followup['results'][0]['seed']==32
sources={r['run']:b['source'] for b in (batch,followup) for r in b['results']}
for row in batch['results']+followup['results']:
    run=Path(row['run']);gate=json.loads((run/'gate_status.json').read_text())
    assert gate['status']==row['status'] and row['cleanup_pass']
    assert 'SITL cleanup verification: PASS' in (run/'run.log').read_text(errors='replace')
    manifest=yaml.safe_load((run/'manifest.yaml').read_text());assert manifest['git_head']==sources[row['run']]
    assert Path(manifest['resolved_uav_mission']).resolve()==(R/'patrol_uav_ws-patrol_planner/src/uav_mission').resolve()
    if gate['status']=='PASS':assert all(gate['checks'].values())
records=json.loads((D/'metrics.json').read_text());assert len(records)==10
for row in records:
    assert row['status']==json.loads((Path(row['run'])/'gate_status.json').read_text())['status']
    if row['status']!='PASS':assert row['completed_mission_s'] is None
for row in json.loads((D/'pairing.json').read_text()):assert row['same_world'] and row['same_targets'] and row['same_camera']
images=re.findall(r'<img[^>]+src="([^"]+)"',(D/'index.html').read_text())
assert len(images)==54 and all((D/p).is_file() for p in images)
source=json.loads((R/'docs/verification/high_fast_five_20260915/frame_inheritance.json').read_text())
subprocess.run(['git','merge-base','--is-ancestor',source['competition_base'],'HEAD'],cwd=R,check=True)
for rel in source['source_files_equal']:
    path='patrol_uav_ws-patrol_planner/src/uav_mission/'+rel
    assert (R/path).read_bytes()==subprocess.check_output(['git','show',source['frame_fix_source']+':'+path],cwd=R)
subprocess.run(['bash','top_level_scripts/check_sim_processes.sh'],cwd=R,check=True)
restored=['vision_ws/src/uav_high_view/src/uav_high_view/local_descent.py','patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/high_view_full.py','patrol_uav_ws-patrol_planner/src/uav_mission/test/test_high_view_full.py']
subprocess.run(['git','diff','--exit-code','a12750b','--']+restored,cwd=R,check=True)
out=dict(status='PASS',source=batch['source'],followup32_source=followup['source'],new_runs=4,primary_runs=3,followup_runs=1,prior_runs=6,image_links=54,paired_inputs='PASS',raw_gate='PASS',inherited_frames='PASS',retained_logic_reference='a12750b',cleanup='PASS')
(D/'validation.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))

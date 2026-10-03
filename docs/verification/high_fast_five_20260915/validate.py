"""Completion checks against raw runs and current repository state."""
import json,re,math,subprocess,os
from pathlib import Path
import yaml
D=Path(__file__).resolve().parent;R=D.parents[2]
def main():
    batch=json.loads((R/'logs/high_fast_five_20260915_batch/matrix.json').read_text())
    assert batch['status']=='COMPLETE' and [v['seed'] for v in batch['results']]==list(range(31,36))
    assert len({v['run'] for v in batch['results']})==5
    for row in batch['results']:
        run=Path(row['run']);gate=json.loads((run/'gate_status.json').read_text())
        assert row['status']==gate['status'] and row['cleanup_pass']
        assert 'SITL cleanup verification: PASS' in (run/'run.log').read_text(errors='replace')
        manifest=yaml.safe_load((run/'manifest.yaml').read_text())
        assert manifest['git_head']==batch['source']
        assert Path(manifest['resolved_uav_mission']).resolve()==(R/'patrol_uav_ws-patrol_planner/src/uav_mission').resolve()
        if gate['status']=='PASS':assert all(gate['checks'].values())
    records=json.loads((D/'metrics.json').read_text());assert len(records)==10
    lookup={(v['seed'],v['label']):v for v in records};assert len(lookup)==10
    for row in records:
        raw=json.loads((Path(row['run'])/'gate_status.json').read_text());assert raw['status']==row['status']
        if row['status']!='PASS':assert row['completed_mission_s'] is None
    for pair in json.loads((D/'paired_comparison.json').read_text()):
        assert pair['same_world'] and pair['same_targets'] and pair['same_camera_intrinsics']
        a=lookup[pair['seed'],'slow_coverage']['completed_mission_s'];b=lookup[pair['seed'],'fast_high']['completed_mission_s']
        if a is None or b is None:assert pair['full_gain_pct'] is None
        else:assert math.isclose(pair['full_gain_pct'],100*(a-b)/a,abs_tol=1e-8)
    page=(D/'index.html').read_text();images=re.findall(r'<img[^>]+src="([^"]+)"',page)
    assert len(images)==53 and all((D/p).is_file() for p in images)
    for link in re.findall(r'href="([^"]+)"',page):assert (D/link).is_file()
    proof=json.loads((D/'frame_inheritance.json').read_text())
    subprocess.run(['git','merge-base','--is-ancestor',proof['competition_base'],'HEAD'],cwd=R,check=True)
    for rel,equal in proof['source_files_equal'].items():
        assert equal
        path='patrol_uav_ws-patrol_planner/src/uav_mission/'+rel
        assert (R/path).read_bytes()==subprocess.check_output(['git','show',proof['frame_fix_source']+':'+path],cwd=R)
    for name,flag in proof['installed_entries'].items():assert flag and os.access(R/'patrol_uav_ws-patrol_planner/devel/lib/uav_mission'/name,os.X_OK)
    cleanup=json.loads((D/'cleanup_result.json').read_text())
    remainder=json.loads((D/'cleanup_main_merged_result.json').read_text())['deleted']+json.loads((D/'cleanup_covered_stages.json').read_text())['removed_refs']
    removed=[('refs/heads/'+v['name'],v['sha']) for v in cleanup['deleted_local']]+[('refs/remotes/origin/'+v['name'],v['sha']) for v in cleanup['deleted_remote']]+[(v['ref'],v['sha']) for v in remainder]
    assert len(removed)==62
    reachable=set(subprocess.check_output(['git','rev-list','--all'],cwd=R,text=True).splitlines())
    for ref,sha in removed:
        assert sha in reachable
        assert subprocess.run(['git','show-ref','--verify','--quiet',ref],cwd=R).returncode!=0
    worktrees=subprocess.check_output(['git','worktree','list','--porcelain'],cwd=R,text=True);assert worktrees.count('worktree ')==3
    for row in cleanup['retired_worktrees']:assert Path(row['old_path']).is_symlink() and Path(row['old_path']).resolve()==Path(row['archive'])
    subprocess.run(['bash','top_level_scripts/check_sim_processes.sh'],cwd=R,check=True)
    result=dict(new_trials=5,baseline_trials=5,unique_scenarios=5,source_frozen=batch['source'],frame_source_and_installed_entries='PASS',removed_refs=62,retired_worktrees=2,remaining_worktrees=3,image_links=53,raw_gate_and_pairing='PASS',status='PASS')
    (D/'validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':main()

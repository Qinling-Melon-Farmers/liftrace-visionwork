"""Static read-only XML/YAML correction checks, without ROS imports or nodes.

Checks final replay overrides and frozen scene inputs, not full ROS expansion.
Original preflight.json and effective_parameters.json remain untouched.
"""
from pathlib import Path
import argparse,json,subprocess,xml.etree.ElementTree as ET
import yaml
R=Path(__file__).resolve().parents[3];D=Path(__file__).resolve().parent
FROZEN_SOURCE='7a9502049b9448856152036632ef31857ada2582'

def frozen_bytes(path):
    return subprocess.check_output(['git','show',FROZEN_SOURCE+':'+path.relative_to(R).as_posix()],cwd=R)

def check_offline(rerun_fixed=False):
    launch=ET.parse(D/'replay.launch').getroot()
    defaults={n.get('name'):n.get('default') for n in launch.findall('arg')}
    assert defaults['high_agl']=='2.16' and defaults['stop_on_collision']=='false'
    collision=launch.find("param[@name='/navigation_vcl06_assertion/stop_on_collision']")
    assert collision is not None and list(launch)[-1] is collision
    assert collision.get('type')=='bool' and collision.get('value')=='$(arg stop_on_collision)'
    passed={n.get('name'):n.get('value') for n in launch.find('include').findall('arg')}
    assert passed['high_agl']=='$(arg high_agl)' and passed['obstacle_inflation_xy']=='0.25'
    assert passed['presentation_recording']=='true' and passed['corridor_fast']=='true'
    assert launch.find("node[@name='review_downward_recorder']") is not None
    assert launch.find("node[@name='center_evidence_recorder']") is not None
    high=R/'vision_ws/src/uav_high_view/launch'
    repair=ET.parse(high/'fov_inner_repair.launch').getroot()
    assert repair.find("include/arg[@name='world']").get('value')=='$(arg scene_dir)/field.world'
    assert repair.find("include/arg[@name='high_agl']").get('value')=='$(arg high_agl)'
    assert [n.get('file') for n in repair.findall('rosparam')]==['$(arg scene_dir)/frame_overrides.yaml','$(arg scene_dir)/repair_overrides.yaml']
    comparison=ET.parse(high/'fast_comparison.launch').getroot()
    assert comparison.find("group[@if='$(arg strategy)']/include/arg[@name='high_agl']").get('value')=='$(arg high_agl)'
    full=ET.parse(high/'full_strategy.launch').getroot()
    assert full.find("param[@name='/navigation/mission_manager/high_view_probe/config/high_agl']").get('value')=='$(arg high_agl)'
    videos=ET.parse(high/'presentation_cameras.launch').getroot()
    for name in ('presentation_overview_recorder','presentation_follow_recorder'):
        assert videos.find(f"node[@name='{name}']") is not None
    assert (D/'scenes.json').read_bytes()==frozen_bytes(D/'scenes.json')
    rows=[]
    for case in json.loads((D/'scenes.json').read_text()):
        if rerun_fixed and case['variant']!='resume_on':continue
        scene=R/case['scene'];config={}
        for name in ('field.world','field_config.yaml','fast_runtime.yaml','fast_gate.yaml','frame_overrides.yaml','repair_overrides.yaml','motion_overrides.yaml','presentation.yaml'):
            path=scene/name
            assert path.read_bytes()==frozen_bytes(path),f'Frozen scene changed: {name}'
            if name.endswith('.yaml'):config[name]=yaml.safe_load(path.read_text())
        ET.parse(scene/'field.world')
        assert case['seed']==38 and case['camera_agl']==2.0 and case['fc_agl']==2.16
        assert config['field_config.yaml']['spawn']['frozen_layout']==case['targets']
        field=config['field_config.yaml']['field']
        assert field['max_x']-field['min_x']==10 and field['max_y']-field['min_y']==10
        assert config['repair_overrides.yaml']['/navigation/mission_manager/high_view_probe/config/survey_xy']==case['survey']
        mission=config['fast_runtime.yaml']['mission']
        assert mission['timeout']==600 and mission['post_delivery_route']==case['post_route']
        # Render only the replay's direct final overrides using its actual runner argv.
        from run_case import launch_command,THREADS
        variant='resume_on_fixed' if rerun_fixed else case['variant']
        cmd=launch_command(38,variant,scene,Path('/offline/model.pt'))
        args=dict(defaults)
        args.update(item.split(':=',1) for item in cmd if ':=' in item)
        rendered={}
        for node in launch.findall('param'):
            value=node.get('value')
            if value.startswith('$(arg '):value=args[value[6:-1]]
            rendered[node.get('name')]=yaml.safe_load(value)
        assert rendered['/navigation_vcl06_assertion/stop_on_collision'] is (not rerun_fixed)
        assert rendered['/navigation/mission_manager/high_view_full/policy/resume_survey_enabled'] is case['resume_enabled']
        assert 'high_agl:=2.16' in cmd and 'SIM_RUN_AUTHORIZED=1' in cmd
        assert 'SIM_REQUIRE_GATE=1' in cmd and 'top_level_scripts/sim_run.sh' in cmd
        assert THREADS==dict(OPENCV_FOR_THREADS_NUM='1',OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='2')
        rows.append(dict(seed=38,variant=variant,scene=case['scene'],status='PASS',replay_parameters=rendered))
    return rows

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rerun-fixed',action='store_true')
    args=parser.parse_args()
    print(json.dumps(check_offline(args.rerun_fixed),indent=2))

if __name__=='__main__':main()

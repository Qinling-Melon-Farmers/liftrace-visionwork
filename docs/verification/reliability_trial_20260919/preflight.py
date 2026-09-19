"""Read-only source-owner/scene/launch checks; never starts a simulator."""
import json,sys
from pathlib import Path
import yaml
import rospkg,roslaunch

D=Path(__file__).resolve().parent
R=D.parents[2]
MODEL=Path('/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt')

def arguments(case):
    return ['strategy:=true','reliability_trial:=true','record_overview_video:=true',
            'overview_z:=7.0',f'field_seed:={case["seed"]}',f'target_model_path:={MODEL}']+[
                f'{k}:={case[k]}' for k in ('world','field_config','runtime_config','gate_geometry_config')]

def main():
    case=json.loads((D/'case.json').read_text());selection=json.loads((D/'selection.json').read_text())
    assert case['seed']==selection['seed'] and MODEL.is_file()
    owners={p:str(Path(rospkg.RosPack().get_path(p)).resolve()) for p in ('uav_mission','uav_vision','uav_high_view','plan_manage')}
    assert owners['uav_mission']==str(R/'patrol_uav_ws-patrol_planner/src/uav_mission'),owners
    assert owners['plan_manage']==str(R/'patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_manage'),owners
    for p in ('uav_vision','uav_high_view'):assert owners[p]==str(R/'vision_ws/src'/p),owners
    cfg=roslaunch.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(str(R/'vision_ws/src/uav_high_view/launch/fast_comparison.launch'),cfg,argv=arguments(case),verbose=False)
    val=lambda p:cfg.params[p].value
    for p in ('/fast_planner_node/fsm/liveness_enabled','/traj_server/traj_server/require_goal_identity',
              '/traj_server/progress/enabled','/navigation/mission_manager/high_view_full/boundary_policy/enabled'):
        assert val(p) is True,p
    assert val('/fast_planner_node/sdf_map/horizontal_avoidance/enabled') is True
    assert '/fast_planner_node/sdf_map/horizontal_avoidance/column_top_z' not in cfg.params
    gate=yaml.safe_load(Path(case['gate_geometry_config']).read_text())
    assert gate['post_delivery_gate']['low_height_region']['max_height']==.7
    pattern=''.join('L' if door['lateral_min']>8.0 else 'R' for door in gate['post_delivery_gate']['doors'][1:])
    assert pattern==selection['pattern']
    field=yaml.safe_load(Path(case['field_config']).read_text())
    scene=json.loads((D/f'seed_{case["seed"]}'/'scene.json').read_text())
    assert sorted((round(t['x'],6),round(t['y'],6)) for t in scene['trees'])==sorted(
        (round(t['world_x'],6),round(t['world_y'],6)) for t in field['static_exclusions'] if t['name'].startswith('combined_tree_'))
    assert any(n.name=='overview_video_recorder' for n in cfg.nodes)
    assert any(n.name=='planner_progress_recorder' for n in cfg.nodes)
    assert not any(n.type in ('gzclient','rviz') for n in cfg.nodes)
    from plan_manage.msg import Bspline,TrajectoryProgress
    assert 'goal_stamp' in Bspline.__slots__ and 'tracking_hold' in TrajectoryProgress.__slots__
    (D/'preflight.json').write_text(json.dumps(dict(status='PASS',seed=case['seed'],python=sys.executable,
        source_owners=owners,headless=True,overview=True,overview_z=7.,gate_height_unchanged=.7,
        navigation_columns_full_height=True,scope='Static scene and launch checks only'),indent=2))
    print('PREFLIGHT PASS seed',case['seed'])

if __name__=='__main__':main()

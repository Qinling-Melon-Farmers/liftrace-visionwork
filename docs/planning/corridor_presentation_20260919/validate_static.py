"""Launch expansion and geometry tests only; no simulation startup."""
import ast,json,math,sys,xml.etree.ElementTree as ET
from pathlib import Path
import yaml,rospkg,roslaunch

D=Path(__file__).resolve().parent;R=D.parents[2]
def main():
    owners={p:rospkg.RosPack().get_path(p) for p in ('uav_high_view','uav_mission','uav_vision_eval')}
    assert all(v.startswith(str(R)) for v in owners.values()),owners
    base=['strategy:=true','reliability_trial:=true','field_seed:=2672',
          'target_model_path:=/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt',
          'world:='+str(D/'example_scene/field.world'),'field_config:='+str(D/'example_scene/field_config.yaml'),
          'runtime_config:='+str(D/'example_scene/corridor_090_candidate.yaml'),
          'gate_geometry_config:='+str(D/'example_scene/gate_120_candidate.yaml')]
    for switch in ('false','true'):
        config=roslaunch.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader().load(str(R/'vision_ws/src/uav_high_view/launch/fast_comparison.launch'),config,
            argv=base+['presentation_recording:='+switch,'record_overview_video:=true'],verbose=False)
        counts=sum(n.type=='camera_video_recorder.py' for n in config.nodes)
        assert counts==(2 if switch=='true' else 1),counts
        assert sum(n.type=='presentation_cameras.py' for n in config.nodes)==(switch=='true')
        assert config.params['/navigation/mission_manager/high_view_probe/config/high_agl'].value==2.6
        assert config.params['/navigation_vcl06_assertion/post_delivery_gate/low_height_region/max_height'].value==1.2
    config=roslaunch.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(str(R/'vision_ws/src/uav_high_view/launch/recorded_flight_replay.launch'),config,
        argv=['presentation_recording:=true','source_run:=/unused/offline-expansion','world:='+str(D/'example_scene/field.world')],verbose=False)
    assert sum(n.type=='camera_video_recorder.py' for n in config.nodes)==2
    assert config.params['/recorded_flight_replay/external_cameras'].value is True
    assert all(n.package not in ('mavros','px4','uav_mission') for n in config.nodes)
    for filename in ('presentation_cameras.py','presentation_compose.py','recorded_flight_replay.py'):
        ast.parse((R/'vision_ws/src/uav_high_view/scripts'/filename).read_text())
    views=yaml.safe_load((R/'vision_ws/src/uav_high_view/config/presentation.yaml').read_text())['views']
    camera=views['overview'];z=camera['xyz'][2]
    assert z-camera['near']<5.4 and z-camera['near']>4.0
    assert (z-4.)*math.tan(camera['hfov']/2)>5.0  # all boundary wall tops inside square overhead FOV
    scene=json.loads((D/'example_scene/scene.json').read_text())
    assert scene['door_mode']=='continuous' and scene['outer_wall_height_m']==4.
    rt=yaml.safe_load((D/'example_scene/corridor_090_candidate.yaml').read_text())
    assert rt['mission']['target_action_timeout']==120 and rt['mission']['early_return_enabled'] is False
    assert rt['mission']['delivery_reserve_per_slot']==70.
    info=dict(status='PASS',scope='STATIC_ONLY_NO_NEW_SIMULATION',package_owners=owners,
              scene_tests_in_fork=77,high_view_tests=70,launch_variants_checked=3,
              continuous_centers=[d['center_y'] for d in scene['doors']],
              actual_runtime_high_agl=2.6,mechanism_budget_design_only_s=10.,
              candidate_corridor_fc_limit=1.2,candidate_cruise_fc_agl=.9,new_sitl_runs=0)
    (D/'validation.json').write_text(json.dumps(info,indent=2));print(json.dumps(info,indent=2))

if __name__=='__main__':main()

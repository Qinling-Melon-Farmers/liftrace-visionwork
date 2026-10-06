#!/usr/bin/env python3
"""Read-only launch/parameter check. Never starts ROS nodes or writes files."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[3]
D = Path(__file__).resolve().parent
FROZEN = '337b583689f46d907546d1d28a8fc52b13f47398'
MODEL = Path('/home/xhj/liftrace/deliverables/liftrace_five_class_20260928_models/flight_5cls_20260928.pt')
SEEDS = (31,38)


def scene_for(seed):
    if seed not in SEEDS:
        raise ValueError('Only the two authorized seeds are available')
    return ROOT/f'docs/verification/snake3_camera2m_20261005/generated/snake3_{seed}/snake3_seed{seed}'


def flatten(value, prefix=''):
    result = {}
    for key, child in value.items():
        path = (prefix+'/'+str(key)).replace('//','/')
        if isinstance(child,dict):
            result.update(flatten(child,path))
        else:
            result[path] = child
    return result


def expected_parameters():
    return {
        '/circle_detector/circle_quality_ordered_nms': False,
        '/drop_aligner/exact_drop_projection': True,
        '/uav_vision/drop_exact_projection_enabled': True,
        '/drop_aligner/use_camera_info': True,
        '/drop_aligner/require_alignment_context': True,
        '/drop_aligner/map_frame': 'camera_init',
        '/drop_aligner/body_frame': 'vision_body',
        '/drop_aligner/ground_z': -.22,
        '/drop_aligner/camera_info_topic': '/downward_camera/camera_info',
        '/drop_aligner/rectify_input_pixels': True,
        '/drop_aligner/slot_compensation_mode': 'zero',
        '/drop_aligner/slot_parameter_namespace': '/drop_system',
        '/drop_aligner/slot_positions_body': [[0.,0.]]*3,
        '/drop_aligner/max_alignment_error_m': 0.,
        '/uav_vision/drop_exact_alignment_max_error_m': 0.,
        '/uav_vision/drop_map_frame': 'camera_init',
        '/uav_vision/drop_camera_frame': 'downward_camera_optical_frame',
        '/uav_vision/drop_camera_info_topic': '/downward_camera/camera_info',
        '/uav_vision/drop_ground_z': -.22,
        '/uav_vision/drop_offset_timeout': .5,
        '/target_map_projector/map_frame': 'camera_init',
        '/target_map_projector/ground_z': -.22,
        '/target_map_projector/camera_info_topic': '/downward_camera/camera_info',
        '/target_map_projector/rectify_input_pixels': True,
        '/vision_pose_tf/parent_frame': 'camera_init',
        '/vision_pose_tf/child_frame': 'vision_body',
        '/drop_system/slot_offsets': [[0.,0.]]*3,
        '/drop_system/dynamic_slot_offsets': [[0.,0.]]*3,
        '/external_mission_mode': True,
        '/uav_vision/require_release_permission': True,
        '/navigation_vcl06_assertion/stop_on_collision': False,
        '/competition_key_recorder/truth_world_offset': [0.,0.,.22],
    }


def check_parameters(params):
    failures = []
    for key, expected in expected_parameters().items():
        actual = params.get(key)
        # Explicit booleans: strings such as "false" must never pass by truthiness.
        same = actual is expected if isinstance(expected,bool) else actual == expected
        if not same:
            failures.append(dict(parameter=key,expected=expected,actual=actual,missing=key not in params))
    if failures:
        raise ValueError('Projection parameter preflight FAIL: '+json.dumps(failures,ensure_ascii=True))
    return dict(status='PASS',checks=len(expected_parameters()),exact_visual=True,exact_control=True,
        body_frame='vision_body',map_frame='camera_init',ground_z=-.22,slot_mode='zero',
        tolerance_policy='existing 30px exposure Jacobian; controller uses message tolerance',
        circle_quality_ordered_nms=False,stop_on_collision=False)


def check_frozen_scene(scene):
    names = ('field.world','field_config.yaml','fast_runtime.yaml','fast_gate.yaml',
             'frame_overrides.yaml','repair_overrides.yaml','motion_overrides.yaml','presentation.yaml')
    for name in names:
        path = scene/name
        original = subprocess.check_output(['git','show',FROZEN+':'+path.relative_to(ROOT).as_posix()],cwd=ROOT)
        if path.read_bytes()!=original:
            raise ValueError('Frozen scene changed: '+str(path))
    return len(names)


def assert_source_switch_names():
    visual = (ROOT/'vision_ws/src/uav_vision/scripts/drop_aligner.py').read_text()
    control = (ROOT/'patrol_uav_ws-patrol_planner/src/patrol_control/src/patrol_control.cpp').read_text()
    if 'get_param("~exact_drop_projection"' not in visual or '"uav_vision/drop_exact_projection_enabled"' not in control:
        raise ValueError('Worker switch names changed; re-review wrapper before launch')


def load_launch(seed, model, launch=None):
    from roslaunch.config import ROSLaunchConfig
    from roslaunch.xmlloader import XmlLoader
    import rospkg
    import roslaunch.substitution_args
    scene = scene_for(seed)
    frozen_count = check_frozen_scene(scene)
    assert_source_switch_names()
    if not model.is_file():
        raise ValueError('Model file missing')
    package_paths = {}
    pack = rospkg.RosPack()
    for name,path in [('uav_mission',ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission'),
                      ('uav_vision',ROOT/'vision_ws/src/uav_vision'),
                      ('uav_high_view',ROOT/'vision_ws/src/uav_high_view')]:
        resolved = Path(pack.get_path(name)).resolve()
        if resolved != path.resolve():
            raise ValueError('Wrong source overlay for '+name+': '+str(resolved))
        package_paths[name] = str(resolved)
    # Add ONLY external dependencies to resolver search. Do not rewrite
    # ROS_PACKAGE_PATH or replace the sourced UAV/VISION overlay order.
    paths = rospkg.get_ros_paths()
    px4 = Path(os.environ.get('PX4_ROOT','/home/xhj/PX4-Autopilot'))
    paths += [str(px4),str(px4/'Tools/simulation/gazebo-classic/sitl_gazebo-classic')]
    roslaunch.substitution_args._rospack = rospkg.RosPack(ros_paths=paths)
    before = os.environ.get('SIM_RUN_DIR')
    os.environ['SIM_RUN_DIR'] = str(ROOT/'logs/PREFLIGHT_ONLY_NOT_CREATED')
    try:
        config = ROSLaunchConfig()
        XmlLoader().load(str(launch or D/'replay.launch'),config,verbose=False,argv=[
            'scene_dir:='+str(scene),'field_seed:='+str(seed),'target_model_path:='+str(model.resolve()),
            'high_agl:=2.16','resume_survey_enabled:=false'])
    finally:
        if before is None:
            os.environ.pop('SIM_RUN_DIR',None)
        else:
            os.environ['SIM_RUN_DIR'] = before
    params = {key:value.value for key,value in config.params.items()}
    result = check_parameters(params)
    params_survey = params['/navigation/mission_manager/high_view_probe/config/survey_xy']
    if params_survey != [[1.,-3.95],[1.,4.1],[3.7,4.1],[3.7,-3.95],[6.4,-3.95],[6.4,4.1]]:
        raise ValueError('Historical three-line survey was changed')
    if params['/navigation/mission_manager/high_view_probe/config/high_agl']!=2.16:
        raise ValueError('Historical camera/FC height was changed')
    if params['/navigation/mission_manager/high_view_full/policy/resume_survey_enabled'] is not False:
        raise ValueError('Resume strategy changed')
    for node in config.nodes:
        if node.package in ('actuator_pwm','livox_ros_driver','livox_ros_driver2'):
            raise ValueError('Hardware node in simulation: '+node.package)
    result.update(seed=seed,scene=str(scene),frozen_scene_files=frozen_count,
                  overlays=package_paths,nodes=len(config.nodes))
    return result,params


def check_build():
    from uav_vision.msg import DropOffset
    expected = {'map_valid','map_point','map_frame','alignment_error_m','alignment_tolerance_m','target_id','target_first_seen'}
    if not expected.issubset(set(DropOffset.__slots__)):
        raise ValueError('New DropOffset message is not built in this overlay')
    targets = [
        (ROOT/'patrol_uav_ws-patrol_planner/devel/lib/patrol_control/patrol_control',
         [ROOT/'patrol_uav_ws-patrol_planner/src/patrol_control/src/patrol_control.cpp',
          ROOT/'patrol_uav_ws-patrol_planner/src/patrol_control/include/patrol_control/patrol_control.h',
          ROOT/'patrol_uav_ws-patrol_planner/src/patrol_control/include/patrol_control/drop_geometry.h']),
        (ROOT/'vision_ws/devel/lib/uav_vision/circle_detector_node',
         [ROOT/'vision_ws/src/uav_vision/src/circle_detector_node.cpp',
          ROOT/'vision_ws/src/uav_vision/include/uav_vision/circle_geometry.h',
          ROOT/'vision_ws/src/uav_vision/include/uav_vision/circular_detector_node.h']),
    ]
    for binary,sources in targets:
        if not binary.is_file() or any(binary.stat().st_mtime < source.stat().st_mtime for source in sources):
            raise ValueError('Missing/stale binary; main must rebuild: '+str(binary))
    return dict(status='PASS',message_fields_checked=sorted(expected),binaries=[str(x[0]) for x in targets])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',type=int,choices=(0,1))
    parser.add_argument('--model',type=Path,default=MODEL)
    parser.add_argument('--params',type=Path,help='Read a launch dump JSON or archived runtime YAML')
    parser.add_argument('--check-build',action='store_true')
    parser.add_argument('--self-check',action='store_true',help='Check both expanded candidates and reject the actual historical runtime')
    args = parser.parse_args()
    if args.self_check:
        expanded = [load_launch(seed,args.model) for seed in SEEDS]
        candidates = [row[0] for row in expanded]
        for mutation in ('missing','enabled'):
            params = dict(expanded[0][1])
            key = '/circle_detector/circle_quality_ordered_nms'
            if mutation=='missing':
                params.pop(key)
            else:
                params[key] = True
            try:
                check_parameters(params)
            except ValueError:
                pass
            else:
                raise AssertionError('Unsafe circle NMS setting was accepted: '+mutation)
        historical = ROOT/'logs/snake3_camera2m_snake3_seed31_20261005_095902/rosparams.yaml'
        try:
            check_parameters(flatten(yaml.safe_load(historical.read_text())))
        except ValueError:
            old_rejected = True
        else:
            raise AssertionError('Old projection-disabled runtime was accepted')
        result = dict(status='PASS',seeds=[r['seed'] for r in candidates],checks_per_seed=len(expected_parameters()),frozen_scene_files_checked=16,old_runtime_rejected=old_rejected,
                      circle_nms_missing_or_enabled_rejected=True,simulation_started=False)
    elif args.params:
        content = yaml.safe_load(args.params.read_text())
        result = check_parameters(flatten(content))
    else:
        if args.case is None:
            parser.error('--case or --params is required')
        result,_ = load_launch(SEEDS[args.case],args.model)
    if args.check_build:
        result['build'] = check_build()
    print(json.dumps(result,indent=2))

if __name__=='__main__':
    main()
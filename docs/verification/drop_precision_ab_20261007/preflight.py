#!/usr/bin/env python3
"""Read-only two-seed exact-ON preparation. Never initialize a ROS node."""
import importlib.util
import json
import math
import os
import shlex
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
D = Path(__file__).resolve().parent


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


base = module('prior_precision_preflight', D.parent/'drop_precision_20261006/projection_preflight.py')
MODEL, SEEDS, scene_for, flatten, check_build = base.MODEL, (31, 38), base.scene_for, base.flatten, base.check_build
START_PROXY = '/drop_precision_camera_guard/start_mission_checked'


def canonical(node):
    return (node.tag, sorted(node.attrib.items()), (node.text or '').strip(), [canonical(x) for x in node])


def camera_inputs(camera_dir):
    camera_dir = Path(camera_dir).resolve()
    contract = json.loads((camera_dir/'camera_contract.json').read_text())
    generator = module('research_camera_generator', ROOT/'vision_ws/src/uav_vision_eval/scripts/generate_drop_research_camera.py')
    template = generator.DEFAULT_TEMPLATE
    profile = json.loads(generator.DEFAULT_PROFILE.read_text())
    expected_tree, expected = generator.make_model(ET.parse(template), profile)
    sdf = camera_dir/'model.sdf'
    if canonical(ET.parse(sdf).getroot()) != canonical(expected_tree.getroot()):
        raise ValueError('Generated SDF differs from reviewed research profile/template')
    for key, value in expected.items():
        if contract.get(key) != value:
            raise ValueError('Camera contract differs from generator: '+key)
    if Path(contract['generated_sdf']).resolve() != sdf:
        raise ValueError('Contract identifies a different SDF')
    if contract['profile_id'] != 'drop_centered_d0_20261007':
        raise ValueError('Only the centered D0 research profile is planned')
    return contract, sdf


def check_camera_info(info, contract):
    for key in ('width', 'height', 'distortion_model'):
        if info.get(key) != contract[key]:
            raise ValueError('CameraInfo mismatch: '+key)
    if info.get('header', {}).get('frame_id') != contract['frame_id']:
        raise ValueError('CameraInfo frame mismatch')
    for key in ('K', 'D', 'R', 'P'):
        got, expected = info.get(key, []), contract[key]
        if len(got) != len(expected) or any(not math.isfinite(float(a)) or not math.isclose(float(a), float(b), rel_tol=1e-8, abs_tol=1e-6) for a,b in zip(got, expected)):
            raise ValueError('CameraInfo mismatch: '+key)
    if info.get('binning_x',0) or info.get('binning_y',0):
        raise ValueError('Unexpected image binning')
    roi=info.get('roi',{})
    if any(roi.get(k,0) for k in ('x_offset','y_offset')):
        raise ValueError('Unexpected image ROI')
    return {'status':'PASS','profile_id':contract['profile_id'],'width':contract['width'],'height':contract['height']}


def check_parameters(params):
    result = base.check_parameters(params)
    if params.get('/navigation_mission_start_gate/start_service') != START_PROXY:
        raise ValueError('Mission start bypasses the camera check')
    if params.get('/navigation/mission_manager/high_view_full/policy/resume_survey_enabled') is not False:
        raise ValueError('Resume must remain false')
    result['scope']='TWO_NEW_EXACT_ON_RUNS_WITH_HISTORICAL_REFERENCE_ONLY'
    return result


def load_launch(seed, model, camera_dir):
    from roslaunch.config import ROSLaunchConfig
    from roslaunch.xmlloader import XmlLoader
    old_result, old_params = base.load_launch(seed, model)
    contract, sdf = camera_inputs(camera_dir)
    before = os.environ.get('SIM_RUN_DIR')
    os.environ['SIM_RUN_DIR'] = str(ROOT/'logs/PREFLIGHT_ONLY_NOT_CREATED')
    try:
        config = ROSLaunchConfig()
        XmlLoader().load(str(D/'replay.launch'), config, verbose=False, argv=[
            'scene_dir:='+str(scene_for(seed)), 'field_seed:='+str(seed),
            'target_model_path:='+str(model.resolve()), 'high_agl:=2.16',
            'resume_survey_enabled:=false', 'vehicle_sdf:='+str(sdf),
            'camera_contract:='+str(Path(camera_dir).resolve()/'camera_contract.json')])
    finally:
        if before is None:os.environ.pop('SIM_RUN_DIR',None)
        else:os.environ['SIM_RUN_DIR']=before
    params={k:v.value for k,v in config.params.items()}
    result=check_parameters(params)
    differences={k:[v,params.get(k)] for k,v in old_params.items()
                 if params.get(k)!=v and k!='/navigation_mission_start_gate/start_service'}
    if differences:
        raise ValueError('Unexpected change from frozen launch parameters: '+json.dumps(differences))
    spawn=[n for n in config.nodes if str(sdf) in shlex.split(n.args)]
    if len(spawn)!=1 or spawn[0].package!='patrol_control' or spawn[0].type!='spawn_model_delayed.py':
        raise ValueError('Research SDF is not the single actual Gazebo spawn input')
    spawn_args=shlex.split(spawn[0].args)
    if spawn_args[spawn_args.index('-file')+1]!=str(sdf) or spawn_args[spawn_args.index('-model')+1]!='iris_mid360':
        raise ValueError('Unexpected actual vehicle spawn arguments')
    guards=[n for n in config.nodes if n.name=='drop_precision_camera_guard']
    if len(guards)!=1 or not guards[0].required:
        raise ValueError('Camera checker must be required')
    if not os.access(D/'camera_guard.py',os.X_OK):
        raise ValueError('Camera checker is not executable')
    result.update(seed=seed, scene=old_result['scene'], overlays=old_result['overlays'],
        frozen_scene_files=old_result['frozen_scene_files'], camera_contract=contract,
        actual_spawn_args=spawn[0].args, node_count=len(config.nodes),
        legacy_parameter_differences_excluding_camera_start_proxy=0,
        startup_gate='CameraInfo plus image dimensions before forwarding the original start Trigger')
    return result,params

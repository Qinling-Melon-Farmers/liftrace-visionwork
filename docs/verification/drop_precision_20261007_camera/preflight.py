#!/usr/bin/env python3
"""Read-only ROS launch expansion and SDF check; no master, nodes or simulation."""
import argparse
import importlib.util
import json
from pathlib import Path
import shlex
import xml.etree.ElementTree as ET
from camera_tools import ROOT, validate_sdf, write_new_json


def preflight(prepared_dir, seed, model):
    manifest = json.loads((prepared_dir / 'prepared.json').read_text())
    contract = json.loads(Path(manifest['camera_contract']).read_text())
    sdf = Path(manifest['vehicle_sdf']).resolve()
    optics = validate_sdf(ET.parse(sdf), contract)
    path = ROOT / 'docs/verification/drop_precision_20261006/projection_preflight.py'
    spec = importlib.util.spec_from_file_location('historical_projection_preflight', path)
    old = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(old)
    # Reuse frozen scene/interface assertions without modifying their tools.
    result, params = old.load_launch(seed, model or old.MODEL, Path(manifest['launch_file']))
    _, historical_params = old.load_launch(seed, model or old.MODEL)
    if params != historical_params:
        raise ValueError('Camera-only replay unexpectedly changed ROS parameters')
    # Inspect the actual expanded spawn args, not a marker parameter claiming a model.
    from roslaunch.config import ROSLaunchConfig
    from roslaunch.xmlloader import XmlLoader
    import os
    previous = os.environ.get('SIM_RUN_DIR')
    os.environ['SIM_RUN_DIR'] = str(ROOT / 'logs/PREFLIGHT_ONLY_NOT_CREATED')
    try:
        config = ROSLaunchConfig()
        XmlLoader().load(manifest['launch_file'], config, verbose=False, argv=[
            'scene_dir:=' + str(old.scene_for(seed)), 'field_seed:=' + str(seed),
            'target_model_path:=' + str((model or old.MODEL).resolve()),
            'high_agl:=2.16', 'resume_survey_enabled:=false'])
    finally:
        if previous is None:
            os.environ.pop('SIM_RUN_DIR', None)
        else:
            os.environ['SIM_RUN_DIR'] = previous
    spawns = [node for node in config.nodes if node.type == 'spawn_model_delayed.py']
    if len(spawns) != 1:
        raise ValueError('Expected exactly one vehicle spawn node')
    argv = shlex.split(spawns[0].args)
    actual = Path(argv[argv.index('-file') + 1]).resolve()
    if actual != sdf or '-sdf' not in argv or argv[argv.index('-model') + 1] != 'iris_mid360':
        raise ValueError('Expanded spawn is not the research vehicle SDF')
    for package in ('uav_vision_eval', 'uav_high_view'):
        import rospkg
        actual_package = Path(rospkg.RosPack().get_path(package)).resolve()
        if actual_package != (ROOT / 'vision_ws/src' / package).resolve():
            raise ValueError('Wrong source overlay: ' + package)
    result.update(simulation_started=False, renderer_sdf=optics, vehicle_sdf=str(actual),
                  spawn_args=spawns[0].args, runtime_camera_info='NOT_OBSERVED',
                  launch_file=manifest['launch_file'], exact_projection=True)
    result['expanded_parameters_identical_to_historical_exactON_wrapper'] = True
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared-dir', type=Path, required=True)
    parser.add_argument('--seed', type=int, choices=(31, 38), required=True)
    parser.add_argument('--model', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = preflight(args.prepared_dir.resolve(), args.seed, args.model)
    if args.output:
        write_new_json(args.output, result)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()

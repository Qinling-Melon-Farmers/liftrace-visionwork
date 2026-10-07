#!/usr/bin/env python3
"""The single operator entry: prepare a research camera/replay; never execute it."""
import argparse
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
from camera_tools import ROOT, generator, validate_sdf, write_new_json

D = Path(__file__).resolve().parent


def unique(parent, xpath):
    nodes = parent.findall(xpath)
    if len(nodes) != 1:
        raise ValueError('Template changed; review selector: ' + xpath)
    return nodes[0]


def prepare(output, enabled, template, profile):
    if not enabled:
        raise ValueError('Explicit --enable-drop-camera-research is required')
    output = output.resolve()
    if output.exists():
        raise ValueError('Output must be a NEW directory')
    # Derive three thin wrappers at generation time; no maintained model/launch copies.
    fov_source = ROOT / 'vision_ws/src/uav_high_view/launch/fov_inner_repair.launch'
    resume_source = ROOT / 'docs/verification/seed38_resume_20261005/replay.launch'
    drop_source = ROOT / 'docs/verification/drop_precision_20261006/replay.launch'
    fov, resume, drop = [ET.parse(p) for p in (fov_source, resume_source, drop_source)]
    vehicle = unique(fov, ".//include/arg[@name='vehicle_sdf']")
    unique(resume, 'include').set('file', str(output / 'fov.launch'))
    unique(drop, 'include').set('file', str(output / 'resume.launch'))
    # Current authorized plan is ON only. The runner consumes this entry unchanged.
    for key in ('/drop_aligner/exact_drop_projection', '/uav_vision/drop_exact_projection_enabled'):
        if unique(drop, "param[@name='%s']" % key).get('value') != 'true':
            raise ValueError('Expected exact-ON historical wrapper')
    contract = generator.generate(template, profile, output / 'camera', enabled=True)
    validate_sdf(ET.parse(output / 'camera/model.sdf'), contract)
    vehicle.set('value', str(output / 'camera/model.sdf'))
    for name, tree in [('fov.launch', fov), ('resume.launch', resume), ('replay.launch', drop)]:
        tree.write(output / name, encoding='utf-8', xml_declaration=True)
    manifest = dict(
        status='PREPARED_NOT_EXECUTED', simulation_started=False,
        source_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        source_git_status=subprocess.check_output(
            ['git', 'status', '--porcelain'], cwd=ROOT, text=True).splitlines(),
        source_root=str(ROOT), research_camera_enabled=True, exact_drop_projection=True,
        authorized_plan='NEW exactON seed31/38 only; historical results reused',
        launch_file=str(output / 'replay.launch'),
        vehicle_sdf=str(output / 'camera/model.sdf'),
        camera_contract=str(output / 'camera/camera_contract.json'),
        launch_templates=[str(p) for p in (fov_source, resume_source, drop_source)],
        operator_preflight=str(D / 'preflight.py'),
        operator_runtime_check=str(D / 'runtime_check.py'),
        runtime_world_sdf_helper_source=str(D / 'read_runtime_world_sdf.cpp'),
        comparison_limit='Camera calibration AND algorithm differ from historical data; no isolated algorithm causal claim')
    write_new_json(output / 'prepared.json', manifest)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--enable-drop-camera-research', action='store_true')
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--template', type=Path, default=generator.DEFAULT_TEMPLATE)
    parser.add_argument('--profile', type=Path, default=generator.DEFAULT_PROFILE)
    args = parser.parse_args()
    try:
        result = prepare(args.output_dir, args.enable_drop_camera_research, args.template, args.profile)
    except (ValueError, OSError, ET.ParseError) as exc:
        parser.error(str(exc))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()

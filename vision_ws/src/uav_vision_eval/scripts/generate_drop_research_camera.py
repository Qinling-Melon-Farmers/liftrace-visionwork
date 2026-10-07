#!/usr/bin/env python3
"""Generate an opt-in simulation camera from an existing SDF; never start ROS.

Only optics and camera publication fields change. Includes, dynamics, mounts,
contact geometry and the source files are preserved. No calibration is fitted.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
DEFAULT_TEMPLATE = PACKAGE / 'models/iris_mid360_start_fov/model.sdf'
DEFAULT_PROFILE = PACKAGE / 'config/drop_camera_centered_d0.json'
RENDER_D = ('k1', 'k2', 'p1', 'p2', 'k3')  # CameraInfo plumb_bob order
PLUGIN_D = ('distortionK1', 'distortionK2', 'distortionT1',
            'distortionT2', 'distortionK3')


def camera_elements(tree, profile):
    model = tree.getroot().find('model')
    if model is None:
        raise ValueError('Expected a standalone model SDF')
    link = model.find("link[@name='%s']" % profile['camera_link'])
    sensor = None if link is None else link.find(
        "sensor[@name='%s']" % profile['sensor_name'])
    if sensor is None or sensor.get('type') != 'camera':
        raise ValueError('Selected camera sensor missing')
    cameras = sensor.findall('camera')
    plugins = sensor.findall("plugin[@name='%s']" % profile['plugin_name'])
    if len(cameras) != 1 or len(plugins) != 1:
        raise ValueError('Expected exactly one selected camera and ROS plugin')
    if plugins[0].get('filename') != 'libgazebo_ros_camera.so':
        raise ValueError('Unsupported CameraInfo publisher')
    camera = cameras[0]
    # This experiment uses the symmetric perspective/FOV path, not custom lens.
    if camera.find('lens') is not None or camera.find('noise') is not None:
        raise ValueError('Custom lens/noise requires a separate reviewed profile')
    return link, sensor, camera, plugins[0]


def put(parent, name, value):
    nodes = parent.findall(name)
    if len(nodes) > 1:
        raise ValueError('Ambiguous SDF field: ' + name)
    node = nodes[0] if nodes else ET.SubElement(parent, name)
    node.text = str(value)


def make_model(template, profile):
    tree = copy.deepcopy(template)
    link, sensor, camera, plugin = camera_elements(tree, profile)
    width = int(camera.findtext('image/width'))
    height = int(camera.findtext('image/height'))
    hfov = float(camera.findtext('horizontal_fov'))
    if [width, height] != profile['expected_image_size']:
        raise ValueError('Template resolution changed; review profile first')
    if not math.isfinite(hfov) or not 0.0 < hfov < math.pi:
        raise ValueError('Invalid perspective horizontal FOV')
    cx, cy = profile['principal_point']
    if [cx, cy] != [width / 2.0, height / 2.0]:
        raise ValueError('Research principal point must equal image half-size')
    if profile['D'] != [0.0] * 5 or profile['distortion_center'] != [0.5, 0.5]:
        raise ValueError('This profile supports only centered zero distortion')
    if not re.fullmatch(r'[a-z][a-z0-9_]*', profile['model_name']):
        raise ValueError('Invalid generated model name')
    f = width / (2.0 * math.tan(hfov / 2.0))
    distortion = camera.find('distortion')
    if distortion is None:
        distortion = ET.SubElement(camera, 'distortion')
    for key in RENDER_D:
        put(distortion, key, '0.0')
    put(distortion, 'center', '0.5 0.5')
    for key in PLUGIN_D:
        put(plugin, key, '0.0')
    for key, value in dict(Cx=cx, CxPrime=cx, Cy=cy, focalLength=format(f, '.17g'),
                           hackBaseline='0.0', autoDistortion='true',
                           borderCrop='false', robotNamespace=profile['robot_namespace'],
                           cameraName=profile['camera_name'],
                           imageTopicName=profile['image_topic_name'],
                           cameraInfoTopicName=profile['camera_info_topic_name'],
                           frameName=profile['optical_frame']).items():
        put(plugin, key, value)
    contract = dict(
        scope='SIMULATION_DROP_COMPARISON_ONLY', profile_id=profile['profile_id'],
        simulation_started=False, camera_info_status='EXPECTED_NOT_RUNTIME_MEASURED',
        width=width, height=height, horizontal_fov=hfov,
        vertical_fov=2.0 * math.atan(height / (2.0 * f)),
        K=[f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0],
        D=[0.0] * 5, R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        P=[f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        distortion_model='plumb_bob', frame_id=profile['optical_frame'],
        camera_link_pose=link.findtext('pose'),
        sensor_pose=sensor.findtext('pose', '0 0 0 0 0 0'),
        renderer='symmetric perspective FOV; all five distortion terms zero',
        robot_namespace=profile['robot_namespace'], camera_name=profile['camera_name'],
        image_topic_name=profile['image_topic_name'],
        camera_info_topic_name=profile['camera_info_topic_name'])
    return tree, contract


def generate(template_path, profile_path, output_dir, *, enabled=False):
    if not enabled:
        raise ValueError('Explicit --enable-drop-camera-research is required')
    template_path, profile_path = Path(template_path).resolve(), Path(profile_path).resolve()
    output_dir = Path(output_dir).resolve()
    profile = json.loads(profile_path.read_text(encoding='utf-8'))
    tree, contract = make_model(ET.parse(template_path), profile)
    # A new directory is mandatory: never overwrite source/historical assets.
    output_dir.mkdir(parents=True, exist_ok=False)
    tree.write(output_dir / 'model.sdf', encoding='utf-8', xml_declaration=True)
    config = ET.Element('model')
    ET.SubElement(config, 'name').text = profile['model_name']
    ET.SubElement(config, 'version').text = '1.0'
    ET.SubElement(config, 'sdf', version='1.6').text = 'model.sdf'
    ET.SubElement(config, 'description').text = 'Opt-in drop comparison: centered K, renderer D0'
    ET.ElementTree(config).write(output_dir / 'model.config', encoding='utf-8', xml_declaration=True)
    contract.update(template=str(template_path), profile=str(profile_path),
                    generated_sdf=str(output_dir / 'model.sdf'))
    (output_dir / 'camera_contract.json').write_text(
        json.dumps(contract, indent=2) + '\n', encoding='utf-8')
    return contract


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--enable-drop-camera-research', action='store_true')
    parser.add_argument('--template', type=Path, default=DEFAULT_TEMPLATE)
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    try:
        result = generate(args.template, args.profile, args.output_dir,
                          enabled=args.enable_drop_camera_research)
    except (ValueError, OSError, KeyError, ET.ParseError) as exc:
        parser.error(str(exc))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()

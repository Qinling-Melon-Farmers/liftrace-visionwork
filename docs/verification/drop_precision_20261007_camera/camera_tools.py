"""Shared offline contracts; no ROS/Gazebo imports or control outputs."""
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
EVAL = ROOT / 'vision_ws/src/uav_vision_eval'
spec = importlib.util.spec_from_file_location(
    'drop_research_generator', EVAL / 'scripts/generate_drop_research_camera.py')
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


def close(actual, expected, label, tolerance=1e-6):
    if len(actual) != len(expected) or any(
            not math.isfinite(float(a)) or abs(float(a) - float(b)) > tolerance
            for a, b in zip(actual, expected)):
        raise ValueError('Camera contract mismatch: ' + label)


def validate_sdf(tree, contract):
    profile = json.loads(Path(contract['profile']).read_text(encoding='utf-8'))
    link, sensor, camera, plugin = generator.camera_elements(tree, profile)
    if [int(camera.findtext('image/width')), int(camera.findtext('image/height'))] != [
            contract['width'], contract['height']]:
        raise ValueError('Renderer image size mismatch')
    close([float(camera.findtext('horizontal_fov'))], [contract['horizontal_fov']], 'FOV', 1e-12)
    close([float(camera.findtext('distortion/' + k)) for k in generator.RENDER_D],
          contract['D'], 'renderer D', 1e-12)
    close([float(v) for v in camera.findtext('distortion/center').split()],
          [0.5, 0.5], 'renderer distortion center', 1e-12)
    close([float(plugin.findtext(k)) for k in generator.PLUGIN_D],
          contract['D'], 'plugin D', 1e-12)
    close([float(plugin.findtext(k)) for k in ('focalLength', 'Cx', 'Cy', 'CxPrime')],
          [contract['K'][0], contract['K'][2], contract['K'][5], contract['K'][2]], 'plugin K')
    close([float(plugin.findtext('hackBaseline'))], [0.0], 'baseline', 1e-12)
    for key, value in [('autoDistortion', 'true'), ('borderCrop', 'false'),
                       ('frameName', contract['frame_id']),
                       ('robotNamespace', contract['robot_namespace']),
                       ('cameraName', contract['camera_name']),
                       ('imageTopicName', contract['image_topic_name']),
                       ('cameraInfoTopicName', contract['camera_info_topic_name'])]:
        if plugin.findtext(key) != value:
            raise ValueError('Camera plugin mismatch: ' + key)
    close([float(v) for v in link.findtext('pose').split()],
          [float(v) for v in contract['camera_link_pose'].split()], 'camera mount', 1e-12)
    close([float(v) for v in sensor.findtext('pose', '0 0 0 0 0 0').split()],
          [float(v) for v in contract['sensor_pose'].split()], 'sensor pose', 1e-12)
    return dict(status='PASS', renderer_center=[contract['width']/2, contract['height']/2],
                renderer_D=[0.0]*5, published_K_expected=contract['K'],
                published_D_expected=contract['D'])


def validate_camera_info(info, contract):
    for name in ('width', 'height', 'distortion_model', 'frame_id'):
        if info.get(name) != contract[name]:
            raise ValueError('CameraInfo mismatch: ' + name)
    for name in ('K', 'P', 'R', 'D'):
        close(info.get(name, []), contract[name], 'CameraInfo ' + name,
              1e-12 if name == 'D' else 1e-6)
    if info.get('binning_x', 0) not in (0, 1) or info.get('binning_y', 0) not in (0, 1):
        raise ValueError('Unexpected CameraInfo binning')
    roi = info.get('roi', {})
    if roi.get('x_offset', 0) or roi.get('y_offset', 0) or roi.get('width', 0) not in (
            0, contract['width']) or roi.get('height', 0) not in (0, contract['height']):
        raise ValueError('Unexpected CameraInfo crop')
    return dict(status='PASS', width=info['width'], height=info['height'],
                K=info['K'], D=info['D'], frame_id=info['frame_id'])


def validate_world_sdf(text, contract, model_name):
    root = ET.fromstring(text)
    models = root.findall("world/model[@name='%s']" % model_name)
    if len(models) != 1:
        raise ValueError('Runtime world must contain exactly one vehicle: ' + model_name)
    selected = ET.Element('sdf', version=root.get('version', '1.6'))
    selected.append(models[0])
    result = validate_sdf(ET.ElementTree(selected), contract)
    result.update(model_name=model_name, source='Gazebo world_sdf transport response')
    return result


def write_new_json(path, value):
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2)
        stream.write('\n')

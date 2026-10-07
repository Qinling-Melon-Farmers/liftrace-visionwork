#!/usr/bin/env python3
"""Offline numerical/negative checks. NOT a Gazebo rendering measurement."""
import argparse
import copy
import json
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from camera_tools import (ROOT, generator, validate_sdf, validate_camera_info,
                          validate_world_sdf, write_new_json)

sys.path.insert(0, str(ROOT / 'vision_ws/src/uav_vision/src'))
from uav_vision.ground_projection import intersect_ground

D = Path(__file__).resolve().parent


def rpy_matrix(rpy):
    # SDF fixed-axis RPY: Rz(yaw) Ry(pitch) Rx(roll).
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array(
        [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array(
        [[1, 0, 0], [0, cr, -sr], [0, sr, cr]])


def projection_check(tree, contract, fixture):
    profile = json.loads(Path(contract['profile']).read_text())
    link, sensor, camera, plugin = generator.camera_elements(tree, profile)
    mount = np.array([float(v) for v in link.findtext('pose').split()])
    sensor_pose = np.array([float(v) for v in sensor.findtext('pose', '0 0 0 0 0 0').split()])
    sensor_r = rpy_matrix(mount[3:]) @ rpy_matrix(sensor_pose[3:])
    sensor_t = mount[:3] + rpy_matrix(mount[3:]) @ sensor_pose[:3]
    # Columns: ROS optical axes expressed in Gazebo sensor coordinates.
    sensor_from_optical = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    body_from_optical = sensor_r @ sensor_from_optical
    if not np.allclose(body_from_optical, np.diag([-1., 1., -1.]), atol=1e-12):
        raise ValueError('Template camera is not the reviewed start-frame extrinsic')
    width, height = contract['width'], contract['height']
    f = width / (2 * np.tan(float(camera.findtext('horizontal_fov')) / 2))
    render_k = np.array([[f, 0, width/2], [0, f, height/2], [0, 0, 1]], dtype=float)
    # Inverse uses independent publisher fields, not the forward matrix.
    pf = float(plugin.findtext('focalLength'))
    info_k = np.array([[pf, 0, float(plugin.findtext('Cx'))],
                       [0, pf, float(plugin.findtext('Cy'))], [0, 0, 1]])
    distortion = np.array([float(plugin.findtext(k)) for k in generator.PLUGIN_D])
    ground = fixture['ground_z']
    points = np.array([[x, y, ground] for x, y in fixture['points_xy']])
    rows, summaries = [], []
    for agl in fixture['camera_agl_m']:
        for rpy in fixture['body_rpy_deg']:
            body_r = rpy_matrix(np.deg2rad(rpy))
            # Fixed FC XY; choose FC Z so camera AGL remains the specified value.
            body_t = np.array([*fixture['body_xy'], ground + agl - (body_r @ sensor_t)[2]])
            optical_r = body_r @ body_from_optical
            origin = body_t + body_r @ sensor_t
            q = Rotation.from_matrix(optical_r).as_quat()
            optical_points = (optical_r.T @ (points - origin).T).T
            if np.any(optical_points[:, 2] <= 0):
                raise ValueError('Fixture unexpectedly places a point behind camera')
            pixels, _ = cv2.projectPoints(optical_points, np.zeros(3), np.zeros(3), render_k, np.zeros(5))
            pixels = pixels.reshape(-1, 2)
            rays = cv2.undistortPoints(pixels.reshape(-1, 1, 2), info_k, distortion).reshape(-1, 2)
            recovered = np.array([intersect_ground((u, v, 1.), tuple(origin), tuple(q), ground) for u, v in rays])
            errors = np.linalg.norm(recovered[:, :2] - points[:, :2], axis=1)
            reprojection, _ = cv2.projectPoints(
                (optical_r.T @ (recovered - origin).T).T, np.zeros(3), np.zeros(3), render_k, np.zeros(5))
            pixel_errors = np.linalg.norm(reprojection.reshape(-1, 2) - pixels, axis=1)
            visible = (pixels[:, 0] >= 0) & (pixels[:, 0] < width) & (pixels[:, 1] >= 0) & (pixels[:, 1] < height)
            edge = visible & ((pixels[:, 0] < 64) | (pixels[:, 0] > width-64) |
                              (pixels[:, 1] < 36) | (pixels[:, 1] > height-36))
            summaries.append(dict(camera_agl_m=agl, body_rpy_deg=rpy, checked_points=len(points),
                                  visible_points=int(sum(visible)), visible_edge_points=int(sum(edge)),
                                  max_ground_error_m=float(max(errors)), max_pixel_roundtrip_error=float(max(pixel_errors))))
            for i, (point, pixel, error, pe) in enumerate(zip(points, pixels, errors, pixel_errors)):
                rows.append(dict(point_index=i, ground_point=point.tolist(), camera_agl_m=agl,
                                 body_rpy_deg=rpy, pixel=pixel.tolist(), in_frame=bool(visible[i]),
                                 edge=bool(edge[i]), ground_error_m=float(error), pixel_error=float(pe)))
    level = [r for r in rows if r['camera_agl_m'] == 1.0 and r['body_rpy_deg'] == [0, 0, 0]]
    if len(level) != 9 or sum(r['edge'] for r in level) != 8:
        raise AssertionError('Fixed fixture must exercise eight visible edges/corners at 1m')
    return dict(samples=len(rows), visible_samples=sum(r['in_frame'] for r in rows),
                offscreen_samples=sum(not r['in_frame'] for r in rows),
                visible_edge_samples=sum(r['edge'] for r in rows),
                max_ground_error_m=max(r['ground_error_m'] for r in rows),
                max_pixel_roundtrip_error=max(r['pixel_error'] for r in rows),
                per_pose=summaries, reference_1m_level=level)


def nonoptical_signature(tree, profile):
    tree = copy.deepcopy(tree)
    _, sensor, camera, plugin = generator.camera_elements(tree, profile)
    camera.remove(camera.find('distortion'))
    sensor.remove(plugin)
    return ET.tostring(tree.getroot())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    fixture = json.loads((D / 'fixed_ground_points.json').read_text())
    before = generator.DEFAULT_TEMPLATE.read_bytes()
    profile = json.loads(generator.DEFAULT_PROFILE.read_text())
    rejected = []

    def must_reject(name, function):
        try:
            function()
        except (ValueError, FileExistsError):
            rejected.append(name)
        else:
            raise AssertionError('Bad input accepted: ' + name)

    with tempfile.TemporaryDirectory(prefix='drop_camera_math_') as tmp:
        out = Path(tmp) / 'camera'
        must_reject('missing_opt_in', lambda: generator.generate(
            generator.DEFAULT_TEMPLATE, generator.DEFAULT_PROFILE, out))
        if out.exists():
            raise AssertionError('Denied generator wrote output')
        contract = generator.generate(generator.DEFAULT_TEMPLATE, generator.DEFAULT_PROFILE, out, enabled=True)
        tree = ET.parse(out / 'model.sdf')
        validate_sdf(tree, contract)
        must_reject('existing_output', lambda: generator.generate(
            generator.DEFAULT_TEMPLATE, generator.DEFAULT_PROFILE, out, enabled=True))
        if nonoptical_signature(ET.parse(generator.DEFAULT_TEMPLATE), profile) != nonoptical_signature(tree, profile):
            raise AssertionError('Non-optical template content changed')
        info = {k: contract[k] for k in ('width', 'height', 'K', 'P', 'D', 'R', 'frame_id', 'distortion_model')}
        validate_camera_info(info, contract)
        for name, field, index, value in [('off_center_info', 'K', 5, 397.566381),
                                         ('wrong_info_focal', 'K', 0, 700.),
                                         ('wrong_rectified_P', 'P', 2, 631.),
                                         ('nonzero_info_D', 'D', 0, .01)]:
            bad = copy.deepcopy(info)
            bad[field][index] = value
            must_reject(name, lambda: validate_camera_info(bad, contract))
        for name, changes in [('wrong_optical_frame', dict(frame_id='other_camera')),
                              ('cropped_camera_info', dict(roi=dict(x_offset=5))),
                              ('wrong_resolution', dict(width=640))]:
            bad = dict(info, **changes)
            must_reject(name, lambda: validate_camera_info(bad, contract))
        bad_tree = copy.deepcopy(tree)
        _, _, bad_camera, _ = generator.camera_elements(bad_tree, profile)
        bad_camera.find('distortion/k1').text = '0.01'
        must_reject('renderer_D_nonzero_even_if_info_D0', lambda: validate_sdf(bad_tree, contract))
        must_reject('historical_SDF_not_research', lambda: validate_sdf(ET.parse(generator.DEFAULT_TEMPLATE), contract))
        synthetic_world = ET.Element('sdf', version='1.6')
        world = ET.SubElement(synthetic_world, 'world', name='test')
        model = copy.deepcopy(tree.getroot().find('model'))
        model.set('name', 'iris_mid360')
        world.append(model)
        validate_world_sdf(ET.tostring(synthetic_world), contract, 'iris_mid360')
        must_reject('wrong_runtime_model', lambda: validate_world_sdf(
            ET.tostring(synthetic_world), contract, 'other_vehicle'))
        projection = projection_check(tree, contract, fixture)
        if projection['max_ground_error_m'] > 1e-9 or projection['max_pixel_roundtrip_error'] > 1e-8:
            raise AssertionError('Numerical self-consistency FAIL')
        # Negative mathematical control: centered forward projection, old K, D0.
        mismatch = copy.deepcopy(tree)
        _, _, _, plugin = generator.camera_elements(mismatch, profile)
        plugin.find('Cx').text = '631.67186313702575'
        plugin.find('Cy').text = '397.56638133116269'
        diagnostic = projection_check(mismatch, contract, fixture)
        if diagnostic['max_ground_error_m'] < .01:
            raise AssertionError('Mismatched principal point was not detected numerically')
        if generator.DEFAULT_TEMPLATE.read_bytes() != before:
            raise AssertionError('Source SDF modified')
        result = dict(status='PASS', scope='OFFLINE_MATH_AND_CONFIG_ONLY',
                      simulation_started=False, gazebo_rendering_measured=False,
                      source_template_unchanged=True, nonoptical_template_unchanged=True,
                      expected_K=contract['K'], expected_D=contract['D'],
                      negative_checks_rejected=rejected, numerical=projection,
                      mismatched_K_D0_diagnostic_max_error_m=diagnostic['max_ground_error_m'],
                      limitations='No renderer image, runtime CameraInfo or flight precision measured; offscreen rays are mathematical checks only.')
    if args.output:
        write_new_json(args.output, result)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()

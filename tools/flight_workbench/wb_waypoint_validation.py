# Pure site checks for embedded waypoint editing.
import math
import yaml

ALLOWED = {'flight_area', 'search_line_x', 'compressed_image_topic', 'high_agl', 'max_agl',
           'terminal_hover_agl', 'auto_start_after_arm', 'initialization_timeout', 'obstacle_columns_enabled',
           'bag_image_hz', 'bag_image_topic', 'record_inflated_cloud', 'record_map_clouds',
           'landing_xy', 'landing_handoff_mode', 'landing_handoff_status_topic', 'landing_posctl',
           'landing_alignment_mode', 'landing_descent_mode', 'landing_descent_speed_mps', 'landing_offboard',
           'corridor_waypoints', 'corridor_geometry'}


def validate(site):
    errors = []
    finite = lambda v: type(v) in (int, float) and math.isfinite(v)
    if not isinstance(site, dict):
        return ['文件顶层必须是配置对象']
    if set(site) - ALLOWED:
        errors.append('含现场配置不支持的字段：' + ', '.join(sorted(set(site) - ALLOWED)))
    area = site.get('flight_area')
    if not isinstance(area, dict):
        return errors + ['需要 flight_area 地图范围']
    if set(area) - {'center_bounds', 'target_bounds', 'search_bounds', 'staging_xy', 'survey_xy', 'map_size'}:
        errors.append('flight_area 含不支持的字段')
    boxes = {}
    for key in ('center_bounds', 'target_bounds', 'search_bounds'):
        box = area.get(key)
        if not isinstance(box, list) or len(box) != 4 or not all(finite(v) for v in box) or box[0] >= box[1] or box[2] >= box[3]:
            errors.append(key + ' 必须是 [X最小, X最大, Y最小, Y最大]')
        else:
            boxes[key] = box
    def inside(point, box):
        return isinstance(point, list) and len(point) == 2 and all(finite(v) for v in point) and box[0] <= point[0] <= box[1] and box[2] <= point[1] <= box[3]
    center = boxes.get('center_bounds')
    if center:
        if not inside([0, 0], center): errors.append('飞行边界必须包含起飞原点')
        if not inside(area.get('staging_xy'), center): errors.append('准备点必须位于飞行边界内')
        for key, outer, inner in [('target_bounds', boxes.get('target_bounds'), center), ('search_bounds', center, boxes.get('search_bounds'))]:
            if outer and inner and not all(inside(p, outer) for p in ([inner[0], inner[2]], [inner[1], inner[3]])):
                errors.append(key + ' 的范围包含关系不正确')
        landing = site.get('landing_xy')
        mission_box = [max(.35, center[0]), center[1], center[2], center[3]]
        if not inside(landing, mission_box): errors.append('H点须位于飞行边界内且 X ≥ 0.35m')
    survey = area.get('survey_xy')
    if not isinstance(survey, list) or not 3 <= len(survey) <= 8:
        errors.append('搜索航点数量须为 3～8 个（与当前高位任务兼容）')
    elif center and any(not inside(p, center) for p in survey):
        errors.append('搜索航点须位于飞行边界内')
    size = area.get('map_size')
    if not isinstance(size, list) or len(size) != 3 or not all(finite(v) and v > 0 for v in size):
        errors.append('地图尺寸必须是三个正数')
    elif center and (max(abs(center[0]), abs(center[1])) + .3 >= size[0]/2 or max(abs(center[2]), abs(center[3])) + .3 >= size[1]/2):
        errors.append('地图 XY 尺寸不足：需覆盖飞行边界并预留 0.30m 起飞参考偏差')
    high = site.get('high_agl')
    cap = site.get('max_agl')
    if not finite(high) or not 2 <= high <= 3: errors.append('高位搜索 AGL 须为 2.0～3.0m')
    if not finite(cap) or (finite(high) and cap < high): errors.append('最大高度须不低于高位搜索高度')
    if isinstance(size, list) and len(size) == 3 and all(finite(v) for v in size) and finite(cap) and size[2] <= cap + .1:
        errors.append('地图 Z 尺寸须大于最大 AGL + 0.10m')
    points = site.get('corridor_waypoints')
    if not isinstance(points, list) or len(points) < 2:
        errors.append('走廊至少需要两个实测航点')
    else:
        for i, point in enumerate(points):
            if not isinstance(point, dict) or set(point) - {'x', 'y', 'agl'}:
                errors.append('走廊点 %d 格式错误' % (i+1)); continue
            if center and not inside([point.get('x'), point.get('y')], mission_box):
                errors.append('走廊点 %d 须位于飞行边界内且 X ≥ 0.35m' % (i+1))
            height = point.get('agl', 1.4)
            if not finite(height) or not .5 <= height <= 1.8:
                errors.append('走廊点 %d 的 AGL 须为 0.5～1.8m' % (i+1))
    geometry = site.get('corridor_geometry')
    if geometry is not None:
        if not isinstance(geometry, dict) or set(geometry) != {'wall_axis', 'wall_coordinates', 'entry_waypoints'}:
            errors.append('门墙配置需要 wall_axis、wall_coordinates、entry_waypoints')
        else:
            if type(geometry['wall_axis']) is not int or geometry['wall_axis'] not in (0, 1): errors.append('墙面轴须为 X=0 或 Y=1')
            walls = geometry['wall_coordinates']
            if not isinstance(walls, list) or not walls or not all(finite(v) for v in walls): errors.append('至少填写一道有效墙面坐标')
            entry = geometry['entry_waypoints']
            if type(entry) is not int or entry not in (1, 2) or not isinstance(points, list) or entry > len(points): errors.append('入口航点数须为 1 或 2，且不超过走廊点数')
            if isinstance(points, list) and type(entry) is int and entry >= 1:
                for point in points[entry-1:]:
                    if isinstance(point, dict) and finite(point.get('agl', 1.4)) and point.get('agl', 1.4) > 1.0:
                        errors.append('默认运动优化的走廊高度上限为 1.0m；入口交接点及之后的 AGL 请设为 ≤1.0m'); break
    line = site.get('search_line_x')
    if not isinstance(line, list) or len(line) < 2 or not all(finite(v) for v in line) or any(a >= b for a, b in zip(line, line[1:])) or (center and any(not max(.35, center[0]) <= v <= center[1] for v in line)):
        errors.append('search_line_x 须为飞行边界内递增的至少两个 X 坐标')
    return errors


def dump_site(site):
    return '# 地图航点工具导出；XY相对静止起飞参考，+X前、+Y左，AGL为飞控中心离地高度。\n' + yaml.safe_dump(site, allow_unicode=True, sort_keys=False)

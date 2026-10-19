"""Offline low-coverage backup configuration; reuse SearchPolicy and MissionRuntime."""
import copy
import math
from uav_mission.search_policy import SearchPolicy

PROFILE = 'low_coverage_full'
FOLDER = '10_low_coverage_full'
HEIGHTS = (1.8, 1.9)
AREA_FIELDS = {'center_bounds', 'target_bounds', 'search_bounds', 'staging_xy', 'survey_xy', 'map_size'}


def is_low_coverage(settings):
    return settings.get('backup_profile') == PROFILE


def inherit_full_settings(full, overlay):
    settings = copy.deepcopy(full)
    # 高位续扫开关不属于普通覆盖任务；CoverageRoute自行保存中断游标。
    settings.pop('resume_survey_enabled', None)
    settings.pop('survey_pattern', None)
    settings.pop('survey_policy', None)
    # 必须由显式现场profile提供几何，08将来新增默认场地也不能泄漏进本备份。
    for key in ('flight_area','search_line_x','search_waypoints_xy','corridor_geometry'):
        settings.pop(key, None)
    settings.update(copy.deepcopy(overlay))
    return settings


def normalize_coverage_site(profile):
    """Accept a measured trial profile or the existing independent waypoint-map schema."""
    if not isinstance(profile, dict):
        raise ValueError('Coverage site must be a mapping')
    if 'flight_area' in profile:
        return copy.deepcopy(profile)
    required = {'flight_bounds','target_bounds','coverage_bounds','staging_xy','survey_xy',
                'map_size','corridor_waypoints','landing_xy'}
    allowed = required | {'search_center_bounds','high_agl','low_agl',
                          'landing_transit_agl','landing_capture_agl','corridor_speed_schedule',
                          'coverage_pattern','coverage_rectangles','coverage_cross_inset',
                          'coverage_lane_spacing','coverage_scan_axis','coverage_loop_inset'}
    if not required <= set(profile) or set(profile)-allowed:
        raise ValueError('Coverage waypoint map requires complete measured bounds/corridor/H')
    result = dict(flight_area={
        'center_bounds': copy.deepcopy(profile['flight_bounds']),
        'target_bounds': copy.deepcopy(profile['target_bounds']),
        'search_bounds': copy.deepcopy(profile['coverage_bounds']),
        'staging_xy': copy.deepcopy(profile['staging_xy']),
        'survey_xy': copy.deepcopy(profile['survey_xy']),
        'map_size': copy.deepcopy(profile['map_size'])},
        corridor_waypoints=copy.deepcopy(profile['corridor_waypoints']),
        landing_xy=copy.deepcopy(profile['landing_xy']))
    for key in ('landing_transit_agl','landing_capture_agl','coverage_pattern','coverage_rectangles',
                'coverage_cross_inset','coverage_lane_spacing','coverage_scan_axis','coverage_loop_inset'):
        if key in profile:
            result[key] = copy.deepcopy(profile[key])
    if 'corridor_speed_schedule' in profile:
        geometry = profile['corridor_speed_schedule']
        if not isinstance(geometry, dict) or set(geometry) != {'axis','wall_coordinates','entry_waypoints'}:
            raise ValueError('Coverage waypoint-map corridor schedule requires axis/walls/entry')
        result['corridor_geometry'] = dict(wall_axis=geometry['axis'],
            wall_coordinates=copy.deepcopy(geometry['wall_coordinates']),entry_waypoints=geometry['entry_waypoints'])
    return result


def prepare_low_coverage(settings, *, site_provided, height=None, lane_spacing=None, scan_axis=None, pattern=None, cross_inset=None):
    if not is_low_coverage(settings):
        if height is not None or lane_spacing is not None or scan_axis is not None or pattern is not None or cross_inset is not None:
            raise ValueError('Coverage options only apply to low_coverage_full')
        return settings
    if not site_provided:
        raise ValueError('low_coverage_full requires a loaded registered or explicit site config')
    if height is not None:
        settings['low_agl'] = height
    if lane_spacing is not None:
        settings['coverage_lane_spacing'] = lane_spacing
    if scan_axis is not None:
        settings['coverage_scan_axis'] = scan_axis
    if pattern is not None:
        settings['coverage_pattern'] = pattern
    if cross_inset is not None:
        settings['coverage_cross_inset'] = cross_inset
    # 现场高位航线高度不参与牛耕；不让兼容字段触发一段高位搜索。
    settings['high_agl'] = 2.0
    settings['obstacle_columns_enabled'] = False
    settings['virtual_ceiling_enabled'] = False
    return settings


def validate_low_coverage(settings):
    if not is_low_coverage(settings):
        return
    if settings.get('mode') != 'low_multi_full' or settings.get('trial_kind') != 'low_multi_corridor':
        raise ValueError('Coverage backup must retain the full corridor/H mission')
    if type(settings.get('low_agl')) not in (int, float) or settings['low_agl'] not in HEIGHTS:
        raise ValueError('Coverage FC AGL must be 1.8 or 1.9 metres')
    if settings.get('coverage_pattern','lawnmower') not in ('lawnmower','nested_rectangles'):
        raise ValueError('coverage_pattern must be lawnmower or nested_rectangles')
    if settings.get('coverage_scan_axis') not in ('x','y'):
        raise ValueError('coverage_scan_axis must be x or y')
    spacing = settings.get('coverage_lane_spacing')
    if type(spacing) not in (int, float) or not math.isfinite(spacing) or not .2 <= spacing <= 3.0:
        raise ValueError('coverage_lane_spacing must be within [0.2,3.0] metres')
    loop_inset=settings.get('coverage_loop_inset')
    if type(loop_inset) not in (int,float) or not math.isfinite(loop_inset) or loop_inset<=0:
        raise ValueError('coverage_loop_inset must be finite and positive')
    inset=settings.get('coverage_cross_inset')
    if type(inset) not in (int,float) or not math.isfinite(inset) or inset<=0:
        raise ValueError('coverage_cross_inset must be finite and positive')
    area = settings.get('flight_area')
    if not isinstance(area, dict) or set(area) != AREA_FIELDS:
        raise ValueError('Coverage requires a complete measured flight_area; default geometry is forbidden')
    if settings.get('obstacle_columns_enabled') is not False or settings.get('virtual_ceiling_enabled') is not False:
        raise ValueError('Coverage backup disables virtual columns and ceiling only')
    if settings.get('delivery_count') != 3:
        raise ValueError('Coverage full mission requires three deliveries')
    if settings.get('terminal_hover_agl') is not None or settings.get('resume_survey_enabled', False):
        raise ValueError('Coverage uses normal route resume and visual H tail')
    # 保留搜索/投后恢复余量；不能沿用低位1.85米上限卡住1.9米航点。
    if float(settings.get('max_agl', 0.)) < settings['low_agl'] + .25:
        raise ValueError('Coverage max_agl requires at least 0.25m above FC search height')
    if len(area.get('map_size', [])) != 3 or area['map_size'][2] < settings['low_agl'] + .5:
        raise ValueError('Coverage planner map Z must retain search/recovery clearance')


def inset_coverage_bounds(settings, area):
    """Inset the cross-scan axis only; keep the measured source bounds unchanged."""
    bounds=list(area['search_bounds'])
    axis=0 if settings['coverage_scan_axis']=='y' else 2
    bounds[axis]+=settings['coverage_cross_inset']
    bounds[axis+1]-=settings['coverage_cross_inset']
    if not bounds[axis]<bounds[axis+1]:
        raise ValueError('coverage_cross_inset leaves no usable cross-scan range')
    return [round(v,10) for v in bounds]


def rectangle_waypoints(settings, area):
    """Two nested loops, explicitly configured or derived from usable measured bounds."""
    explicit='coverage_rectangles' in settings
    rectangles=copy.deepcopy(settings.get('coverage_rectangles'))
    if not explicit:
        outer=inset_coverage_bounds(settings,area)
        inner=outer[:]
        cross=0 if settings['coverage_scan_axis']=='y' else 2
        along=2 if cross==0 else 0
        loop_inset=settings['coverage_loop_inset']
        inner[cross]+=loop_inset;inner[cross+1]-=loop_inset
        inner[along]+=loop_inset/2;inner[along+1]-=loop_inset/2
        rectangles=[outer,[round(v,10) for v in inner]]
    if not isinstance(rectangles,list) or len(rectangles)!=2:
        raise ValueError('coverage_rectangles requires two explicit [xmin,xmax,ymin,ymax] bounds')
    for box in rectangles:
        if (not isinstance(box,list) or len(box)!=4
                or any(type(v) not in (int,float) or not math.isfinite(v) for v in box)
                or not box[0]<box[1] or not box[2]<box[3]):
            reason=('Invalid coverage rectangle bounds' if explicit else
                    'nested_rectangles unavailable for this area/cross-inset/loop-inset; provide valid explicit rectangles or adjust parameters')
            raise ValueError(reason)
    outer,inner=rectangles
    if not (outer[0]<inner[0]<inner[1]<outer[1] and outer[2]<inner[2]<inner[3]<outer[3]):
        raise ValueError('Second coverage rectangle must lie strictly inside the first')
    points=[]
    for index,(x0,x1,y0,y1) in enumerate(rectangles):
        loop=([[x0,y0],[x0,y1],[x1,y1],[x1,y0],[x0,y0]]
              if settings['coverage_scan_axis']=='y' else
              [[x0,y0],[x1,y0],[x1,y1],[x0,y1],[x0,y0]])
        if index:
            # 圈间先沿横向再沿纵向连接，保留折点，不用对角替代。
            connector=([loop[0][0],points[-1][1]] if settings['coverage_scan_axis']=='y'
                       else [points[-1][0],loop[0][1]])
            points.append(connector)
        points.extend(loop)
    return points


def coverage_waypoints(settings, area):
    """Use editor waypoints when present; otherwise reuse SearchPolicy with the selected sweep axis."""
    bounds = area['search_bounds']
    usable=inset_coverage_bounds(settings,area)
    if 'search_waypoints_xy' in settings:
        points = copy.deepcopy(settings['search_waypoints_xy'])
        source = 'site.search_waypoints_xy'
    elif settings.get('coverage_pattern','lawnmower')=='nested_rectangles':
        points=rectangle_waypoints(settings,area)
        source=('nested_rectangles(coverage_rectangles)' if 'coverage_rectangles' in settings
                else 'nested_rectangles(inset_search_bounds)')
    else:
        x0, x1, y0, y1 = usable
        cross_span=(x1-x0) if settings['coverage_scan_axis']=='y' else (y1-y0)
        if math.ceil(cross_span/settings['coverage_lane_spacing']) + 1 > 50:
            raise ValueError('Coverage route would exceed 100 endpoints; adjust measured area or spacing')
        if settings['coverage_scan_axis']=='y':
            route = SearchPolicy(y0, y1, x0, x1, settings['coverage_lane_spacing'], settings['low_agl'])
            points = [[point.y, point.x] for point in route.waypoints]
        else:
            route = SearchPolicy(x0, x1, y0, y1, settings['coverage_lane_spacing'], settings['low_agl'])
            points = [[point.x, point.y] for point in route.waypoints]
        source = 'SearchPolicy(inset_search_bounds)'
    if not isinstance(points, list) or not 2 <= len(points) <= 100:
        raise ValueError('Coverage requires 2..100 measured/generated XY waypoints')
    for point in points:
        if (not isinstance(point, list) or len(point) != 2
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in point)
                or not bounds[0] <= point[0] <= bounds[1]
                or not bounds[2] <= point[1] <= bounds[3]):
            raise ValueError('Coverage waypoint must lie inside measured search_bounds')
    return points, source

"""Load unmodified source without replacing the current generated ROS messages."""
import importlib
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[1]
SOURCE = HERE/'source'
FAST_SCRIPTS = SOURCE/'patrol_uav_ws-patrol_planner/src/FAST_LIO/scripts'
MISSION_SOURCE = SOURCE/'patrol_uav_ws-patrol_planner/src/uav_mission/src'
MISSION_PACKAGE = MISSION_SOURCE/'uav_mission'
MISSION_SCRIPTS = SOURCE/'patrol_uav_ws-patrol_planner/src/uav_mission/scripts'
MISSION_TESTS = SOURCE/'patrol_uav_ws-patrol_planner/src/uav_mission/test'
TOOLS = SOURCE/'tools/ev_continuity'


def under(path, root):
    try:
        Path(path).resolve().relative_to(root.resolve())
        return True
    except ValueError:
        return False


def components(ros=False):
    if ros:
        package = importlib.import_module('uav_mission')
        # Message module remains the one selected by the 0928 workspace.
        if str(MISSION_PACKAGE) not in package.__path__:
            package.__path__.insert(0, str(MISSION_PACKAGE))
    else:
        sys.path.insert(0, str(MISSION_SOURCE))
    for path in (FAST_SCRIPTS, MISSION_SCRIPTS, MISSION_TESTS):
        sys.path.insert(0, str(path))


def message_checks(prediction=False):
    result = {}
    expected = ROOT/'patrol_uav_ws-patrol_planner/devel'
    from uav_mission.msg import ReleasePermission
    module = importlib.import_module(ReleasePermission.__module__)
    if not under(module.__file__, expected):
        raise RuntimeError('ReleasePermission must come from the current 0928 workspace: '+module.__file__)
    required = {'mission_id', 'decision_seq', 'attempt', 'target_first_seen',
                'permission_epoch', 'permission_revision'}
    if not required.issubset(ReleasePermission.__slots__):
        raise RuntimeError('current ReleasePermission generation is stale; rebuild common update first')
    result['ReleasePermission'] = dict(path=module.__file__, md5=ReleasePermission._md5sum,
                                      slots=list(ReleasePermission.__slots__))
    if prediction:
        from fast_lio.msg import PredictionState
        module = importlib.import_module(PredictionState.__module__)
        if not under(module.__file__, expected):
            raise RuntimeError('PredictionState must be newly generated in the SAME workspace: '+module.__file__)
        fields = {'header', 'imu_frame', 'epoch', 'valid', 'pose', 'velocity',
                  'gyro_bias', 'accel_bias', 'gravity', 'covariance', 'process_noise',
                  'accel_scale', 'imu_time_offset'}
        if set(PredictionState.__slots__) != fields:
            raise RuntimeError('PredictionState schema mismatch')
        result['PredictionState'] = dict(path=module.__file__, md5=PredictionState._md5sum)
    return result

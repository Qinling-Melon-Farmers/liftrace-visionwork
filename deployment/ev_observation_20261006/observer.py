#!/usr/bin/env python3
"""Preview/check by default. Explicit --start runs only an observer on an existing master."""
import argparse
import json
import os
from pathlib import Path
import socket
import sys

from ev_suite.bootstrap import HERE, ROOT, SOURCE, components, message_checks
from ev_suite.isolation import SHADOW, RESET, PublisherGuard, config_check


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode', choices=['preview', 'check', 'shadow', 'reset', 'graph-check'])
    p.add_argument('--kind', choices=['shadow', 'reset', 'all'], default='all')
    p.add_argument('--config', type=Path)
    p.add_argument('--start', action='store_true')
    return p


def load_config(kind, path=None):
    import yaml
    default = (HERE/'config/shadow.yaml' if kind == 'shadow' else
               SOURCE/'patrol_uav_ws-patrol_planner/src/uav_mission/config/ev_task_boundary.example.yaml')
    config = yaml.safe_load((path or default).read_text(encoding='utf-8'))
    config_check(kind, config)
    return config


def master():
    from urllib.parse import urlparse
    uri = os.environ.get('ROS_MASTER_URI', '')
    parsed = urlparse(uri)
    if parsed.hostname not in ('127.0.0.1', 'localhost') or parsed.scheme != 'http':
        raise RuntimeError('only the existing local board master is allowed; source source.sh')
    import rosgraph
    socket.setdefaulttimeout(3.0)
    result = rosgraph.Master('/ev_observation_check')
    result.getPid()  # Read-only; never creates a master or starts any service.
    return result


def graph_check():
    state = master().getSystemState()
    publishers, subscribers, services = state
    expected = {'/ev_shadow': set(SHADOW), '/ev_task_boundary': set(RESET)}
    details = {}
    for node, allowed in expected.items():
        topics = {topic for topic, nodes in publishers if node in nodes}
        if topics and topics != allowed:
            raise RuntimeError('unexpected publisher set for '+node+': '+str(sorted(topics)))
        advertised = {name for name, nodes in services if node in nodes}
        if advertised - {node+'/get_loggers', node+'/set_logger_level'}:
            raise RuntimeError('observer unexpectedly advertises non-logging services')
        details[node] = dict(present=bool(topics), publishers=sorted(topics), services=sorted(advertised))
    details['formal_ev_publishers'] = next((nodes for topic, nodes in publishers
                                          if topic == '/mavros/vision_pose/pose'), [])
    details['note'] = 'Compare formal EV publisher list with pre-start baseline; subscribers may consume observers but must not bridge outputs to control.'
    return details


def start(kind, config):
    import rospy
    # Reject inherited namespace/name changes and every ROS command-line remap.
    if os.environ.get('ROS_NAMESPACE', '/') not in ('', '/'):
        raise RuntimeError('ROS_NAMESPACE override is not allowed')
    master_api = master()
    node_name = '/ev_shadow' if kind == 'shadow' else '/ev_task_boundary'
    publishers, subscribers, services = master_api.getSystemState()
    if any(node_name in nodes for _, nodes in publishers+subscribers+services):
        raise RuntimeError('observer already registered; refusing duplicate node')
    output_topics = SHADOW if kind == 'shadow' else RESET
    if any(topic in output_topics and nodes for topic, nodes in publishers):
        raise RuntimeError('another publisher already owns an observation output')
    rospy.init_node(node_name[1:], argv=[sys.argv[0]], disable_signals=False, disable_rosout=True)
    if rospy.get_name() != node_name:
        raise RuntimeError('observer node name changed')
    for key, value in config.items():
        rospy.set_param('~'+key, value)
    # Replace all parameter access with this validated mapping. Old private
    # output/namespace parameters on an existing master cannot steer the node.
    def param(key, default=None):
        if not key.startswith('~'):
            raise ValueError('observer must use private parameters only')
        return config.get(key[1:], default)
    rospy.get_param = param
    guard = PublisherGuard(rospy, kind)
    guard.install()
    if kind == 'shadow':
        from ev_shadow import Shadow
        observer = Shadow()
    else:
        from ev_task_boundary import EvTaskBoundary
        observer = EvTaskBoundary()
        observer.core._block('calibration_not_verified;online_reset_and_lio_health_contracts_unavailable')
    guard.complete()
    print(json.dumps(dict(observe_only=True, node=node_name,
                         publishers=sorted(guard.created)), indent=2), flush=True)
    rospy.spin()


def main():
    args = parser().parse_args()
    if args.start and args.mode not in ('shadow', 'reset'):
        parser().error('--start is only valid for shadow or reset')
    if args.config and args.mode not in ('shadow', 'reset'):
        parser().error('--config is only valid for shadow or reset')
    if args.mode == 'preview':
        print(json.dumps(dict(default='no nodes started', suite=str(HERE), shared_workspace=str(ROOT),
            whitelist={'shadow': SHADOW, 'reset': RESET}, prediction_state_default=False,
            calibration_verified=False, missing_contracts=['authoritative reset', 'independent LIO health'],
            start_requires='explicit shadow/reset --start; already running local ROS master'), indent=2))
        return 0
    if args.mode == 'graph-check':
        print(json.dumps(graph_check(), indent=2))
        return 0
    kind = args.mode if args.mode in ('shadow', 'reset') else args.kind
    components(ros=True)
    schemas = message_checks(prediction=kind != 'reset')
    configs = {k: load_config(k, args.config) for k in
               (('shadow', 'reset') if kind == 'all' else (kind,))}
    print(json.dumps(dict(prerequisites='PASS', messages=schemas, observe_only=True,
                         calibration_verified=False, started=False), indent=2), flush=True)
    if args.start:
        start(kind, configs[kind])
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ImportError, RuntimeError, ValueError, KeyError, OSError) as exc:
        print('BLOCKED (nothing auto-started): '+str(exc), file=sys.stderr)
        sys.exit(3)

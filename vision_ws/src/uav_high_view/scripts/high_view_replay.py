#!/usr/bin/env python3
"""Bounded JSONL research replay. Does not import rospy or communicate remotely."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
from uav_high_view.core import (Catalog, Config, Epoch, Key, Observation, Edge,
                                rank_routes, survey_decision)


def replay(config, stream, output):
    catalog = Catalog(Config(**config['catalog']), config['weights'])
    edges = {}
    while True:
        line = stream.readline(65537)
        if not line:
            break
        if len(line) > 65536:
            raise ValueError('event exceeds 64 KiB')
        event = json.loads(line)
        kind = event['type']
        result = {'scope': 'OFFLINE_ONLY_NO_FLIGHT_OUTPUT', 'type': kind}
        if kind == 'context':
            catalog.reset(Epoch(**event['epoch']))
            edges.clear()
            result['reason'] = 'context_loaded'
        elif kind == 'observation':
            data = dict(event['observation'])
            data['epoch'] = Epoch(**data['epoch'])
            data['key'] = Key(**data['key'])
            data['xy'] = tuple(data['xy'])
            result['reason'] = catalog.observe(Observation(**data), event['now_ns'])
        elif kind == 'delivery_ack':
            result['accepted'] = catalog.record_delivery(event['slot'], event['class_name'])
        elif kind == 'revisit':
            result['accepted'] = catalog.begin_revisit(Key(**event['key']), event['now_ns'])
        elif kind == 'edge':
            data = dict(event['edge']); data['epoch'] = Epoch(**data['epoch'])
            edge = Edge(**data); pair = edge.source, edge.target
            if pair not in edges and len(edges) >= 64:
                raise ValueError('edge cap reached')
            edges[pair] = edge
            result['reason'] = 'declared_cost_loaded_not_planner_verified'
        elif kind == 'snapshot':
            now = event['now_ns']
            hints = catalog.hints(now)
            result['hints'] = [asdict(h) for h in hints]
            result['delivered_slots'] = dict(catalog.delivered)
            proposal = None
            if catalog.epoch is not None and len(catalog.delivered) < 3:
                rows = event.get('service_costs', [])
                if len(rows) > 48:
                    raise ValueError('service cost cap reached')
                costs = {(v['target'], v['slot']): v['seconds'] for v in rows}
                if len(costs) != len(rows):
                    raise ValueError('duplicate service cost')
                proposal = rank_routes(hints, catalog.weights, list(edges.values()),
                    catalog.epoch, now, 3-len(catalog.delivered), costs,
                    event['remaining_seconds'], event['reserve_seconds'],
                    event.get('fixed_seconds', 0.))
            result['route'] = asdict(proposal) if proposal else None
            # A data file cannot turn an unvalidated P0 into flight authorization.
            result['decision'] = survey_decision(event['elapsed_seconds'],
                event.get('survey_budget', 45.), proposal, event['desired_weight'],
                catalog.epoch, now, p0_passed=False)
        else:
            raise ValueError('unknown event type: '+str(kind))
        output.write(json.dumps(result, sort_keys=True, allow_nan=False)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if Path(args.input).resolve() == Path(args.output).resolve():
        raise ValueError('input must not be overwritten')
    config = json.loads(Path(args.config).read_text())
    with open(args.input) as source, open(args.output, 'x') as destination:
        replay(config, source, destination)


if __name__ == '__main__':
    main()

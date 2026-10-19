#!/usr/bin/env python3
"""Read cached flight records; extract only missing timing topics from bags."""
import argparse, collections, json, math, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PRODUCTS = ROOT / '试飞产物'
OUTPUT = PRODUCTS / 'analysis_flight_efficiency_20261011'
TOPICS = ['/mavros/state', '/mavros/extended_state', '/mavros/local_position/odom',
 '/navigation/local_pose', '/mavros/setpoint_position/local', '/navigation/setpoint_mission',
 '/mission/control_ready', '/rosout_agg', '/board_trials/run_metadata',
 '/navigation/mission_status', '/navigation/mission_command_raw', '/navigation/mission_result',
 '/navigation/planner_bridge_status', '/fastplanner/goal', '/planning/bspline',
 '/planning/pos_cmd', '/planning/progress', '/planning/goal_status', '/uav_high_view/probe_status',
 '/mission/release_result']

def load(n):
    OUTPUT.mkdir(parents=True, exist_ok=True)
    cache = OUTPUT / ('timing_topics_%s.json' % n)
    if cache.exists():
        return json.loads(cache.read_text())
    rows = collections.defaultdict(list)
    sources = []
    if n == 1:
        source = PRODUCTS / 'analysis_flight_10111009_1/collision_1_records.json'
        doc = json.loads(source.read_text())
        start = doc['metadata']['start_epoch']
        duration = doc['metadata']['duration']
        for topic in TOPICS:
            rows[topic] = doc['rows'].get(topic, [])
        sources.append(str(source.relative_to(ROOT)))
    else:
        inventory = json.loads((PRODUCTS / 'analysis_10111009_2/inventory.json').read_text())
        start, duration = inventory['start'], inventory['duration']
        for name in ('events.json', 'extra_events.json'):
            source = PRODUCTS / 'analysis_10111009_2' / name
            for row in json.loads(source.read_text()):
                if row['topic'] in TOPICS:
                    rows[row['topic']].append(dict(t=row['t'], m=row['msg']))
            sources.append(str(source.relative_to(ROOT)))
        source = PRODUCTS / 'replay_10111009_2/data.json'
        replay = json.loads(source.read_text())
        for key, topic in replay['topics'].items():
            if topic in TOPICS and not rows[topic]:
                rows[topic] = replay['rows'][key]
        sources.append(str(source.relative_to(ROOT)))
    missing = [topic for topic in TOPICS if not rows[topic]]
    sys.path.insert(0, str(ROOT / 'tools/bag_replay'))
    from bag_replay import plain
    import rosbag
    bag_path = PRODUCTS / ('flight_debug_0_10111009_%s.bag' % n)
    with rosbag.Bag(str(bag_path), 'r') as bag:
        available = bag.get_type_and_topic_info().topics
        needed = [topic for topic in missing if topic in available]
        for topic, msg, stamp in bag.read_messages(topics=needed):
            rows[topic].append(dict(t=stamp.to_sec()-start, m=plain(msg)))
    for topic in rows:
        rows[topic].sort(key=lambda r: r['t'])
    result = dict(round=n, bag=str(bag_path.relative_to(ROOT)), start=start,
                  duration=duration, cache_sources=sources, bag_extracted_topics=needed,
                  missing_topics=[t for t in TOPICS if not rows[t]], rows=rows)
    cache.write_text(json.dumps(result, ensure_ascii=False), encoding='utf-8')
    return result

def decoded(rows, topic):
    return [dict(r, m=json.loads(r['m']['data'])) for r in rows[topic]]

def compact(doc):
    rows = doc['rows']
    stages, events, seen, previous = [], [], set(), None
    for r in decoded(rows, '/uav_high_view/probe_status'):
        m = r['m']
        key = (m.get('stage'), m.get('ascent_verified'))
        if key != previous:
            stages.append(dict(t=r['t'], stage=key[0], ascent_verified=key[1]))
            previous = key
        for event in m.get('events', []):
            token = json.dumps(event, sort_keys=True)
            if token not in seen:
                seen.add(token)
                events.append(dict(event, t=event.get('time', doc['start'])-doc['start']))
    return dict(round=doc['round'], missing=doc['missing_topics'], extracted=doc['bag_extracted_topics'],
                counts={t:len(r) for t,r in rows.items()}, stages=stages, events=events)

def first(rows, predicate):
    return next((r for r in rows if predicate(r)), None)

def prior(rows, t):
    return next((r for r in reversed(rows) if r['t'] <= t), None)

def sustained(rows, begin, end, predicate):
    """0.2s persistence is an analysis noise filter, never a control gate."""
    run, previous = None, None
    for row in rows:
        if row['t'] < begin or row['t'] > end:
            continue
        if not predicate(row) or (previous is not None and row['t']-previous > .15):
            run = None
        if predicate(row):
            run = run or row
            if row['t']-run['t'] >= .2:
                return run
        previous = row['t']
    return None

def position(row):
    return row['m']['pose']['pose']['position']

def summarize(doc):
    import statistics
    rows, start = doc['rows'], doc['start']
    meta = [json.loads(r['m']['data']) for r in rows['/board_trials/run_metadata']]
    meta = next(m for m in reversed(meta) if 'ground_reference' in m)
    ref = meta['ground_reference']
    ground, target = ref['ground_z'], ref['takeoff_z']
    commands = rows['/navigation/mission_command_raw']
    off = first(rows['/mavros/state'], lambda r: r['m']['armed'] and r['m']['mode']=='OFFBOARD')
    ready = first(rows['/mission/control_ready'], lambda r: r['m']['data'])
    odom = rows['/mavros/local_position/odom']
    lift = sustained(odom, off['t'], ready['t'], lambda r: position(r)['z'] >= ref['fc_xyz'][2]+.05)
    lift10 = sustained(odom, off['t'], ready['t'], lambda r: position(r)['z'] >= ref['fc_xyz'][2]+.10)
    probes = decoded(rows, '/uav_high_view/probe_status')
    ascent = first(probes, lambda r: r['m'].get('ascent_verified') is True)
    compact_doc = compact(doc)
    begin_event = first(compact_doc['events'], lambda r: r['stage']=='DESCEND')
    interrupt = first(compact_doc['events'], lambda r: r['stage']=='SURVEY_INTERRUPTED_TOP3')
    descend = first(commands, lambda r: r['m']['reason']=='high_view_full:DESCEND')
    revisit = first(commands, lambda r: r['m']['reason']=='high_view_full:REVISIT')
    def segment(command):
        seq=command['m']['decision_seq']
        trajectory=first(rows['/planning/bspline'], lambda r: r['m']['goal_stamp']==command['m']['header']['stamp'])
        result=first(rows['/navigation/mission_result'], lambda r: r['m']['decision_seq']==seq and r['m']['reason']=='motion_arrival_confirmed')
        initial=position(prior(odom,command['t']))
        if command is descend:
            motion=sustained(odom,trajectory['t'],result['t'],lambda r:position(r)['z']<=initial['z']-.05)
        else:
            goal=command['m']['goal']['pose']['position']
            dx,dy=goal['x']-initial['x'],goal['y']-initial['y']
            length=math.hypot(dx,dy)
            motion=sustained(odom,trajectory['t'],result['t'],lambda r: ((position(r)['x']-initial['x'])*dx+(position(r)['y']-initial['y'])*dy)/length>=.05)
        return dict(seq=seq,goal=command['m']['goal']['pose']['position'],command=command['t'],
                    first_bspline=trajectory['t'],traj_id=trajectory['m']['traj_id'],
                    motion_5cm=motion['t'] if motion else None,arrival=result['t'])
    ds,rv=segment(descend),segment(revisit)
    low_physical=sustained(odom,ds['first_bspline'],ds['arrival'],lambda r:abs(position(r)['z']-ref['low_z'])<=.1)
    high_physical=sustained(odom,ready['t'],ascent['t'],lambda r:abs(position(r)['z']-ref['high_z'])<=.1)
    high_command=first(commands,lambda r:abs(r['m']['goal']['pose']['position']['z']-ref['high_z'])<1e-6)
    scan_command=first(commands,lambda r:r['m']['decision_seq']>high_command['m']['decision_seq'])
    lead=[]
    for sp in rows['/mavros/setpoint_position/local']:
        if lift['t']<=sp['t']<=ready['t']:
            actual=prior(odom,sp['t'])
            if actual and sp['t']-actual['t']<=.1:
                lead.append(sp['m']['pose']['position']['z']-position(actual)['z'])
    wait=[]
    statuses=decoded(rows,'/navigation/mission_status')
    for label,begin,end in [('descent_admission',begin_event['t'],ds['command']),('low_stage_handoff',ds['arrival'],rv['command'])]:
        reasons=collections.defaultdict(list)
        for r in statuses:
            reason=r['m'].get('last_reason','')
            if begin<=r['t']<=end and ('waiting_' in reason or 'unavailable' in reason):
                reasons[reason].append(r['t'])
        wait.append(dict(window=label,begin=begin,end=end,observed_reasons={k:dict(first=v[0],last=v[-1],samples=len(v)) for k,v in reasons.items()}))
    result=dict(round=doc['round'],bag=doc['bag'],time_basis='receipt seconds from bag start except explicitly named node_event',
        ground_z=ground,takeoff_z=target,high_z=ref['high_z'],low_z=ref['low_z'],
        takeoff=dict(first_setpoint=rows['/mavros/setpoint_position/local'][0]['t'],
            armed=first(rows['/mavros/state'],lambda r:r['m']['armed'])['t'],offboard=off['t'],
            in_air=first(rows['/mavros/extended_state'],lambda r:r['m']['landed_state']==2)['t'],
            measured_rise_5cm=lift['t'],measured_rise_10cm=lift10['t'],ready=ready['t'],
            ready_agl=position(prior(odom,ready['t']))['z']-ground,
            first_search_command=commands[0]['t'],high_climb_command=high_command['t'],
            high_within_10cm=high_physical['t'] if high_physical else None,ascent_verified=ascent['t'],
            first_scan_command=scan_command['t'],
            z_lead_m=dict(samples=len(lead),min=min(lead),median=statistics.median(lead),max=max(lead)),
            rise_5cm_to_ready_average_mps=(position(prior(odom,ready['t']))['z']-position(lift)['z'])/(ready['t']-lift['t'])),
        high_scan_exit=dict(kind='SUPPORTED_TOP3_INTERRUPT' if interrupt else 'SURVEY_ROUTE_COMPLETE',
            node_event=begin_event['t'],first_descend_status=first(probes,lambda r:r['m']['stage']=='DESCEND')['t'],
            previous_route_arrival=None if interrupt else first(rows['/navigation/mission_result'],lambda r:r['m']['decision_seq']==descend['m']['decision_seq']-1 and r['m']['reason']=='motion_arrival_confirmed')['t']),
        descent=ds,low_height_within_10cm=low_physical['t'] if low_physical else None,
        first_low_target=rv,observed_waits=wait,
        missing_topics=doc['missing_topics'])
    dwell_rows=[r for r in decoded(rows,'/navigation/planner_bridge_status')
        if r['t']<=ds['arrival'] and r['m'].get('executor',{}).get('active_decision_seq')==ds['seq']
        and r['m']['executor'].get('dwell_start_ns',0)>0]
    result['descent_arrival_dwell']=dict(last_start_source_relative=dwell_rows[-1]['m']['executor']['dwell_start_ns']/1e9-start,
        first_observed_receipt=dwell_rows[0]['t'],note='Source observation clock; not pure receipt elapsed time') if dwell_rows else None
    result['durations']=dict(offboard_to_ready=ready['t']-off['t'],offboard_to_measured_rise_5cm=lift['t']-off['t'],
        measured_rise_5cm_to_ready=ready['t']-lift['t'],ready_to_first_search=commands[0]['t']-ready['t'],
        first_search_to_high_verified=ascent['t']-commands[0]['t'],
        descent_command_to_first_bspline=ds['first_bspline']-ds['command'],
        descent_command_to_motion_5cm=ds['motion_5cm']-ds['command'],
        descent_command_to_arrival=ds['arrival']-ds['command'],
        low_physical_to_arrival=ds['arrival']-low_physical['t'] if low_physical else None,
        low_arrival_to_target_command=rv['command']-ds['arrival'],
        low_target_command_to_first_bspline=rv['first_bspline']-rv['command'],
        low_target_command_to_motion_5cm=rv['motion_5cm']-rv['command'],
        low_target_command_to_arrival=rv['arrival']-rv['command'])
    assert ds['command'] <= ds['first_bspline'] <= ds['arrival'] < rv['command'] <= rv['first_bspline'] <= rv['arrival']
    assert (interrupt is not None) == (doc['round']==1)
    return result

if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--round',type=int,choices=(1,2),nargs='+',default=[1,2])
    rounds=parser.parse_args().round
    summaries=[summarize(load(n)) for n in rounds]
    (OUTPUT/('takeoff_handoff_summary.json' if len(rounds)==2 else 'takeoff_handoff_summary_%s.json'%rounds[0])).write_text(json.dumps(summaries,ensure_ascii=False,indent=2),encoding='utf-8')
    for summary in summaries:
        print(json.dumps(summary,ensure_ascii=False))

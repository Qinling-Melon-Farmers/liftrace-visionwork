#!/usr/bin/env python3
"""Offline seed38 resume A/B report. No ROS, subprocesses, Git writes or simulation.

Run after both matrix cases finish. --dry-run never writes report files.
Existing analysis artifacts are optional; raw records remain the authority.
"""
import argparse
import bisect
from collections import Counter, defaultdict
from datetime import datetime, timezone
import csv
import html
import json
import math
from pathlib import Path
import statistics
import sys
import xml.etree.ElementTree as ET

import yaml

DEFAULT_ROOT = Path('/home/xhj/liftrace-worktrees/r2026-high-view-search')
DOC = Path('docs/verification/seed38_resume_20261005')
VARIANTS = ('resume_off', 'resume_on')
FACTS = {0: 'EXECUTION_UNKNOWN', 1: 'NOT_STARTED', 2: 'RAW_CALL_STARTED', 3: 'COMPLETED'}


def canonical(x):
    return json.dumps(x, sort_keys=True, ensure_ascii=False, allow_nan=False)


def number(x):
    try:
        v = float(x)
        return v if math.isfinite(v) else None
    except (TypeError, ValueError):
        return None


def stamp(x):
    if isinstance(x, dict):
        if 'stamp_ns' in x:
            return (number(x['stamp_ns']) or 0) / 1e9
        return (number(x.get('secs')) or 0) + (number(x.get('nsecs')) or 0) / 1e9
    return number(x)


def read_json(path, warnings, required=False):
    if not path.is_file():
        if required:
            warnings.append('Missing file: ' + str(path))
        return {}
    try:
        return json.loads(path.read_text(encoding='utf-8-sig'))
    except (ValueError, OSError) as exc:
        warnings.append('Cannot read ' + str(path) + ': ' + str(exc))
        return {}


def read_yaml(path, warnings):
    if not path.is_file():
        warnings.append('Missing file: ' + str(path))
        return {}
    try:
        return yaml.safe_load(path.read_text(encoding='utf-8-sig')) or {}
    except (yaml.YAMLError, OSError) as exc:
        warnings.append('Cannot read ' + str(path) + ': ' + str(exc))
        return {}


def read_jsonl(path, warnings):
    if not path.is_file():
        warnings.append('Missing file: ' + str(path))
        return []
    rows = []
    with path.open(encoding='utf-8-sig') as source:
        for i, line in enumerate(source, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError('Expected an object')
                rows.append(row)
            except ValueError as exc:
                warnings.append(f'Malformed JSONL {path}:{i}: {exc}')
    return rows


def read_pose(path, warnings):
    if not path.is_file():
        warnings.append('Missing file: ' + str(path))
        return []
    rows, bad = [], 0
    with path.open(encoding='utf-8-sig', newline='') as source:
        for row in csv.DictReader(source):
            vals = [number(row.get(k)) for k in ('t', 'x', 'y', 'z')]
            if None in vals:
                bad += 1
            else:
                rows.append(vals)
    if bad:
        warnings.append(f'{path}: {bad} invalid pose rows')
    return sorted(rows)


def nearest_pose(rows, times, t, max_age):
    if not rows or t is None:
        return None
    i = bisect.bisect_left(times, t)
    indices = [k for k in (i-1, i) if 0 <= k < len(rows)]
    p = min((rows[k] for k in indices), key=lambda p: abs(p[0]-t))
    return dict(sample_ros_s=p[0], sample_age_s=abs(p[0]-t), xy=p[1:3],
                z=p[3], fresh=abs(p[0]-t) <= max_age)


def freeze_high(rows):
    """Keep first publication of each exact embedded event; snapshots repeat lists."""
    frozen, memory, changes, seen, memseen = [], [], [], set(), set()
    previous = None
    valid = [e for e in rows if number(e.get('t')) is not None and isinstance(e.get('status'), dict)
             and e['status'].get('scope') == 'HIGH_VIEW_FULL_MISSION']
    valid.sort(key=lambda e: e['t'])
    for row in valid:
        t, s = row['t'], row['status']
        if s.get('stage') != previous:
            changes.append(dict(ros_s=t, stage=s.get('stage', 'UNKNOWN'),
                                timing_source='first status publication'))
            previous = s.get('stage')
        for event in s.get('events', []):
            key = canonical(event)
            if key not in seen:
                seen.add(key)
                frozen.append(dict(event=event, first_publication_ros_s=t))
        for event in s.get('navigation_memory_events', []):
            key = canonical(event)
            if key not in memseen:
                memseen.add(key)
                memory.append(dict(event=event, first_publication_ros_s=t))
    return valid, frozen, memory, changes


def intervals(changes, start, end):
    result, totals = [], defaultdict(float)
    if start is None or end is None or end < start:
        return result, {}
    for i, c in enumerate(changes):
        left = max(start, c['ros_s'])
        right = min(end, changes[i+1]['ros_s'] if i+1 < len(changes) else end)
        if right <= left:
            continue
        result.append(dict(stage=c['stage'], start_ros_s=left, end_ros_s=right,
                           duration_s=right-left, timing_source=c.get('timing_source'),
                           last_interval=i+1 == len(changes)))
        totals[c['stage']] += right-left
    return result, dict(totals)


def truth_targets(truth):
    offset = truth.get('spawn_offset', {})
    out = []
    for t in truth.get('targets', []):
        x, y = number(t.get('world_x', t.get('x'))), number(t.get('world_y', t.get('y')))
        if x is None or y is None:
            continue
        out.append(dict(name=t.get('model'), class_name=t.get('class'), world_xy=[x, y],
                        eval_xy=[x-(number(offset.get('x')) or 0), y-(number(offset.get('y')) or 0)],
                        yaw=number(t.get('yaw')) or 0,
                        nominal_half_side_m=.175 if t.get('class') == 'red_cross' else .5))
    return out


def truth_association(label, xy, truth):
    if not truth or xy is None:
        return None
    ranked = sorted(((math.dist(xy, t['eval_xy']), t) for t in truth), key=lambda p: p[0])
    distance, target = ranked[0]
    dx, dy = xy[0]-target['eval_xy'][0], xy[1]-target['eval_xy'][1]
    c, s = math.cos(target['yaw']), math.sin(target['yaw'])
    local = [c*dx+s*dy, -s*dx+c*dy]
    return dict(nearest_class=target['class_name'], nearest_instance=target['name'],
                nearest_distance_m=distance, label_matches=label == target['class_name'],
                next_nearest_distance_m=ranked[1][0] if len(ranked)>1 else None,
                board_local_xy=local, nominal_half_side_m=target['nominal_half_side_m'],
                within_nominal_board=all(abs(v) <= target['nominal_half_side_m'] for v in local))


def fixed_identity(d):
    return [d.get('mission_id'), d.get('decision_seq'), d.get('attempt'), d.get('payload_slot'),
            d.get('target_id'), stamp(d.get('target_first_seen')), d.get('target_class')]


def release_summary(events, poses, truth, max_age):
    groups, rejected, anomalies = {}, [], []
    times = [p[0] for p in poses]
    seen = set()
    for e in events:
        if e.get('kind') != 'release' or not isinstance(e.get('data'), dict):
            continue
        d = e['data']
        state = d.get('execution_state', 0)
        valid = state in FACTS and (state == 0 or bool(d.get('success')) == (state == 3))
        if not valid:
            anomalies.append(dict(reason='success_fact_mismatch', raw_event=e))
            continue
        state = 3 if state == 0 and d.get('success') else state
        identity = fixed_identity(d)
        key = canonical([identity, d.get('align_mode'), d.get('execution_id')])
        token = canonical(d)
        g = groups.setdefault(key, dict(identity=identity, execution_id=d.get('execution_id'),
                         align_mode=d.get('align_mode'), messages=[], duplicate_messages=0,
                         identity_fields_recorded=all(k in d for k in ('mission_id','decision_seq',
                                                        'attempt','target_first_seen','execution_state','terminal'))))
        if (key, token) in seen:
            g['duplicate_messages'] += 1
            continue
        seen.add((key, token))
        t = stamp(d.get('header', {}).get('stamp')) or number(e.get('ros_sec'))
        terminal = bool(d.get('terminal')) if d.get('execution_state', 0) else True
        g['messages'].append(dict(ros_s=t, received_ros_s=e.get('ros_sec'), fact=FACTS[state],
                                 terminal=terminal, success=bool(d.get('success')), reason=d.get('reason')))
        if state == 1:
            rejected.append(dict(identity=identity, execution_id=d.get('execution_id'), ros_s=t,
                                 terminal=terminal, reason=d.get('reason')))
    completed = []
    for g in groups.values():
        starts = [m['ros_s'] for m in g['messages'] if m['fact']=='RAW_CALL_STARTED']
        ends = [m for m in g['messages'] if m['fact']=='COMPLETED' and m['terminal']]
        g['positive_execution_evidence'] = any(m['fact'] in ('RAW_CALL_STARTED','COMPLETED') for m in g['messages'])
        g['not_started_only'] = bool(g['messages']) and all(m['fact']=='NOT_STARTED' for m in g['messages'])
        g['execution_role'] = ('ACTUATOR_EXECUTION' if g['positive_execution_evidence'] else
                               'PRE_CALL_DENIAL' if g['not_started_only'] else 'UNKNOWN')
        g['call_started_ros_s'] = min(starts) if starts else None
        g['completion_ros_s'] = min(m['ros_s'] for m in ends) if ends else None
        g['call_to_ack_s'] = (g['completion_ros_s']-g['call_started_ros_s']
                             if starts and ends else None)
        if ends:
            pose = nearest_pose(poses, times, g['completion_ros_s'], max_age)
            association = truth_association(g['identity'][-1], pose['xy'] if pose else None, truth)
            correct = (bool(association['label_matches'] and association['within_nominal_board'])
                       if pose and pose['fresh'] and association else None)
            completed.append(dict(identity=g['identity'], slot=g['identity'][3],
                class_name=g['identity'][-1], execution_id=g['execution_id'],
                ros_s=g['completion_ros_s'], truth_pose=pose, truth_association=association,
                correct_mock_release=correct, call_to_ack_s=g['call_to_ack_s']))
    completed.sort(key=lambda d: d['ros_s'])
    by_slot = {str(s): [d for d in completed if d['slot']==s] for s in (1,2,3)}
    for slot, records in by_slot.items():
        if len(records)>1:
            anomalies.append(dict(reason='multiple_completed_executions_for_slot',slot=int(slot),count=len(records)))
    commits, commit_seen = [], set()
    for e in events:
        d=e.get('data', {})
        if e.get('kind')=='result' and d.get('payload_committed'):
            key=canonical(fixed_identity(d))
            if key not in commit_seen:
                commit_seen.add(key)
                commits.append(dict(identity=fixed_identity(d),ros_s=e.get('ros_sec'),reason=d.get('reason')))
    return dict(executions=list(groups.values()), not_started=rejected, completed=completed,
                positive_execution_count=sum(g['positive_execution_evidence'] for g in groups.values()),
                not_started_only_id_count=sum(g['not_started_only'] for g in groups.values()),
                duplicate_message_count=sum(g['duplicate_messages'] for g in groups.values()),
                slots=by_slot, completed_execution_count=len(completed),
                three_slots_exactly_once=all(len(v)==1 for v in by_slot.values()) and len(completed)==3,
                completed_slot_sequence=[d['slot'] for d in completed],
                correct_mock_release_count=sum(d['correct_mock_release'] is True for d in completed),
                truth_unknown_count=sum(d['correct_mock_release'] is None for d in completed),
                bridge_payload_commits=commits, anomalies=anomalies)


def local_retry_summary(keys, release):
    """Correlate strict action identity, preflight denial, REVISIT and FREE slot.

    Different denial IDs are receipt identities, not separate actuator calls.
    A later positive fact on this same action cancels the no-call conclusion.
    A different decision/attempt can legitimately reuse the still-free slot.
    """
    actions=defaultdict(list)
    for g in release['executions']:
        actions[canonical([g['identity'],g['align_mode']])].append(g)
    rows=[]
    for groups in actions.values():
        denied=[g for g in groups if g['not_started_only']]
        if not denied:continue
        identity=groups[0]['identity'];mission,seq,attempt,slot,target,first_seen,label=identity
        if not mission or number(seq) is None:continue
        positive=any(g['positive_execution_evidence'] for g in groups)
        related=[e for e in keys if e.get('kind')=='result' and fixed_identity(e['data'])==identity]
        failures=[e for e in related if e['data'].get('terminal') and e['data'].get('retryable')
                  and str(e['data'].get('reason','')).split(':',1)[0]=='release_preflight_rejected'
                  and e['data'].get('payload_committed') is False]
        failure=failures[0] if failures else None
        failure_t=failure['ros_sec'] if failure else None
        # Require the next decision, not an unrelated REVISIT much later.
        later=[e for e in keys if e.get('kind')=='decision' and e['data'].get('mission_id')==mission
               and number(e['data'].get('decision_seq')) is not None and e['data']['decision_seq']>seq
               and failure_t is not None and e['ros_sec']>=failure_t]
        revisit=later[0] if later and later[0]['data'].get('reason')=='high_view_full:REVISIT' else None
        slot_evidence=None
        if revisit is not None and isinstance(slot,int) and 1<=slot<=3:
            # End at the next decision so a later successful reuse cannot mask the denial.
            stop=later[1]['ros_sec'] if len(later)>1 else math.inf
            for e in keys:
                d=e['data'];states=d.get('slot_status',[])
                if (e.get('kind')=='mission' and d.get('mission_id')==mission
                        and revisit['ros_sec']<=e['ros_sec']<stop and len(states)>=slot and states[slot-1]=='FREE'):
                    slot_evidence=dict(received_ros_s=e['ros_sec'],slot_status=states,
                        committed_slots=d.get('committed_slots'),phase=d.get('phase'),last_reason=d.get('last_reason'))
                    break
        commit=any(d['identity']==identity for d in release['bridge_payload_commits'])
        no_waste=bool(not positive and not commit and failure and revisit and slot_evidence)
        recoveries=[d for d in release['completed'] if d['identity'][0]==mission and d['slot']==slot
                    and d['identity'][4:]==identity[4:] and d['identity'][1]>seq]
        recovered=recoveries[0] if recoveries else None
        rows.append(dict(identity=identity,align_mode=groups[0]['align_mode'],
            classification='LOCAL_PREFLIGHT_RETRY_NO_SLOT_CONSUMED' if no_waste else
                           'DENIAL_THEN_POSITIVE_EXECUTION' if positive else 'PRE_CALL_DENIAL_NOT_FULLY_CORROBORATED',
            not_started_execution_ids=[g['execution_id'] for g in denied],
            not_started_messages=[dict(execution_id=g['execution_id'],**m) for g in denied for m in g['messages']],
            positive_execution_ids=[g['execution_id'] for g in groups if g['positive_execution_evidence']],
            no_raw_call_observed_for_action=not positive,slot_not_consumed_verified=no_waste,
            bridge_preflight_failure=failure,revisit_decision=revisit,free_slot_evidence=slot_evidence,
            subsequent_successful_attempt=recovered,
            denial_to_recovered_ack_s=recovered['ros_s']-failure_t if recovered and failure_t is not None else None,
            note='Only RAW_CALL_STARTED/COMPLETED identify actuator executions. Multiple NOT_STARTED IDs are not duplicate actuation; exact message replays are counted separately.'))
    return rows


def preflight_clock_diagnostic(rows):
    if not isinstance(rows,list):return dict(status='NOT_RECORDED',evidence=[])
    findings=[]
    for refusal in rows:
        if refusal.get('reason')!='permission_stale':continue
        t=number(refusal.get('source_time'))
        permits=[p for p in rows if p.get('topic')=='/mission/release_permission' and p.get('permitted') is True
                 and p.get('slot')==refusal.get('slot') and p.get('decision')==refusal.get('decision')
                 and number(p.get('source_time')) is not None and number(p.get('valid_until')) is not None
                 and number(p.get('bag_time')) is not None and number(refusal.get('bag_time')) is not None
                 and p['bag_time']<=refusal['bag_time']]
        latest=max(permits,key=lambda p:p['bag_time']) if permits else None
        skew=t-latest['source_time'] if t is not None and latest else None
        likely=bool(skew is not None and -.01<=skew<0 and latest['valid_until']>t)
        findings.append(dict(refusal=refusal,prior_recorded_permits=permits,
            latest_permit_by_bag_receipt=latest,candidate_age_s=skew,
            interpretation='LIKELY_TINY_FUTURE_CLOCK_SKEW' if likely else 'CAUSE_UNRESOLVED',
            long_expiry_confirmed=False,node_internal_trace_available=False,
            note='permission_stale covers negative age as well as expiry. Bag receipt and source stamps do not prove the exact permission object/current ROS time used by the proxy. No threshold change in this A/B.'))
    return dict(status='RECORDED' if findings else 'NO_STALE_REFUSAL',evidence=findings)


def contact_summary(contacts,gate):
    events=[]
    for e in contacts.get('events',[]):
        pairs=e.get('pairs',[])
        vehicle_parts=[p for pair in pairs for p in pair if str(p).startswith('iris_mid360::')]
        guard_only=bool(vehicle_parts) and all(str(p).endswith('::competition_guard_collision') for p in vehicle_parts)
        events.append(dict(first_ros_s=e.get('ros_stamp'),last_ros_s=e.get('last_ros_stamp'),
            duration_s=e.get('duration_sec'),sample_count=e.get('sample_count'),pairs=pairs,
            peak_depth_m=e.get('peak_sampled_depth_m'),
            peak_depth_mm=(1000*e['peak_sampled_depth_m'] if number(e.get('peak_sampled_depth_m')) is not None else None),
            peak_force_n=e.get('peak_sampled_force_n'),guard_only_recorded=guard_only,
            actual_airframe_part_contacts_recorded=[p for p in vehicle_parts if not str(p).endswith('::competition_guard_collision')],
            descriptive_label='GUARD_ENVELOPE_CONTACT' if guard_only else 'AIRFRAME_OR_UNRESOLVED_CONTACT',
            details=e.get('details',[])))
    return dict(actual_collision_count=contacts.get('actual_collision_count'),events=events,
        stopped_by_contact_gate=gate.get('reason')=='actual_collision',
        note='The recorded seed38 on event is guard-envelope grazing: report sampled depth/force/duration, not a severe-crash claim. No actual airframe-part pair is recorded for that event. Guard contact remains a Gate collision; absence of a part pair is not an independent damage assessment.')


def resume_summary(high, frozen, keys, poses, changes, finished=True):
    final=high[-1]['status'] if high else {}
    ev=[v['event'] for v in frozen]
    once=[e for e in ev if e.get('stage')=='SURVEY_RESUME_ONCE']
    resumed=[e for e in ev if e.get('stage')=='SURVEY_RESUMED']
    failed=[e for e in ev if e.get('stage')=='SURVEY_RESUME_FAILED']
    unavailable=[e for e in ev if e.get('stage')=='SURVEY_RESUME_UNAVAILABLE']
    found=[e for e in ev if e.get('stage')=='SURVEY_RESUME_FOUND_MISSING']
    enabled=next((e['status'].get('survey_policy',{}).get('resume_survey_enabled')
                  for e in high if 'resume_survey_enabled' in e['status'].get('survey_policy',{})), None)
    attempted=bool(final.get('resume_attempted') or once or unavailable)
    physically_started=bool(resumed or final.get('resume_started') is not None)
    outcome=('UNKNOWN' if not high else 'DISABLED' if enabled is False else
             'UNAVAILABLE' if unavailable and not once else ('NOT_TRIGGERED' if finished else 'NOT_TRIGGERED_YET') if not attempted else
             'FAILED' if failed else
             'FOUND_MISSING' if found else 'RESUMED_ENDED' if physically_started and final.get('resume_completed') else
             'RESUMED_INCOMPLETE' if physically_started else 'ASCENT_OR_REJOIN_INCOMPLETE')
    actions=[]
    for e in keys:
        d=e.get('data',{})
        if e.get('kind')!='decision':continue
        reason=str(d.get('reason','')).lower()
        action_kind=('resume_ascent' if reason in ('resume_ascent','high_view_full:resume_ascend') else
                     'resume_rejoin' if reason in ('resume_rejoin','high_view_full:resume_join') else
                     'remaining_survey' if reason=='remaining_survey' else None)
        if action_kind is None and reason=='high_view_full:survey':
            t=stamp(d.get('header',{}).get('stamp')) or e['ros_sec']
            if any(number(v.get('time')) is not None and v['time']<=t for v in resumed):
                action_kind='remaining_survey'
        if action_kind is None:continue
        related=[x for x in keys if x.get('kind')=='result'
                 and x.get('data',{}).get('mission_id')==d.get('mission_id')
                 and x.get('data',{}).get('decision_seq')==d.get('decision_seq')]
        start=stamp(d.get('header',{}).get('stamp')) or e['ros_sec']
        def first(status):
            rows=[x for x in related if x['data'].get('status')==status]
            return min((stamp(x['data'].get('header',{}).get('stamp')) or x['ros_sec'] for x in rows), default=None)
        accepted, started, arrived=first(0),first(1),first(3)
        actions.append(dict(mission_id=d.get('mission_id'),decision_seq=d.get('decision_seq'),reason=action_kind,raw_reason=d.get('reason'),
            published_ros_s=start,goal=d.get('goal'),deadline_ros_s=stamp(d.get('deadline')),
            bridge_accepted_ros_s=accepted,planner_started_ros_s=started,arrived_ros_s=arrived,
            dispatch_to_planner_started_s=started-start if started is not None else None,
            motion_duration_s=arrived-start if arrived is not None else None,
            terminal_failures=[x for x in related if x['data'].get('terminal') and x['data'].get('status') in (4,5,6,7)]))
    ascent=next((a for a in actions if a['reason']=='resume_ascent'),None)
    request_time=number(once[0].get('time')) if once else None
    return dict(config_enabled=enabled,attempted=attempted,attempt_count=len(once)+len(unavailable),
        ascent_requested=bool(once),ascent_published=ascent is not None,actual_survey_started=physically_started,
        outcome=outcome,observation_final=finished,request_events=once,unavailable_events=unavailable,failure_events=failed,
        resumed_events=resumed,found_missing_events=found,actions=actions,
        request_to_ascent_dispatch_s=(ascent['published_ros_s']-request_time if ascent and request_time is not None else None),
        height_constraint_ack=dict(direct_payload_available=False,status='NOT_DIRECTLY_RECORDED',
            dispatch_admission_inferred=ascent is not None,
            note='Height ACK is a ROS parameter, not a recorded topic. Under this source, resume dispatch requires matching ACK/readback; exact ACK ID/time/max_z cannot be independently measured. Request-to-dispatch includes all readiness waits.'),
        waiting_height_ack_statuses=[e for e in keys if e.get('kind')=='mission'
            and 'height_constraint_ack' in str(e.get('data',{}).get('last_reason',''))],
        saved_remainder=[e for e in ev if e.get('stage')=='SURVEY_REMAINDER_SAVED'],
        resume_completed_flag=final.get('resume_completed'),
        completion_flag_note='resume_completed is also set on failure; it is not a success verdict.',
        mission_final_failure_separate=dict(done=final.get('done'),succeeded=final.get('succeeded'),
            reason=final.get('failure'),note='Final mission/manual abort never overrides an earlier resume outcome.'))


def stable_window(poses,times,end,seconds=2.,tolerance=.02,max_gap=.5,max_age=.25):
    """Use samples available by end, covering the whole preceding window."""
    last=bisect.bisect_right(times,end)-1
    first=bisect.bisect_right(times,end-seconds)-1
    if first<0 or last<=first or end-times[last]>max_age:return None
    window=poses[first:last+1]
    gaps=[b[0]-a[0] for a,b in zip(window,window[1:])]
    if max(gaps,default=math.inf)>max_gap:return None
    ranges=[max(p[i] for p in window)-min(p[i] for p in window) for i in (1,2,3)]
    if any(v>tolerance for v in ranges):return None
    return dict(start_ros_s=window[0][0],last_sample_ros_s=window[-1][0],
                confirmed_at_ros_s=end,samples=len(window),xyz_range_m=ranges,
                max_gap_s=max(gaps),max_range_threshold_m=tolerance,
                required_window_s=seconds,last_sample_age_s=end-window[-1][0])


def physical_summary(keys,poses,contacts,operator,start,observed,max_age):
    """Physical completion is separate from software COMPLETE/raw Gate.

    State/extended_state are change-only records: a held ON_GROUND/disarmed
    value is not an odometry freshness failure. We require the actual entry
    transitions, sustained contact and a continuous stable truth window.
    """
    decisions=[e for e in keys if e.get('kind')=='decision']
    land=min((stamp(e['data'].get('header',{}).get('stamp')) or e['ros_sec']
              for e in decisions if e['data'].get('command')==5),default=None)
    supports=[s for s in contacts.get('support_events',[]) if land is not None
              and number(s.get('ros_stamp')) is not None and s['ros_stamp']>=land]
    supports.sort(key=lambda s:s['ros_stamp'])
    state=[e for e in keys if e.get('kind')=='state' and land is not None and e['ros_sec']>=land]
    extended=[e for e in keys if e.get('kind')=='extended_state' and land is not None and e['ros_sec']>=land]
    ground=[e for e in extended if e['data'].get('landed_state')==1]
    disarmed=[e for e in state if e['data'].get('connected') is True and e['data'].get('armed') is False]
    times=[p[0] for p in poses]
    proof=None;support=None;window=None;g=None;d=None
    for s in supports:
        contact_end=number(s.get('last_ros_stamp'))
        if contact_end is None:contact_end=number(s.get('ended_ros_stamp'))
        if contact_end is None:continue  # duration alone is not a sampled end timestamp
        for ge in ground:
            for de in disarmed:
                t=max(s['ros_stamp']+2.,ge['ros_sec'],de['ros_sec'])
                if t>contact_end or (proof is not None and t>=proof):continue
                latest_s=next((e for e in reversed(state) if e['ros_sec']<=t),None)
                latest_e=next((e for e in reversed(extended) if e['ros_sec']<=t),None)
                if (latest_s is None or latest_s['data'].get('connected') is not True
                        or latest_s['data'].get('armed') is not False or latest_e is None
                        or latest_e['data'].get('landed_state')!=1):continue
                candidate=stable_window(poses,times,t,max_age=max_age)
                if candidate is not None:proof,support,window,g,d=t,s,candidate,ge,de
    stop_t=number(operator.get('ros_sec'))
    aborts=[e for e in decisions if e['data'].get('command')==7 and
            'manual' in str(e['data'].get('reason','')).lower()]
    abort=aborts[0] if aborts else None
    abort_t=(stamp(abort['data'].get('header',{}).get('stamp')) or abort['ros_sec']) if abort else None
    tail_end=min(v for v in (observed,stop_t) if v is not None) if observed is not None else stop_t
    tail=[p for p in poses if proof is not None and tail_end is not None and proof<=p[0]<=tail_end]
    ranges=[max(p[i] for p in tail)-min(p[i] for p in tail) for i in (1,2,3)] if tail else None
    contradictions=[e for e in state+extended if proof is not None and tail_end is not None and
        proof<e['ros_sec']<=tail_end and ((e.get('kind')=='state' and
        (e['data'].get('armed') is True or e['data'].get('connected') is False)) or
        (e.get('kind')=='extended_state' and e['data'].get('landed_state')!=1))]
    gaps=[b[0]-a[0] for a,b in zip(tail,tail[1:])]
    stable_tail=bool(tail and ranges is not None and all(v<=.02 for v in ranges)
                     and not contradictions and max(gaps,default=math.inf)<=.5)
    operator_after_ground=bool(operator and abort and proof is not None and
        stop_t is not None and proof<stop_t<=abort_t and stable_tail)
    def evidence(e):
        if e is None:return None
        return dict(received_ros_s=e['ros_sec'],source_ros_s=stamp(e['data'].get('header',{}).get('stamp')),
                    data=e['data'])
    return dict(verified=proof is not None,status='VERIFIED_STABLE_GROUND' if proof is not None else
        'CONTACT_ONLY' if supports else 'UNVERIFIED',landing_command_ros_s=land,
        first_touchdown_contact_ros_s=supports[0]['ros_stamp'] if supports else None,
        sustained_support_start_ros_s=support.get('ros_stamp') if support else None,
        support_episode=support,ground_state_evidence=evidence(g),disarmed_evidence=evidence(d),
        stable_truth_window=window,physical_completion_ros_s=proof,
        physical_completion_mission_s=proof-start if proof is not None and start is not None else None,
        touchdown_mission_s=support['ros_stamp']-start if support is not None and start is not None else None,
        stable_ground_tail=dict(end_ros_s=tail_end,samples=len(tail),xyz_range_m=ranges,
            max_gap_s=max(gaps,default=None),verified=stable_tail,state_contradictions=contradictions),
        operator_stop=dict(record_present=bool(operator),reason=operator.get('reason'),request_ros_s=stop_t,
            abort_command_ros_s=abort_t,abort_received_ros_s=abort.get('ros_sec') if abort else None,
            after_verified_stable_ground=operator_after_ground,
            ground_wait_before_stop_s=stop_t-proof if operator_after_ground else None,
            record=operator),
        evidence_note='Touchdown, sustained support, ON_GROUND, disarm, and stable truth are separate. State records are change-only; no 0.5s odom-age gate is applied to held state.')


def pose_metrics(rows,start,end):
    selected=[r for r in rows if start is not None and end is not None and start<=r[0]<=end]
    gaps=[b[0]-a[0] for a,b in zip(selected,selected[1:])]
    return dict(samples=len(selected),xy_distance_m=sum(math.dist(a[1:3],b[1:3]) for a,b in zip(selected,selected[1:])),
        xyz_distance_m=sum(math.dist(a[1:],b[1:]) for a,b in zip(selected,selected[1:])),
        max_gap_s=max(gaps,default=None),gaps_over_0_5s=sum(v>.5 for v in gaps),
        min_recorded_z_m=min((p[3] for p in selected),default=None),max_recorded_z_m=max((p[3] for p in selected),default=None))


def analyze_case(row,root,doc,args):
    if not row.get('run'):
        return dict(variant=row['variant'],status='NOT_RUN',warnings=['No run directory in matrix'])
    run=Path(row['run']);run=run if run.is_absolute() else root/run
    warnings=[]
    keys=read_jsonl(run/'key_events.jsonl',warnings)
    keys=[e for e in keys if number(e.get('ros_sec')) is not None and isinstance(e.get('data'),dict)]
    keys.sort(key=lambda e:e['ros_sec'])
    high,frozen,memory,changes=freeze_high(read_jsonl(run/'high_view_full_events.jsonl',warnings))
    poses=read_pose(run/'truth_pose.csv',warnings)
    gate=read_json(run/'gate_status.json',warnings,True)
    contacts=read_json(run/'gazebo_contact_status.json',warnings,True)
    clock_diagnostic=preflight_clock_diagnostic(read_json(run/'release_preflight_diagnostic.json',warnings))
    operator=read_json(run/'operator_ground_stop.json',warnings)
    raw_truth=read_yaml(run/'random_field_truth.yaml',warnings)
    truth=truth_targets(raw_truth)
    cache=read_json(doc/f"{row['seed']}_{row['variant']}"/'metrics.json',warnings)
    if cache and cache.get('run')!=str(run):
        warnings.append('Ignored metrics cache: run path mismatch');cache={}
    decisions=[e for e in keys if e.get('kind')=='decision']
    start=number(cache.get('start_ros_s'))
    start_source='validated metrics cache'
    if start is None:
        # Decision header is the issue time; receipt can be delayed.
        start=min((stamp(e['data'].get('header',{}).get('stamp')) or e['ros_sec'] for e in decisions),default=None)
        start_source='first decision issue time (approximate mission start)'
    final=high[-1]['status'] if high else {}
    observed=max([p[0] for p in poses]+[e['ros_sec'] for e in keys]+[e['t'] for e in high],default=None)
    terminal_rows=[e for e in keys if e.get('kind')=='mission' and e['data'].get('phase') in ('COMPLETE','ABORTED')]
    terminal_ros=terminal_rows[0]['ros_sec'] if terminal_rows else None
    raw_end=terminal_ros if terminal_ros is not None else observed
    physical=physical_summary(keys,poses,contacts,operator,start,observed,args.pose_max_age)
    physical_end=physical['physical_completion_ros_s']
    end=min(v for v in (raw_end,physical_end) if v is not None) if raw_end is not None or physical_end is not None else None
    phase_changes=[];prev=None
    for e in keys:
        if e.get('kind')=='mission' and e['data'].get('phase')!=prev:
            prev=e['data'].get('phase');phase_changes.append(dict(ros_s=e['ros_sec'],stage=prev,timing_source='mission status receipt'))
    segments,totals=intervals(changes,start,end)
    phases,phase_totals=intervals(phase_changes,start,end)
    release=release_summary(keys,poses,truth,args.pose_max_age)
    for d in release['completed']:
        d['mission_s']=d['ros_s']-start if start is not None else None
    resume=resume_summary(high,frozen,keys,poses,changes,finished=bool(row.get('finished_wall') or row.get('cleanup_pass')))
    target_events=[]
    names={'TARGET_DEFERRED','UNCONFIRMED_LOCATION_RETIRED','DELIVERY_POINT_UNREACHABLE',
           'LOW_VIEW_LABEL_RESOLVED','CONFLICT_LOCATION_INADMISSIBLE','SURVEY_SKIPPED',
           'LOW_COVERAGE_SKIPPED_HIGH_PRIORITY'}
    for v in frozen:
        e=v['event']
        if e.get('stage') in names:
            target_events.append(dict(**v,truth_association=truth_association(
                e.get('target',e.get('class_name')),e.get('xy',e.get('target_xy',e.get('region'))),truth)))
    land=min((e['ros_sec'] for e in decisions if e['data'].get('command')==5),default=None)
    support=physical['support_episode']
    failures=[e for e in keys if e.get('kind')=='result' and e['data'].get('terminal') and e['data'].get('status') in (4,5,6,7)]
    scene=root/DOC/'generated'/f"{row['variant']}_{row['seed']}"/f"{row['variant']}_seed{row['seed']}"
    params=read_yaml(run/'rosparams.yaml',warnings)
    offset=params.get('competition_key_recorder',{}).get('truth_world_offset')
    if offset is None:
        warnings.append('Recorder XY offset not found in rosparams; assuming random-field spawn_offset XY')
    else:
        for target in truth:
            target['eval_xy']=[target['world_xy'][0]-offset[0],target['world_xy'][1]-offset[1]]
        # Recompute when the recorder transform differs from the scene spawn offset.
        release=release_summary(keys,poses,truth,args.pose_max_age)
        for d in release['completed']:d['mission_s']=d['ros_s']-start if start is not None else None
        for v in target_events:
            e=v['event'];v['truth_association']=truth_association(e.get('target',e.get('class_name')),
                e.get('xy',e.get('target_xy',e.get('region'))),truth)
    matched_slots=len(release['bridge_payload_commits'])
    retries=local_retry_summary(keys,release)
    recovered_ids={canonical(r['identity']) for r in retries if r['slot_not_consumed_verified']
                   and r['subsequent_successful_attempt'] is not None}
    unresolved=[e for e in failures if not (str(e['data'].get('reason','')).split(':',1)[0]=='release_preflight_rejected'
                and canonical(fixed_identity(e['data'])) in recovered_ids)]
    return dict(variant=row['variant'],seed=row['seed'],source=row.get('source'),run=str(run),scene=str(scene),
        status=gate.get('status',row.get('status','UNKNOWN')),matrix_status=row.get('status'),
        exit_code=row.get('exit_code'),cleanup_pass=row.get('cleanup_pass'),gate=gate,
        first_failure=failures[0] if failures else None,action_failures=failures,
        local_preflight_retries=retries,unresolved_action_failures=unresolved,
        mission_start_ros_s=start,mission_start_source=start_source,observed_end_ros_s=observed,
        mission_terminal_ros_s=terminal_ros,duration_censored=terminal_ros is None and not physical['verified'],
        completed_mission_s=(terminal_ros-start if terminal_ros is not None and start is not None and
                            terminal_rows[0]['data'].get('phase')=='COMPLETE' else None),
        observed_duration_s=observed-start if start is not None and observed is not None else None,
        flight_metrics_end_ros_s=end,flight_metrics_end_source='verified physical completion' if physical_end is not None and end==physical_end else 'software terminal or censored observation',
        flight_duration_s=end-start if start is not None and end is not None else None,
        physical_completion=physical,
        raw_gate=dict(status=gate.get('status',row.get('status')),reason=gate.get('reason',row.get('reason')),
            failed_checks=gate.get('failed_checks',row.get('failed_checks')),errors=gate.get('errors'),
            software_complete=bool(terminal_rows and terminal_rows[0]['data'].get('phase')=='COMPLETE'),
            software_terminal_ros_s=terminal_ros),
        outcome_classification=('PHYSICAL_LANDED_OPERATOR_STOP_SOFTWARE_INCOMPLETE'
            if physical['operator_stop']['after_verified_stable_ground'] else
            'PHYSICAL_LANDED_SOFTWARE_COMPLETE' if physical['verified'] and terminal_rows and terminal_rows[0]['data'].get('phase')=='COMPLETE' else
            'PHYSICAL_LANDED_SOFTWARE_NOT_COMPLETE' if physical['verified'] else
            'CONTACT_GATE_STOPPED_BEFORE_LANDING' if gate.get('reason')=='actual_collision' else 'NO_VERIFIED_PHYSICAL_COMPLETION'),
        raw_gate_mission_s=gate.get('metrics',{}).get('mission_ros_sec'),
        high_view_final=final,resume=resume,stage_intervals=segments,stage_durations_s=totals,
        mission_phase_intervals=phases,mission_phase_durations_s=phase_totals,
        deduplicated_high_events=frozen,navigation_memory_events=memory,target_events=target_events,
        target_event_counts=dict(Counter(v['event'].get('stage') for v in target_events)),
        truth_targets=truth,truth_source=str(run/'random_field_truth.yaml'),
        truth_csv_transform=dict(recorded_offset=offset,xy_note='truth_pose.csv is world minus recorder offset; target XY uses the same transform'),
        releases=release,slots_committed_final=final.get('slots_committed'),
        release_preflight_clock_diagnostic=clock_diagnostic,contact_outcome=contact_summary(contacts,gate),
        slot_accounting=dict(proxy_completed=len(release['completed']),bridge_commit_count=matched_slots,
            high_view_slots=final.get('slots_committed'),gate_release_commit_count=gate.get('metrics',{}).get('release_commit_count'),
            counts_agree=(len(release['completed'])==matched_slots==final.get('slots_committed')==gate.get('metrics',{}).get('release_commit_count'))),
        pose_csv={name:pose_metrics(poses if name=='truth_pose' else read_pose(run/(name+'.csv'),warnings),start,end)
                  for name in ('truth_pose','mavros_pose','lio_pose','planner_setpoint','mavros_setpoint')},
        collisions=contacts.get('actual_collision_count'),landing_support_event=support,
        reused_analysis=dict(metrics_cache=str(doc/f"{row['seed']}_{row['variant']}"/'metrics.json') if cache else None,
            available_files=[str(doc/p) for p in ('metrics.json','release_truth.json','physical_completion.json') if (doc/p).is_file()]),
        artifacts={name:str(run/name) for name in ('run.log','gate_status.json','high_view_full_events.jsonl',
                   'key_events.jsonl','truth_pose.csv','gazebo_contact_status.json','operator_ground_stop.json',
                   'release_preflight_diagnostic.json','presentation_review.mp4') if (run/name).exists()},
        warnings=warnings)


def scene_signature(path):
    if not path.is_file():return None
    tree=ET.parse(path).getroot()
    for world in tree.findall('world'):world.attrib.pop('name',None)
    return ET.tostring(tree,encoding='unicode')


def corridor_contact_comparison(runs,max_age=.25):
    """One spatial cross-section on the same post-delivery route, no replay."""
    on=next(r for r in runs if r['variant']=='resume_on')
    contact=next((e for e in on.get('contact_outcome',{}).get('events',[])
                  if any('Wall_22_north_collision' in p for pair in e['pairs'] for p in pair)),None)
    if contact is None:return dict(status='NOT_RECORDED')
    result=dict(status='PARTIAL',contact_ros_s=contact['first_ros_s'],samples=[],
        note='Compare one spatial cross-section at the on contact y, within route segment 3. CSV samples are asynchronous. Quaternion yaw is directly measured; this is not a causal attribution to resume or a collision simulation. No Gate change.')
    try:
        tree=ET.parse(Path(on['scene'])/'field.world')
        model=next(m for m in tree.iter('model') if m.get('name')=='toudi2')
        model_pose=[float(v) for v in model.findtext('pose','0 0 0 0 0 0').split()]
        link=next(l for l in model.findall('link') if l.get('name')=='Wall_22_north')
        lp=[float(v) for v in link.findtext('pose').split()]
        size=[float(v) for v in link.findtext('collision/geometry/box/size').split()]
        if any(abs(v)>1e-9 for v in model_pose[3:]+lp[3:]):
            return dict(result,status='UNSUPPORTED_ROTATED_WALL')
        wall=[model_pose[i]+lp[i] for i in range(3)]
        result['wall_world_center']=wall;result['wall_box_size_m']=size
        result['wall_world_east_face_x']=wall[0]+size[0]/2
        def read_quat(path):
            rows=[]
            with path.open(newline='') as f:
                for raw in csv.DictReader(f):
                    vals=[number(raw.get(k)) for k in ('t','x','y','z','qx','qy','qz','qw')]
                    if None not in vals:rows.append(vals)
            return rows
        def sample(rows,t):
            p=min(rows,key=lambda p:abs(p[0]-t),default=None)
            if p is None:return None
            qx,qy,qz,qw=p[4:];norm=math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw)
            if norm<1e-9:return None
            qx,qy,qz,qw=[q/norm for q in (qx,qy,qz,qw)]
            return dict(ros_s=p[0],sample_age_s=abs(p[0]-t),fresh=abs(p[0]-t)<=max_age,
                xyz=p[1:4],yaw_deg=math.degrees(math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))))
        ontruth=read_quat(Path(on['run'])/'truth_pose.csv')
        reference=sample(ontruth,contact['first_ros_s'])
        if reference is None or not reference['fresh']:return dict(result,status='STALE_CONTACT_TRUTH')
        result['reference_contact_truth']=reference
        target_y=reference['xyz'][1];result['matched_truth_y']=target_y
        for r in runs:
            fences=r.get('gate',{}).get('decision_fences',[])
            start=next((e['issued_ns']/1e9 for e in fences if str(e.get('reason','')).startswith('post_delivery_route:3/')),None)
            end=next((e['issued_ns']/1e9 for e in fences if str(e.get('reason','')).startswith('post_delivery_route:4/')),r.get('observed_end_ros_s'))
            if start is None or end is None:continue
            rows=ontruth if r['variant']=='resume_on' else read_quat(Path(r['run'])/'truth_pose.csv')
            route=[p for p in rows if start<=p[0]<=end]
            match=min(route,key=lambda p:abs(p[2]-target_y),default=None)
            if match is None:continue
            offset=r.get('truth_csv_transform',{}).get('recorded_offset') or [0,0,0]
            samples={name:sample(rows if name=='truth_pose' else read_quat(Path(r['run'])/(name+'.csv')),match[0])
                     for name in ('truth_pose','mavros_pose','planner_setpoint','mavros_setpoint')}
            for s in samples.values():
                if s is not None:s['center_east_of_wall_face_m']=s['xyz'][0]+offset[0]-result['wall_world_east_face_x']
            result['samples'].append(dict(variant=r['variant'],route_segment_start_ros_s=start,
                matched_y_error_m=abs(match[2]-target_y),samples=samples))
        result['status']='RECORDED' if len(result['samples'])==2 else 'PARTIAL'
        result['interpretation']='At this cross-section compare truth and commanded x offsets and yaw. A closer trajectory/command supports a geometric explanation; it does not establish resume as the cause. Center-to-wall distances are not guard clearances.'
    except (OSError,ET.ParseError,ValueError,StopIteration,TypeError) as exc:
        result['input_issue']=str(exc)
    return result


def comparison(runs,batch):
    cases={r['variant']:r for r in runs};off=cases['resume_off'];on=cases['resume_on']
    reasons=[]
    if batch.get('active') or batch.get('status')!='COMPLETE':reasons.append('BATCH_INCOMPLETE')
    for r in runs:
        if r.get('status')!='PASS':reasons.append(r['variant']+':RUN_NOT_PASS')
        if r.get('cleanup_pass') is not True:reasons.append(r['variant']+':CLEANUP_NOT_PASS')
        if r.get('completed_mission_s') is None:reasons.append(r['variant']+':NO_COMPLETE_MISSION_TIME')
        releases=r.get('releases',{})
        if not releases.get('three_slots_exactly_once'):reasons.append(r['variant']+':THREE_SLOT_ACK_NOT_VERIFIED')
        if releases.get('correct_mock_release_count')!=3:reasons.append(r['variant']+':THREE_CORRECT_TRUTH_RELEASES_NOT_VERIFIED')
        if not r.get('slot_accounting',{}).get('counts_agree'):reasons.append(r['variant']+':ACCOUNTING_NOT_RECONCILED')
        if r.get('collisions')!=0:reasons.append(r['variant']+':ZERO_COLLISION_NOT_VERIFIED')
        if r.get('warnings'):reasons.append(r['variant']+':INPUT_WARNINGS')
    sources=[r.get('source') for r in runs]
    same_source=bool(batch.get('source')) and all(s==batch['source'] for s in sources)
    if not same_source:reasons.append('SOURCE_MISMATCH_OR_MISSING')
    same_truth=bool(off.get('truth_targets')) and off.get('truth_targets')==on.get('truth_targets')
    if not same_truth:reasons.append('TRUTH_LAYOUT_MISMATCH_OR_MISSING')
    scene_equal=None;config_equal=None;config_differences=[]
    if all(r.get('scene') for r in runs):
        try:
            signatures=[scene_signature(Path(r['scene'])/'field.world') for r in runs]
            scene_equal=signatures[0] is not None and signatures[0]==signatures[1]
            for name in ('motion_overrides.yaml','repair_overrides.yaml','frame_overrides.yaml','fast_gate.yaml','fast_runtime.yaml'):
                paths=[Path(r['scene'])/name for r in runs]
                values=[yaml.safe_load(p.read_text()) if p.is_file() else None for p in paths]
                if values[0] is None or values[0]!=values[1]:config_differences.append(name)
            config_equal=not config_differences
        except (OSError,ET.ParseError,yaml.YAMLError) as exc:
            config_differences.append(str(exc))
    if scene_equal is not True:reasons.append('WORLD_EQUIVALENCE_NOT_VERIFIED')
    if config_equal is not True:reasons.append('SCENE_CONFIG_EQUIVALENCE_NOT_VERIFIED')
    if off.get('resume',{}).get('config_enabled') is not False:reasons.append('OFF_SWITCH_NOT_VERIFIED')
    if on.get('resume',{}).get('config_enabled') is not True:reasons.append('ON_SWITCH_NOT_VERIFIED')
    resume=on.get('resume',{})
    if not resume.get('actual_survey_started'):reasons.append('RESUME_NOT_ACTUALLY_STARTED')
    if resume.get('outcome') in ('FAILED','UNAVAILABLE','UNKNOWN','RESUMED_INCOMPLETE','ASCENT_OR_REJOIN_INCOMPLETE'):
        reasons.append('RESUME_FAILED_OR_INCOMPLETE')
    dropclasses=lambda r:sorted(d['class_name'] for d in r.get('releases',{}).get('completed',[]))
    if dropclasses(off)!=dropclasses(on):reasons.append('DELIVERED_CLASS_SETS_DIFFER')
    a,b=off.get('completed_mission_s'),on.get('completed_mission_s')
    delta=a-b if a is not None and b is not None else None
    physical_reasons=[s for s in reasons if not s.endswith(':RUN_NOT_PASS') and not s.endswith(':NO_COMPLETE_MISSION_TIME')]
    ground_stop_gate_checks={'contract_errors_zero','land_success','manager_complete',
                            'mission_ros_within_limit','return_before_land'}
    for r in runs:
        physical=r.get('physical_completion',{})
        if not physical.get('verified'):physical_reasons.append(r['variant']+':PHYSICAL_COMPLETION_NOT_VERIFIED')
        if r.get('status')!='PASS':
            raw=r.get('raw_gate',{})
            allowed=bool(physical.get('operator_stop',{}).get('after_verified_stable_ground') and
                raw.get('reason')=='manager_failed' and
                set(raw.get('errors') or []).issubset({'manager_failed','manager_aborted'}) and
                set(raw.get('failed_checks') or []).issubset(ground_stop_gate_checks) and
                r.get('gate',{}).get('checks',{}).get('post_delivery_return_sequence') is True and
                not r.get('unresolved_action_failures',r.get('action_failures')))
            if not allowed:physical_reasons.append(r['variant']+':FAIL_NOT_EXPLAINED_BY_POST_GROUND_OPERATOR_STOP')
    pa,pb=[r.get('physical_completion',{}).get('physical_completion_mission_s') for r in (off,on)]
    physical_delta=pa-pb if pa is not None and pb is not None else None
    third=[next((d.get('mission_s') for d in r.get('releases',{}).get('completed',[]) if d['slot']==3),None) for r in (off,on)]
    return dict(benefit_measurable=not reasons,non_measurable_reasons=sorted(set(reasons)),
        conclusion=('Single-seed, single-pair descriptive comparison only.' if not reasons else
                    'Resume benefit cannot be measured from this pair; lack of trigger/failure/missing evidence is not zero benefit.'),
        same_source=same_source,same_truth_layout=same_truth,same_world=scene_equal,
        same_scene_configs=config_equal,scene_config_differences=config_differences,
        completed_mission_time_off_minus_on_s=delta,
        descriptive_time_delta_note='Raw completed-run difference only; do not attribute it to resume when benefit_measurable=false.',
        resume_attributable_time_saved_s=delta if not reasons else None,
        resume_attributable_percent_saved=100*delta/a if not reasons and a else None,
        physical_endpoint_comparison=dict(comparable=not physical_reasons,
            non_comparable_reasons=sorted(set(physical_reasons)),off_mission_s=pa,on_mission_s=pb,
            descriptive_off_minus_on_s=physical_delta,
            comparable_off_minus_on_s=physical_delta if not physical_reasons else None,
            note='Independent physical endpoint, not a software Gate PASS. A post-ground operator stop may explain software failure only when independently corroborated and other physical/mission checks hold. Single-pair descriptive difference is not a causal estimate.'),
        third_slot_ack=dict(off_mission_s=next((d.get('mission_s') for d in off.get('releases',{}).get('completed',[]) if d['slot']==3),None),
            on_mission_s=next((d.get('mission_s') for d in on.get('releases',{}).get('completed',[]) if d['slot']==3),None),
            off_ros_s=next((d['ros_s'] for d in off.get('releases',{}).get('completed',[]) if d['slot']==3),None),
            on_ros_s=next((d['ros_s'] for d in on.get('releases',{}).get('completed',[]) if d['slot']==3),None),
            descriptive_off_minus_on_s=third[0]-third[1] if all(t is not None for t in third) else None,
            three_correct_slots_each=all(r.get('releases',{}).get('three_slots_exactly_once') and
                r.get('releases',{}).get('correct_mock_release_count')==3 for r in (off,on)),
            observation_final=not batch.get('active') and batch.get('status')=='COMPLETE',
            note='Delivery milestone only. While a run is active this is not completed-flight savings, a Gate verdict or causal resume benefit.'),
        low_coverage_time_off_minus_on_s=(off.get('stage_durations_s',{}).get('LOW_COVERAGE',0)-on.get('stage_durations_s',{}).get('LOW_COVERAGE',0)
            if off.get('stage_intervals') and on.get('stage_intervals') and not batch.get('active') and batch.get('status')=='COMPLETE' else None))


def fmt(v):
    if v is None:return 'N/A'
    if isinstance(v,float):return f'{v:.3f}'
    return str(v)


def render(payload,out):
    runs=payload['runs'];c=payload['comparison']
    lines=['# seed38 高位续扫对照','',
        '本报告只读取本批原始日志；SITL/mock ACK 不代表实物投递落点或板端验收。', '',
        '**收益可衡量：'+('是（仅单 seed 单对照的描述性差值）' if c['benefit_measurable'] else '否')+'。**',
        '未触发、回高/接回失败、任务失败或证据不足时，续扫收益保持 `null`，不能解释成收益为零。','',
        '不可衡量原因：'+(', '.join(c['non_measurable_reasons']) or '无')+'。','',
        '| 轮次 | 原 Gate | 续扫状态 | 请求/回高派发/实际续扫 | 软件完成秒 | 物理完成秒 | 三槽 ACK | 真值正确模拟释放 | 碰撞 |',
        '|---|---|---|---|---|---|---|---|---|']
    table=[]
    for r in runs:
        s=r.get('resume',{});d=r.get('releases',{})
        values=[r['variant'],r.get('status'),s.get('outcome'),
            '/'.join(fmt(s.get(k)) for k in ('attempted','ascent_published','actual_survey_started')),
            r.get('completed_mission_s'),r.get('physical_completion',{}).get('physical_completion_mission_s'),
            d.get('completed_execution_count'),d.get('correct_mock_release_count'),r.get('collisions')]
        table.append(values);lines.append('| '+' | '.join(fmt(v) for v in values)+' |')
    lines+=['','完整任务原始差值（off−on）：'+fmt(c['completed_mission_time_off_minus_on_s'])+' s。',
        '可归因于续扫的差值：'+fmt(c['resume_attributable_time_saved_s'])+' s。','',
        '物理完成端点原始差值（off−on）：'+fmt(c['physical_endpoint_comparison']['descriptive_off_minus_on_s'])+' s。',
        '物理端点可比：'+str(c['physical_endpoint_comparison']['comparable'])+'；阻断原因：'+', '.join(c['physical_endpoint_comparison']['non_comparable_reasons'])+'。','',
        '第三槽 ACK 任务秒：off '+fmt(c['third_slot_ack']['off_mission_s'])+'，on '+fmt(c['third_slot_ack']['on_mission_s'])+
        '；阶段差值 off−on '+fmt(c['third_slot_ack']['descriptive_off_minus_on_s'])+' s。这是投递里程碑，不作为整轮飞行节省或 Gate 结论。','',
        '## 计量口径','',
        '- 内嵌历史事件按完整内容去重，保留首次发布；阶段时长来自状态首次发布，含录制采样延迟。',
        '- `SURVEY_RESUME_ONCE` 是请求；`resume_ascent` 是实际派发；`SURVEY_RESUMED` 才是接回后续扫。`resume_completed` 在失败时也会置位。',
        '- 限高 ACK 参数载荷未直接录制。回高派发只提供按该源码准入检查推断的证据；规划 STARTED 回执另行统计。请求至派发时间不是纯 ACK 延迟。',
        '- 以释放回执源时间匹配 truth_pose.csv 最近样本；最大样本年龄 '+fmt(payload['pose_max_age_s'])+' s。类别匹配且机体 XY 位于真值靶板名义旋转方形内才算正确模拟释放：普通靶 1m、红十字 0.35m。',
        '- CSV 真值已减 recorder offset；目标采用同一 XY 变换。判别跳过/证伪位置时，仅依据日志内明确坐标，不猜测位置。',
        '- 原 Gate、物理支撑接触、执行 ACK 与任务记账各自保留。物理完成要求持续接触、ON_GROUND、解除武装及至少2秒连续真值稳定；短暂碰地不当作稳定落地。',
        '- MAVROS state/extended_state 日志只记录状态变化；持有的地面/解除武装状态不套用0.5秒 odom 新鲜度门槛。',
        '- 阶段和五类 CSV 飞行指标在已验证物理完成处截尾；之后地面等待及人工停止另列，不加进飞行时长。软件完成仍只认可 COMPLETE，原 FAIL 不改写。',
        '- 续扫结论只依据续扫专用事件；后续人工 abort 不覆盖此前 FOUND_MISSING/RESUMED_ENDED。','']
    lines+=['- NOT_STARTED 的不同 execution_id 仅表示拒绝回执，不计重复执行或占槽。执行次数只计 RAW_CALL_STARTED/COMPLETED；完整相同回执重播另计。',
        '- 本地重试须同时关联严格 action 身份的 Bridge preflight 拒绝、下一条 REVISIT 与该槽 FREE 状态；后续成功投递另按 decision/attempt 记账，原失败回执仍保留。','']
    lines+=['## 同门接触与轨迹快速比较','',
        '以下只比较 on 接触时真值 y 对应的两轮第三段返航横截面。采样不同步，机体中心到墙面的距离不是防护包络净距；不能据此归因续扫策略。','']
    corridor=payload.get('corridor_contact_comparison',{})
    lines+=['记录状态：'+str(corridor.get('status'))+'。',
            'on 接触时刻：'+fmt(corridor.get('contact_ros_s'))+' ROS s。','',
            '| 轮次 | 真值 ROS秒 | 真值 x/y/z | 真值 yaw度 | 中心在墙东侧距离 m | 规划 x/y/z | 规划 yaw度 |',
            '|---|---|---|---|---|---|---|']
    for case in corridor.get('samples',[]):
        s=case['samples'];truth=s.get('truth_pose') or {};plan=s.get('planner_setpoint') or {}
        lines.append('| '+' | '.join(fmt(v) for v in (case['variant'],truth.get('ros_s'),truth.get('xyz'),truth.get('yaw_deg'),
            truth.get('center_east_of_wall_face_m'),plan.get('xyz'),plan.get('yaw_deg')))+' |')
    for r in runs:
        lines+=['## '+r['variant'],'','运行目录：`'+str(r.get('run'))+'`。',
            '首次失败：`'+canonical(r.get('first_failure'))+'`。','',
            '原 Gate：`'+canonical(r.get('raw_gate'))+'`。',
            '结果分类：`'+str(r.get('outcome_classification'))+'`。','',
            '### 物理完成与人工停止','']
        physical=r.get('physical_completion',{});stop=physical.get('operator_stop',{})
        for label,key in (('首次支撑接触','first_touchdown_contact_ros_s'),('持续支撑开始','sustained_support_start_ros_s'),
                          ('物理完成确认','physical_completion_ros_s')):
            lines.append('- '+label+'：'+fmt(physical.get(key))+' ROS s。')
        lines+=['- ON_GROUND：'+fmt((physical.get('ground_state_evidence') or {}).get('received_ros_s'))+' ROS s。',
                '- 解除武装：'+fmt((physical.get('disarmed_evidence') or {}).get('received_ros_s'))+' ROS s。',
                '- 人工停止请求：'+fmt(stop.get('request_ros_s'))+' ROS s；ABORT 派发：'+fmt(stop.get('abort_command_ros_s'))+' ROS s。',
                '- 已确认地面等待：'+fmt(stop.get('ground_wait_before_stop_s'))+' s。',
                '- 飞行指标终点：'+fmt(r.get('flight_metrics_end_ros_s'))+' ROS s（'+str(r.get('flight_metrics_end_source'))+'）。',
                '- 稳定地面尾段：`'+canonical(physical.get('stable_ground_tail'))+'`。','',
                '### 接触 Gate 与完成状态','',
                '实际碰撞次数：'+fmt(r.get('collisions'))+'；接触 Gate 停止：'+fmt(r.get('contact_outcome',{}).get('stopped_by_contact_gate'))+'。',
                '防护包络擦碰保留为 Gate 碰撞；根据采样深度、力和接触对描述，不表述为严重坠毁，也不把 Gate 停止前的返航称为完成。','',
                '| 首次/末次 ROS秒 | 时长 s | 样本数 | 最大深度 mm | 峰值力 N | 仅记录 guard | 接触对 |',
                '|---|---|---|---|---|---|---|']
        for e in r.get('contact_outcome',{}).get('events',[]):
            lines.append('| '+' | '.join(fmt(v) for v in (str(e['first_ros_s'])+'/'+str(e['last_ros_s']),e['duration_s'],e['sample_count'],
                e['peak_depth_mm'],e['peak_force_n'],e['guard_only_recorded'],e['pairs']))+' |')
        lines+=['',
                '### 阶段时长（ROS 秒，截于物理完成或软件终点）','','| 阶段 | 时长 |','|---|---|']
        lines+=['| '+str(k)+' | '+fmt(v)+' |' for k,v in r.get('stage_durations_s',{}).items()]
        lines+=['','### 回高与接回','','| 动作 | decision_seq | 请求→派发秒 | 派发→规划 STARTED 秒 | 派发→到达秒 |','|---|---|---|---|---|']
        for a in r.get('resume',{}).get('actions',[]):
            lines.append('| '+' | '.join(fmt(v) for v in (a['reason'],a['decision_seq'],
                r['resume']['request_to_ascent_dispatch_s'] if a['reason']=='resume_ascent' else None,
                a['dispatch_to_planner_started_s'],a['motion_duration_s']))+' |')
        lines+=['','### 三槽回执与真值','','| 槽 | 执行 ID | 类别 | ACK ROS秒 | ACK任务秒 | 最近真值类别 | 距离 m | 样本年龄 s | 正确模拟释放 |','|---|---|---|---|---|---|---|---|---|']
        for d in r.get('releases',{}).get('completed',[]):
            a=d.get('truth_association') or {};p=d.get('truth_pose') or {}
            lines.append('| '+' | '.join(fmt(v) for v in (d['slot'],d['execution_id'],d['class_name'],
                d['ros_s'],d.get('mission_s'),a.get('nearest_class'),a.get('nearest_distance_m'),p.get('sample_age_s'),d['correct_mock_release']))+' |')
        lines+=['','记账核对：`'+canonical(r.get('slot_accounting'))+'`。','',
            '### 本地 preflight 重试（未消耗槽位）','',
            '| 原 decision/attempt/槽 | NOT_STARTED IDs | 未耗槽确认 | REVISIT decision | 后续成功 decision/attempt/exec | 拒绝至后续 ACK秒 |',
            '|---|---|---|---|---|---|']
        for retry in r.get('local_preflight_retries',[]):
            identity=retry['identity'];rv=(retry.get('revisit_decision') or {}).get('data',{})
            recovered=retry.get('subsequent_successful_attempt')
            values=['/'.join(map(str,identity[1:4])),retry['not_started_execution_ids'],retry['slot_not_consumed_verified'],rv.get('decision_seq'),
                    '/'.join(map(str,[recovered['identity'][1],recovered['identity'][2],recovered['execution_id']])) if recovered else None,
                    retry['denial_to_recovered_ack_s']]
            lines.append('| '+' | '.join(fmt(v) for v in values)+' |')
        if not r.get('local_preflight_retries'):lines.append('未记录 NOT_STARTED 本地重试。')
        lines+=['','执行/拒绝 ID/回执重播数：'+ '/'.join(fmt(r.get('releases',{}).get(k)) for k in
            ('positive_execution_count','not_started_only_id_count','duplicate_message_count'))+'。','',
            '### permission_stale 时间诊断','']
        diag=r.get('release_preflight_clock_diagnostic',{})
        if not diag.get('evidence'):lines.append('未记录 bag 摘要；不猜测许可过期原因。')
        for finding in diag.get('evidence',[]):
            lines.append('- `'+canonical(finding)+'`')
        if diag.get('evidence'):
            lines.append('记录支持约1 ms未来时间戳/跨节点时钟回调差异这一推断；并未确认长期过期。缺少代理内部许可对象与当时 now 的 trace，不能给出确定原因。本对照未改阈值。')
        lines+=['',
            '### 跳过、否定与类别修正','']
        for v in r.get('target_events',[]):lines.append('- `'+canonical(v)+'`')
        if not r.get('target_events'):lines.append('未记录到相关事件；数据缺失见输入问题，不能自动视为全部目标通过。')
        lines+=['','输入问题：'+('; '.join(r.get('warnings',[])) or '无')+'。','']
    md='\n'.join(lines)+'\n'
    h=html.escape
    cards=[]
    for r in runs:
        bars=''
        totals=r.get('stage_durations_s',{});scale=max(totals.values(),default=1) or 1
        for stage,seconds in totals.items():
            bars+=f'<div class="bar"><span>{h(stage)} {seconds:.2f}s</span><i style="width:{100*seconds/scale:.2f}%"></i></div>'
        links=' '.join('<a href="'+h(str(Path(path).relative_to(out) if Path(path).is_relative_to(out) else __import__('os').path.relpath(path,out)),quote=True)+'">'+h(name)+'</a>' for name,path in r.get('artifacts',{}).items())
        physical=r.get('physical_completion',{});stop=physical.get('operator_stop',{})
        completion_text=('<p>原 Gate：'+h(fmt(r.get('status')))+'；结果分类：'+h(fmt(r.get('outcome_classification')))+
            '。物理确认：'+h(fmt(physical.get('physical_completion_ros_s')))+' ROS s；人工停止请求：'+
            h(fmt(stop.get('request_ros_s')))+' ROS s；排除地面等待：'+h(fmt(stop.get('ground_wait_before_stop_s')))+' s。</p>')
        retry_text=''.join('<p>本地重试 decision '+h(fmt(retry['identity'][1]))+'：NOT_STARTED IDs '+
            h(fmt(retry['not_started_execution_ids']))+'；未耗槽确认 '+h(fmt(retry['slot_not_consumed_verified']))+
            '；后续成功 exec '+h(fmt((retry.get('subsequent_successful_attempt') or {}).get('execution_id')))+'。不同拒绝 IDs 不计重复投递。</p>'
            for retry in r.get('local_preflight_retries',[]))
        contact_text=''.join('<p>防护包络接触 '+h(fmt(e['first_ros_s']))+'–'+h(fmt(e['last_ros_s']))+
            ' ROS s；持续 '+h(fmt(e['duration_s']))+' s，深度 '+h(fmt(e['peak_depth_mm']))+' mm，峰值力 '+h(fmt(e['peak_force_n']))+
            ' N；仅记录 guard：'+h(fmt(e['guard_only_recorded']))+'。原碰撞 Gate 保留；不作严重坠毁或返航完成结论。</p>' for e in r.get('contact_outcome',{}).get('events',[]))
        diag_text='<p>permission_stale：可能约1 ms未来时钟偏差，非已确认长期过期；缺少节点内部 trace。</p>' if r.get('release_preflight_clock_diagnostic',{}).get('evidence') else ''
        cards.append('<section><h2>'+h(r['variant'])+'</h2>'+completion_text+retry_text+contact_text+diag_text+bars+'<p>'+links+'</p><details><summary>完整本轮数据</summary><pre>'+h(json.dumps(r,ensure_ascii=False,indent=2))+'</pre></details></section>')
    header=''.join('<th>'+h(x)+'</th>' for x in ('轮次','原 Gate','续扫状态','请求/派发/续扫','软件完成秒','物理完成秒','ACK','正确释放','碰撞'))
    rows=''.join('<tr>'+''.join('<td>'+h(fmt(v))+'</td>' for v in row)+'</tr>' for row in table)
    page='<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>seed38 续扫对照</title><style>body{max-width:1200px;margin:32px auto;padding:0 20px;font:16px/1.6 system-ui;background:#f5f7fa;color:#203040}table{border-collapse:collapse;width:100%;background:white}td,th{border:1px solid #ccd5df;padding:8px}section{background:white;padding:20px;margin:20px 0;border-radius:8px}pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:12px}.bar{position:relative;min-height:28px;margin:5px 0;background:#edf2f7}.bar span{position:relative;z-index:2;padding-left:8px}.bar i{position:absolute;left:0;top:0;bottom:0;background:#b8d5ef}a{margin-right:12px}</style><h1>seed38 高位续扫对照</h1><p>收益可衡量：<b>'+('是，单 seed 描述性对照' if c['benefit_measurable'] else '否')+'</b>。未触发或失败不能衡量收益。</p><p>'+h(', '.join(c['non_measurable_reasons']))+'</p><p>SITL/mock 执行确认；不代表实物落点。限高 ACK 未直接录制。</p><p><a href="REPORT.md">完整报告及口径</a><a href="comparison.json">机器可读数据</a></p><table><tr>'+header+'</tr>'+rows+'</table>'+''.join(cards)+'</html>'
    return md,page


def self_test():
    target=dict(name='panzer',class_name='panzer',eval_xy=[0,0],world_xy=[0,0],yaw=0,nominal_half_side_m=.5)
    assert truth_association('panzer',[.4,0],[target])['within_nominal_board']
    assert not truth_association('panzer',[.6,0],[target])['within_nominal_board']
    common=dict(mission_id='m',decision_seq=4,attempt=1,payload_slot=1,target_id=7,
                target_first_seen={'stamp_ns':1},target_class='panzer',align_mode='drop_circle')
    def event(state,execution,t):
        d=dict(common,execution_id=execution,execution_state=state,success=state==3,
               terminal=state!=2,header={'stamp':{'stamp_ns':int(t*1e9)}})
        return dict(kind='release',ros_sec=t,data=d)
    events=[event(1,1,1),event(2,2,2),event(3,2,3),event(3,2,3)]
    r=release_summary(events,[[3,0,0,2]],[target],.25)
    assert r['completed_execution_count']==1 and r['correct_mock_release_count']==1
    assert r['completed'][0]['execution_id']==2 and len(r['not_started'])==1
    assert r['positive_execution_count']==1 and r['not_started_only_id_count']==1 and r['duplicate_message_count']==1
    denied=[event(1,3,1),event(1,4,1.01)]
    failure=dict(kind='result',ros_sec=1.01,data=dict(common,terminal=True,retryable=True,
        reason='release_preflight_rejected',payload_committed=False,status=4))
    revisit=dict(kind='decision',ros_sec=1.01,data=dict(mission_id='m',decision_seq=5,reason='high_view_full:REVISIT'))
    free=dict(kind='mission',ros_sec=1.01,data=dict(mission_id='m',slot_status=['FREE','FREE','FREE'],committed_slots=0))
    negative=release_summary(denied,[],[target],.25)
    assert negative['positive_execution_count']==0 and negative['not_started_only_id_count']==2
    assert not negative['anomalies'] and negative['duplicate_message_count']==0
    retries=local_retry_summary(denied+[failure,revisit,free],negative)
    assert len(retries)==1 and retries[0]['slot_not_consumed_verified'] and retries[0]['not_started_execution_ids']==[3,4]
    assert not local_retry_summary(denied+[failure,revisit],negative)[0]['slot_not_consumed_verified']
    later=event(3,5,3);later['data'].update(decision_seq=6,attempt=2)
    recovered=release_summary(denied+[later],[[3,0,0,2]],[target],.25)
    assert local_retry_summary(denied+[failure,revisit,free,later],recovered)[0]['subsequent_successful_attempt']['execution_id']==5
    same_action=release_summary(denied+[event(2,5,2)],[],[target],.25)
    assert not local_retry_summary(denied+[failure,revisit,free],same_action)[0]['slot_not_consumed_verified']
    assert release_summary(events,[[1,0,0,2]],[target],.25)['truth_unknown_count']==1
    snap=dict(scope='HIGH_VIEW_FULL_MISSION',stage='RESUME_ASCEND',events=[dict(stage='SURVEY_RESUME_ONCE',time=1)],
              resume_attempted=True,resume_completed=True,survey_policy=dict(resume_survey_enabled=True))
    high,frozen,memory,changes=freeze_high([dict(t=1,status=snap),dict(t=2,status=snap)])
    assert len(frozen)==1
    failed=dict(snap,events=snap['events']+[dict(stage='SURVEY_RESUME_FAILED',time=2,reason='unreachable')])
    high,frozen,_,changes=freeze_high([dict(t=1,status=snap),dict(t=2,status=failed)])
    summary=resume_summary(high,frozen,[],[],changes)
    assert summary['outcome']=='FAILED' and not summary['actual_survey_started']
    snap=dict(scope='HIGH_VIEW_FULL_MISSION',stage='SURVEY',events=[],survey_policy=dict(resume_survey_enabled=True))
    high,frozen,_,changes=freeze_high([dict(t=1,status=snap)])
    assert resume_summary(high,frozen,[],[],changes)['outcome']=='NOT_TRIGGERED'
    assert resume_summary(high,frozen,[],[],changes,finished=False)['outcome']=='NOT_TRIGGERED_YET'
    resumed=dict(snap,resume_started=2.,resume_completed=True,done=True,succeeded=False,
                 failure='manual_abort_requested',events=[dict(stage='SURVEY_RESUME_ONCE',time=1),
                 dict(stage='SURVEY_RESUMED',time=2),dict(stage='SURVEY_RESUME_FOUND_MISSING',time=3)])
    high,frozen,_,changes=freeze_high([dict(t=4,status=resumed)])
    assert resume_summary(high,frozen,[],[],changes)['outcome']=='FOUND_MISSING'
    production_decisions=[dict(kind='decision',ros_sec=1.1,data=dict(mission_id='m',decision_seq=4,
                          reason='high_view_full:RESUME_ASCEND')),
                          dict(kind='decision',ros_sec=1.5,data=dict(mission_id='m',decision_seq=5,
                          reason='high_view_full:RESUME_JOIN')),
                          dict(kind='decision',ros_sec=2.1,data=dict(mission_id='m',decision_seq=6,
                          reason='high_view_full:SURVEY'))]
    rs=resume_summary(high,frozen,production_decisions,[],changes)
    assert [a['reason'] for a in rs['actions']]==['resume_ascent','resume_rejoin','remaining_survey']
    assert abs(rs['request_to_ascent_dispatch_s']-.1)<1e-9
    def key(t,kind,**data):return dict(ros_sec=t,kind=kind,data=data)
    keys=[key(9,'decision',command=5),key(14,'extended_state',landed_state=1),
          key(15,'state',connected=True,armed=False),key(22.1,'decision',command=7,reason='manual_abort_requested')]
    poses=[[i/10,0,0,0] for i in range(100,251)]
    contacts=dict(support_events=[dict(ros_stamp=10,last_ros_stamp=10.01),dict(ros_stamp=12,last_ros_stamp=25)])
    phys=physical_summary(keys,poses,contacts,dict(ros_sec=22,reason='operator_ground_stop'),1,25,.25)
    assert phys['verified'] and phys['physical_completion_ros_s']==15
    assert phys['first_touchdown_contact_ros_s']==10 and phys['sustained_support_start_ros_s']==12
    assert phys['operator_stop']['after_verified_stable_ground'] and phys['operator_stop']['ground_wait_before_stop_s']==7
    assert not physical_summary(keys,poses,dict(support_events=[]),dict(ros_sec=22),1,25,.25)['verified']
    assert not physical_summary(keys[:-2],poses,contacts,dict(ros_sec=22),1,25,.25)['verified']
    moving=[[p[0],p[0]*.1,0,0] for p in poses]
    assert not physical_summary(keys,moving,contacts,dict(ros_sec=22),1,25,.25)['verified']
    assert sum(intervals([dict(ros_s=1,stage='FLIGHT')],1,phys['physical_completion_ros_s'])[1].values())==14
    segments,totals=intervals([dict(ros_s=1,stage='A'),dict(ros_s=3,stage='B')],2,5)
    assert totals=={'A':1.,'B':2.}
    print('SELF-TEST PASS: release IDs/duplicates/truth, phase clipping, failed/not-triggered/success-then-manual-abort resume, contact+ground+disarm+stable truth, bounce/missing evidence/motion rejection, post-ground stop')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,default=DEFAULT_ROOT)
    p.add_argument('--matrix',type=Path)
    p.add_argument('--out',type=Path)
    p.add_argument('--pose-max-age',type=float,default=.25)
    p.add_argument('--dry-run',action='store_true',help='Print JSON without creating directories or files')
    p.add_argument('--allow-incomplete',action='store_true',help='Diagnostic only: include active/missing cases, never claim benefit')
    p.add_argument('--self-test',action='store_true',help='In-memory checks only; no fixture/report files')
    args=p.parse_args()
    if args.self_test:self_test();return
    if not math.isfinite(args.pose_max_age) or args.pose_max_age<=0:p.error('--pose-max-age must be positive')
    root=args.root.resolve();doc=root/DOC;out=(args.out or doc).resolve()
    matrix=(args.matrix or root/'logs/seed38_resume_20261005_batch/matrix.json').resolve()
    batch=json.loads(matrix.read_text(encoding='utf-8-sig'))
    rows={}
    for row in batch.get('results',[]):
        if row.get('seed')!=38 or row.get('variant') not in VARIANTS:p.error('Unexpected matrix case')
        if row['variant'] in rows:p.error('Duplicate matrix case; choose a single frozen pair')
        rows[row['variant']]=row
    if not args.allow_incomplete and (batch.get('active') or batch.get('status')!='COMPLETE' or set(rows)!=set(VARIANTS)):
        p.error('Matrix is not a completed two-case pair. For read-only inspection use --allow-incomplete --dry-run.')
    if args.allow_incomplete:
        active=batch.get('active')
        if active and active.get('variant') in VARIANTS:rows.setdefault(active['variant'],active)
        for v in VARIANTS:rows.setdefault(v,dict(seed=38,variant=v,source=batch.get('source'),status='NOT_RUN'))
    runs=[analyze_case(rows[v],root,doc,args) for v in VARIANTS]
    payload=dict(schema_version=1,generated_at_utc=datetime.now(timezone.utc).isoformat(),
        matrix=str(matrix),matrix_status=batch.get('status'),source=batch.get('source'),pose_max_age_s=args.pose_max_age,
        runs=runs,comparison=comparison(runs,batch),corridor_contact_comparison=corridor_contact_comparison(runs,args.pose_max_age))
    encoded=json.dumps(payload,ensure_ascii=False,indent=2,allow_nan=False)+'\n'
    if args.dry_run:sys.stdout.write(encoded);return
    md,page=render(payload,out)
    out.mkdir(parents=True,exist_ok=True)
    for name,text in (('comparison.json',encoded),('REPORT.md',md),('index.html',page)):
        (out/name).write_text(text,encoding='utf-8')
    print(json.dumps(dict(out=str(out),benefit_measurable=payload['comparison']['benefit_measurable'],
                         reasons=payload['comparison']['non_measurable_reasons']),ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()

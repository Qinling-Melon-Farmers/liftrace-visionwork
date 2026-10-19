"""Offline waypoint dwell accounting from existing extracts; never reads a bag."""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import numpy as np


def load(path):
    with path.open(encoding='utf-8') as handle:
        return json.load(handle)


def xyz(p):
    return np.array([p[k] for k in ('x', 'y', 'z')], dtype=float)


def stamp(m, start):
    s = m.get('header', {}).get('stamp', {})
    return s.get('secs', 0) + s.get('nsecs', 0) / 1e9 - start


def sample(row, start):
    m = row['m']
    return dict(t=row['t'], s=stamp(m, start),
                p=xyz(m['pose']['pose']['position']),
                v=float(np.linalg.norm(xyz(m['twist']['twist']['linear']))))


def persistent_motion(samples, lower, upper, goal, speed=.15, span=.5, displacement=.08):
    """Require a full speed interval AND net displacement toward this goal."""
    a = [r for r in samples if lower <= r['s'] < upper]
    for i, r in enumerate(a):
        if r['v'] < speed:
            continue
        for j in range(i+1, len(a)):
            if a[j]['s']-a[j-1]['s'] > .15 or a[j]['v'] < speed:
                break
            if a[j]['s']-r['s'] >= span:
                moved = float(np.linalg.norm(a[j]['p']-r['p']))
                progress = float(np.linalg.norm(goal-r['p'])-np.linalg.norm(goal-a[j]['p']))
                if moved >= displacement and progress >= displacement:
                    return dict(start=r['s'], confirmed=a[j]['s'], displacement=moved,
                                progress=progress, receipt=r['t'])
                break
    return None


def stationary_runs(samples, lo, hi):
    """Continuous <=.10m/s for >=1s with 3D bounding-box diagonal <=.10m."""
    a = [r for r in samples if lo <= r['s'] <= hi]
    runs, active = [], []
    def close():
        if len(active)>1 and active[-1]['s']-active[0]['s']>=1.:
            points=np.array([r['p'] for r in active])
            diameter=float(np.linalg.norm(np.ptp(points,axis=0)))
            if diameter<=.10:
                runs.append(dict(start=active[0]['s'],end=active[-1]['s'],
                                 duration=active[-1]['s']-active[0]['s'],extent=diameter))
    for row in a:
        if row['v']>.10 or (active and row['s']-active[-1]['s']>.15):
            close();active=[]
        if row['v']<=.10:
            active.append(row)
    close()
    return runs


def stats(values):
    a = np.array([v for v in values if v is not None], dtype=float)
    if not len(a):return dict(n=0,total=None,p50=None,p95=None,max=None)
    return dict(n=len(a),total=float(a.sum()),p50=float(np.percentile(a,50)),
                p95=float(np.percentile(a,95)),max=float(a.max()))


def analyze(root, bag):
    base=root/'试飞产物'
    replay=load(base/('replay_10111009_%d'%bag)/'data.json')
    start=replay['start'];rows=replay['rows']
    if bag==1:
        extras=load(base/'analysis_flight_10111009_1/collision_1_records.json')['rows']
    else:
        extras={}
        for row in load(base/'analysis_10111009_2/extra_events.json'):
            extras.setdefault(row['topic'],[]).append(dict(t=row['t'],m=row['msg']))
    metadata=json.loads(extras['/board_trials/run_metadata'][0]['m']['data'])
    nav=[sample(r,start) for r in extras['/navigation/local_odom']]
    fc=[sample(r,start) for r in rows['odom']]
    assert all(r['s']<=q['s'] for r,q in zip(nav,nav[1:])), 'navigation odom clock rewind'
    assert all(r['s']<=q['s'] for r,q in zip(fc,fc[1:])), 'FC odom clock rewind'
    static=[t for r in rows['tf_static'] for t in r['m']['transforms']
            if t['header']['frame_id']=='map' and t['child_frame_id']=='camera_init']
    assert static and all(np.linalg.norm(xyz(t['transform']['translation']))<1e-9 and
              abs(t['transform']['rotation']['w']-1)<1e-9 for t in static), 'Frame mapping is not identity'
    plans=extras.get('/planning/goal_status',[])
    bstatus=[dict(t=r['t'],m=json.loads(r['m']['data']))
             for r in extras.get('/navigation/planner_bridge_status',[])]
    commands=rows['command'];records=[];windows=[]
    for index,cmd in enumerate(commands):
        if cmd['m']['reason']!='high_view_full:SURVEY':continue
        m=cmd['m'];seq=m['decision_seq'];mission=m['mission_id']
        next_cmd=commands[index+1] if index+1<len(commands) else None
        end=next_cmd['t'] if next_cmd else replay['duration']
        goal_rows=[g for g in rows['goal'] if g['m']['header']['seq']==seq and g['t']>=cmd['t']-.001 and g['t']<end]
        assert len(goal_rows)==1, ('Ambiguous or missing goal',bag,seq)
        goal_row=goal_rows[0];goal=xyz(goal_row['m']['pose']['position'])
        results=[r for r in rows['result'] if r['m']['mission_id']==mission and r['m']['decision_seq']==seq]
        def first(reason):return next((r for r in results if r['m']['reason']==reason),None)
        ready=first('planner_trajectory_ready');arrival=first('motion_arrival_confirmed')
        accepted=first('planner_goal_accepted')
        same_plan=[p for p in plans if p['m']['goal_seq']==seq and p['t']<end]
        effective=next((xyz(p['m']['effective_goal']['pose']['position']) for p in reversed(same_plan)
                        if p['t']<=(arrival['t'] if arrival else end)),goal)
        reached=next((p for p in same_plan if p['m']['status']==4),None)
        markers=[r for r in rows['trajectory'] if goal_row['t']<=r['t']<end
                 and r['m']['action']==0 and r['m']['points']]
        payloads=[r for r in extras.get('/planning/bspline',[]) if r['m']['goal_stamp']==m['header']['stamp'] and r['m']['goal_frame']==m['goal']['header']['frame_id'] and r['t']<end]
        move=persistent_motion(fc,goal_row['t'],end,goal)
        eligible=[r for r in nav if r['s']>=(ready['stamp'] if ready else goal_row['t']) and
                  r['t']<=(arrival['t'] if arrival else end)]
        for r in eligible:
            r['distance']=float(np.linalg.norm(r['p']-effective))
            r['qualified']=r['distance']<=.12 and r['v']<=.20 and 0<=r['t']-r['s']<=.50
        # A finished-trajectory event forbids earlier odom from completing arrival.
        finished_source=stamp(reached['m'],start) if reached else None
        first_position=next((r['s'] for r in eligible if r['distance']<=.12),None)
        last_position_start=None
        position_exits=0
        was_inside=False
        for r in eligible:
            inside=r['distance']<=.12
            if inside and last_position_start is None:last_position_start=r['s']
            if not inside:
                if was_inside:position_exits+=1
                last_position_start=None
            was_inside=inside
        # Persistent geometric entry, not a single sample.
        enter=None;position_start=None
        for r in eligible:
            if r['distance']>.12:
                position_start=None
            elif position_start is None:
                position_start=r['s']
            elif r['s']-position_start>=.20 and enter is None:
                enter=position_start
        final_start=None;resets=0;was_qualified=False;previous=None
        for r in eligible:
            q=r['qualified'] and (finished_source is None or r['s']>=finished_source) and (previous is None or 0<=r['s']-previous['s']<=.50)
            if q:
                if final_start is None:final_start=r['s']
            else:
                if was_qualified:resets+=1
                final_start=None
            was_qualified=q;previous=r
        # Only an observed terminal arrival establishes final dwell.
        if not arrival:final_start=None
        observed=[r for r in bstatus if r['m']['executor']['active_decision_seq']==seq and
                  r['m']['executor']['dwell_start_ns']>0 and r['t']<(arrival['t'] if arrival else end)]
        reported=observed[-1]['m']['executor']['dwell_start_ns']/1e9-start if observed else None
        speed_block=sum(min(.15,b['s']-a['s']) for a,b in zip(eligible,eligible[1:])
                        if a['distance']<=.12 and a['v']>.20)
        hold_end=arrival['t'] if arrival else end
        holds=stationary_runs(fc,goal_row['t'],hold_end)
        for h in holds:windows.append(dict(bag=bag,seq=seq,**h))
        record=dict(bag=bag,mission_id=mission,decision_seq=seq,object_target_id=m['target_id'],
             role='staging_low' if index==0 else 'ascent_high' if index==1 else 'high_survey',
             x=float(goal[0]),y=float(goal[1]),z=float(goal[2]),
             command_source_s=cmd['stamp'],command_published_s=cmd['t'],goal_published_s=goal_row['t'],
             planner_accepted_s=accepted['t'] if accepted else None,
             first_trajectory_ready_s=ready['t'] if ready else None,
             first_visual_trajectory_s=markers[0]['t'] if markers else None,
             first_bspline_payload_s=payloads[0]['t'] if payloads else None,
             first_bspline_traj_id=payloads[0]['m']['traj_id'] if payloads else None,
             fc_motion_start_s=move['start'] if move else None,
             fc_motion_confirmed_s=move['confirmed'] if move else None,
             planner_reached_s=reached['t'] if reached else None,
             first_position_sample_s=first_position,sustained_position_entry_s=enter,
             final_continuous_position_start_s=last_position_start,position_exits=position_exits,
             final_qualified_start_s=final_start,reported_dwell_start_s=reported,
             arrival_confirmed_s=arrival['t'] if arrival else None,
             next_decision_seq=next_cmd['m']['decision_seq'] if next_cmd else None,
             next_reason=next_cmd['m']['reason'] if next_cmd else None,
             next_command_published_s=next_cmd['t'] if next_cmd else None,
             goal_to_first_trajectory_s=ready['t']-goal_row['t'] if ready else None,
             ready_to_fc_motion_s=move['start']-ready['t'] if ready and move else None,
             goal_to_arrival_s=arrival['t']-goal_row['t'] if arrival else None,
             planner_reached_to_arrival_s=arrival['t']-reached['t'] if reached and arrival else None,
             position_entry_to_final_qualified_s=final_start-enter if final_start is not None and enter is not None else None,
             final_qualified_to_arrival_s=arrival['t']-final_start if final_start is not None else None,
             arrival_to_next_command_s=next_cmd['t']-arrival['t'] if next_cmd and arrival else None,
             inside_radius_speed_block_s=speed_block,qualification_resets=resets,
             confirmed_stationary_s=sum(h['duration'] for h in holds),
             confirmed_stationary_count=len(holds),
             terminal='arrived' if arrival else 'superseded_without_arrival' if next_cmd else 'right_censored')
        # Partition the planner-reached interval by geometry and speed, never
        # equate a planner event to a physical stop. Integrate source-time spans.
        after=[r for r in eligible if reached and finished_source<=r['s']]
        outside=inside_fast=inside_slow=unknown=0.
        for left,right in zip(after,after[1:]):
            dt=right['s']-left['s']
            if dt>.15:
                unknown+=dt
            elif left['distance']>.12:
                outside+=dt
            elif left['v']>.20:
                inside_fast+=dt
            else:
                inside_slow+=dt
        record.update(after_planner_reached_outside_radius_s=outside if reached else None,
                      after_planner_reached_inside_fast_s=inside_fast if reached else None,
                      after_planner_reached_inside_slow_s=inside_slow if reached else None,
                      after_planner_reached_sample_gap_s=unknown if reached else None,
                      trajectory_ready_count=sum(q['m']['reason']=='planner_trajectory_ready' for q in results),
                      planner_replan_attempt_count=sum(q['m']['status']==3 for q in same_plan))
        records.append(record)
    # Link the outgoing leg by unique decision ID, including stage changes.
    for r in records:
        seq=r['next_decision_seq'];next_cmd=next((c for c in commands if c['m']['decision_seq']==seq),None)
        next_goal=next((g for g in rows['goal'] if g['m']['header']['seq']==seq),None)
        next_ready=next((q for q in rows['result'] if q['m']['decision_seq']==seq and
                         q['m']['reason']=='planner_trajectory_ready'),None)
        next_index=commands.index(next_cmd) if next_cmd else None
        next_end=commands[next_index+1]['t'] if next_index is not None and next_index+1<len(commands) else replay['duration']
        move=persistent_motion(fc,next_goal['t'],next_end,xyz(next_goal['m']['pose']['position'])) if next_goal else None
        outgoing=[q for q in extras.get('/planning/bspline',[]) if next_cmd and q['m']['goal_stamp']==next_cmd['m']['header']['stamp'] and q['t']<next_end]
        r.update(next_bspline_payload_s=outgoing[0]['t'] if outgoing else None,
                 next_goal_published_s=next_goal['t'] if next_goal else None,
                 next_first_trajectory_ready_s=next_ready['t'] if next_ready else None,
                 next_fc_motion_start_s=move['start'] if move else None,
                 arrival_to_next_motion_s=move['start']-r['arrival_confirmed_s'] if move and r['arrival_confirmed_s'] is not None else None)
        if r['arrival_confirmed_s'] is not None and move:
            extra_hold=stationary_runs(fc,r['arrival_confirmed_s'],move['start'])
            for h in extra_hold:windows.append(dict(bag=bag,seq=r['decision_seq'],phase='after_arrival',**h))
        previous=next((a for a in records if a['decision_seq']==r['decision_seq']-1),None)
        if previous and next_goal:
            u=np.array([r['x']-previous['x'],r['y']-previous['y']]);v=xyz(next_goal['m']['pose']['position'])[:2]-np.array([r['x'],r['y']])
            r['turn_degrees']=float(np.degrees(np.arccos(np.clip(np.dot(u,v)/np.linalg.norm(u)/np.linalg.norm(v),-1,1)))) if np.linalg.norm(u)>.01 and np.linalg.norm(v)>.01 else None
        else:r['turn_degrees']=None
    ordinary=[r for r in records if r['role']=='high_survey' and r['terminal']=='arrived' and r['next_reason']=='high_view_full:SURVEY']
    high=[r for r in records if r['role']=='high_survey']
    metrics=['goal_to_first_trajectory_s','goal_to_arrival_s','planner_reached_to_arrival_s',
             'position_entry_to_final_qualified_s','final_qualified_to_arrival_s',
             'inside_radius_speed_block_s','confirmed_stationary_s']
    out=dict(bag=bag,source_revision=metadata['deployment']['source_revision'],high_agl=metadata['settings']['high_agl'],
             low_agl=metadata['settings']['low_agl'],records=records,stationary_windows=windows,
             metric_scope='high_survey only; missing terminal excluded, observed partial slow-speed accounting retained',
             summary={k:stats([r[k] for r in high]) for k in metrics},
             ordinary_switch_summary={k:stats([r[k] for r in ordinary]) for k in
                                    ('arrival_to_next_command_s','arrival_to_next_motion_s')})
    return out


def main():
    p=argparse.ArgumentParser(__doc__)
    p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[3])
    p.add_argument('--output-dir',type=Path,default=Path(__file__).resolve().parent)
    p.add_argument('--detail-dir',type=Path)
    a=p.parse_args();a.output_dir.mkdir(parents=True,exist_ok=True)
    results=[analyze(a.root,n) for n in (1,2)]
    fields=list(results[0]['records'][0])
    with (a.output_dir/'waypoint_dwell.csv').open('w',encoding='utf-8',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for result in results:writer.writerows(result['records'])
    dest=a.detail_dir or a.root/'试飞产物/waypoint_dwell_10111009'
    dest.mkdir(parents=True,exist_ok=True)
    (dest/'summary.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
    for result in results:
        print(json.dumps({k:v for k,v in result.items() if k not in ('records','stationary_windows')},ensure_ascii=False))
        for r in result['records']:
            print('ROW',json.dumps(r,ensure_ascii=False))

if __name__=='__main__':main()

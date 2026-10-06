#!/usr/bin/env python3
"""Read-only release/center evaluation. No ROS imports, launches or file writes.
Run under existing conda rl_drone. JSON goes to stdout; redirect only to a NEW
approved report path. Ground truth is evaluation-only.
"""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[3]
OLD = ROOT / 'docs/verification/seed38_resume_20261005'
CAM = ROOT / 'docs/verification/snake3_camera2m_20261005'
MODEL = Path('/home/xhj/liftrace/deliverables/liftrace_five_class_20260928_models/flight_5cls_20260928.pt')
B = Path('/home/xhj/liftrace-worktrees/r2026-board-vision-tests')
BASELINES = [
    (31, 'snake3', CAM, ROOT/'logs/snake3_camera2m_20261005_batch/matrix.json'),
    (38, 'snake3', CAM, ROOT/'logs/snake3_camera2m_20261005_batch/matrix.json'),
    (38, 'resume_on_fixed', OLD/'fixed_rerun', ROOT/'logs/seed38_resume_20261005_fixed_batch/matrix.json'),
]

def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))

def events(path):
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)

def source_stamp(data):
    value = data.get('header', {}).get('stamp', {})
    if 'stamp_ns' in value:
        return value['stamp_ns']/1e9
    if 'secs' in value:
        return value['secs']+value.get('nsecs', 0)/1e9
    return None

def identity(data):
    return tuple(data.get(k) for k in ('mission_id', 'decision_seq', 'attempt', 'payload_slot', 'execution_id'))

def recorder_offset(run):
    params = yaml.safe_load((run/'rosparams.yaml').read_text(encoding='utf-8'))
    found = []
    def visit(value, path):
        if isinstance(value, dict):
            if 'truth_world_offset' in value:
                found.append((path, value['truth_world_offset']))
            for key, child in value.items():
                visit(child, path+'/'+str(key))
    visit(params, '')
    offsets = {tuple(float(v) for v in offset) for _, offset in found}
    if len(offsets) != 1:
        raise ValueError('Missing/ambiguous recorded truth_world_offset: '+repr(found))
    return list(offsets.pop()), found

def truth_catalog(run, scene, offset):
    document = yaml.safe_load((run/'random_field_truth.yaml').read_text(encoding='utf-8'))
    targets = []
    for target in document['targets']:
        targets.append(dict(name=target['model'], class_name=target['class'],
            world_xy=[target['world_x'], target['world_y']], yaw=target['yaw'],
            half_side_m=.175 if target['class']=='red_cross' else .5,
            source='recorded random_field_truth.yaml (rounded to 0.1 mm)'))
    for include in ET.parse(scene/'field.world').getroot().find('world').findall('include'):
        if not include.findtext('uri', '').split('/')[-1].startswith('landing_h'):
            continue
        pose = include.find('pose')
        if pose is not None and pose.get('relative_to'):
            raise ValueError('Relative H pose requires explicit resolution')
        values = list(map(float, include.findtext('pose', '0 0 0 0 0 0').split()))
        targets.append(dict(name=include.findtext('name'), class_name='landing_pad',
            world_xy=values[:2], yaw=values[5], half_side_m=.4,
            source='frozen field.world H include pose'))
    for target in targets:
        target['csv_xy'] = [target['world_xy'][i]-offset[i] for i in range(2)]
    return targets

class Poses:
    def __init__(self, path):
        with path.open(newline='') as stream:
            rows = list(csv.DictReader(stream))
        self.rows = sorted(rows, key=lambda row: float(row['t']))
        self.times = [float(row['t']) for row in self.rows]
        if not self.rows or any(b <= a for a,b in zip(self.times, self.times[1:])):
            raise ValueError('Empty/non-monotonic truth pose samples')

    def nearest(self, time):
        i = bisect.bisect_left(self.times, time)
        candidates = [j for j in (i-1, i) if 0 <= j < len(self.rows)]
        j = min(candidates, key=lambda j: abs(self.times[j]-time))
        row = self.rows[j]
        return dict(t=self.times[j], age_s=abs(self.times[j]-time), xy=[float(row[k]) for k in ('x','y')])

    def interpolate(self, time, max_gap):
        i = bisect.bisect_left(self.times, time)
        near = self.nearest(time)
        if i < len(self.times) and abs(self.times[i]-time)<1e-9:
            return dict(status='exact', xy=near['xy'], nearest_age_s=near['age_s'], bracket_gap_s=0.)
        if i == 0 or i == len(self.times):
            return dict(status='outside_sample_range', xy=None, nearest_age_s=near['age_s'])
        span = self.times[i]-self.times[i-1]
        if span > max_gap:
            return dict(status='pose_gap_exceeds_limit', xy=None, bracket_gap_s=span, nearest_age_s=near['age_s'])
        fraction = (time-self.times[i-1])/span
        xy = [float(self.rows[i-1][k])+fraction*(float(self.rows[i][k])-float(self.rows[i-1][k])) for k in ('x','y')]
        return dict(status='interpolated', xy=xy, bracket_gap_s=span, nearest_age_s=near['age_s'])

def error_at(poses, time, targets, target_class, max_gap):
    pose = poses.interpolate(time, max_gap)
    if pose['xy'] is None:
        return pose
    xy = pose['xy']
    ranked = sorted(targets, key=lambda target: math.dist(xy, target['csv_xy']))
    nearest = ranked[0]
    own = next((target for target in ranked if target['class_name']==target_class), None)
    dx,dy = [xy[i]-nearest['csv_xy'][i] for i in range(2)]
    co,si = math.cos(nearest['yaw']), math.sin(nearest['yaw'])
    local = [co*dx+si*dy, -si*dx+co*dy]
    pose.update(nearest_instance=nearest['name'], nearest_class=nearest['class_name'],
        nearest_error_m=math.hypot(dx,dy), label_matches=nearest['class_name']==target_class,
        same_class_error_m=math.dist(xy, own['csv_xy']) if own else None,
        board_local_xy=local, inside_nominal_board=all(abs(v)<=nearest['half_side_m'] for v in local))
    return pose

def evaluate(run, scene, center_summary=None, max_gap=.25):
    run,scene = Path(run),Path(scene)
    gate = read_json(run/'gate_status.json')
    offset, provenance = recorder_offset(run)
    truth = truth_catalog(run, scene, offset)
    poses = Poses(run/'truth_pose.csv')
    raw = list(events(run/'key_events.jsonl'))
    starts, completed, rejects = {},{},[]
    duplicate_count = 0
    for event in raw:
        if event.get('kind')!='release':
            continue
        data = event['data']; key = identity(data)
        legacy = 'execution_state' not in data
        if legacy:
            # Pre-transaction schema has ACK only. Never invent a start/decision.
            key = ('legacy_run_local', data.get('execution_id'), data.get('payload_slot'), data.get('target_id'), data.get('target_class'))
            if data.get('success') and data.get('reason')=='raw_actuator_ack':
                store = completed
            else:
                rejects.append(dict(identity=key, reason=data.get('reason'), receipt_ros_s=event['ros_sec']))
                continue
        elif any(value is None for value in key):
            raise ValueError('Typed release lacks strict transaction identity')
        elif data.get('execution_state')==2 and not data.get('terminal'):
            store = starts
        elif data.get('execution_state')==3 and data.get('success') and data.get('terminal'):
            store = completed
        else:
            rejects.append(dict(identity=key, state=data.get('execution_state'), reason=data.get('reason'), receipt_ros_s=event['ros_sec']))
            continue
        if key in store:
            duplicate_count += 1
        else:
            store[key] = event
    drops = []
    for key, ack in completed.items():
        data = ack['data']; start = starts.get(key)
        row = dict(identity=key, class_name=data['target_class'], slot=data['payload_slot'],target_id=data['target_id'])
        for name,event in [('raw_call_started',start), ('completed',ack)]:
            if event is None:
                row[name] = dict(status='missing_event'); continue
            t = source_stamp(event['data'])
            if t is None or t<=0:
                row[name] = dict(status='missing_source_stamp', receipt_ros_s=event['ros_sec']); continue
            row[name] = dict(source_ros_s=t, receipt_ros_s=event['ros_sec'],
                receipt_minus_source_s=event['ros_sec']-t,
                pose=error_at(poses,t,truth,data['target_class'],max_gap))
        t0,t1 = source_stamp(start['data']) if start else None, source_stamp(data)
        row['raw_call_interval_s'] = t1-t0 if t0 is not None and t1 is not None else None
        # Exactly preserve the old receipt-time/nearest-sample metric separately.
        near = poses.nearest(ack['ros_sec'])
        nearest = min(truth, key=lambda target: math.dist(near['xy'],target['csv_xy']))
        row['legacy_ack_receipt_nearest'] = dict(**near, nearest_class=nearest['class_name'],
            error_m=math.dist(near['xy'],nearest['csv_xy']), usable_age_le_025=near['age_s']<=.25)
        if t0 is not None and t1 is not None and t1>=t0:
            points = [error_at(poses,t,truth,data['target_class'],max_gap) for t in [t0,t1]]
            points += [error_at(poses,t,truth,data['target_class'],max_gap) for t in poses.times if t0<=t<=t1]
            distances = [point['same_class_error_m'] for point in points if point.get('xy') is not None and point.get('same_class_error_m') is not None]
            row['interval_sampled_same_class_max_m'] = max(distances,default=None)
            row['interval_note'] = 'Sampled center envelope, not continuous bound or physical detachment/impact time'
        drops.append(row)
    contacts = read_json(run/'gazebo_contact_status.json')
    centers = {}
    if center_summary is not None:
        summary = read_json(center_summary)
        if Path(summary['run']).resolve()!=run.resolve():
            raise ValueError('Center summary belongs to another run')
        centers = dict(path=str(center_summary), statistics=summary['statistics'],
            counts=summary['counts'], transform=summary['world_to_eval_xyz'])
        if summary.get('world_to_eval_yaw') != 0 or any(abs(summary['world_to_eval_xyz'][i]+offset[i])>1e-6 for i in range(3)):
            raise ValueError('Visual/body metrics need an explicit common frame')
        sample_path = Path(center_summary).parent/'center_samples.csv'
        with sample_path.open(newline='') as stream:
            samples = list(csv.DictReader(stream))
        for drop in drops:
            for endpoint in ('raw_call_started','completed'):
                event = drop[endpoint]; t = event.get('source_ros_s')
                if t is None:
                    continue
                selected = []
                for stream_name in ('mapped','targets','selected'):
                    valid = [row for row in samples if row['stream']==stream_name and
                        row['class_name']==drop['class_name'] and not row['exclusion'] and
                        row['duplicate']!='True' and row['in_mission']=='True' and
                        row['fresh']=='True' and float(row['receipt_ros_s'])<=t and
                        0 <= t-float(row['source_ros_s']) <= .5 and
                        (stream_name=='mapped' or row['id']==str(completed[tuple(drop['identity'])]['data']['target_id']))]
                    if not valid:
                        selected.append(dict(stream=stream_name,status='no_fresh_identity_matched_sample'));continue
                    row = max(valid,key=lambda row:(float(row['source_ros_s']),float(row['receipt_ros_s'])))
                    selected.append(dict(stream=stream_name,source_ros_s=float(row['source_ros_s']),
                        age_at_release_s=t-float(row['source_ros_s']),center_source=row['center_source'],
                        center_refined=row['center_refined'],center_xy=[float(row['eval_x']),float(row['eval_y'])],
                        center_truth_error_m=float(row['xy_error_m']), nearest_any_class=row['nearest_any_class'],
                        association_note='Mapped rows use class association only; targets/selected also match target_id'))
                event['visual_centers_at_event'] = selected
    return dict(run=str(run),scene=str(scene), raw_gate=gate.get('status'), reason=gate.get('reason'),
        failed_checks=gate.get('failed_checks'), truth_world_offset=offset, offset_provenance=provenance,
        pose_gap_limit_s=max_gap, truth=truth, completed_transactions=len(completed),
        started_without_completed=[key for key in starts if key not in completed],
        duplicate_receipts=duplicate_count, other_release_receipts=rejects, drops=drops,
        actual_collision_count=contacts.get('actual_collision_count'),
        support_events=contacts.get('support_events'), centers=centers,
        note='Body/model-origin XY center at mock execution/ACK. No physical detachment or parcel impact measurement; truth Pose ModelStates is stamped at recorder receipt.')

def inventory():
    result = []
    for seed,variant,report,matrix_path in BASELINES:
        matrix = read_json(matrix_path)
        row = next(row for row in matrix['results'] if row['seed']==seed and row['variant']==variant)
        if 'scene' in row:
            scene = Path(row['scene'])
        else:
            scenes = read_json(report/'scenes.json')
            scene = ROOT/next(s['scene'] for s in scenes if s['seed']==seed and s['variant']==variant)
        run = Path(row['run']); centers = report/f'{seed}_{variant}_centers'
        paths = [run/name for name in ('manifest.yaml','rosparams.yaml','truth_pose.csv','lio_pose.csv',
            'mavros_pose.csv','planner_setpoint.csv','mavros_setpoint.csv','actual_camera_info.json',
            'vision_metrics.bag','center_export.jsonl','downward.mp4','downward.csv','overview.mp4',
            'high_view_full_events.jsonl','key_events.jsonl','random_field_truth.yaml','gate_status.json')]
        paths += [report/'metrics.json',report/'release_truth.json',centers/'center_summary.json',
            centers/'center_samples.csv',centers/'center_errors.png',centers/'stages/phase_statistics.csv',
            centers/'stages/PHASE_REPORT.md']
        result.append(dict(seed=seed,variant=variant,source=matrix['source'],matrix=str(matrix_path),
            command=row['command'],scene=str(scene), artifacts=[dict(path=str(p),exists=p.exists(),bytes=p.stat().st_size if p.is_file() else None) for p in paths],
            evaluation=evaluate(run,scene,centers/'center_summary.json')))
    return result

def attach_exact_offsets(result, path):
    records=[row for row in events(path) if row.get('kind')=='drop_offset']
    truth=result['truth']
    for drop in result['drops']:
        for endpoint in ('raw_call_started','completed'):
            event=drop[endpoint];t=event.get('source_ros_s')
            if t is None:
                continue
            matches=[]
            for record in records:
                msg=record['m'];source=source_stamp(msg)
                if (not msg.get('map_valid') or msg.get('target_id')!=drop['target_id'] or
                    source is None or record['t']>t or not 0<=t-source<=.5 or
                    msg.get('map_frame')!='camera_init'):
                    continue
                point=msg['map_point'];xy=[point['x'],point['y']]
                if not all(math.isfinite(v) for v in (*xy,point['z'])) or abs(point['z']+.22)>1e-5:
                    continue
                matches.append((source,record['t'],msg))
            if not matches:
                event['latest_exact_offset']=dict(status='no_fresh_matching_exact_sample');continue
            source,receipt,msg=max(matches,key=lambda row:row[:2])
            xy=[msg['map_point']['x'],msg['map_point']['y']]
            # These candidate files use explicit world-to-camera_init offset.
            nearest=min(truth,key=lambda target:math.dist(xy,target['csv_xy']))
            own=min((target for target in truth if target['class_name']==drop['class_name']),
                    key=lambda target:math.dist(xy,target['csv_xy']))
            event['latest_exact_offset']=dict(source_ros_s=source,receipt_ros_s=receipt,
                age_at_event_s=t-source,goal_xy=xy,same_class_truth_error_m=math.dist(xy,own['csv_xy']),
                nearest_class=nearest['class_name'],nearest_distance_m=math.dist(xy,nearest['csv_xy']),
                current_alignment_error_m=msg['alignment_error_m'],tolerance_m=msg['alignment_tolerance_m'],
                body_to_goal_m=math.dist(event['pose']['xy'],xy) if event['pose'].get('xy') is not None else None,
                note='Post-slot-compensation FC goal; with verified sim zero mode this is instantaneous observed target XY. No parcel impact.')
    return result

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='mode',required=True)
    inv = sub.add_parser('inventory');inv.add_argument('--summary',action='store_true')
    sub.add_parser('verify')
    one = sub.add_parser('analyze')
    one.add_argument('--run',type=Path,required=True);one.add_argument('--scene',type=Path,required=True)
    one.add_argument('--center-summary',type=Path);one.add_argument('--max-pose-gap',type=float,default=.25)
    one.add_argument('--exact-offset-data',type=Path)
    matrix = sub.add_parser('matrix-row')
    matrix.add_argument('--run',type=Path,required=True);matrix.add_argument('--scene',type=Path,required=True)
    matrix.add_argument('--seed',type=int,choices=(31,38),required=True);matrix.add_argument('--source',required=True)
    args = parser.parse_args()
    if args.mode=='verify':
        reports = inventory()
        checked = 0
        for report in reports:
            assert not [p for p in report['artifacts'] if not p['exists']], 'Missing baseline artifact'
            ev = report['evaluation']
            assert ev['completed_transactions']==3 and ev['actual_collision_count']==0
            previous = read_json(next(p['path'] for p in report['artifacts'] if p['path'].endswith('/release_truth.json')))
            prior = next(p for p in previous if p['seed']==report['seed'] and p['variant']==report['variant'])
            for drop in ev['drops']:
                original = next(p for p in prior['drops'] if p['slot']==drop['slot'])
                assert abs(original['nearest_distance_m']-drop['legacy_ack_receipt_nearest']['error_m'])<1e-9
                assert drop['completed']['pose']['inside_nominal_board'] and drop['completed']['pose']['label_matches']
                checked += 1
        output = dict(status='PASS',baselines=len(reports),ack_metrics_reproduced=checked,missing_artifacts=0,simulation_started=False)
    elif args.mode=='inventory':
        output = inventory()
        if args.summary:
            output = [dict(seed=r['seed'],variant=r['variant'],source=r['source'],missing=[p['path'] for p in r['artifacts'] if not p['exists']],drops=[dict(class_name=d['class_name'],slot=d['slot'],legacy_ack_error_cm=100*d['legacy_ack_receipt_nearest']['error_m'],source_interpolated_ack_error_cm=100*d['completed']['pose']['same_class_error_m'],start_recorded=d['raw_call_started'].get('status')!='missing_event') for d in r['evaluation']['drops']],collision_count=r['evaluation']['actual_collision_count'],raw_gate=r['evaluation']['raw_gate']) for r in output]
    else:
        if args.mode=='analyze':
            output = evaluate(args.run,args.scene,args.center_summary,args.max_pose_gap)
            if args.exact_offset_data:
                output = attach_exact_offsets(output,args.exact_offset_data)
        else:
            if not (args.run/'gate_status.json').is_file() or not (args.scene/'field.world').is_file():
                raise ValueError('Closed run and existing frozen scene required')
            output = dict(source=args.source,results=[dict(seed=args.seed,variant='precision',run=str(args.run.resolve()),scene=str(args.scene.resolve()))])
    print(json.dumps(output,indent=2,ensure_ascii=True,allow_nan=False))

if __name__=='__main__':
    main()
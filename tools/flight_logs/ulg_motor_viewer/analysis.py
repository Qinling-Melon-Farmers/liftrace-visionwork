"""Offline source-aware ULog exports. Motor control is a command, not RPM/current."""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from pyulog import ULog

TOPICS = (
    'actuator_motors', 'actuator_outputs', 'actuator_armed', 'vehicle_status',
    'vehicle_land_detected', 'vehicle_attitude', 'vehicle_attitude_setpoint',
    'vehicle_angular_velocity', 'vehicle_rates_setpoint', 'input_rc', 'rc_channels',
    'manual_control_setpoint', 'manual_control_switches', 'battery_status',
    'vehicle_local_position', 'vehicle_local_position_setpoint',
    'vehicle_visual_odometry', 'estimator_selector_status', 'estimator_status_flags',
    'estimator_event_flags', 'estimator_status', 'estimator_innovations',
    'estimator_innovation_test_ratios', 'estimator_aid_src_ev_hgt',
    'estimator_aid_src_ev_pos', 'estimator_aid_src_ev_vel', 'estimator_aid_src_ev_yaw',
    'distance_sensor', 'esc_status',
)
MODE_NAMES = {0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 3: 'AUTO_MISSION',
              4: 'AUTO_LOITER', 5: 'AUTO_RTL', 10: 'ACRO', 12: 'DESCEND',
              13: 'TERMINATION', 14: 'OFFBOARD', 15: 'STAB', 17: 'AUTO_TAKEOFF',
              18: 'AUTO_LAND', 20: 'AUTO_PRECLAND'}
RESET_FIELDS = {'z_reset_counter': 'delta_z', 'heading_reset_counter': 'delta_heading',
                'quat_reset_counter': None, 'reset_counter': None}
WINDOWS = ('ground', 'motor_airborne', 'transition_guard', 'stopped_or_blocked',
           'suspected_impact', 'logging_gap', 'unknown')


@dataclass
class Options:
    state_age_sec: float = 2.0
    gap_sec: float = 0.3
    guard_sec: float = 3.0
    impact_accel_mps2: float = 15.0
    impact_rate_dps: float = 300.0
    motor_count: int = 4


@dataclass
class Stream:
    name: str
    instance: int
    time: np.ndarray
    data: dict
    rewinds: int = 0

    @property
    def key(self):
        return f'{self.name}[{self.instance}]'


def normalize(name, instance, data, start_us):
    ts = np.asarray(data.get('timestamp', []), dtype=float)
    rewinds = int(np.count_nonzero(np.diff(ts) < 0))
    order = np.flatnonzero(np.isfinite(ts))
    order = order[np.argsort(ts[order], kind='stable')]
    if len(order):
        order = order[np.r_[ts[order][1:] != ts[order][:-1], True]]
    return Stream(name, instance, (ts[order] - start_us) / 1e6,
                  {k: np.asarray(v)[order] for k, v in data.items()}, rewinds)


def at(stream, field, times, max_age=math.inf):
    """Causal previous publication; no future interpolation or pre-first backfill."""
    result = np.full(len(times), np.nan)
    if stream is None or field not in stream.data or not len(stream.time):
        return result
    indices = np.searchsorted(stream.time, times, side='right') - 1
    valid = indices >= 0
    indices = np.maximum(indices, 0)
    valid &= (times - stream.time[indices] <= max_age)
    result[valid] = np.asarray(stream.data[field], dtype=float)[indices[valid]]
    return result


def finite_stats(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return {'samples': len(values), 'min': float(values.min()), 'mean': float(values.mean()),
            'p50': float(np.median(values)), 'p95': float(np.percentile(values, 95)),
            'max': float(values.max())}


def euler(data, prefix='q'):
    keys = [f'{prefix}[{i}]' for i in range(4)]
    if not all(k in data for k in keys):
        return None
    q = np.stack([data[k] for k in keys], axis=1).astype(float)
    norm = np.linalg.norm(q, axis=1)
    valid = np.isfinite(norm) & (norm > 1e-6)
    q[~valid] = np.nan
    q[valid] /= norm[valid, None]
    w, x, y, z = q.T
    return np.rad2deg(np.stack((np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                               np.arcsin(np.clip(2*(w*y-z*x), -1, 1)),
                               np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))), axis=1))


def event_fields(stream):
    fields = {
        'actuator_armed': ('armed', 'lockdown', 'manual_lockdown', 'force_failsafe'),
        'vehicle_status': ('nav_state', 'arming_state', 'failsafe'),
        'vehicle_land_detected': ('landed', 'ground_contact', 'maybe_landed', 'freefall'),
        'manual_control_switches': ('kill_switch', 'arm_switch', 'offboard_switch', 'mode_slot'),
        'input_rc': ('rc_lost', 'rc_failsafe'),
        'estimator_selector_status': ('primary_instance', 'instance_changed_count'),
        'vehicle_local_position': ('z_reset_counter', 'heading_reset_counter', 'z_valid',
                                   'xy_valid', 'dead_reckoning'),
        'vehicle_attitude': ('quat_reset_counter',),
        'vehicle_visual_odometry': ('reset_counter', 'quality', 'pose_frame'),
        'estimator_status': ('control_mode_flags', 'filter_fault_flags', 'timeout_flags'),
    }.get(stream.name, ())
    if stream.name == 'estimator_status_flags':
        fields = tuple(k for k in stream.data if k.startswith(('cs_ev_', 'reject_', 'fs_bad_')))
    elif stream.name == 'estimator_event_flags':
        fields = tuple(k for k in stream.data if k.startswith(('reset_', 'starting_vision_'))
                       or k in ('vision_data_stopped', 'height_sensor_timeout',
                                'information_event_changes', 'warning_event_changes'))
    elif stream.name.startswith('estimator_aid_src_ev_'):
        fields = ('fused', 'innovation_rejected')
    return [k for k in fields if k in stream.data]


def scalar(value):
    value = value.item() if hasattr(value, 'item') else value
    return None if isinstance(value, float) and not math.isfinite(value) else value


class Analysis:
    def __init__(self, path, ulog, options=None):
        self.path = Path(path)
        self.options = options or Options()
        self.start_us = int(ulog.start_timestamp)
        self.streams = {}
        for dataset in ulog.data_list:
            if dataset.name in TOPICS:
                stream = normalize(dataset.name, dataset.multi_id, dataset.data, self.start_us)
                if len(stream.time):
                    self.streams[(stream.name, stream.instance)] = stream
        self.dropouts = [{'time_s': (d.timestamp-self.start_us)/1e6,
                          'duration_s': d.duration/1000} for d in ulog.dropouts]
        self.end = max((s.time[-1] for s in self.streams.values()), default=0.0)
        self.metadata = {k: scalar(v) for k, v in ulog.msg_info_dict.items()
                         if k in ('sys_name', 'ver_hw', 'ver_sw', 'ver_sw_branch')}
        self.parameters = {k: scalar(v) for k, v in ulog.initial_parameters.items()
                           if k.startswith(('EKF2_EV', 'RC_MAP_', 'PWM_', 'CA_ROTOR'))
                           or k in ('EKF2_HGT_REF', 'EKF2_HGT_MODE', 'EKF2_AID_MASK',
                                    'SDLOG_PROFILE', 'SDLOG_MODE')}
        self.events = self.make_events()
        self.exclusions = self.make_exclusions()

    def get(self, name, instance=0):
        return self.streams.get((name, instance))

    def all(self, name):
        return [s for (n, _), s in sorted(self.streams.items()) if n == name]

    def make_events(self):
        result = []
        selector = self.get('estimator_selector_status')
        for stream in self.streams.values():
            for field in event_fields(stream):
                values = stream.data[field]
                finite = np.isfinite(values)
                changes = np.flatnonzero(finite[1:] & finite[:-1] &
                                         (values[1:] != values[:-1])) + 1
                for i in np.r_[0, changes]:
                    if not finite[i]:
                        continue
                    t = float(stream.time[i])
                    event = {'time_s': t, 'boot_sec': t+self.start_us/1e6,
                             'topic': stream.name, 'instance': stream.instance, 'field': field,
                             'kind': 'initial_observation' if i == 0 else 'change',
                             'before': scalar(values[i-1]) if i else None,
                             'after': scalar(values[i]), 'delta': None, 'counter_step_mod256': None,
                             'selected_estimator': scalar(at(selector, 'primary_instance', [t],
                                                             self.options.state_age_sec)[0])}
                    if field in RESET_FIELDS and i:
                        event['kind'] = 'reset_counter_change'
                        event['counter_step_mod256'] = (int(values[i])-int(values[i-1])) % 256
                        delta_field = RESET_FIELDS[field]
                        if delta_field in stream.data:
                            event['delta'] = scalar(stream.data[delta_field][i])
                    result.append(event)
        ev = self.get('vehicle_visual_odometry')
        if ev:
            for i in np.flatnonzero(np.diff(ev.time) > self.options.gap_sec) + 1:
                result.append({'time_s': float(ev.time[i]), 'boot_sec': float(ev.time[i])+self.start_us/1e6,
                               'topic': ev.name, 'instance': ev.instance, 'field': 'logged_gap_s',
                               'kind': 'logged_gap', 'before': float(ev.time[i-1]),
                               'after': float(ev.time[i]-ev.time[i-1]), 'delta': None,
                               'counter_step_mod256': None, 'selected_estimator': None})
        return sorted(result, key=lambda e: (e['time_s'], e['topic'], e['instance'], e['field']))

    def make_exclusions(self):
        """Exclude guard bands around contact/stops and observed motion spikes."""
        guard = self.options.guard_sec
        exclusions = []
        for event in self.events:
            if event['kind'] == 'initial_observation':
                continue
            if ((event['topic'] == 'vehicle_land_detected' and event['field'] in
                 ('landed', 'ground_contact', 'maybe_landed'))
                or event['topic'] == 'actuator_armed'
                or (event['topic'] == 'manual_control_switches' and event['field'] == 'kill_switch')):
                exclusions.append((event['time_s']-guard, event['time_s']+guard, 'transition_guard'))
        for name, fields, threshold in (
            ('vehicle_local_position', ('ax', 'ay', 'az'), self.options.impact_accel_mps2),
            ('vehicle_angular_velocity', ('xyz[0]', 'xyz[1]', 'xyz[2]'),
             np.deg2rad(self.options.impact_rate_dps)),
        ):
            stream = self.get(name)
            if stream and all(k in stream.data for k in fields):
                magnitude = np.linalg.norm(np.stack([stream.data[k] for k in fields]), axis=0)
                for time in stream.time[np.isfinite(magnitude) & (magnitude > threshold)]:
                    exclusions.append((float(time)-guard, float(time)+guard, 'suspected_impact'))
        for dropout in self.dropouts:
            exclusions.append((dropout['time_s']-guard,
                               dropout['time_s']+dropout['duration_s']+guard, 'logging_gap'))
        for stream in self.all('actuator_motors'):
            for i in np.flatnonzero(np.diff(stream.time) > self.options.gap_sec):
                exclusions.append((float(stream.time[i]), float(stream.time[i+1]), 'logging_gap'))
        return exclusions

    def context(self, times):
        age = self.options.state_age_sec
        mapping = {'armed': ('actuator_armed', 'armed'),
                   'lockdown': ('actuator_armed', 'lockdown'),
                   'manual_lockdown': ('actuator_armed', 'manual_lockdown'),
                   'force_failsafe': ('actuator_armed', 'force_failsafe'),
                   'landed': ('vehicle_land_detected', 'landed'),
                   'ground_contact': ('vehicle_land_detected', 'ground_contact'),
                   'maybe_landed': ('vehicle_land_detected', 'maybe_landed'),
                   'kill_switch': ('manual_control_switches', 'kill_switch'),
                   'nav_state': ('vehicle_status', 'nav_state')}
        values = {key: at(self.get(topic), field, times, age)
                  for key, (topic, field) in mapping.items()}
        labels = np.full(len(times), 'unknown', dtype=object)
        blocked = ((values['armed'] == 0) | (values['lockdown'] == 1)
                   | (values['manual_lockdown'] == 1) | (values['force_failsafe'] == 1)
                   | (values['kill_switch'] == 2))
        ground = (values['landed'] == 1) | (values['ground_contact'] == 1)
        airborne = ((values['armed'] == 1) & (values['landed'] == 0)
                    & (values['ground_contact'] == 0) & ~blocked)
        # Safety flags must be recorded and clear; kill switch is optional but,
        # if logged, 0/unconfigured is not treated as a positive kill assertion.
        airborne &= ((values['lockdown'] == 0) & (values['manual_lockdown'] == 0)
                     & (values['force_failsafe'] == 0) & (values['maybe_landed'] != 1))
        labels[airborne] = 'motor_airborne'
        labels[blocked] = 'stopped_or_blocked'
        labels[ground] = 'ground'
        for kind in ('transition_guard', 'suspected_impact', 'logging_gap'):
            for begin, end, reason in self.exclusions:
                if reason == kind:
                    labels[(times >= begin) & (times <= end)] = kind
        return values, labels

    def motor_streams(self):
        return self.all('actuator_motors')

    def motor_fields(self, stream):
        return [f'control[{i}]' for i in range(self.options.motor_count)
                if f'control[{i}]' in stream.data
                and np.any(np.isfinite(stream.data[f'control[{i}]']))]

    def motor_stats(self):
        rows = []
        for stream in self.motor_streams():
            _, labels = self.context(stream.time)
            dt = np.diff(stream.time)
            labels = labels.copy()
            for i in np.flatnonzero(dt > self.options.gap_sec):
                labels[i:i+2] = 'logging_gap'
            fields = self.motor_fields(stream)
            for window in WINDOWS:
                mask = labels == window
                for field in fields:
                    values = np.asarray(stream.data[field], dtype=float)
                    finite = np.isfinite(values)
                    valid = mask & finite
                    data = finite_stats(values[valid])
                    if not data:
                        continue
                    interval = (valid[:-1] & valid[1:] & (dt > 0)
                                & (dt <= self.options.gap_sec))
                    rows.append({'topic': stream.key, 'window': window, 'motor': field,
                                 **data, 'sample_pct_ge_095': float(np.mean(values[valid] >= .95)*100),
                                 'observed_interval_s': float(dt[interval].sum()),
                                 'held_command_s_ge_0999': float(dt[interval & (values[:-1] >= .999)].sum())})
            # Compare motors at identical eligible timestamps only; never compare
            # separate per-motor means with mismatched NaN populations.
            if len(fields) >= 2:
                matrix = np.stack([stream.data[k] for k in fields], axis=1).astype(float)
                valid = (labels == 'motor_airborne') & np.all(np.isfinite(matrix), axis=1)
                spread = np.ptp(matrix[valid], axis=1)
                data = finite_stats(spread)
                if data:
                    rows.append({'topic': stream.key, 'window': 'motor_airborne',
                                 'motor': 'simultaneous_max_minus_min', **data})
        return rows

    def selected(self, topic, field, times):
        selector = self.get('estimator_selector_status')
        primary = at(selector, 'primary_instance', times, self.options.state_age_sec)
        result = np.full(len(times), np.nan)
        # No selector means no claim that instance 0 was the active estimator.
        for stream in self.all(topic):
            mask = primary == stream.instance
            result[mask] = at(stream, field, np.asarray(times)[mask], self.options.state_age_sec)
        return result

    def report(self):
        missing = [name for name in TOPICS if not self.all(name)]
        warnings = []
        if not self.motor_streams():
            warnings.append('No actuator_motors: raw actuator_outputs may be exported, but no normalized motor comparison.')
        for name in ('actuator_armed', 'vehicle_land_detected'):
            if not self.get(name):
                warnings.append(f'{name} absent: airborne motor comparison unavailable; phases remain unknown.')
        if not self.get('vehicle_visual_odometry'):
            warnings.append('EV input topic absent: input freshness/latency unknown; estimator EV flags do not measure input rate.')
        if not self.all('estimator_aid_src_ev_hgt'):
            warnings.append('EV aid-source topic absent: use recorded flags/legacy innovations; absence does not prove fusion disabled.')
        if not self.get('estimator_selector_status'):
            warnings.append('Estimator selector absent: instances are kept separate; selected-estimator timeline unknown.')
        if self.dropouts:
            warnings.append('Logger dropouts present; logged gaps do not establish controller input interruptions.')
        if not self.all('esc_status'):
            warnings.append('ESC feedback absent: motor RPM/current unavailable. Battery current, if logged, is pack current.')
        batteries = self.all('battery_status')
        usable_voltage = False
        for stream in batteries:
            for field in ('voltage_v', 'voltage_filtered_v'):
                if field in stream.data:
                    values = np.asarray(stream.data[field], float)
                    valid = np.isfinite(values) & (values > 0)
                    if 'connected' in stream.data:
                        valid &= np.asarray(stream.data['connected']) == 1
                    usable_voltage |= bool(np.any(valid))
        if not usable_voltage:
            warnings.append('No positive connected battery voltage samples; voltage observation unavailable.')
        for topic, fields in (
            ('actuator_armed', ('armed', 'lockdown', 'manual_lockdown', 'force_failsafe')),
            ('vehicle_land_detected', ('landed', 'ground_contact')),
        ):
            stream = self.get(topic)
            if stream:
                absent = [f for f in fields if f not in stream.data]
                if absent:
                    warnings.append(f'{topic} missing {absent}: airborne command comparison remains unknown.')
        return {'source_file': str(self.path.resolve()), 'time_basis': 'seconds relative to ULog start; boot_sec=start_us/1e6+time_s',
                'start_us': self.start_us, 'selected_topic_span_s': float(self.end),
                'metadata': self.metadata, 'initial_parameters': self.parameters,
                'options': vars(self.options), 'missing_topics': missing,
                'inventory': {s.key: {'samples': len(s.time), 'first_s': float(s.time[0]),
                                      'last_s': float(s.time[-1]), 'timestamp_rewinds': s.rewinds,
                                      'fields': sorted(s.data)} for s in self.streams.values()},
                'logger_dropouts': self.dropouts, 'motor_statistics': self.motor_stats(),
                'reset_events': [e for e in self.events if e['kind'] == 'reset_counter_change'],
                'warnings': warnings,
                'limitations': ['motor_airborne is a logged state/command window, not proof of spinning motors or stable hover.',
                                'ground/contact are flight-controller estimates; suspected_impact is a configurable spike exclusion, not collision diagnosis.',
                                'z/up and dist_bottom are estimates, not measured actual AGL; filename is not altitude truth.',
                                'Reset counter changes mark first logged observation, not cause; deltas describe latest reset only.',
                                'Windows exclude state transitions, blocked commands, spikes and logger gaps; remaining samples can still include maneuvering.',
                                'No physical motor mapping or causal diagnosis is inferred.']}


def read_log(path, options=None):
    path = Path(path)
    with path.open('rb') as stream:
        if not stream.read(16).startswith(b'ULog\x01\x12\x35'):
            raise ValueError('Invalid ULog header')
    return Analysis(path, ULog(str(path), message_name_filter_list=TOPICS), options)


def write_csv(path, rows, fields=None):
    rows = list(rows)
    if fields is None:
        fields = list(dict.fromkeys(k for row in rows for k in row))
    with Path(path).open('w', encoding='utf-8-sig', newline='') as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: scalar(v) for k, v in row.items()})


def export(analysis, destination, plots=True):
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=False)
    report = analysis.report()
    (destination / 'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2,
                                                       allow_nan=False), encoding='utf-8')
    write_csv(destination / 'events.csv', analysis.events,
              ['time_s', 'boot_sec', 'topic', 'instance', 'field', 'kind', 'before', 'after',
               'delta', 'counter_step_mod256', 'selected_estimator'])
    write_csv(destination / 'motor_statistics.csv', report['motor_statistics'],
              ['topic', 'window', 'motor', 'samples', 'min', 'mean', 'p50', 'p95', 'max',
               'sample_pct_ge_095', 'observed_interval_s', 'held_command_s_ge_0999'])
    for stream in analysis.streams.values():
        fields = sorted(stream.data)
        derived = {}
        if stream.name == 'vehicle_attitude':
            angles = euler(stream.data)
            if angles is not None:
                derived.update({f'{axis}_deg': angles[:, i] for i, axis in enumerate(('roll', 'pitch', 'yaw'))})
        elif stream.name == 'vehicle_local_position' and 'z' in stream.data:
            derived['estimated_local_up_m'] = -np.asarray(stream.data['z'], dtype=float)
        elif stream.name == 'vehicle_visual_odometry' and 'timestamp_sample' in stream.data:
            sample = np.asarray(stream.data['timestamp_sample'], dtype=float)
            derived['publish_minus_sample_ms'] = np.where(sample > 0,
                    (stream.data['timestamp'].astype(float)-sample)/1000, np.nan)
        _, labels = analysis.context(stream.time)
        if stream.name == 'actuator_motors':
            for i in np.flatnonzero(np.diff(stream.time) > analysis.options.gap_sec):
                labels[i:i+2] = 'logging_gap'
        write_csv(destination / f'{stream.name}_instance{stream.instance}.csv',
                  ({'time_s': t, 'window': labels[i],
                    **{k: stream.data[k][i] for k in fields},
                    **{k: v[i] for k, v in derived.items()}} for i, t in enumerate(stream.time)),
                  ['time_s', 'window'] + fields + list(derived))
    grid = np.unique(np.concatenate([s.time for s in analysis.streams.values()])) if analysis.streams else np.array([])
    _, labels = analysis.context(grid)
    if len(grid):
        changed = np.r_[True, labels[1:] != labels[:-1]]
        bounds = np.flatnonzero(changed)
        write_csv(destination / 'windows.csv',
                  ({'begin_s': grid[i], 'end_s': grid[bounds[j+1]] if j+1 < len(bounds) else grid[-1],
                    'window': labels[i]} for j, i in enumerate(bounds)))
        context, _ = analysis.context(grid)
        selector = analysis.get('estimator_selector_status')
        health = {'primary_instance': at(selector, 'primary_instance', grid, analysis.options.state_age_sec)}
        for field in ('cs_ev_pos', 'cs_ev_vel', 'cs_ev_hgt', 'cs_ev_yaw', 'cs_ev_yaw_fault',
                      'reject_ver_pos', 'reject_yaw'):
            health[field] = analysis.selected('estimator_status_flags', field, grid)
        ev = analysis.get('vehicle_visual_odometry')
        published = at(ev, 'timestamp', grid)
        sample = at(ev, 'timestamp_sample', grid)
        health['ev_last_logged_publish_age_ms'] = (grid*1e6+analysis.start_us-published)/1000
        health['ev_last_logged_sample_age_ms'] = np.where(sample > 0,
                (grid*1e6+analysis.start_us-sample)/1000, np.nan)
        write_csv(destination / 'health_timeline.csv',
                  ({'time_s': t, 'window': labels[i],
                    **{k: v[i] for k, v in context.items()},
                    **{k: v[i] for k, v in health.items()}} for i, t in enumerate(grid)),
                  ['time_s', 'window'] + list(context) + list(health))
    lines = [analysis.path.name, 'Motor command units: normalized allocation command; never RPM or current.',
             'Comparison uses motor_airborne only. Ground/stops/transition/spike/gap samples are separate.',
             'Height curves are ESTIMATES in the local frame, not actual AGL.',
             f'Recorded reset counter changes: {len(report["reset_events"])}']
    for row in report['motor_statistics']:
        if row['window'] == 'motor_airborne':
            lines.append(f'{row["topic"]} {row["motor"]}: n={row["samples"]}, mean={row["mean"]:.4f}, max={row["max"]:.4f}')
    if not any(r['window'] == 'motor_airborne' for r in report['motor_statistics']):
        lines.append('No eligible airborne motor samples; comparison UNKNOWN.')
    lines.extend('UNKNOWN/LIMIT: ' + w for w in report['warnings'])
    (destination / 'summary.txt').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    if plots:
        from plotting import overview
        overview(analysis, destination / 'overview.png')
    return report


def discover(paths, recursive=False):
    files = set()
    for value in paths:
        path = Path(value).expanduser()
        if path.is_dir():
            files.update(p.resolve() for p in (path.rglob('*') if recursive else path.glob('*'))
                         if p.is_file() and p.suffix.lower() == '.ulg')
        else:
            files.add(path.resolve())
    return sorted(files, key=str)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--recursive', action='store_true')
    parser.add_argument('--no-plots', action='store_true')
    parser.add_argument('--headless', action='store_true', help='App entry supports headless export without Tk')
    for name, default in vars(Options()).items():
        parser.add_argument('--'+name.replace('_', '-'), type=int if name == 'motor_count' else float,
                            default=default)
    args = parser.parse_args(argv)
    options = Options(**{k: getattr(args, k) for k in vars(Options())})
    if any(not math.isfinite(v) or v <= 0 for k, v in vars(options).items() if k != 'guard_sec') \
            or not math.isfinite(options.guard_sec) or options.guard_sec < 0 or options.motor_count > 12:
        parser.error('Thresholds must be finite and positive; guard >= 0; motor-count <= 12')
    files = discover(args.inputs, args.recursive)
    if not files:
        parser.error('No ULog files found')
    # Never overwrite an earlier batch. Same-name logs get deterministic numeric prefixes.
    args.output.mkdir(parents=True, exist_ok=False)
    failures = []
    for index, path in enumerate(files, 1):
        stem = re.sub(r'[^\w.-]', '_', path.stem)[:100]
        destination = args.output / f'{index:03d}_{stem}'
        try:
            report = export(read_log(path, options), destination, not args.no_plots)
            print(f'OK {path.name}: {destination}; reset changes={len(report["reset_events"])}')
        except Exception as exc:
            failures.append({'source_file': str(path), 'error': f'{type(exc).__name__}: {exc}'})
            print(f'FAIL {path.name}: {exc}')
    write_csv(args.output / 'batch_errors.csv', failures, ['source_file', 'error'])
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())

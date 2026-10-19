#!/usr/bin/env python3
"""Summarize closed PX4 ULogs offline; never connect to ROS or a vehicle.

Run in the existing rl_drone conda environment (numpy and pyulog installed).
Times are PX4 boot seconds, NOT ROS epoch seconds. Missing fields are unknown,
not evidence of no resets/fusion. Logged EV gaps can include logger losses.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from pyulog import ULog


TOPICS = (
    'vehicle_local_position', 'vehicle_visual_odometry', 'actuator_armed',
    'vehicle_status', 'vehicle_land_detected', 'estimator_selector_status',
    'estimator_status_flags', 'estimator_event_flags', 'estimator_status',
    'estimator_innovations', 'estimator_innovation_variances',
    'estimator_innovation_test_ratios', 'timesync_status', 'vehicle_air_data',
    'sensor_baro', 'vehicle_imu_status', 'estimator_aid_src_ev_hgt',
    'estimator_aid_src_ev_pos', 'estimator_aid_src_ev_vel',
    'estimator_aid_src_ev_yaw', 'estimator_aid_src_baro_hgt',
    'estimator_baro_bias', 'battery_status',
)
COUNTERS = {
    'xy_reset_counter': ('delta_xy[0]', 'delta_xy[1]'),
    'z_reset_counter': ('delta_z',),
    'vxy_reset_counter': ('delta_vxy[0]', 'delta_vxy[1]'),
    'vz_reset_counter': ('delta_vz',),
    'heading_reset_counter': ('delta_heading',),
}


def scalar(value):
    value = value.item() if hasattr(value, 'item') else value
    return None if isinstance(value, float) and not math.isfinite(value) else value


def stats(values):
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return dict(zip(('min', 'p50', 'p95', 'p99', 'max'),
                    map(float, np.percentile(values, (0, 50, 95, 99, 100)))))


def change_indices(values):
    return np.flatnonzero(values[1:] != values[:-1]) + 1


def transitions(data, field, limit):
    values = data[field]
    indices = change_indices(values)
    return {
        'initial': scalar(values[0]) if len(values) else None,
        'change_count': len(indices),
        'changes': [{'boot_sec': float(data['timestamp'][i]) / 1e6,
                     'before': scalar(values[i-1]), 'after': scalar(values[i])}
                    for i in indices[:limit]],
        'truncated': len(indices) > limit,
    }


def parameters(ulog):
    def relevant(name):
        return (name.startswith(('EKF2_EV', 'EKF2_BARO_', 'EKF2_GPS_', 'EKF2_RNG_',
                                 'EKF2_IMU_POS_', 'SENS_BOARD_'))
                or name in ('EKF2_HGT_REF', 'EKF2_HGT_MODE', 'EKF2_AID_MASK',
                            'EKF2_DELAY_MAX',
                            'EKF2_MULTI_IMU', 'EKF2_MULTI_MAG', 'SDLOG_PROFILE',
                            'SDLOG_MODE', 'SENS_IMU_MODE'))
    return {
        'initial': {k: scalar(v) for k, v in ulog.initial_parameters.items() if relevant(k)},
        'changes': [{'boot_sec': t / 1e6, 'name': k, 'value': scalar(v)}
                    for t, k, v in ulog.changed_parameters if relevant(k)],
    }


def analyze(path, gap_sec, limit):
    ulog = ULog(str(path), message_name_filter_list=TOPICS)
    topics = {}
    warnings = []
    armed = next((d.data for d in ulog.data_list if d.name == 'actuator_armed'
                  and 'armed' in d.data and len(d.data['timestamp'])), None)

    def armed_at(t):
        if armed is None:
            return None
        i = int(np.searchsorted(armed['timestamp'], t, side='right')) - 1
        return bool(armed['armed'][i]) if i >= 0 else None

    for dataset in ulog.data_list:
        data = dataset.data
        ts = data.get('timestamp')
        if ts is None or not len(ts):
            continue
        name = dataset.name
        times = ts.astype(np.float64) / 1e6
        gaps = np.diff(times)
        out = {
            'count': len(ts), 'first_boot_sec': float(times[0]),
            'last_boot_sec': float(times[-1]), 'available_fields': sorted(data),
            'logged_interval_sec': stats(gaps),
            'timestamp_rewinds': int(np.count_nonzero(gaps < 0)),
        }
        topics[f'{name}[{dataset.multi_id}]'] = out
        if name == 'vehicle_local_position':
            out['missing_reset_fields'] = [f for f in COUNTERS if f not in data]
            resets = {}
            for field, delta_fields in COUNTERS.items():
                if field not in data:
                    continue
                indices = change_indices(data[field])
                events = []
                for i in indices[:limit]:
                    event = {
                        'boot_sec': float(times[i]), 'armed': armed_at(ts[i]),
                        'before': int(data[field][i-1]), 'after': int(data[field][i]),
                        'counter_step_mod256': (int(data[field][i])-int(data[field][i-1])) % 256,
                        'latest_reset_delta': {f: scalar(data[f][i])
                                               for f in delta_fields if f in data},
                    }
                    if field == 'z_reset_counter' and 'delta_z' in data:
                        delta = scalar(data['delta_z'][i])
                        event['latest_reset_delta_z_enu_up_m'] = -delta if delta is not None else None
                    events.append(event)
                resets[field] = {
                    'initial_counter': int(data[field][0]), 'change_count': len(indices),
                    'events': events, 'truncated': len(indices) > limit,
                }
            out['resets'] = resets
            out['states'] = {f: transitions(data, f, limit) for f in
                             ('xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid', 'dead_reckoning')
                             if f in data}
        elif name == 'vehicle_visual_odometry':
            indices = np.flatnonzero(gaps > gap_sec) + 1
            out['gap_count'] = len(indices)
            out['gaps'] = [{'previous_boot_sec': float(times[i-1]),
                            'boot_sec': float(times[i]), 'gap_sec': float(gaps[i-1]),
                            'armed': armed_at(ts[i])} for i in indices[:limit]]
            out['gaps_truncated'] = len(indices) > limit
            if 'timestamp_sample' in data:
                sample = data['timestamp_sample'].astype(np.float64)
                valid = sample > 0
                ages_ms = (ts.astype(np.float64)[valid]-sample[valid]) / 1e3
                out['publish_minus_sample_ms'] = stats(ages_ms)
                out['future_sample_count'] = int(np.count_nonzero(ages_ms < 0))
                out['zero_sample_count'] = int(np.count_nonzero(~valid))
                out['sample_timestamp_rewinds'] = int(np.count_nonzero(np.diff(sample[valid]) < 0))
            else:
                out['missing_sample_timestamp'] = True
            out['states'] = {f: transitions(data, f, limit)
                             for f in ('pose_frame', 'reset_counter') if f in data}
        else:
            if name == 'estimator_status_flags':
                fields = [f for f in data if f.startswith(('cs_ev_', 'reject_', 'fs_bad_acc_'))
                          or f in ('cs_in_air', 'cs_baro_hgt', 'cs_rng_hgt', 'cs_gps_hgt',
                                   'cs_inertial_dead_reckoning', 'cs_vehicle_at_rest')]
            elif name == 'estimator_event_flags':
                fields = [f for f in data if f.startswith(('reset_', 'starting_vision_'))
                          or f in ('information_event_changes', 'warning_event_changes',
                                   'vision_data_stopped')]
            elif name.startswith('estimator_aid_src_'):
                fields = [f for f in ('fused', 'innovation_rejected', 'estimator_instance') if f in data]
                out['numeric'] = {f: stats(v) for f, v in data.items()
                                  if f.startswith(('innovation', 'test_ratio', 'observation'))}
            else:
                fields = [f for f in ('primary_instance', 'instance_changed_count',
                                      'arming_state', 'nav_state', 'failsafe', 'armed',
                                      'landed', 'xy_valid', 'z_valid', 'control_mode_flags') if f in data]
                if name == 'battery_status':
                    fields += [f for f in ('connected', 'warning', 'cell_count') if f in data]
                if name in ('timesync_status', 'vehicle_air_data', 'sensor_baro',
                            'vehicle_imu_status', 'estimator_baro_bias', 'battery_status',
                            'estimator_innovations', 'estimator_innovation_variances',
                            'estimator_innovation_test_ratios'):
                    out['numeric'] = {f: stats(v) for f, v in data.items()
                                      if f != 'timestamp' and np.issubdtype(v.dtype, np.number)}
            out['states'] = {f: transitions(data, f, limit) for f in fields}

    names = {d.name for d in ulog.data_list}
    for required in ('vehicle_local_position', 'vehicle_visual_odometry'):
        if not any(k.startswith(required + '[') for k in topics):
            warnings.append(f'{required} has no logged samples; corresponding conclusions are unknown.')
    if 'estimator_aid_src_ev_hgt' not in names:
        warnings.append('EV height aid-source topic absent; this does not prove EV height fusion was inactive.')
    if 'estimator_status_flags' not in names:
        warnings.append('Decoded estimator flags absent; inspect legacy estimator_status bitfields for this firmware.')
    if 'estimator_event_flags' not in names:
        warnings.append('Estimator event flags absent; reset causes cannot be identified from event markers.')
    if armed is None:
        warnings.append('No actuator_armed samples; reset/gap events have unknown arming state.')
    dropouts = [{'boot_sec': d.timestamp / 1e6, 'duration_ms': d.duration}
                for d in ulog.dropouts]
    if dropouts:
        warnings.append('Logger dropouts present; logged EV gaps alone do not prove flight-controller input interruptions.')
    return {
        'ulog': str(path), 'time_basis': 'PX4 boot seconds; manually align with ROS before comparing events',
        'start_boot_sec': ulog.start_timestamp / 1e6,
        'end_boot_sec': ulog.last_timestamp / 1e6,
        'metadata': {k: scalar(ulog.msg_info_dict[k]) for k in
                     ('sys_name', 'ver_hw', 'ver_sw', 'ver_sw_branch', 'ver_sw_release')
                     if k in ulog.msg_info_dict},
        'gap_threshold_sec': gap_sec, 'events_limit_per_field': limit,
        'parameters': parameters(ulog), 'logger_dropouts': dropouts,
        'topics': topics, 'warnings': warnings,
        'limitations': [
            'Counter changes identify resets, not their cause; correlate fusion flags, innovations and selector changes.',
            'A counter step greater than one may hide multiple resets; delta fields describe only the latest reset.',
            'EV timestamp intervals reflect logged publications and may be affected by logging rate/losses.',
            'Estimator multi_id streams must be compared with the selected primary estimator.',
            'Counter timestamps are first logged observations; sparse local-position logging can trail the reset event.',
            'Numeric innovation statistics summarize logged samples and do not reconstruct every fusion-cycle rejection.',
            'The reported end time reflects selected topics and may precede the final sample in the full ULog.',
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ulogs', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gap-sec', type=float, default=.3)
    parser.add_argument('--events-limit', type=int, default=100)
    args = parser.parse_args()
    if not math.isfinite(args.gap_sec) or args.gap_sec <= 0 or args.events_limit <= 0:
        parser.error('gap-sec must be finite and positive; events-limit must be positive')
    reports = [analyze(p, args.gap_sec, args.events_limit) for p in args.ulogs]
    args.output.write_text(json.dumps(reports, indent=2, allow_nan=False), encoding='utf-8')
    for report in reports:
        print(f"{report['ulog']}: boot {report['start_boot_sec']:.3f}..{report['end_boot_sec']:.3f}s")
        for name, topic in report['topics'].items():
            if 'resets' in topic:
                for field, reset in topic['resets'].items():
                    print(f"  {name}.{field}: {reset['change_count']} counter changes")
                    for event in reset['events']:
                        print(f"    boot {event['boot_sec']:.3f}s armed={event['armed']} delta={event['latest_reset_delta']}")
        for warning in report['warnings']:
            print('  UNKNOWN: ' + warning)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Offline three-ULog analysis; no hardware, ROS, network or archive execution.

Use the existing rl_drone environment. Boot seconds are not UTC. A local-z
height is an estimate, not an independent AGL measurement. Motor controls are
dimensionless commands, not measured thrust/current. Full parse cache stays
under logs/. All asynchronous alignment uses previous samples, never future
state flags; sample ages are retained in snapshots.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from pyulog import ULog
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

INPUTS = [('hover04', '飞行0.4m.ulg'),
          ('above06', '飞行0.4m-实际飞行0.6m以上.ulg'),
          ('forward3', '飞行0.4m-前进3段.ulg')]
RESET_FIELDS = {'z_reset_counter': ['delta_z'],
                'xy_reset_counter': ['delta_xy[0]', 'delta_xy[1]'],
                'vz_reset_counter': ['delta_vz'],
                'vxy_reset_counter': ['delta_vxy[0]', 'delta_vxy[1]'],
                'heading_reset_counter': ['delta_heading']}
FLAGS = ['cs_ev_hgt', 'cs_ev_pos', 'cs_ev_yaw', 'cs_ev_vel', 'cs_baro_hgt',
         'cs_rng_hgt', 'cs_gps_hgt', 'cs_in_air', 'cs_inertial_dead_reckoning',
         'reject_yaw', 'reject_ver_pos', 'reject_hor_pos', 'reject_ver_vel',
         'reject_hor_vel', 'cs_mag_hdg', 'cs_mag_3d']
WARNINGS = {'gps_quality_poor', 'gps_fusion_timout', 'gps_data_stopped',
            'gps_data_stopped_using_alternate', 'height_sensor_timeout',
            'stopping_navigation', 'invalid_accel_bias_cov_reset',
            'bad_yaw_using_gps_course', 'stopping_mag_use', 'vision_data_stopped',
            'emergency_yaw_reset_mag_stopped', 'emergency_yaw_reset_gps_yaw_stopped'}
NEEDED = ['vehicle_visual_odometry', 'estimator_aid_src_ev_hgt',
          'estimator_aid_src_ev_yaw', 'estimator_aid_src_ev_pos',
          'estimator_aid_src_ev_vel', 'distance_sensor', 'esc_status']


def clean(x):
    if isinstance(x, dict): return {str(k): clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)): return [clean(v) for v in x]
    if isinstance(x, np.ndarray): return clean(x.tolist())
    if isinstance(x, np.generic): return clean(x.item())
    if isinstance(x, float) and not np.isfinite(x): return None
    return x


def stats(x):
    a = np.asarray(x, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a): return None
    return dict(n=len(a), **dict(zip(['min', 'p05', 'median', 'p95', 'max'],
                                   np.percentile(a, [0, 5, 50, 95, 100]))))


def changes(d, f):
    if d is None or f not in d: return []
    return [{'t': d['timestamp'][i]/1e6,
             'previous_t': d['timestamp'][i-1]/1e6,
             'before': d[f][i-1], 'after': d[f][i]}
            for i in np.flatnonzero(d[f][1:] != d[f][:-1])+1]


def intervals(t, mask):
    # Exclusive end: first false sample, or last sample for right censoring.
    ix = np.flatnonzero(np.diff(np.r_[False, mask, False].astype(int)))
    return [{'start': t[a], 'end': t[min(b, len(t)-1)],
             'right_censored': b == len(t), 'samples': b-a}
            for a, b in zip(ix[::2], ix[1::2])]


def asof(d, t, fields=None):
    if d is None or not len(d.get('timestamp', [])): return None
    i = np.searchsorted(d['timestamp']/1e6, t, side='right')-1
    if i < 0: return None
    return {'sample_t': d['timestamp'][i]/1e6,
            'age_s': t-d['timestamp'][i]/1e6,
            **{k: d[k][i] for k in (fields or d) if k in d and k != 'timestamp'}}


def analyze(root, short, filename, out, cache, landed_fc_clearance):
    u = ULog(str(root/'试飞产物'/filename))
    ds = {(d.name, d.multi_id): d.data for d in u.data_list}
    def get(n, i=0): return ds.get((n, i))
    def hold(n, f, t, default=np.nan, i=0):
        d = get(n, i)
        if d is None or f not in d: return np.full(len(t), default)
        ix = np.searchsorted(d['timestamp']/1e6, t, side='right')-1
        return np.where(ix >= 0, d[f][np.maximum(ix, 0)], default)
    def context(t):
        s = asof(get('estimator_selector_status'), t,
                 ['primary_instance', 'instance_changed_count'])
        inst = int(s['primary_instance']) if s else 0
        return {'selector': s, 'flags': asof(get('estimator_status_flags', inst), t, FLAGS),
                'land': asof(get('vehicle_land_detected'), t,
                             ['landed', 'ground_contact', 'maybe_landed', 'at_rest']),
                'status': asof(get('vehicle_status'), t, ['nav_state', 'arming_state', 'failsafe']),
                'takeoff': asof(get('takeoff_status'), t, ['takeoff_state']),
                'lp': asof(get('vehicle_local_position'), t,
                           ['x', 'y', 'z', 'vx', 'vy', 'vz', 'heading', 'dist_bottom',
                            'dist_bottom_valid', 'epv', 'z_valid']),
                'sp': asof(get('vehicle_local_position_setpoint'), t,
                           ['x', 'y', 'z', 'vx', 'vy', 'vz', 'yaw']),
                'att_sp': asof(get('vehicle_attitude_setpoint'), t,
                              ['roll_body', 'pitch_body', 'yaw_body', 'thrust_body[2]'])}
    report = {'id': short, 'filename': filename, 'start': u.start_timestamp/1e6,
              'end': u.last_timestamp/1e6, 'firmware': u.msg_info_dict,
              'initial_parameters': u.initial_parameters, 'changed_parameters': u.changed_parameters,
              'logged_messages': [{'t': m.timestamp/1e6, 'level': m.log_level, 'message': m.message}
                                  for m in u.logged_messages],
              'dropouts': [{'t': d.timestamp/1e6, 'ms': d.duration} for d in u.dropouts],
              'missing_topics': [n for n in NEEDED if get(n) is None],
              'topics': {}, 'transitions': {}, 'resets': [], 'events': [], 'flags': {}}
    report['conditional_landed_fc_clearance_m'] = landed_fc_clearance
    for (n, i), d in ds.items():
        if not len(d.get('timestamp', [])): continue
        report['topics'][f'{n}[{i}]'] = {'n': len(d['timestamp']), 'fields': list(d),
                    'dt_s': stats(np.diff(d['timestamp'].astype(float))/1e6),
                    'first': d['timestamp'][0]/1e6, 'last': d['timestamp'][-1]/1e6}
        if n == 'estimator_status_flags':
            report['flags'][str(i)] = {f: {'initial': d[f][0], 'changes': changes(d, f),
                       'true_intervals': intervals(d['timestamp']/1e6, d[f].astype(bool))}
                       for f in FLAGS if f in d}
        if n == 'estimator_event_flags':
            info = {x['t'] for x in changes(d, 'information_event_changes')}
            warn = {x['t'] for x in changes(d, 'warning_event_changes')}
            for j, t in enumerate(d['timestamp']/1e6):
                active = [f for f in d if f not in ['timestamp', 'timestamp_sample',
                          'information_event_changes', 'warning_event_changes'] and bool(d[f][j])
                          and t in (warn if f in WARNINGS else info)]
                if not active: continue
                c = context(t)
                report['events'].append({'t': t, 'instance': i, 'fields': active,
                         'information_counter': d['information_event_changes'][j],
                         'warning_counter': d['warning_event_changes'][j], 'context': c})
    for n, fs in [('vehicle_land_detected', ['landed', 'ground_contact', 'maybe_landed']),
                  ('vehicle_status', ['arming_state', 'nav_state', 'failsafe']),
                  ('actuator_armed', ['armed', 'lockdown', 'manual_lockdown']),
                  ('takeoff_status', ['takeoff_state']),
                  ('estimator_selector_status', ['primary_instance', 'instance_changed_count'])]:
        d = get(n)
        report['transitions'][n] = {f: changes(d, f) for f in fs}
        if d is not None:
            report['transitions'][n]['initial'] = asof(d, d['timestamp'][0]/1e6, fs)
    lp = get('vehicle_local_position')
    for f, df in RESET_FIELDS.items():
        for e in changes(lp, f):
            e.update(field=f, step=(int(e['after'])-int(e['before'])) % 256,
                     delta=asof(lp, e['t'], df), context=context(e['t']))
            report['resets'].append(e)
    att = get('vehicle_attitude')
    for e in changes(att, 'quat_reset_counter'):
        e.update(field='quat_reset_counter', delta=asof(att, e['t'],
                    [f'delta_q_reset[{i}]' for i in range(4)]), context=context(e['t']))
        report['resets'].append(e)
    report['initial_reset_counters'] = {f: lp[f][0] for f in RESET_FIELDS if f in lp}
    report['initial_reset_counters']['quat_reset_counter'] = att['quat_reset_counter'][0]
    # All raw numeric topics are local-only cache, never a committed deliverable.
    np.savez_compressed(cache/f'{short}.npz', **{f'{n}__{i}__{f}': a
                          for (n, i), d in ds.items() for f, a in d.items()})
    t = lp['timestamp']/1e6
    flying = (hold('actuator_armed', 'armed', t, 0) == 1) & (hold('vehicle_land_detected', 'landed', t, 1) == 0)
    report['airborne_lp_intervals'] = intervals(t, flying)
    report['flight_height'] = {'estimate_up_m': stats(-lp['z'][flying]),
                             'target_up_m': stats(-hold('vehicle_local_position_setpoint', 'z', t)[flying]),
                             'tracking_est_minus_target_m': stats((-lp['z']+hold('vehicle_local_position_setpoint', 'z', t))[flying]),
                             'epv_m': stats(lp['epv'][flying])}
    ground = hold('vehicle_land_detected', 'landed', t, 0) == 1
    first_air = t[flying][0] if flying.any() else t[-1]
    pre = ground & (t < first_air)
    report['preflight_ground_z'] = stats(lp['z'][pre])
    ground_z = np.median(lp['z'][pre]) if pre.any() else np.nan
    report['flight_height']['estimated_lift_from_logged_ground_m'] = stats(ground_z-lp['z'][flying])
    report['flight_height']['conditional_fc_agl_m'] = stats(landed_fc_clearance+ground_z-lp['z'][flying])
    peak_i = int(np.argmax(-lp['z']))
    report['height_peak'] = {'t': t[peak_i], 'local_up_m': -lp['z'][peak_i],
             'conditional_fc_agl_m': landed_fc_clearance+ground_z-lp['z'][peak_i],
             'context': context(t[peak_i])}
    # Keep per-source height/innovation statistics; sensor altitude is not AGL.
    report['sensor_stats'] = {}
    for (n, i), d in ds.items():
        if n in ['vehicle_air_data', 'sensor_baro', 'battery_status', 'system_power',
                 'estimator_baro_bias', 'estimator_innovations', 'estimator_innovation_test_ratios',
                 'estimator_innovation_variances']:
            report['sensor_stats'][f'{n}[{i}]'] = {f: stats(a) for f, a in d.items()
                                    if f not in ['timestamp', 'timestamp_sample']}
    q = np.array([att[f'q[{i}]'] for i in range(4)]).T
    roll = np.rad2deg(np.arctan2(2*(q[:, 0]*q[:, 1]+q[:, 2]*q[:, 3]),
                               1-2*(q[:, 1]**2+q[:, 2]**2)))
    pitch = np.rad2deg(np.arcsin(np.clip(2*(q[:, 0]*q[:, 2]-q[:, 3]*q[:, 1]), -1, 1)))
    yaw = np.rad2deg(np.arctan2(2*(q[:, 0]*q[:, 3]+q[:, 1]*q[:, 2]),
                              1-2*(q[:, 2]**2+q[:, 3]**2)))
    mot = get('actuator_motors'); mt = mot['timestamp']/1e6
    c = np.array([mot[f'control[{i}]'] for i in range(4)]).T
    airm = (hold('actuator_armed', 'armed', mt, 0) == 1) & (hold('vehicle_land_detected', 'landed', mt, 1) == 0)
    # Conservative stable local-estimate window: >2s after detected takeoff,
    # armed, no ground_contact, rates small, low estimated speed and tilt.
    rr = hold('vehicle_local_position', 'vx', mt)**2+hold('vehicle_local_position', 'vy', mt)**2
    rt = get('vehicle_angular_velocity')
    rate_norm = np.sqrt(sum(hold('vehicle_angular_velocity', f'xyz[{i}]', mt)**2 for i in range(3)))
    stable = airm & (mt > first_air+2) & (hold('vehicle_land_detected', 'ground_contact', mt, 1) == 0)
    stable &= hold('vehicle_status', 'nav_state', mt) == 14
    stable &= hold('takeoff_status', 'takeoff_state', mt) == 5
    stable &= (rr < .3**2) & (np.abs(hold('vehicle_local_position', 'vz', mt)) < .2)
    stable &= (np.abs(np.interp(mt, att['timestamp']/1e6, roll)) < 10)
    stable &= (np.abs(np.interp(mt, att['timestamp']/1e6, pitch)) < 10) & (rate_norm < .3)
    good = [r for r in intervals(mt, stable) if r['end']-r['start'] >= 1]
    # End each continuous accepted interval at its last accepted sample to avoid
    # printing the following POSCTL/moving sample as if included in the window.
    for rr in good:
        rr['end_exclusive_first_false'] = rr['end']
        rr['last_included'] = mt[(mt >= rr['start']) & (mt < rr['end'])][-1]
    stable = np.zeros(len(mt), bool)
    for r in good: stable |= (mt >= r['start']) & (mt < r['end'])
    report['stable_intervals'] = good
    report['phases'] = {}
    for label, mask in [('airborne', airm), ('stable', stable)]:
        p = {'n': int(mask.sum()), 'commands': [stats(c[mask, i]) for i in range(4)],
             'M1_minus_mean_others': stats(c[mask, 0]-c[mask, 1:].mean(axis=1)),
             'highest_fraction': [np.mean(c[mask, i] == np.max(c[mask], axis=1)) if mask.any() else None for i in range(4)],
             'motor_near_upper_fraction': [np.mean(c[mask, i] >= .999) if mask.any() else None for i in range(4)],
             'roll_deg': stats(np.interp(mt, att['timestamp']/1e6, roll)[mask]),
             'pitch_deg': stats(np.interp(mt, att['timestamp']/1e6, pitch)[mask]),
             'estimate_height_m': stats(-hold('vehicle_local_position', 'z', mt)[mask]),
             'target_height_m': stats(-hold('vehicle_local_position_setpoint', 'z', mt)[mask]),
             'estimated_lift_from_logged_ground_m': stats(ground_z-hold('vehicle_local_position', 'z', mt)[mask]),
             'target_lift_from_logged_ground_m': stats(ground_z-hold('vehicle_local_position_setpoint', 'z', mt)[mask]),
             'conditional_fc_agl_m': stats(landed_fc_clearance+ground_z-hold('vehicle_local_position', 'z', mt)[mask]),
             'battery_voltage': stats(hold('battery_status', 'voltage_v', mt)[mask]),
             'battery_current_total': stats(hold('battery_status', 'current_a', mt)[mask]),
             'rate_integrals': {f: stats(hold('rate_ctrl_status', f, mt)[mask])
                                for f in ['rollspeed_integ', 'pitchspeed_integ', 'yawspeed_integ']},
             'rate_error_abs': {f: stats(np.abs(hold('vehicle_angular_velocity', f'xyz[{i}]', mt)-
                                hold('vehicle_rates_setpoint', f, mt))[mask]) for i, f in enumerate(['roll', 'pitch', 'yaw'])},
             'allocator_saturation': {str(i): stats(hold('control_allocator_status', f'actuator_saturation[{i}]', mt)[mask]) for i in range(4)}}
        report['phases'][label] = p
    report['snapshots'] = [{'t': t, 'context': context(t)} for t in sorted(set(
               [e['t'] for e in report['resets']]+[e['t'] for e in report['events']]+
               [e['t'] for f in report['transitions']['takeoff_status'].values() if isinstance(f, list) for e in f]))]
    fig, ax = plt.subplots(6, 1, figsize=(13, 16), sharex=True)
    ax[0].plot(t, -lp['z'], label='estimated local height (-z)')
    sp = get('vehicle_local_position_setpoint'); st = sp['timestamp']/1e6
    ax[0].plot(st, -sp['z'], label='local height setpoint', linestyle='--')
    if 'dist_bottom' in lp:
        ax[0].plot(t, np.where(lp['dist_bottom_valid'], lp['dist_bottom'], np.nan), label='valid dist_bottom')
    ax[0].set_ylabel('m; NOT proven AGL')
    at = att['timestamp']/1e6
    ax[1].plot(at, yaw, label='estimated yaw deg')
    asp = get('vehicle_attitude_setpoint')
    ax[1].plot(asp['timestamp']/1e6, np.rad2deg(asp['yaw_body']), label='yaw setpoint', linestyle='--')
    ax[1].set_ylabel('yaw deg')
    ax[2].plot(at, roll, label='roll deg'); ax[2].plot(at, pitch, label='pitch deg')
    for i in range(4): ax[3].plot(mt, c[:, i], label=f'M{i+1} command')
    ax[3].set_ylabel('dimensionless command')
    sel = get('estimator_selector_status')
    primary = int(sel['primary_instance'][0]) if sel is not None else 0
    flags = get('estimator_status_flags', primary); ft = flags['timestamp']/1e6
    for i, f in enumerate(['cs_ev_hgt', 'cs_ev_pos', 'cs_ev_yaw', 'reject_yaw', 'reject_ver_pos']):
        ax[4].step(ft, flags[f]+i*1.5, where='post', label=f)
    bat = get('battery_status')
    ax[5].plot(bat['timestamp']/1e6, bat['voltage_v'], label='reported battery V')
    ax[5].plot(bat['timestamp']/1e6, bat['current_a'], label='reported TOTAL battery A')
    ax[5].set_xlabel(f'PX4 boot seconds; flag panel = estimator instance {primary} (initial primary)')
    for a in ax:
        for e in report['resets']:
            if e['field'] in ['heading_reset_counter', 'z_reset_counter']:
                a.axvline(e['t'], alpha=.4, color='r', linestyle=':')
        a.grid(alpha=.3); a.legend(loc='best', fontsize=8)
    fig.suptitle(f'{short}: estimates / setpoints / commands; not independent height or thrust measurements')
    fig.tight_layout(); fig.savefig(out/f'{short}_timeline.png', dpi=125); plt.close(fig)
    (cache/f'{short}_details.json').write_text(json.dumps(clean(report), indent=2, ensure_ascii=False), encoding='utf-8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[3])
    parser.add_argument('--landed-fc-clearance-m', type=float, default=.22,
                        help='Conditional geometry only; remeasure before treating as AGL')
    args = parser.parse_args()
    out = Path(__file__).resolve().parent
    cache = args.root/'logs/low_hover_diagnosis_20261006'; cache.mkdir(parents=True, exist_ok=True)
    results = []
    for short, name in INPUTS:
        r = analyze(args.root, short, name, out, cache, args.landed_fc_clearance_m); results.append(r)
        print(short, 'resets', clean(r['resets']), 'stable', clean(r['stable_intervals']), flush=True)
    fields = ['id', 'start', 'end', 'z_resets', 'yaw_resets', 'xy_resets', 'quat_resets',
              'motor_n', 'M1', 'M2', 'M3', 'M4', 'M1_minus_others', 'height_median', 'target_median',
              'ground_z_ned_m', 'target_lift_median', 'estimated_lift_median',
              'conditional_landed_fc_clearance_m', 'conditional_fc_agl_median', 'conditional_fc_agl_peak',
              'height_peak_boot_s', 'voltage_median', 'voltage_max', 'current_total_median', 'dropout_n', 'dropout_ms']
    with (out/'comparison.csv').open('w', encoding='utf-8', newline='') as fp:
        w = csv.DictWriter(fp, fieldnames=fields); w.writeheader()
        for r in results:
            p = r['phases']['stable']
            w.writerow(clean({'id': r['id'], 'start': r['start'], 'end': r['end'],
                **{f'{label}_resets': sum(e['field'] == f for e in r['resets'])
                   for label, f in [('z', 'z_reset_counter'), ('yaw', 'heading_reset_counter'),
                                   ('xy', 'xy_reset_counter'), ('quat', 'quat_reset_counter')]},
                'motor_n': p['n'], **{f'M{i+1}': s['median'] if s else None for i, s in enumerate(p['commands'])},
                'M1_minus_others': (p['M1_minus_mean_others'] or {}).get('median'),
                'height_median': (p['estimate_height_m'] or {}).get('median'),
                'target_median': (p['target_height_m'] or {}).get('median'),
                'ground_z_ned_m': (r['preflight_ground_z'] or {}).get('median'),
                'target_lift_median': (p['target_lift_from_logged_ground_m'] or {}).get('median'),
                'estimated_lift_median': (p['estimated_lift_from_logged_ground_m'] or {}).get('median'),
                'conditional_landed_fc_clearance_m': args.landed_fc_clearance_m,
                'conditional_fc_agl_median': (p['conditional_fc_agl_m'] or {}).get('median'),
                'conditional_fc_agl_peak': r['height_peak']['conditional_fc_agl_m'],
                'height_peak_boot_s': r['height_peak']['t'],
                'voltage_median': (p['battery_voltage'] or {}).get('median'),
                'voltage_max': (r['sensor_stats'].get('battery_status[0]', {}).get('voltage_v') or {}).get('max'),
                'current_total_median': (p['battery_current_total'] or {}).get('median'),
                'dropout_n': len(r['dropouts']), 'dropout_ms': sum(d['ms'] for d in r['dropouts'])}))
    with (out/'events.csv').open('w', encoding='utf-8', newline='') as fp:
        w = csv.DictWriter(fp, fieldnames=['id', 'kind', 'instance', 't', 'previous_t', 'field', 'before', 'after', 'delta', 'fields'])
        w.writeheader()
        for r in results:
            for e in r['resets']:
                w.writerow(clean({'id': r['id'], 'kind': 'reset', **{k: e.get(k) for k in ['t', 'previous_t', 'field', 'before', 'after']}, 'delta': json.dumps(clean(e['delta']))}))
            for e in r['events']:
                w.writerow(clean({'id': r['id'], 'kind': 'fresh_estimator_event', 'instance': e['instance'], 't': e['t'], 'fields': ','.join(e['fields'])}))
            for n, fs in r['transitions'].items():
                for f, ee in fs.items():
                    if not isinstance(ee, list): continue
                    for e in ee:
                        w.writerow(clean({'id': r['id'], 'kind': 'state_change', 'instance': 0,
                           'field': n+'.'+f, **{k: e[k] for k in ['t', 'previous_t', 'before', 'after']}}))
            for e in r['logged_messages']:
                w.writerow(clean({'id': r['id'], 'kind': 'text_message', 't': e['t'], 'fields': e['message'].rstrip()}))
    (cache/'summary.json').write_text(json.dumps(clean(results), indent=2, ensure_ascii=False), encoding='utf-8')
    print('Saved small CSV/plots beside script; full details and NPZ under', cache)


if __name__ == '__main__': main()

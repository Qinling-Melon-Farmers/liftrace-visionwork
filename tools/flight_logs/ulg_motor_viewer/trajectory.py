"""Agg trajectory export, adapted from tools/飞控日志小工具.zip trajectory_viewer.py.

Retains its NED→ENU mapping and breaks at resets/invalid positions/log gaps.
No Tk, SSH settings, downloaded logs or remote process controls are imported.
"""
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg


def position_samples(data, start, check_valid=False):
    required = ('timestamp', 'x', 'y', 'z')
    if not all(key in data for key in required):
        raise ValueError('位置话题缺少 timestamp/x/y/z 字段')
    order = np.argsort(data['timestamp'], kind='stable')
    time = (np.asarray(data['timestamp'], dtype=float)[order] - start) / 1e6
    points = np.column_stack([np.asarray(data[key], dtype=float)[order] for key in ('x', 'y', 'z')])
    finite = np.isfinite(time)
    if check_valid:
        for key in ('xy_valid', 'z_valid'):
            if key in data:
                points[~np.asarray(data[key], dtype=bool)[order]] = np.nan
    breaks = np.r_[False, np.diff(time) > .5] if len(time) else np.array([], dtype=bool)
    if check_valid:
        for key in ('xy_reset_counter', 'z_reset_counter'):
            if key in data:
                counter = np.asarray(data[key])[order]
                breaks |= np.r_[False, counter[1:] != counter[:-1]] if len(time) else False
    points[breaks] = np.nan
    return time[finite], points[finite], int(np.sum(breaks))


def enu(points, origin, relative):
    values = points - origin if relative else points
    return values[:, [1, 0, 2]] * np.array([1., 1., -1.])


def display_indices(points, budget=12000):
    if len(points) <= budget:
        return np.arange(len(points))
    gaps = np.flatnonzero(~np.all(np.isfinite(points), axis=1))
    return np.unique(np.r_[np.linspace(0, len(points)-1, budget, dtype=int),
                           gaps, np.maximum(0, gaps-1), np.minimum(len(points)-1, gaps+1)])


def export_trajectory(analysis, destination):
    from analysis import write_csv
    from plotting import configure_fonts
    configure_fonts()
    figure = Figure(figsize=(14, 7), constrained_layout=True)
    FigureCanvasAgg(figure)
    spatial = figure.add_subplot(121, projection='3d')
    planar = figure.add_subplot(222)
    height = figure.add_subplot(224)
    stream = analysis.get('vehicle_local_position')
    messages = []
    if stream:
        try:
            times, points, breaks = position_samples(stream.data, analysis.start_us, True)
            good = np.flatnonzero(np.all(np.isfinite(points), axis=1))
            if len(good) < 2:
                raise ValueError('有效位置样本不足')
            origin = points[good[0]]
            values = enu(points, origin, True)
            idx = display_indices(values)
            spatial.plot(*values[idx].T, label='estimated position')
            planar.plot(values[idx, 0], values[idx, 1], label='estimated position')
            height.plot(times[idx], values[idx, 2], label='estimated local up')
            target = analysis.get('vehicle_local_position_setpoint')
            if target and all(k in target.data for k in ('timestamp', 'x', 'y', 'z')):
                tt, tp, _ = position_samples(target.data, analysis.start_us)
                tv = enu(tp, origin, True); j = display_indices(tv)
                spatial.plot(*tv[j].T, linestyle='--', label='setpoint')
                planar.plot(tv[j, 0], tv[j, 1], linestyle='--', label='setpoint')
                height.plot(tt[j], tv[j, 2], linestyle='--', label='setpoint')
            messages.append(f'Position reset / >0.5s gap breaks: {breaks}. Invalid positions are not connected.')
            write_csv(destination / 'trajectory.csv',
                      ({'time_s': t, 'east_m': p[0], 'north_m': p[1], 'local_up_m': p[2]}
                       for t, p in zip(times, values)), ['time_s', 'east_m', 'north_m', 'local_up_m'])
            planar.set_aspect('equal', adjustable='datalim')
            planar.legend(); height.legend()
        except ValueError as error:
            messages.append(str(error))
    else:
        messages.append('vehicle_local_position absent: trajectory UNKNOWN.')
    spatial.set(xlabel='East / m', ylabel='North / m', zlabel='Local up / m', title='Relative local trajectory')
    planar.set(xlabel='East / m', ylabel='North / m', title='Top view')
    height.set(xlabel='Seconds since log start', ylabel='Local up / m', title='Position estimate; not AGL')
    figure.suptitle(analysis.path.name + '\nOrigin = first valid position; E=NED y, N=NED x, U=-NED z')
    if not planar.lines:
        planar.text(.05, .5, '\n'.join(messages), transform=planar.transAxes)
    figure.savefig(destination / 'trajectory.png', dpi=120)
    (destination / 'trajectory.txt').write_text('\n'.join(messages), encoding='utf-8')

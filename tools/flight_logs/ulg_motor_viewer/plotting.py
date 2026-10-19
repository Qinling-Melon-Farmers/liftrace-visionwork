"""Matplotlib Agg rendering; safe to use without DISPLAY or Tk."""
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib import font_manager, rcParams
from analysis import euler


def configure_fonts():
    available = {font.name for font in font_manager.fontManager.ttflist}
    if not any('CJK' in name or 'YaHei' in name for name in available):
        # Refresh available system CJK fonts when the existing Matplotlib cache
        # predates their installation; no package/font installation is performed.
        for path in font_manager.findSystemFonts():
            if 'cjk' in path.lower():
                font_manager.fontManager.addfont(path)
        available = {font.name for font in font_manager.fontManager.ttflist}
    preferred = [name for name in ('Noto Sans CJK SC', 'Noto Sans CJK JP',
                                   'Microsoft YaHei', 'WenQuanYi Zen Hei') if name in available]
    rcParams['font.sans-serif'] = preferred + ['DejaVu Sans']
    rcParams['axes.unicode_minus'] = False


def overview(analysis, destination):
    configure_fonts()
    figure = Figure(figsize=(16, 24), constrained_layout=True)
    FigureCanvasAgg(figure)
    axes = figure.subplots(10, 1, sharex=True)
    age = analysis.options.state_age_sec

    def lines(ax, name, fields, scale=1, selected=False):
        streams = analysis.all(name)
        if selected and streams:
            grid = np.unique(np.concatenate([s.time for s in streams]
                             + [s.time for s in analysis.all('estimator_selector_status')]))
            for field in fields:
                values = analysis.selected(name, field, grid)*scale
                if np.any(np.isfinite(values)):
                    ax.plot(grid, values, label=f'{name}.selected.{field}', linewidth=.9)
            return streams
        for stream in streams:
            for field in fields:
                if field not in stream.data:
                    continue
                values = (analysis.selected(name, field, stream.time) if selected
                          else np.asarray(stream.data[field], dtype=float)) * scale
                if np.any(np.isfinite(values)):
                    draw = ax.step if name in ('vehicle_status', 'manual_control_switches') else ax.plot
                    draw(stream.time, values, label=f'{stream.key}.{field}', linewidth=.9,
                         **({'where': 'post'} if draw == ax.step else {}))
        return streams

    for stream in analysis.motor_streams():
        _, windows = analysis.context(stream.time)
        for field in analysis.motor_fields(stream):
            values = stream.data[field]
            axes[0].plot(stream.time, values, alpha=.25, linewidth=.8, label=f'{stream.key}.{field} all')
            axes[0].plot(stream.time, np.where(windows == 'motor_airborne', values, np.nan),
                         linewidth=1.2, label=f'{field} eligible')
    axes[0].set_ylabel('Motor COMMAND\nnormalized')
    if not analysis.motor_streams():
        axes[0].text(.02, .4, 'Normalized motor command unavailable; outputs remain raw CSV.', transform=axes[0].transAxes)
    yaw_axis = axes[1].twinx()
    stream = analysis.get('vehicle_attitude')
    if stream:
        angles = euler(stream.data)
        if angles is not None:
            for i, name in enumerate(('roll', 'pitch')):
                axes[1].plot(stream.time, angles[:, i], label=f'estimated {name}', linewidth=.9)
            yaw_axis.plot(stream.time, angles[:, 2], color='green', alpha=.7, label='estimated yaw')
    lines(axes[1], 'vehicle_attitude_setpoint', ('roll_body', 'pitch_body'), 180/np.pi)
    lines(yaw_axis, 'vehicle_attitude_setpoint', ('yaw_body',), 180/np.pi)
    yaw_axis.set_ylabel('Yaw (deg)')
    if yaw_axis.get_legend_handles_labels()[0]:
        yaw_axis.legend(fontsize=6, loc='upper right')
    axes[1].set_ylabel('Roll / pitch (deg)')
    lines(axes[2], 'vehicle_angular_velocity', ('xyz[0]', 'xyz[1]', 'xyz[2]'), 180/np.pi)
    lines(axes[2], 'vehicle_rates_setpoint', ('roll', 'pitch', 'yaw'), 180/np.pi)
    axes[2].set_ylabel('Body angular rate\n(deg/s)')
    manual = lines(axes[3], 'manual_control_setpoint', ('roll', 'pitch', 'yaw', 'throttle', 'valid'))
    if not manual:
        rc = analysis.get('input_rc')
        if rc:
            lines(axes[3], 'input_rc', [f'values[{i}]' for i in range(4)])
    axes[3].set_ylabel('RC / manual\nsource-native units')
    tracks = [('actuator_armed', 'armed'), ('vehicle_land_detected', 'landed'),
              ('vehicle_land_detected', 'ground_contact'), ('actuator_armed', 'lockdown'),
              ('actuator_armed', 'manual_lockdown'), ('input_rc', 'rc_lost'),
              ('input_rc', 'rc_failsafe'), ('manual_control_switches', 'kill_switch')]
    for index, (topic, field) in enumerate(tracks):
        stream = analysis.get(topic)
        if stream and field in stream.data:
            axes[4].step(stream.time, np.asarray(stream.data[field], float)*.3+index,
                         where='post', linewidth=1, label=f'{topic}.{field} (native)')
    axes[4].set_yticks(range(len(tracks)))
    axes[4].set_yticklabels([field for _, field in tracks], fontsize=8)
    axes[4].set_ylabel('State tracks\n0/1; kill native 0/1/2')
    lines(axes[5], 'vehicle_status', ('nav_state', 'arming_state', 'failsafe'))
    lines(axes[5], 'manual_control_switches', ('mode_slot', 'offboard_switch', 'arm_switch'))
    axes[5].set_ylabel('Mode / switch\nnative enum ID')
    for stream in analysis.all('battery_status'):
        for field in ('voltage_v', 'voltage_filtered_v'):
            if field in stream.data:
                values = np.asarray(stream.data[field], float)
                valid = np.isfinite(values) & (values > 0)
                if 'connected' in stream.data:
                    valid &= np.asarray(stream.data['connected']) == 1
                if np.any(valid):
                    axes[6].plot(stream.time, np.where(valid, values, np.nan),
                                 label=f'{stream.key}.{field} connected positive only')
    axes[6].set_ylabel('PACK voltage (V)')
    local = analysis.get('vehicle_local_position')
    if local and 'z' in local.data:
        up = -np.asarray(local.data['z'], dtype=float)
        valid = np.asarray(local.data.get('z_valid', np.zeros(len(up)))) == 1
        axes[7].plot(local.time, np.where(valid, up, np.nan), label='estimated local up=-z (valid only)')
        if 'dist_bottom' in local.data:
            valid = np.asarray(local.data.get('dist_bottom_valid', np.zeros(len(up)))) == 1
            axes[7].plot(local.time, np.where(valid, local.data['dist_bottom'], np.nan),
                         label='estimated dist_bottom (valid only)')
    sp = analysis.get('vehicle_local_position_setpoint')
    if sp and 'z' in sp.data:
        axes[7].plot(sp.time, -np.asarray(sp.data['z'], float), '--', label='local up SETPOINT=-z')
    axes[7].set_ylabel('Height ESTIMATE (m)\nNOT actual AGL')
    flags = ('cs_ev_pos', 'cs_ev_vel', 'cs_ev_hgt', 'cs_ev_yaw', 'cs_ev_yaw_fault',
             'reject_ver_pos', 'reject_yaw')
    streams = analysis.all('estimator_status_flags')
    if streams:
        grid = np.unique(np.concatenate([s.time for s in streams]
                         + [s.time for s in analysis.all('estimator_selector_status')]))
        for index, field in enumerate(flags):
            values = analysis.selected('estimator_status_flags', field, grid)
            if np.any(np.isfinite(values)):
                axes[8].step(grid, values*.4+index, where='post', label=field+' selected')
        axes[8].set_yticks(range(len(flags)))
        axes[8].set_yticklabels(flags, fontsize=8)
    else:
        lines(axes[8], 'estimator_status', ('control_mode_flags', 'timeout_flags'))
    axes[8].set_ylabel('EV flags\nselected EKF; 0/1')
    lines(axes[9], 'estimator_innovation_test_ratios', ('ev_hpos[0]', 'ev_hpos[1]', 'ev_vpos', 'heading'), selected=True)
    for topic in ('estimator_aid_src_ev_hgt', 'estimator_aid_src_ev_yaw'):
        lines(axes[9], topic, ('test_ratio', 'innovation_rejected'), selected=True)
    ev = analysis.get('vehicle_visual_odometry')
    if ev and 'timestamp_sample' in ev.data:
        sample = np.asarray(ev.data['timestamp_sample'], float)
        delay = np.where(sample > 0, (ev.data['timestamp'].astype(float)-sample)/1000, np.nan)
        delay_axis = axes[9].twinx()
        delay_axis.plot(ev.time, delay, ':', color='black', label='EV publish-sample ms')
        delay_axis.set_ylabel('EV publish-sample (ms)')
        delay_axis.legend(loc='upper right', fontsize=7)
    else:
        axes[9].text(.02, .5, 'EV input freshness UNKNOWN: vehicle_visual_odometry not logged',
                     transform=axes[9].transAxes, fontsize=8)
    axes[9].axhline(1, color='grey', linestyle=':', linewidth=.8)
    axes[9].set_ylabel('Innovation test ratio\nheading not EV-specific')
    # Render all motor classifications as a bottom strip on the command panel.
    palette = {'ground': '#bbbbbb', 'motor_airborne': '#44aa77', 'transition_guard': '#ddaa44',
               'stopped_or_blocked': '#555555', 'suspected_impact': '#dd5555',
               'logging_gap': '#aa66cc', 'unknown': '#eeeeee'}
    for stream in analysis.motor_streams():
        _, labels = analysis.context(stream.time)
        for i in range(len(stream.time)-1):
            axes[0].axvspan(stream.time[i], stream.time[i+1], ymin=0, ymax=.05,
                           color=palette[labels[i]], alpha=.7, linewidth=0)
    for event in analysis.events:
        if event['kind'] == 'reset_counter_change':
            for ax in (axes[0], axes[7], axes[8]):
                ax.axvline(event['time_s'], color='red', linestyle=':', alpha=.55, linewidth=.8)
            axes[7].annotate(event['field'], (event['time_s'], .98),
                             xycoords=('data', 'axes fraction'), rotation=90,
                             va='top', fontsize=6, color='red')
    for ax in axes:
        ax.grid(alpha=.25)
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(handles, labels, fontsize=6, ncol=3, loc='upper left')
        elif ax not in (axes[0], axes[9]):
            ax.text(.02, .5, 'No usable logged fields', transform=ax.transAxes)
    axes[-1].set_xlabel('Seconds relative to this ULog start (not calendar time)')
    figure.suptitle(analysis.path.name+'\nCommands, estimates and logged states; no motor/height ground truth\n'
                    'Window strip: green airborne; grey ground/stopped; amber transition; red spike; purple gap', fontsize=11)
    figure.savefig(destination, dpi=140)
    figure.clear()

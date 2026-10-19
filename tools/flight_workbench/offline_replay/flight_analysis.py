"""Offline delivery/landing analysis. Does not publish, repair, or alter bags."""
import argparse
import bisect
import csv
import html
import json
import math
import re
import sys
from collections import Counter
from datetime import datetime, timedelta, timezone
from pathlib import Path

from bag_replay import open_bag, select_reader

ANALYSIS_TOPICS = {
    'metadata': '/board_trials/run_metadata', 'logs': '/rosout_agg',
    'pose': '/navigation/local_pose', 'odom': '/navigation/local_odom',
    'setpoint': '/navigation/setpoint_mission', 'fc_pose': '/mavros/local_position/pose',
    'fc_odom': '/mavros/local_position/odom', 'fc_setpoint': '/mavros/setpoint_position/local',
    'authorization': '/mission/release_authorization', 'permission': '/mission/release_permission',
    'release': '/mission/release_result', 'command': '/mission/command',
    'mission': '/navigation/mission_status', 'fc': '/mavros/state',
    'extended': '/mavros/extended_state', 'raw_setpoint': '/mavros/setpoint_raw/target_local',
    'handoff': '/patrol_control/external_landing_handoff',
}
NAMES = {'panzer': '坦克', 'bridge': '桥', 'red_cross': '红十字', 'landing_pad': 'H点'}


def read_data(bag, reader='auto', topics=None):
    mapping = dict(ANALYSIS_TOPICS)
    if topics:
        overrides = json.loads(Path(topics).read_text(encoding='utf-8-sig'))
        if not isinstance(overrides, dict) or not all(isinstance(k, str) and isinstance(v, str) for k, v in overrides.items()):
            raise ValueError('话题 JSON 必须是字符串键和值组成的对象')
        mapping.update({k: v for k, v in overrides.items() if k in mapping})
    rows = {k: [] for k in mapping}
    reverse = {}
    for key, topic in mapping.items():
        reverse.setdefault(topic, []).append(key)
    try:
        backend = select_reader(reader)
    except RuntimeError:
        import ros1_bridge
        if reader != 'auto' or not ros1_bridge.available():
            raise
        return ros1_bridge.analysis_data(bag, topics)
    with open_bag(Path(bag), list(mapping.values()), backend) as (start, end, available, messages):
        for topic, msgtype, message, stamp in messages:
            if msgtype == 'std_msgs/String':
                try:
                    message = json.loads(message['data'])
                except (ValueError, TypeError, KeyError):
                    pass
            for key in reverse[topic]:
                rows[key].append({'t': stamp-start, 'm': message})
    for data in rows.values():
        data.sort(key=lambda r: r['t'])
    return dict(source=str(Path(bag).resolve()), start=start, duration=end-start,
                topics=mapping, missing=[k for k, v in mapping.items() if v not in available], rows=rows)


def logtext(row):
    return str(row['m'].get('msg', '')) if isinstance(row['m'], dict) else ''


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def position(row):
    m = row['m']
    p = m.get('pose', {})
    return p.get('pose', p)


def ground_info(rows):
    metadata = {}
    for row in rows['metadata']:
        if isinstance(row['m'], dict):
            metadata.update(row['m'])
    reference = metadata.get('ground_reference', {})
    ground = reference.get('ground_z')
    settings = reference.get('settings', metadata.get('settings', {}))
    rig = reference.get('known_rig', metadata.get('known_rig', {}))
    return ground if finite(ground) else None, settings, rig


def raw_channels(rows, start, end):
    counts = Counter()
    frames = Counter()
    nonzero = 0
    for row in rows:
        if not start <= row['t'] <= end:
            continue
        m = row['m']
        mask = m.get('type_mask')
        if not isinstance(mask, int):
            continue
        pos, vel = not bool(mask & 4), not bool(mask & 32)
        counts['Z位置+Z速度' if pos and vel else 'Z位置' if pos else 'Z速度' if vel else 'Z位置与速度均忽略'] += 1
        frames[str(m.get('coordinate_frame', '未知'))] += 1
        vz = m.get('velocity', {}).get('z')
        if vel and finite(vz) and abs(vz) > .01:
            nonzero += 1
    return {'channels_by_type_mask': dict(counts), 'coordinate_frames': dict(frames), 'active_nonzero_vz_samples': nonzero}


def extract_deliveries(data):
    rows = data['rows']
    starts = []
    for row in rows['logs']:
        match = re.search(r'External ALIGN target=(\d+) class=(\S+)', logtext(row))
        if match:
            starts.append(dict(t=row['t'], target_id=int(match[1]), target_class=match[2], start_source='controller_log'))
    for row in rows['command']:
        m = row['m']
        if m.get('command') == m.get('ALIGN', 2) and not any(abs(s['t']-row['t']) < .5 for s in starts):
            starts.append(dict(t=row['t'], target_id=m.get('target_id'), target_class=m.get('target_class', ''), start_source='command'))
    starts.sort(key=lambda s: s['t'])
    runs = []
    for i, start in enumerate(starts):
        stop = starts[i+1]['t'] if i+1 < len(starts) else data['duration'] + 1e-6
        logs = [r for r in rows['logs'] if start['t'] <= r['t'] < stop]
        releases = [r for r in rows['release'] if start['t'] <= r['t'] < stop and
                    r['m'].get('target_id') == start['target_id'] and r['m'].get('target_class') == start['target_class']]
        calls = [r for r in releases if r['m'].get('reason') == 'raw_actuator_call_started']
        acks = [r for r in releases if r['m'].get('success') is True and r['m'].get('terminal') is True]
        freeze = next((r for r in logs if '[DropGeometry] frozen slot=' in logtext(r)), None)
        descent = next((r for r in logs if '[DropGeometry] descent_started slot=' in logtext(r)), None)
        call = calls[0] if calls else None
        ack = next((r for r in acks if not call or r['t'] >= call['t']), None)
        terminal = next((r for r in releases if r['m'].get('terminal')), None)
        end_event = ack or terminal or next((r for r in logs if 'External ALIGN completed' in logtext(r)), None)
        end = min(stop, end_event['t']+1 if end_event else stop)
        ready = next((r for r in logs if '[DropGeometry] release_ready slot=' in logtext(r)), None)
        mode = next((r for r in logs if '[DropGeometry] descent_mode=' in logtext(r)), None)
        checks = [r for r in logs if '[DropGeometry]' in logtext(r) and 'release_xy_check=' in logtext(r)]
        enabled = sorted(set(re.findall(r'release_xy_check=(\w+)', '\n'.join(logtext(r) for r in checks))))
        configured = data['settings'].get('drop_descent_mode')
        observed = re.search(r'descent_mode=(\d+).*?z_speed=([\d.]+)', logtext(mode)) if mode else None
        tolerance_row = ready or descent
        tolerance = re.search(r'xy_tolerance=([\d.]+)', logtext(tolerance_row)) if tolerance_row else None
        run = dict(index=i+1, **start, end=end, outcome='成功ACK' if ack else '失败回执' if terminal else '未记录成功ACK',
                   slot=(call or ack or terminal or {}).get('m', {}).get('payload_slot'),
                   events={'align': start['t'], 'freeze': freeze['t'] if freeze else None,
                           'descent': descent['t'] if descent else None, 'call': call['t'] if call else None,
                           'ack': ack['t'] if ack else None},
                   align_to_ack_s=ack['t']-start['t'] if ack else None,
                   configured_descent_mode=configured,
                   observed_descent_mode=int(observed[1]) if observed else None,
                   observed_z_speed_mps=float(observed[2]) if observed else None,
                   release_xy_check=enabled or ['未记录'],
                   xy_tolerance_m=float(tolerance[1]) if tolerance else None,
                   mode_evidence=[{'t': r['t'], 'text': logtext(r)} for r in (mode, ready) if r],
                   raw_control=raw_channels(rows['raw_setpoint'], descent['t'] if descent else start['t'], call['t'] if call else end))
        geometry = re.search(r'center=\(([-\d.]+),([-\d.]+)\) FC=\(([-\d.]+),([-\d.]+)\)', logtext(freeze)) if freeze else None
        run['frozen_xy'] = [float(v) for v in geometry.groups()] if geometry else None
        if run['slot'] is None and freeze:
            slotmatch = re.search(r'frozen slot=(\d+)', logtext(freeze))
            run['slot'] = int(slotmatch[1]) if slotmatch else None
        runs.append(run)
    return runs


def landing_summary(data):
    rows, settings = data['rows'], data['settings']
    commands = [r for r in rows['command'] if r['m'].get('command') == r['m'].get('LAND', 5)]
    runtime = [r for r in rows['logs'] if '[ExternalLanding]' in logtext(r) and
               re.search(r'descent_started|descend_started|descent_mode=|enter_descend|handoff_request_accepted|handoff mode observed', logtext(r))]
    handoff_runtime = [r for r in rows['handoff'] if isinstance(r['m'], dict) and
                       r['m'].get('stage') in ('REQUESTED', 'OBSERVED') and r['m'].get('decision_seq', 0)]
    auto_land = [r for r in rows['fc'] if r['m'].get('mode') == 'AUTO.LAND']
    starts = [r['t'] for r in commands] + [r['t'] for r in runtime] + [r['t'] for r in handoff_runtime] + [r['t'] for r in auto_land]
    start = min(starts) if starts else None
    transitions = []
    previous = None
    for row in rows['fc']:
        m = row['m']
        signature = (m.get('mode'), m.get('armed'))
        if signature != previous:
            transitions.append({'t': row['t'], 'mode': signature[0], 'armed': signature[1]})
            previous = signature
    landed = None
    if start is not None:
        airborne = False
        for row in rows['extended']:
            if row['m'].get('landed_state') == 2:
                airborne = True
            if row['t'] >= start and airborne and row['m'].get('landed_state') == 1:
                landed = row['t']
                break
    modes = [r for r in transitions if start is not None and r['t'] >= start]
    h = [r for r in rows['handoff'] if start is not None and r['t'] >= start]
    mission = rows['mission'][-1]['m'] if rows['mission'] else {}
    observed_descent_modes = sorted(set(int(m[1]) for r in runtime
                                       for m in [re.search(r'descent_mode=(\d+)', logtext(r))] if m))
    observed_handoff_modes = sorted(set(r['m'].get('mode') for r in h if r['m'].get('stage') == 'OBSERVED' and r['m'].get('mode')))
    return dict(configured_descent_mode=settings.get('landing_descent_mode'),
                configured_descent_speed_mps=settings.get('landing_descent_speed_mps'),
                configured_alignment_mode=settings.get('landing_alignment_mode'),
                configured_handoff_mode=settings.get('landing_handoff_mode'),
                observed_descent_modes=observed_descent_modes, observed_handoff_modes=observed_handoff_modes,
                execution_recorded=start is not None, start_bag_s=start,
                land_command_count=len(commands), observed_transitions=modes,
                fc_transitions=transitions, runtime_evidence=[{'t': r['t'], 'text': logtext(r)} for r in runtime],
                handoff_evidence=h, landed_after_landing_s=landed,
                last_mission_phase=mission.get('phase'),
                raw_control=raw_channels(rows['raw_setpoint'], start, data['duration']) if start is not None else {},
                note='配置与实际模式分别列出；仅落地状态由 IN_AIR 转 ON_GROUND 作为落地证据。POSCTL 切换不能单独证明 H 点降落。')


def geometry_curve(data, run):
    import numpy as np
    errors = []
    warnings = []
    offsets = data['rig'].get('slot_offsets', [])
    slot = run['slot']
    xy = run['frozen_xy']
    if xy and isinstance(slot, int) and 1 <= slot <= len(offsets) and run['events']['freeze'] is not None:
        arm = np.asarray(offsets[slot-1], dtype=float)
        if len(arm) == 2:
            arm = np.r_[arm, 0.]
        mission_frame = data['rig'].get('mission_frame')
        odom = data['rows']['odom']
        for row in odom:
            if not run['events']['freeze'] <= row['t'] <= run['end']:
                continue
            if not mission_frame or row['m'].get('header', {}).get('frame_id') != mission_frame:
                continue
            p = position(row)
            point, q = p.get('position', {}), p.get('orientation', {})
            values = [point.get(k) for k in ('x', 'y')] + [q.get(k) for k in ('x', 'y', 'z', 'w')]
            if not all(finite(v) for v in values) or arm.shape != (3,):
                continue
            quat = np.array(values[2:])
            if abs(np.linalg.norm(quat)-1) > .01:
                continue
            quat /= np.linalg.norm(quat)
            vector = quat[:3]
            lever = arm + 2*quat[3]*np.cross(vector, arm) + 2*np.cross(vector, np.cross(vector, arm))
            fc_error = np.array(values[:2])-xy[2:]
            outlet_error = np.array(values[:2])+lever[:2]-xy[:2]
            errors.append([row['t'], max(np.linalg.norm(fc_error), np.linalg.norm(outlet_error))])
    logged = []
    for row in data['rows']['logs']:
        if not run['t'] <= row['t'] <= run['end']:
            continue
        text = logtext(row)
        wait = re.search(r'\[DropGeometry\] waiting: xy=([\d.]+)', text)
        ready = re.search(r'fc_xy_error=([\d.]+).*?outlet_xy_error=([\d.]+)', text)
        if wait:
            logged.append([row['t'], float(wait[1])])
        elif '[DropGeometry]' in text and ready:
            logged.append([row['t'], max(float(ready[1]), float(ready[2]))])
    if errors and logged:
        times = [r[0] for r in errors]
        differences = []
        for t, error in logged:
            i = bisect.bisect_right(times, t)-1
            if i >= 0 and t-times[i] <= .2:
                differences.append(abs(errors[i][1]-error))
        median = float(np.median(differences)) if differences else None
        run['geometry_check_median_m'] = median
        if median is not None and median > .003:
            warnings.append('重建误差与控制器日志不一致，使用日志离散误差点')
            errors = []
    source = '姿态与冻结XY重建（XY精度0.001m）' if errors else '控制器日志离散样本' if logged else '缺少冻结目标、偏移或同坐标系运动数据'
    return errors or logged, source, warnings


def plot_outputs(data, runs, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.lines import Line2D
    plt.rcParams.update({'font.family': ['Microsoft YaHei', 'Noto Sans CJK SC', 'DejaVu Sans'],
                         'axes.unicode_minus': False, 'font.size': 10})
    # Select one available font to avoid repeated fallback warnings on Linux.
    available = {f.name for f in font_manager.fontManager.ttflist}
    plt.rcParams['font.family'] = [next((f for f in ('Microsoft YaHei', 'Noto Sans CJK SC', 'SimHei') if f in available), 'DejaVu Sans')]
    rows, ground = data['rows'], data['ground_z']
    images = []
    styles = [('freeze', '冻结靶心', '#c8a23b'), ('descent', '开始下降', '#97528f'),
              ('call', '舵机调用开始', '#d6545d'), ('ack', '成功ACK', '#333333')]

    def height(ax, start, end, show_release=True):
        feedback = rows['pose'] or rows['fc_pose']
        command = rows['setpoint'] or rows['fc_setpoint']
        mission_frame = data['rig'].get('mission_frame')
        for series, label, color in [(feedback, '实际高度', '#377fa5'), (command, '指令高度', '#e18a3a')]:
            points = []
            for r in series:
                z = position(r).get('position', {}).get('z')
                frame = r['m'].get('header', {}).get('frame_id')
                if start <= r['t'] <= end and finite(z):
                    if ground is not None and frame != mission_frame:
                        continue
                    points.append((r['t']-start, z-(ground if ground is not None else 0)))
            if points:
                ax.plot(*zip(*points), color=color, lw=1.3, label=label)
        if ground is not None and runs and show_release:
            low, high, goal = [data['settings'].get(k) for k in ('release_min_agl', 'release_max_agl', 'drop_agl')]
            if finite(low) and finite(high):
                ax.axhspan(low, high, color='#4c7935', alpha=.13)
            if finite(goal):
                ax.axhline(goal, color='#4c7935', ls=':', lw=1)
        if ax.lines:
            ax.legend(fontsize=9, loc='best')
        else:
            ax.text(.5, .5, '缺少同坐标系高度记录', transform=ax.transAxes, ha='center')
        ax.set_ylabel('AGL（m）' if ground is not None else '本地Z（m）；无地面参考')

    def permission(ax, start, end):
        series = rows['authorization'] or rows['permission']
        values = [(r['t'], int(r['m']['permitted'])) for r in series if isinstance(r['m'].get('permitted'), bool)]
        prev = [r for r in values if r[0] < start]
        values = ([ (start, prev[-1][1])] if prev else []) + [r for r in values if start <= r[0] <= end]
        if values:
            ax.step([r[0]-start for r in values], [r[1] for r in values], where='post', color='#377fa5', label='投递授权 permitted')
            ax.legend(loc='upper left', fontsize=9)
        else:
            ax.text(.5, .5, '未记录投递授权', transform=ax.transAxes, ha='center')
        ax.set_ylim(-.07, 1.12)
        ax.set_yticks([0, 1], labels=['0 禁止', '1 允许'])
        ax.set_ylabel('投递许可')

    pages = [runs[i:i+3] for i in range(0, len(runs), 3)] if runs else [[]]
    for page_index, page in enumerate(pages):
        count = max(1, len(page))
        fig, axes = plt.subplots(3, count, squeeze=False, sharex='col', figsize=(6.4*count if page else 12, 10.5))
        for column, run in enumerate(page or [None]):
            start, end = (run['t'], run['end']) if run else (0, data['duration'])
            height(axes[0, column], start, end)
            title = f"投递 {run['index']} · {NAMES.get(run['target_class'], run['target_class'])}\n" if run else '记录概览：没有投递 ALIGN\n'
            title += f"ALIGN → ACK {run['align_to_ack_s']:.2f} 秒" if run and run['align_to_ack_s'] is not None else run['outcome'] if run else '未记录快递投递阶段'
            axes[0, column].set_title(title, fontsize=12, pad=12)
            ax = axes[1, column]
            if run:
                curve, source, warnings = geometry_curve(data, run)
                run['horizontal_source'], run['warnings'] = source, warnings
                if curve:
                    ax.plot([r[0]-start for r in curve], [r[1]*100 for r in curve], lw=1.3, color='#377fa5',
                            marker='.' if source.startswith('控制器') else None, label='max(飞控中心误差, 投口误差)')
                else:
                    ax.text(.5, .5, '水平误差不可计算\n'+source, transform=ax.transAxes, ha='center', fontsize=9)
                tol = run['xy_tolerance_m']
                if tol is not None:
                    disabled = 'disabled' in run['release_xy_check']
                    ax.axhline(tol*100, color='#4c7935', ls='--', label=f'XY参考 {tol*100:g} cm'+('（释放检查关闭）' if disabled else '（启用状态见报告）'))
                if ax.lines:
                    ax.legend(fontsize=8, loc='best')
                with (out/f"delivery_{run['index']:02d}_horizontal.csv").open('w', encoding='utf-8-sig', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(['bag_seconds', 'seconds_since_align', 'max_horizontal_error_m'])
                    writer.writerows([[t, t-start, value] for t, value in curve])
            else:
                ax.text(.5, .5, '无 ALIGN 或冻结靶心记录\n水平投递误差不可计算', transform=ax.transAxes, ha='center', fontsize=14, color='#586471')
                ax.set_yticks([])
            ax.set_ylabel('水平位置误差（cm）')
            permission(axes[2, column], start, end)
            for ax in axes[:, column]:
                if run:
                    for key, label, color in styles:
                        if run['events'][key] is not None:
                            ax.axvline(run['events'][key]-start, color=color, ls=':', lw=1.2)
                ax.grid(alpha=.22)
                ax.set_xlim(-.3 if run else 0, max(end-start, .1)+(.3 if run else 0))
            axes[2, column].set_xlabel('距接受 ALIGN 的时间（秒）' if run else '距 bag 开始（秒）；没有 ALIGN 起点')
        if page:
            fig.legend(handles=[Line2D([0], [0], color=c, ls=':', label=l) for _, l, c in styles], loc='upper center', bbox_to_anchor=(.5, .95), ncol=4, frameon=False)
        fig.suptitle(Path(data['source']).name+'：高度 · 水平误差 · 投递许可', fontsize=16, y=.99)
        fig.text(.06, .025, '高度采用日志地面参考；没有地面参考时标为本地Z。水平误差从冻结目标后开始，缺失记录留空。\n'
                 'ACK 表示舵机成功应答；配置值与实际执行模式请查看分析报告。曲线不做平滑，各列时间轴独立。', fontsize=9)
        fig.tight_layout(rect=(.01, .085, .995, .92 if page else .95))
        stem = 'delivery_curves' if page_index == 0 else f'delivery_curves_{page_index+1}'
        fig.savefig(out/(stem+'.png'), dpi=180, facecolor='white')
        plt.close(fig)
        images.append(stem+'.png')

    landing = data['landing']
    if landing['execution_recorded']:
        start = landing['start_bag_s']
        fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
        height(axes[0], start, data['duration'], show_release=False)
        series = [r for r in rows['fc'] if r['t'] >= start]
        modes = list(dict.fromkeys(r['m'].get('mode', '未知') for r in series))
        if series:
            axes[1].step([r['t']-start for r in series], [modes.index(r['m'].get('mode', '未知')) for r in series], where='post', marker='.', markersize=5)
            axes[1].set_yticks(range(len(modes)), labels=modes)
        extended = [r for r in rows['extended'] if r['t'] >= start]
        if extended:
            axes[2].step([r['t']-start for r in extended], [r['m'].get('landed_state', 0) for r in extended], where='post', marker='.', markersize=5)
        axes[1].set_ylabel('实际飞控模式')
        axes[2].set_ylabel('落地状态')
        axes[2].set_yticks([0, 1, 2, 3, 4], labels=['未知', 'ON_GROUND', 'IN_AIR', 'TAKEOFF', 'LANDING'])
        axes[2].set_xlabel('距首次降落执行记录（秒）')
        for ax in axes:
            ax.grid(alpha=.22)
        fig.suptitle('降落阶段：高度、实际飞控模式与落地状态', fontsize=16)
        fig.tight_layout(rect=(0, 0, 1, .95))
        fig.savefig(out/'landing_curves.png', dpi=180, facecolor='white')
        plt.close(fig)
        images.append('landing_curves.png')
    return images


def write_report(data, runs, images, out):
    landing = data['landing']
    summary = dict(source=data['source'], start=data['start'], duration=data['duration'], ground_z=data['ground_z'],
                   configured_settings=data['settings'], missing_topics=data['missing'], deliveries=runs,
                   success_count=sum(r['outcome'] == '成功ACK' for r in runs), landing=landing, images=images)
    (out/'analysis_summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    start = datetime.fromtimestamp(data['start'], timezone(timedelta(hours=8)))
    lines = ['# 日志投递与降落分析', '', f"文件：{data['source']}", f"北京时间：{start.isoformat(timespec='milliseconds')}，时长 {data['duration']:.3f} 秒。", '',
             f"投递 ALIGN {len(runs)} 次，成功 ACK {summary['success_count']} 次。", '', '## 投递模式与结果', '']
    for run in runs:
        duration = f"{run['align_to_ack_s']:.3f} 秒" if run['align_to_ack_s'] is not None else '未记录'
        lines += [f"### 投递 {run['index']} · {run['target_class']}",
                  f"- 结果：{run['outcome']}；ALIGN 至 ACK：{duration}",
                  f"- 配置下降模式：{run['configured_descent_mode']}；实际日志下降模式：{run['observed_descent_mode']}；实际记录 z_speed：{run['observed_z_speed_mps']} m/s",
                  f"- 释放 XY 检查：{', '.join(run['release_xy_check'])}；配置参考容差：{run['xy_tolerance_m']} m",
                  f"- 实际 setpoint 掩码识别的 Z 控制通道：{json.dumps(run['raw_control'], ensure_ascii=False)}",
                  f"- 水平误差来源：{run.get('horizontal_source', '未知')}", '']
        for evidence in run['mode_evidence']:
            lines += [f"  {evidence['t']:.3f}s：{evidence['text']}"]
    if not runs:
        lines += ['未记录投递 ALIGN，无法绘制三个目标的投递阶段；输出图为已有高度与授权记录的概览。', '']
    lines += ['## 降落模式', '', f"- 配置下降模式：{landing['configured_descent_mode']}；配置下降速度：{landing['configured_descent_speed_mps']} m/s",
              f"- 配置对准模式：{landing['configured_alignment_mode']}；配置交接模式：{landing['configured_handoff_mode']}",
              f"- 实际日志下降模式：{landing['observed_descent_modes'] or '未记录'}；交接 OBSERVED 模式：{landing['observed_handoff_modes'] or '未记录'}",
              f"- 实际降落阶段：{'已记录' if landing['execution_recorded'] else '未记录（配置不能作为执行证据）'}",
              f"- LAND 指令数量：{landing['land_command_count']}",
              f"- 降落后的实际模式切换：{json.dumps(landing['observed_transitions'], ensure_ascii=False)}",
              f"- 全程实际飞控模式/解锁状态变化：{json.dumps(landing['fc_transitions'], ensure_ascii=False)}",
              f"- 降落阶段后落地状态确认时刻：{landing['landed_after_landing_s'] if landing['landed_after_landing_s'] is not None else '未记录'}",
              f"- 末尾任务阶段：{landing['last_mission_phase']}", '', '### 降落实时日志证据', '']
    lines += [f"- {r['t']:.3f}s：{r['text']}" for r in landing['runtime_evidence']]
    lines += ['', '## 数据口径', '', 'None / 未记录代表证据不足，不代表关闭或模式0。下降模式编号保持日志原值，不将编号擅自解释为某种控制算法。',
              'Z控制通道按 MAVLink type_mask 的位置Z与速度Z忽略位判断；位置+速度通道不等于纯速度控制。',
              '图中高度与目标仅使用同坐标系数据；固定地面参考与瞬时离地测距不同。',
              '授权曲线不是全部释放门控，舵机成功回执也不能独立证明物品已触地。',
              '只有 LAND 指令或明确 ExternalLanding 执行日志可开启降落阶段，启动配置与 WayPointDetectDone 的 Landing completed 不计为降落完成。',
              landing['note'], '', '缺失话题：'+', '.join(data['missing'])]
    (out/'ANALYSIS.md').write_text('\n'.join(lines), encoding='utf-8')
    body = '<h1>日志投递与降落分析</h1>' + ''.join(f'<p><a href="{html.escape(image)}"><img src="{html.escape(image)}" style="max-width:100%"></a></p>' for image in images)
    body += '<pre style="white-space:pre-wrap">'+html.escape('\n'.join(lines))+'</pre>'
    (out/'analysis_report.html').write_text('<!doctype html><html lang="zh"><meta charset="utf-8"><title>日志分析</title><body style="font-family:Microsoft YaHei,sans-serif;max-width:1600px;margin:24px auto;padding:16px">'+body+'</body></html>', encoding='utf-8')


def analyze(bag, out, reader='auto', topics=None):
    bag, out = Path(bag).resolve(), Path(out).resolve()
    if not bag.is_file():
        raise ValueError(f'日志文件不存在：{bag}')
    if out == bag or (out.exists() and not out.is_dir()):
        raise ValueError('输出必须是独立目录')
    print('Stage: analyze', flush=True)
    data = read_data(bag, reader, topics)
    data['ground_z'], data['settings'], data['rig'] = ground_info(data['rows'])
    runs = extract_deliveries(data)
    data['landing'] = landing_summary(data)
    out.mkdir(parents=True, exist_ok=True)
    print(f'识别 ALIGN {len(runs)} 次，降落执行记录：{data["landing"]["execution_recorded"]}', flush=True)
    print('Stage: plot', flush=True)
    images = plot_outputs(data, runs, out)
    write_report(data, runs, images, out)
    print(f'Done. Open: {out / "analysis_report.html"}', flush=True)
    return data, runs


def main():
    parser = argparse.ArgumentParser(description='分析 ROS1 bag 的投递与降落，导出曲线、模式证据及报告；不生成视频。')
    parser.add_argument('bag')
    parser.add_argument('out')
    parser.add_argument('--reader', choices=['auto', 'rosbags', 'ros1'], default='auto')
    parser.add_argument('--topics', help='分析话题映射JSON；键名见 ANALYSIS_TOPICS')
    args = parser.parse_args()
    try:
        analyze(args.bag, args.out, args.reader, args.topics)
    except Exception as exc:
        print(f'日志分析失败：{exc}', file=sys.stderr)
        if isinstance(exc, ImportError):
            print('缺少依赖时，请用当前解释器运行 python -m pip install -r requirements.txt。', file=sys.stderr)
        elif any(word in str(exc).lower() for word in ('indexed', 'reindex', 'truncated', 'corrupt')):
            print('无索引或损坏的 bag 请先修复副本后再分析。原文件未修改。', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())

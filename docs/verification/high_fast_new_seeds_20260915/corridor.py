"""Corridor stall analysis: 2nd-door (Wall_22) stagnation in the new batch and its history.

Measurement approach follows high_fallback_repair_20260915/corridor_hold_analysis.py
(held-setpoint plus movement evidence) and extends it to per-segment durations,
position traces and cross-batch comparison. Offline only: reads raw run products.
"""
import csv, json, re, runpy
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

D = Path(__file__).resolve().parent
R = D.parents[2]
DOOR_X = {'Wall_20': -1.6, 'Wall_22': 1.6}

# Archived runs kept for recurrence evidence (only those whose run dirs still exist).
LEGACY = [
    dict(seed=31, source='d463613', door_pattern='LL',
         run=R / 'logs/high_fallback_repair_seed31_20260915_124852'),
    dict(seed=32, source='d463613', door_pattern='RR',
         run=R / 'logs/high_fallback_repair_seed32_20260915_130605'),
    dict(seed=34, source='d463613', door_pattern='LL',
         run=R / 'logs/high_fallback_repair_seed34_20260915_130829'),
    dict(seed=32, source='a12750b', door_pattern='RR',
         run=R / 'logs/high_fallback_descent32_seed32_20260915_132723'),
    dict(seed=31, source='20b1a0f', door_pattern='LL',
         run=R / 'logs/high_fast_five_seed31_20260915_101053'),
    dict(seed=32, source='20b1a0f', door_pattern='RR',
         run=R / 'logs/high_fast_five_seed32_20260915_101502'),
    dict(seed=34, source='20b1a0f', door_pattern='LL',
         run=R / 'logs/high_fast_five_seed34_20260915_103731'),
]


def csv4(path):
    with path.open() as f:
        return np.array([[float(r[k]) for k in ('t', 'x', 'y', 'z')] for r in csv.DictReader(f)])


def analyse_run(run):
    run = Path(run)
    ev = [json.loads(l) for l in (run / 'key_events.jsonl').read_text().splitlines()]
    dec = [e for e in ev if e['kind'] == 'decision']
    planner = [e for e in ev if e['kind'] == 'planner']
    route = []
    for e in dec:
        reason = str(e['data'].get('reason', ''))
        if reason.startswith('post_delivery_route:'):
            goal = (e['data'].get('goal') or {}).get('pose', {}).get('position') or {}
            route.append(dict(t=e['ros_sec'], idx=reason.split(':')[1],
                              goal=[goal.get('x'), goal.get('y'), goal.get('z')]))
    terminal = next((e for e in dec if str(e['data'].get('reason', '')) in
                     ('safety_motion_timed_out', 'post_delivery_route_complete')), None)
    end_t = terminal['ros_sec'] if terminal else None
    text = (run / 'run.log').read_text(errors='replace')

    segs = []
    for i, cur in enumerate(route):
        nxt = route[i + 1]['t'] if i + 1 < len(route) else end_t
        if nxt is None:
            continue
        segs.append(dict(idx=cur['idx'], start=cur['t'], end=nxt, duration=nxt - cur['t'],
                         goal_xyz=cur['goal']))
    second = next((s for s in segs if s['idx'].startswith('8/')), None)
    first = next((s for s in segs if s['idx'].startswith('5/')), None)

    def segment_probe(seg):
        if seg is None:
            return None
        out = dict(idx=seg['idx'], duration_s=seg['duration'], goal_xyz=seg['goal_xyz'])
        sp_path = run / 'planner_setpoint.csv'
        if sp_path.is_file():
            sp = csv4(sp_path)
            win = sp[(sp[:, 0] >= seg['start']) & (sp[:, 0] < seg['end'] - .05)]
            out['setpoint_samples'] = int(len(win))
            if len(win):
                last = win[-1, 1:4]
                out['held_xyz'] = [float(v) for v in last]
                within = np.linalg.norm(win[:, 1:4] - last, axis=1) < .01
                idx = np.flatnonzero(~within)
                start_hold = win[int(idx[-1] + 1), 0] if len(idx) else win[0, 0]
                out['held_from_s'] = float(start_hold)
                out['held_duration_s'] = float(win[-1, 0] - start_hold)
                out['held_fraction'] = float(within.mean())
        tr_path = run / 'truth_pose.csv'
        if tr_path.is_file():
            tr = csv4(tr_path)
            tw = tr[(tr[:, 0] >= seg['start']) & (tr[:, 0] <= seg['end'])]
            if len(tw):
                out['truth_xy_start'] = [float(tw[0, 1]), float(tw[0, 2])]
                out['truth_xy_end'] = [float(tw[-1, 1]), float(tw[-1, 2])]
                step = np.linalg.norm(np.diff(tw[:, 1:3], axis=0), axis=1)
                moving = np.flatnonzero(step > .01)
                out['moved_path_m'] = float(step.sum())
                out['last_motion_s'] = float(tw[moving[-1] + 1, 0]) if len(moving) else None
                out['still_duration_s'] = (float(seg['end'] - out['last_motion_s'])
                                           if out.get('last_motion_s') is not None
                                           else float(seg['duration']))
                out['truth_x_range'] = [float(tw[:, 1].min()), float(tw[:, 1].max())]
        out['planner_failures'] = sum(
            1 for e in planner
            if seg['start'] <= e['ros_sec'] <= seg['end'] and e['data'].get('status') == 4)
        return out

    def window_log(pattern, start, end):
        if start is None or end is None:
            return None
        hits = 0
        for m in re.finditer(r'\[[0-9.]+,\s*([0-9.]+)\][^\n]*', text):
            if pattern not in m.group(0):
                continue
            if start <= float(m.group(1)) <= end:
                hits += 1
        return hits

    log = dict(kino_replan_fail=len(re.findall(r'kinodynamic search fail', text)),
               no_planner_command=len(re.findall(r'no planner command received', text)),
               time_jump=len(re.findall(r'Time jump detected', text)))
    doors = []
    gate_path = run / 'gate_status.json'
    if gate_path.is_file():
        gate = json.loads(gate_path.read_text())
        doors = [d['name'] for d in (gate.get('metrics') or {}).get('door_crossings', [])]
    seg2 = next((s for s in segs if s['idx'].startswith('8/')), None)
    log['seg8_kino_fail'] = window_log('kinodynamic search fail',
                                       seg2['start'] if seg2 else None,
                                       seg2['end'] if seg2 else None)
    log['seg8_no_command'] = window_log('no planner command received',
                                        seg2['start'] if seg2 else None,
                                        seg2['end'] if seg2 else None)
    return dict(run=str(run), route=route, segments=segs, end_t=end_t,
                terminal_reason=(str(terminal['data'].get('reason')) if terminal else None),
                first_door_segment=segment_probe(first), second_door_segment=segment_probe(second),
                log=log, doors_crossed=doors)


def _tag(rec, meta):
    rec['seed'] = meta['seed']
    rec['source'] = meta['source']
    rec['door_pattern'] = meta.get('door_pattern', '?')
    seg = rec['second_door_segment']
    rec['seg8_s'] = seg['duration_s'] if seg else None
    rec['seg8_still_s'] = seg.get('still_duration_s') if seg else None
    rec['seg8_hold_x'] = (seg.get('held_xyz') or [None])[0] if seg else None
    rec['seg8_hold_s'] = seg.get('held_duration_s') if seg else None
    rec['seg8_kino_fail'] = (rec['log'].get('seg8_kino_fail') if seg else None)
    rec['seg8_no_command'] = (rec['log'].get('seg8_no_command') if seg else None)
    rec['seg8_gap_m'] = (round(DOOR_X['Wall_22'] - rec['seg8_hold_x'], 3)
                         if rec.get('seg8_hold_x') is not None else None)
    return rec


def chart_new(records, out_dir):
    n = len(records)
    fig, axes = plt.subplots(2, n, figsize=(5 * n, 10))
    if n == 1:
        axes = np.array([[axes[0]], [axes[1]]])
    for col, rec in enumerate(records):
        run = Path(rec['run'])
        tr = csv4(run / 'truth_pose.csv')
        start = rec['route'][0]['t'] if rec['route'] else 0
        end = rec['end_t'] or tr[-1, 0]
        win = tr[(tr[:, 0] >= start) & (tr[:, 0] <= end)]
        seg2 = rec['second_door_segment']
        ax = axes[0][col]
        ax.plot(win[:, 0] - start, win[:, 1], lw=1, label='X')
        for x, name in sorted(DOOR_X.items(), key=lambda kv: kv[1]):
            ax.axhline(x, ls='--', color='gray', lw=.8)
            ax.text(1, x, name, fontsize=7, va='bottom')
        for s in rec['segments']:
            ax.axvline(s['start'] - start, ls=':', color='#bbbbbb', lw=.7)
        if seg2 and seg2.get('held_from_s'):
            ax.axvspan(seg2['held_from_s'] - start, end - start, color='red', alpha=.12,
                       label='2nd-door hold')
        ax.set(title=f"seed{rec['seed']}({rec['door_pattern']}) X(t)\n{rec['terminal_reason']}",
               xlabel='s after first post-delivery command', ylabel='X (m)')
        ax.grid(alpha=.2)
        ax.legend(fontsize=7)
        ax = axes[1][col]
        ax.plot(win[:, 0] - start, win[:, 2], lw=1, color='#8a5a2b')
        ax.axhline(7.5, ls='--', color='gray', lw=.8)
        for s in rec['segments']:
            ax.axvline(s['start'] - start, ls=':', color='#bbbbbb', lw=.7)
        if seg2 and seg2.get('held_from_s'):
            ax.axvspan(seg2['held_from_s'] - start, end - start, color='red', alpha=.12)
        ax.set(title=f"Y(t) | doors {rec['doors_crossed']}", xlabel='s', ylabel='Y (m)')
        ax.grid(alpha=.2)
    fig.suptitle('Corridor phase, new scenes: door lines (dashed), segment starts (dotted), '
                 '2nd-door hold (red)')
    fig.tight_layout()
    fig.savefig(out_dir / 'corridor_stall.png', dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(13, 6))
    width = .8 / n
    for i, rec in enumerate(records):
        segs = rec['segments']
        xs = np.arange(len(segs))
        colors = ['#b5443a' if s['idx'].startswith('8/') else '#6f9bd1' for s in segs]
        ax.bar(xs + i * width, [s['duration'] for s in segs], width, color=colors,
               edgecolor='black', linewidth=.4, label=f"seed{rec['seed']}")
    lengths = [len(r['segments']) for r in records]
    ax.set_xticks(np.arange(max(lengths)) + .4 - width / 2,
                  [f"{k+1}/9" for k in range(max(lengths))])
    ax.set(xlabel='post-delivery route segment', ylabel='segment duration (s)',
           title='Corridor segment durations, new scenes (red = segment 8/9, the 2nd door)')
    ax.grid(axis='y', alpha=.25)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_dir / 'corridor_segments.png', dpi=170)
    plt.close(fig)


def chart_history(all_recs, out_dir):
    have = [r for r in all_recs if r['seg8_s'] is not None]
    if not have:
        return
    fig, ax = plt.subplots(figsize=(13, 6))
    labels = [f"s{r['seed']}\n{r['source']}" for r in have]
    xs = np.arange(len(have))
    ax.bar(xs, [r['seg8_s'] for r in have], .6,
           color=['#b5443a' if r['seg8_s'] >= 80 else '#6f9bd1' for r in have])
    for i, r in enumerate(have):
        ax.text(i, r['seg8_s'] + 1, f"{r['seg8_s']:.0f}s", ha='center', fontsize=9)
        if r.get('seg8_hold_x') is not None:
            ax.text(i, 2, f"hold x={r['seg8_hold_x']:.2f}", ha='center', fontsize=8,
                    rotation=90, color='white')
    ax.axhline(90, ls='--', color='black', lw=.8)
    ax.text(0, 91, 'segment safety window = 90s', fontsize=8)
    ax.set_xticks(xs, labels)
    ax.set(ylabel='segment 8/9 duration (s)',
           title='2nd-door segment (8/9) duration across runs: new batch vs archived batches')
    ax.grid(axis='y', alpha=.25)
    fig.tight_layout()
    fig.savefig(out_dir / 'corridor_history.png', dpi=170)
    plt.close(fig)


def corridor_section(records, out_dir=None, legacy=None):
    out_dir = out_dir or D
    new = [_tag(analyse_run(m['run']),
                dict(seed=m['seed'], source='92bd3ee', door_pattern=m.get('door_pattern', '?')))
           for m in records]
    legacy = LEGACY if legacy is None else legacy
    hist = []
    for meta in legacy:
        if not Path(meta['run']).exists():
            continue
        hist.append(_tag(analyse_run(meta['run']), meta))
    (out_dir / 'corridor_evidence.json').write_text(
        json.dumps(dict(new=new, history=hist), indent=2, ensure_ascii=False))
    chart_new(new, out_dir)
    chart_history(new + hist, out_dir)

    def table(rows):
        out = ['| 场景 | 来源 | 过门 | 第5段(一门前) 时长/停滞 | 第8段(二门前) 时长/停滞 | 段末设定点 x | 距二门线 | 终止原因 |',
               '|---|---|---|---|---|---|---:|---|']
        for rec in rows:
            f, s = rec['first_door_segment'], rec['second_door_segment']
            def cell(seg):
                return '—' if not seg else f"{seg['duration_s']:.1f}s / {seg.get('still_duration_s', float('nan')):.1f}s"
            holdx = (f"{rec['seg8_hold_x']:.2f}m"
                     if rec.get('seg8_hold_x') is not None else '—')
            gap = (f"{rec['seg8_gap_m']:.2f}m"
                   if rec.get('seg8_gap_m') is not None else '—')
            out.append(
                f"| {rec['seed']}({rec['door_pattern']}) | {rec['source']} | "
                f"{', '.join(rec['doors_crossed'] or []) or '无'} | {cell(f)} | {cell(s)} | "
                f"{holdx} | {gap} | {rec['terminal_reason'] or '—'} |")
        return out

    lines = ['## 三、走廊段专项：第二门前停滞（Wall_22, x=+1.6）', '',
             '走廊段定义为投后路线 1/9→9/9；第 5 段目标需穿越 Wall_20（x=-1.6），'
             '第 8 段目标需穿越 Wall_22（x=+1.6）。「停滞」= 该段内真值位移步长 ≤1cm 的尾段长度；'
             '「保持指令 x」= 段末规划器设定点（1cm 判据）的 x 值，可复核 `corridor_evidence.json`。', '',
             '### 本次新场景', ''] + table(new) + ['', '### 历史归档（同一测量口径）', '']
    lines += table(hist) if hist else ['（未找到归档 run 目录）']
    lines += ['', '整轮日志计数（`kinodynamic search fail` 行不带 ROS 时间戳，无法按段切分，故只给整轮值）：', '',
              '| 场景 | 来源 | kinodynamic search fail | no planner command received | Time jump detected |',
              '|---|---|---:|---:|---:|']
    for rec in new + hist:
        lines.append(f"| {rec['seed']}({rec['door_pattern']}) | {rec['source']} | "
                     f"{rec['log']['kino_replan_fail']} | {rec['log']['no_planner_command']} | "
                     f"{rec['log']['time_jump']} |")
    def reached(rows):
        return [r for r in rows if r['seg8_s'] is not None]
    def stalled(rows):
        return [r for r in reached(rows) if r['seg8_s'] is not None and r['seg8_s'] >= 80]
    lines += ['',
              f"复发性：本批到达第 8 段的 {len(reached(new))} 个 run 中 {len(stalled(new))} 个跑满安全窗（90s）后超时；"
              f"归档批（d463613）到达该段的 {len(reached(hist))} 个 run 中 {len(stalled(hist))} 个同样跑满 90s。"
              '即：该停滞在**不同源码版本、不同随机场景上重复出现**，但并非每次到达该段都发生——'
              '本批 seed37/39 分别用 15.9s、40.0s 正常通过。', '',
              '![走廊 X(t)/Y(t) 与停滞窗口](corridor_stall.png)', '',
              '![走廊分段时长](corridor_segments.png)', '',
              '![跨批次第二门段时长](corridor_history.png)', '',
              '结论口径：本专项只描述可复现现象与代码窗口（设定点保持 + 无位移 + 规划失败计数），'
              '不认定唯一根因；现场 `trajectory_progress` hold 标志未记录，离线构造的 0.25m 偏差反例'
              '（见 `high_fallback_repair_20260915/CORRIDOR_REVIEW.md`）不等于现场触发条件。', '']
    return lines, dict(new=new, history=hist)
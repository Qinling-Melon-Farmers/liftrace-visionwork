"""New-scene (36-40) robustness analysis: takeoff-search-delivery loop first, tail separate.

Reuses the existing chart pipeline: fast_full_random_20260914/analyze_fast.py,
which itself reuses high_view_full_20260914/analyze.py (base). Offline only:
never starts ROS, never alters raw run products.

The combined section additionally folds in the archived 31-35 high-view metrics
(different code revision) to give a 10-scenario loop view; the revision boundary
is stated explicitly and never blurred.
"""
import json, runpy
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import yaml

D = Path(__file__).resolve().parent
R = D.parents[2]
NS = runpy.run_path(str(R / 'docs/verification/fast_full_random_20260914/analyze_fast.py'))
base = NS['base']
BATCH = R / 'logs/high_fast_new_seeds_20260915_batch/matrix.json'
ARCHIVED = R / 'docs/verification/high_fast_five_20260915/metrics.json'
ARCHIVED_SOURCE = '20b1a0f'
ARCHIVED_PATTERNS = {31: 'LL', 32: 'RR', 33: 'LR', 34: 'LL', 35: 'RR'}


def fmt(x):
    return '—' if x is None else f'{x:.2f}'


def loop_metrics(m):
    """Front-loop (takeoff -> search -> 3 deliveries) milestones, tail excluded."""
    final = m.get('high_view_final') or {}
    events = {e.get('stage'): e.get('time') for e in final.get('events', [])}
    deliveries = [e for e in final.get('events', []) if e.get('stage') == 'DELIVERY']
    timeouts = [e for e in final.get('events', []) if e.get('stage') == 'REVISIT_TIMEOUT']
    hints = sorted((final.get('first_hint_ready') or {}).keys())
    return dict(
        armed_s=m.get('armed_ros_s'), airborne_s=m.get('airborne_ros_s'),
        survey_interrupt_s=events.get('SURVEY_INTERRUPTED_TOP3'),
        descend_s=events.get('DESCEND'), catalog_entries=final.get('catalog_entries'),
        hints_ready=hints, required_classes=final.get('required_classes'),
        reacquisitions=len(final.get('reacquisitions', []) or []),
        deliveries=len(deliveries), deliveries_via_event=[e.get('target') for e in deliveries],
        revisit_timeouts=len(timeouts), releases=len(m.get('releases', []) or []),
        commits=len(m.get('commit_times', []) or []), first_commit_s=m.get('first_commit_s'),
        third_commit_s=m.get('third_commit_s'), loop_ok=len(m.get('commit_times', []) or []) == 3,
        terminal_stage=final.get('stage'), loop_failure=final.get('failure') or '')


def tail_metrics(run, m):
    gm = m.get('gate_metrics') or {}
    gate = json.loads((Path(run) / 'gate_status.json').read_text())
    checks = gate.get('checks', {})
    return dict(doors=len(gm.get('door_crossings', []) or []),
                door_names=[d.get('name') for d in gm.get('door_crossings', []) or []],
                post_route_done=gm.get('post_delivery_return_success_count'),
                post_route_size=gm.get('post_delivery_route_size'),
                height_violations=gm.get('height_violations'),
                boundary_violations=gm.get('boundary_violations'),
                land_ok=checks.get('land_success'), disarmed=checks.get('final_vehicle_disarmed'),
                mission_s=m.get('completed_mission_s'), status=m.get('status'), reason=m.get('reason'))


def wall_of(pairs):
    """First wall link mentioned in normalized contact pairs, else '?'."""
    for item in pairs:
        head = item.split(' vs ')[0]
        if head.startswith('Wall'):
            return head
    return '?'


def contact_events(run):
    """Raw gazebo contact pairs for this run (wall names are the useful part)."""
    path = Path(run) / 'gazebo_contact_status.json'
    if not path.is_file():
        return []
    data = json.loads(path.read_text())
    out = []
    for e in data.get('events', []):
        pairs = [' vs '.join(p.split('::')[-2:]) if '::' in p else p
                 for p in (e.get('pairs') or [[]])[0]]
        out.append(dict(t=e.get('ros_stamp'), pairs=pairs))
    return out


def combined_section(new_records, out_dir=None):
    """Fold archived 31-35 high-view metrics into a 10-scenario loop view."""
    out_dir = out_dir or D
    rows = []
    if ARCHIVED.is_file():
        for m in json.loads(ARCHIVED.read_text()):
            if m.get('label') != 'fast_high':
                continue
            v = m.get('loop') or loop_metrics(m)
            rows.append(dict(group='archived', source=ARCHIVED_SOURCE, seed=m['seed'],
                             pattern=ARCHIVED_PATTERNS.get(m['seed'], '?'), status=m.get('status'),
                             reason=m.get('reason'), collisions=m.get('collisions'), **v))
    for m in new_records:
        v = m.get('loop') or loop_metrics(m)
        rows.append(dict(group='new', source='92bd3ee', seed=m['seed'],
                         pattern=m.get('door_pattern', '?'), status=m.get('status'),
                         reason=m.get('reason'), collisions=m.get('collisions'), **v))
    rows.sort(key=lambda r: (r['group'], r['seed']))
    labels = [f"{r['seed']}{r['pattern']}" for r in rows]
    colors = ['#3f8f4f' if r['loop_ok'] else '#b5443a' for r in rows]

    fig, axes = plt.subplots(1, 2, figsize=(19, 6))
    ax = axes[0]
    ax.bar(range(len(rows)), [r['deliveries'] for r in rows], .6, color=colors)
    for i, r in enumerate(rows):
        ax.text(i, r['deliveries'] + .05, str(r['deliveries']), ha='center', fontsize=9)
    ax.set_xticks(range(len(rows)), labels, rotation=45)
    ax.set(ylabel='deliveries completed (0-3)',
           title='Front loop outcome by scenario (archived 31-35 | new 36-40)')
    ax.axvline(4.5, ls='--', color='gray')
    ax.text(2, 2.75, f'archived @{ARCHIVED_SOURCE}', ha='center', fontsize=9, color='gray')
    ax.text(7, 2.75, 'new @92bd3ee', ha='center', fontsize=9, color='gray')
    ax.legend(handles=[Patch(color='#3f8f4f', label='loop OK (3 deliveries)'),
                       Patch(color='#b5443a', label='loop not completed')], fontsize=9)
    ax.grid(axis='y', alpha=.25)

    ax = axes[1]
    y = np.arange(len(rows))
    survey = [r['survey_interrupt_s'] if r['survey_interrupt_s'] is not None else 0 for r in rows]
    ax.barh(y, survey, .5, color='#8fb8de', label='airborne -> 3-class interrupt')
    for i, r in enumerate(rows):
        if r['survey_interrupt_s'] is None:
            ax.text(.5, i, 'search not interrupted', va='center', fontsize=8, color='gray')
        if r['third_commit_s'] is not None:
            ax.scatter(r['third_commit_s'], i, marker='|', s=300, color='black', zorder=5)
    ax.set_yticks(y, labels)
    ax.invert_yaxis()
    ax.set(xlabel='simulation seconds after first mission command',
           title='Search interrupt (bar) and 3rd delivery (tick)')
    ax.legend(fontsize=9)
    ax.grid(axis='x', alpha=.25)
    fig.tight_layout()
    fig.savefig(out_dir / 'combined_loop.png', dpi=170)
    plt.close(fig)

    arch = [r for r in rows if r['group'] == 'archived']
    new = [r for r in rows if r['group'] == 'new']
    lines = ['## 四、跨批次汇总（31–40 共 10 场景，闭环口径）', '',
             f"归档高位批次（源码 {ARCHIVED_SOURCE}）闭环 {sum(r['loop_ok'] for r in arch)}/{len(arch)}；"
             f"本批新场景（源码 92bd3ee）闭环 {sum(r['loop_ok'] for r in new)}/{len(new)}。"
             '两组源码不同（本批含 d463613 兜底修复与 a12750b 下降解耦），因此这是覆盖度汇总而非单变量对照。', '',
             '| 场景 | 门型 | 来源 | 环路 | 投递 | 搜索中断(s) | 三投(s) | 整场 Gate | 原因 |',
             '|---|---|---|---|---:|---:|---:|---|---|']
    for r in rows:
        lines.append(f"| {r['seed']} | {r['pattern']} | {r['source']} | "
                     f"{'OK' if r['loop_ok'] else 'FAIL'} | {r['deliveries']}/3 | "
                     f"{fmt(r['survey_interrupt_s'])} | {fmt(r['third_commit_s'])} | "
                     f"{r['status']} | {r['reason']} |")
    lines += ['', '![跨批次闭环汇总](combined_loop.png)', '',
              '解释边界：归档批次的运行条件（场景随机种子、机型、相机、模型质量门槛）与参数在 '
              '`high_fast_five_20260915/REPORT.md` 中单列；本表只做「前段闭环是否完成」的覆盖度合并，'
              '不把两批的不同源码与不同场景混为一次受控对比。', '']
    return lines, rows, arch, new


def main():
    batch = json.loads(BATCH.read_text())
    assert batch['status'] == 'COMPLETE' and len(batch['results']) == 5, batch['status']
    cases = json.loads((D / 'cases.json').read_text())
    scene = json.loads((D / 'scenarios.json').read_text())
    records = []
    for case in cases:
        result = next(v for v in batch['results'] if v['seed'] == case['seed'])
        assert result['cleanup_pass'], f"cleanup failed seed {case['seed']}"
        run = Path(result['run'])
        item = dict(seed=case['seed'], label='fast_high_new', run=str(run),
                    world=case['world'], source=batch['source'])
        dest = D / f"{case['seed']}_fast_high_new"
        metrics_file = dest / 'metrics.json'
        if metrics_file.exists():
            m = json.loads(metrics_file.read_text())
        else:
            m = NS['analyze'](item, D)
        m['door_pattern'] = scene[str(case['seed'])]['door_pattern']
        m['world'] = case['world']
        m['run'] = str(run)
        m['source'] = batch['source']
        m['loop'] = loop_metrics(m)
        m['tail'] = tail_metrics(run, m)
        records.append(m)
    (D / 'metrics.json').write_text(json.dumps(records, indent=2))
    (D / 'matrix.json').write_text(json.dumps(batch, indent=2))

    loops = [m['loop'] for m in records]
    summary = dict(seeds=[m['seed'] for m in records],
                   loop_ok=sum(v['loop_ok'] for v in loops),
                   loop_total=len(loops),
                   deliveries_total=sum(v['deliveries'] for v in loops),
                   releases_total=sum(v['releases'] for v in loops),
                   survey_interrupt_times=[v['survey_interrupt_s'] for v in loops],
                   third_commit_times=[v['third_commit_s'] for v in loops],
                   collisions=sum(m['collisions'] for m in records),
                   gate_pass=sum(m['status'] == 'PASS' for m in records),
                   doors_total=sum(m['tail']['doors'] for m in records))
    (D / 'summary.json').write_text(json.dumps(summary, indent=2))

    # ---- chart 1: five routes on real scene geometry (reuse base.scene) ----
    fig, axes = plt.subplots(1, 5, figsize=(27, 6.4))
    for ax, m in zip(axes, records):
        a = base.csv4(Path(m['run']) / 'truth_pose.csv')
        base.scene(ax, Path(m['world']),
                   yaml.safe_load((Path(m['run']) / 'random_field_truth.yaml').read_text()))
        ax.plot(a[:, 1], a[:, 2], lw=1, color='#dd8b25')
        for event in m['commit_times']:
            ax.scatter(*[np.interp(event['t'], a[:, 0], a[:, k]) for k in (1, 2)],
                       marker='*', s=70, color='red', zorder=5)
        if m['status'] != 'PASS':
            ax.scatter(a[-1, 1], a[-1, 2], marker='x', s=90, color='red', zorder=5)
        ax.set_title(f"seed{m['seed']} {m['door_pattern']}\n{m['status']} | "
                     f"{m['loop']['commits']}/3 drops | contacts {m['collisions']}", fontsize=11)
    fig.suptitle('New random scenes 36-40: routes, targets (squares), commits (stars)')
    fig.tight_layout()
    fig.savefig(D / 'all_routes.png', dpi=160)
    plt.close(fig)

    # ---- chart 2: front-loop milestone timeline (takeoff -> search -> deliveries) ----
    fig, ax = plt.subplots(figsize=(13, 6))
    for i, (m, v) in enumerate(zip(records, loops)):
        y = i
        ax.plot([v['airborne_s'] or 0, v['survey_interrupt_s'] or v['airborne_s'] or 0], [y, y],
                lw=6, color='#8fb8de', solid_capstyle='butt')
        final = m.get('high_view_final') or {}
        for e in final.get('events', []):
            if e.get('stage') == 'DELIVERY':
                ax.scatter(e['time'], y, marker='*', s=170, color='red', zorder=5)
                ax.annotate(e.get('target', ''), (e['time'], y), xytext=(0, 9),
                            textcoords='offset points', ha='center', fontsize=8)
        if v['third_commit_s']:
            ax.scatter(v['third_commit_s'], y, marker='|', s=260, color='black', zorder=4)
        label = f"seed{m['seed']}({m['door_pattern']}) {v['deliveries']}/3"
        ax.text(-3, y, label, ha='right', va='center', fontsize=10)
    ax.set(xlabel='Simulation seconds after first mission command', ylabel='',
           title='Front loop: airborne -> survey interrupt (bar), deliveries (stars), 3rd commit (tick)')
    ax.set_yticks([])
    ax.grid(axis='x', alpha=.25)
    fig.tight_layout()
    fig.savefig(D / 'loop_timeline.png', dpi=170)
    plt.close(fig)

    # ---- chart 3: loop vs tail outcome summary ----
    fig, axes = plt.subplots(1, 3, figsize=(19, 5.4))
    keys = [('survey_interrupt_s', 'Search: airborne -> 3-class interrupt (s)'),
            ('third_commit_s', 'Loop: time to 3rd delivery (s)'),
            ('catalog_entries', 'Survey: catalog entries')]
    for ax, (key, title) in zip(axes, keys):
        vals, colors = [], []
        for m, v in zip(records, loops):
            value = v[key]
            vals.append(0 if value is None else value)
            colors.append('#3f8f4f' if v['loop_ok'] else '#b5443a')
        ax.bar(range(5), vals, .55, color=colors)
        for i, (value, m, v) in enumerate(zip(vals, records, loops)):
            if v[key] is None:
                ax.text(i, .02, 'N/A', rotation=90, ha='center',
                        transform=ax.get_xaxis_transform(), fontsize=9)
            elif key != 'catalog_entries':
                ax.text(i, value, fmt(value), ha='center', va='bottom', fontsize=9)
        ax.set_xticks(range(5), [f"{m['seed']}\n{m['door_pattern']}" for m in records])
        ax.set_title(title)
        ax.grid(axis='y', alpha=.25)
    axes[0].legend(handles=[Patch(color='#3f8f4f', label='3 deliveries done (loop OK)'),
                            Patch(color='#b5443a', label='loop not completed')], fontsize=9)
    fig.tight_layout()
    fig.savefig(D / 'loop_summary.png', dpi=170)
    plt.close(fig)

    combined_lines, combined_rows, arch, new = combined_section(records)
    corr_ns = runpy.run_path(str(D / 'corridor.py'))
    corridor_lines, corridor_records = corr_ns['corridor_section'](records)

    # ---- report ----
    lines = ['# 新随机场景 36–40 高位策略闭环鲁棒性报告', '',
             f"本轮在 5 个**全新全随机场景**（树 obstacle_seed、门 door_seed/门型、靶 field_seed 均随机）上各跑 1 轮，"
             f"评估现有高位策略「起飞→搜索→投递」闭环鲁棒性。结果：闭环（三投齐备）**{summary['loop_ok']}/5**，"
             f"投递合计 {summary['deliveries_total']}/15，整场 Gate PASS **{summary['gate_pass']}/5**，碰撞合计 {summary['collisions']}。"
             f"所有预定场景均保留，失败未重跑替换。", '',
             '本批按用户要求**重点分析走廊问题**：专项定位投后路线第 8 段（第二门 Wall_22，x=+1.6）的停滞现象，'
             '见第三节；后段其余失败原因单列在第二节。', '',
             '门型分配：36=LL、37=LR、38=RL、39=RR、40=LL（LL 为 seed31 同类，加权两组）。'
             '场景由本仓 `simulation_tools/tools/export_r2026_scene.py` 生成；生成器已用归档 seed31 复现校验（field.world/'
             'field_config/gate_geometry 逐字节一致）。', '',
             '## 一、前段闭环（本次重点）', '',
             '| Seed | 门型 | 起飞(s) | 搜索中断(s) | 目录条目 | 齐备线索 | 投递事件 | 已提交 | 三投(s) | 环路 | 退出阶段 |',
             '|---|---|---:|---:|---:|---|---:|---:|---:|---|---|']
    for m, v in zip(records, loops):
        lines.append(f"| {m['seed']} | {m['door_pattern']} | {fmt(v['airborne_s'])} | "
                     f"{fmt(v['survey_interrupt_s'])} | {v['catalog_entries']} | "
                     f"{', '.join(v['hints_ready']) or '无'} | {v['deliveries']}/3 | {v['commits']}/3 | "
                     f"{fmt(v['third_commit_s'])} | {'OK' if v['loop_ok'] else 'FAIL'} | "
                     f"{v['terminal_stage']}`{(' ' + v['loop_failure']) if v['loop_failure'] else ''}` |")
    collision_rows, walls = [], {}
    for m in records:
        for e in contact_events(m['run']):
            wall = wall_of(e['pairs'])
            walls[wall] = walls.get(wall, 0) + 1
            collision_rows.append(
                f"| {m['seed']}({m['door_pattern']}) | {batch['source'][:8]} | {fmt(e['t'])} | "
                f"{', '.join(e['pairs'])} | 投递事件 {m['loop']['deliveries']}/3、"
                f"已提交 {m['loop']['commits']}/3 |")
    for meta in corr_ns['LEGACY']:
        if not Path(meta['run']).exists():
            continue
        for e in contact_events(meta['run']):
            wall = wall_of(e['pairs'])
            walls[wall] = walls.get(wall, 0) + 1
            collision_rows.append(f"| {meta['seed']}({meta['door_pattern']}) | {meta['source']} | "
                                  f"{fmt(e['t'])} | {', '.join(e['pairs'])} | 归档轮 |")
    if collision_rows:
        lines += ['', '### 环内碰撞（原始接触对，含归档轮）', '',
                  '| 场景 | 来源 | 时刻(s) | 接触对 | 当时进度 |', '|---|---:|---|---|---|'] + collision_rows
        top = ', '.join(f'{k} x{v}' for k, v in sorted(walls.items(), key=lambda kv: -kv[1]))
        lines += ['', f"碰撞对象分布：{top}。三次 Wall_11 接触分别发生在 a12750b/seed32（t~76.4s）、"
                  '本批 seed38（t~85.2s）与 seed40（t~90.2s），均在第三次投递前后；'
                  '该墙体附近是本策略在搜索-投递末段的高风险区，具体净空/绕行机制需单列分析。']
    lines += ['', '![闭环时间线](loop_timeline.png)', '', '![闭环汇总](loop_summary.png)', '',
              '## 二、后段（走廊/门/降落，单列不计入闭环结论）', '',
              '| Seed | 过门 | 投后路线 | 走廊限高违规 | 降落 | disarm | 整场 Gate | 原因 |',
              '|---|---:|---|---:|---|---|---|---|']
    for m in records:
        t = m['tail']
        route = f"{t['post_route_done']}/{t['post_route_size']}" if t['post_route_size'] else '—'
        lines.append(f"| {m['seed']} | {t['doors']} | {route} | {t['height_violations']} | "
                     f"{'OK' if t['land_ok'] else 'FAIL'} | {'OK' if t['disarmed'] else 'FAIL'} | "
                     f"{t['status']} | {t['reason']} |")
    lines += ['', '![五场景航迹](all_routes.png)', ''] + corridor_lines + combined_lines + ['## 五、逐场景图表', '']
    images = [('五场景航迹与靶位', 'all_routes.png'), ('闭环时间线', 'loop_timeline.png'),
              ('闭环汇总', 'loop_summary.png'), ('跨批次 10 场景闭环', 'combined_loop.png'),
              ('走廊段 X(t)/Y(t) 与停滞窗口', 'corridor_stall.png'),
              ('走廊分段时长（红=第二门 8/9）', 'corridor_segments.png')]
    for m in records:
        folder = f"{m['seed']}_fast_high_new"
        for name, title in [('flight_charts', '航迹/高度/速度'), ('route_stages', '阶段航线'),
                            ('route_3d', '三维航迹'), ('speed_profile', '速度阶段'),
                            ('phase_target_timeline', '阶段与投递事件')]:
            images.append((f"seed{m['seed']} {title}", f'{folder}/{name}.png'))
    for m in records:
        tail = m['tail']
        contacts = contact_events(m['run'])
        contact_txt = ('；'.join(f"t={fmt(c['t'])}s " + ', '.join(c['pairs']) for c in contacts)
                       if contacts else '无')
        lines += [f"### seed{m['seed']}（{m['door_pattern']}）", '',
                  f"环路：{'完成三投' if m['loop']['loop_ok'] else '未完成'}"
                  f"（投递事件 {m['loop']['deliveries']}/3、已提交 {m['loop']['commits']}/3）；"
                  f"高位终态 `{m['loop']['terminal_stage']}`，失败原因 `{m['loop']['loop_failure'] or '无'}`。",
                  f"后段：过门 {tail['doors']}、投后路线 "
                  f"{tail['post_route_done']}/{tail['post_route_size']}、整场 Gate `{tail['status']}`（{tail['reason']}）。",
                  f"碰撞：{contact_txt}。",
                  f"原始目录：`{m['run']}`。", '']
    lines += ['## 六、口径与边界', '',
              '- 闭环成功定义为「完成 3 次投递（3 次 release/commit）」；后段走廊、过门与降落结果单列，不与闭环鲁棒性混算。',
              '- 每轮为独立样本，n=1/场景；本报告度量的是新场景覆盖，不是同一场景的重复性统计。',
              '- 高位仍只做导航提示，低空新鲜重捕/释放门控与低空走廊 0.7m 工程阈值均未改动；',
              '  零碰撞不等于规则/实机验收，仿真结果不外推到板端部署。',
              '', '[全部图表浏览](index.html) · [指标](metrics.json) · [汇总](summary.json) · [场景校验](scene_validation.json)。']
    (D / 'REPORT.md').write_text('\n'.join(lines) + '\n')
    gallery = ''.join(f'<details open><summary>{t}</summary><a href="{p}"><img loading="lazy" src="{p}"></a></details>'
                      for t, p in images)
    (D / 'index.html').write_text(
        '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>新随机场景36-40闭环鲁棒性</title>'
        '<style>body{max-width:1500px;margin:24px auto;padding:0 18px;font:16px/1.7 system-ui;background:#f5f7fa}'
        'details{background:white;padding:16px;margin:20px 0}summary{font-weight:bold;cursor:pointer}'
        'img{width:100%;height:auto}</style><h1>新随机场景 36–40 高位策略闭环鲁棒性</h1>'
        '<p><a href="REPORT.md">完整结论</a> · <a href="metrics.json">指标</a></p>' + gallery + '</html>')
    print(json.dumps(dict(new=summary, archived_loop_ok=sum(r['loop_ok'] for r in arch),
                          new_group_loop_ok=sum(r['loop_ok'] for r in new)), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
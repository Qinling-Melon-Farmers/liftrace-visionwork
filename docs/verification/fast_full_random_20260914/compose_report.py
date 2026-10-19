"""Assemble actual four-run results, retaining incomplete trials as failures."""
import json,html
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze_fast import base

D=Path(__file__).resolve().parent
def fmt(x):return '—' if x is None else f'{x:.2f}'
def main():
    items=json.loads((D/'runs.json').read_text())
    metrics=json.loads((D/'summary.json').read_text())
    assert {(m['seed'],m['label']) for m in metrics}=={(s,k) for s in (32,34) for k in ('baseline','strategy')}
    pairs=[]
    fig,axes=plt.subplots(1,2,figsize=(14,7))
    for seed,ax in zip((32,34),axes):
        cases=[i for i in items if i['seed']==seed]
        truths=[base.yaml.safe_load((Path(i['run'])/'random_field_truth.yaml').read_text()) for i in cases]
        coordinates=lambda truth:sorted((t['class'],t['world_x'],t['world_y']) for t in truth['targets'])
        assert coordinates(truths[0])==coordinates(truths[1]),'paired target layouts differ'
        assert Path(cases[0]['world']).read_text()==Path(cases[1]['world']).read_text(),'paired worlds differ'
        base.scene(ax,Path(cases[0]['world']),truths[0])
        for item in cases:
            a=base.csv4(Path(item['run'])/'truth_pose.csv')
            ax.plot(a[:,1],a[:,2],lw=1.1,label=item['label'])
        ax.set_title(f'Seed {seed}: same random trees, doors and targets');ax.legend()
        b=next(m for m in metrics if m['seed']==seed and m['label']=='baseline')
        h=next(m for m in metrics if m['seed']==seed and m['label']=='strategy')
        row={'seed':seed,'same_world_and_targets':True}
        for key in ('third_commit_s','completed_mission_s'):
            row[key+'_gain_pct']=100*(b[key]-h[key])/b[key] if b[key] is not None and h[key] is not None else None
        pairs.append(row)
    fig.tight_layout();fig.savefig(D/'paired_routes.png',dpi=170);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(15,4))
    for ax,key,title in zip(axes,('third_commit_s','completed_mission_s','third_commit_to_corridor_entry_s'),('Third delivery (s)','Successful full mission (s)','Third delivery to corridor entry (s)')):
        for i,m in enumerate(metrics):
            if m[key] is not None:ax.bar(i,m[key],color='#c7862e' if m['label']=='strategy' else '#3979b5')
            else:ax.text(i,.02,'N/A',ha='center',transform=ax.get_xaxis_transform())
        ax.set_xticks(range(len(metrics)),[f"{m['seed']} {m['label']}" for m in metrics],rotation=25,ha='right');ax.set_title(title);ax.grid(axis='y',alpha=.2)
    fig.tight_layout();fig.savefig(D/'time_comparison.png',dpi=170);plt.close(fig)
    (D/'paired_comparison.json').write_text(json.dumps(pairs,indent=2))
    lines=['# 提速与高位策略：全随机seed32/34四轮对照','',
           '本报告只统计本轮实际运行。两组各采用同一套随机树、门、靶布局及相同提速配置；不以旧固定树四轮代替本轮。失败保留，不对未完成轮计算成功完赛节时。研究分支未替换正赛部署。','',
           '巡航前视1.0m、规划速度上限1.2m/s、接收限幅1.2m；精细阶段0.4m；第三投后保持巡航，到走廊入口第一个航点才降至0.15m。距离参数不是实测速度。','',
           '| Seed | 策略 | Gate | 三投(s) | 成功完赛(s) | 碰撞 | 末投至走廊入口(s) | XY航程(m) |',
           '|---|---|---|---:|---:|---:|---:|---:|']
    for m in metrics:
        lines.append(f"| {m['seed']} | {m['label']} | {m['status']} | {fmt(m['third_commit_s'])} | {fmt(m['completed_mission_s'])} | {m['collisions']} | {fmt(m['third_commit_to_corridor_entry_s'])} | {fmt(m['xy_distance_m'])} |")
    lines+=['','## 同布局对比','']
    for p in pairs:lines.append(f"- seed{p['seed']}：三投节时比例 {fmt(p['third_commit_s_gain_pct'])}%；成功整场节时比例 {fmt(p['completed_mission_s_gain_pct'])}%。正值表示高位策略更快，缺失表示无法配对计算。")
    lines+=['','## 各轮速度、失败与完整图表','',
            '速度由真值位置差分计算，剔除大于0.5s的采样断档；移动统计阈值为0.03m/s，仍可能包含悬停漂移。P95不能当作全程平均巡航速度。阶段时长使用任务事件和状态边界。','']
    images=[]
    for m in metrics:
        folder=f"{m['seed']}_{m['label']}"
        lines += [f"### seed{m['seed']} {m['label']}",'',f"原始运行：`{m['run']}`。Gate原因：`{m['reason']}`。最高FC离地高度 {fmt(m['max_fc_agl_m'])}m；真值采样断档 {m['sample_gap_count']} 次。",'']
        if m['failed_checks']:lines += ['失败项：'+', '.join(m['failed_checks'])+'。','']
        for phase,st in m['following_speed_stats'].items():
            lines.append(f"- {phase}：阶段累计 {fmt(st['duration_s'])}s，移动速度中位数 {fmt(st['median_mps'])}m/s，P95 {fmt(st['p95_mps'])}m/s。")
        lines+=['']
        for name in ('flight_charts','route_stages','route_3d','speed_profile','phase_target_timeline'):
            link=f'{folder}/{name}.png';lines.append(f'![{folder} {name}]({link})');lines.append('');images.append(link)
    lines+=['## 图表总览','', '![同布局完整航迹](paired_routes.png)','', '![耗时对比](time_comparison.png)','',
            '## 范围与复现','',
            '仅两布局各一次配对，不能据此证明泛化稳定性。先导与启动失败记录见pilot目录及STATUS.md；正式运行索引见runs.json，逐轮指标见summary.json，同图校验与差值见paired_comparison.json。先导不计入四轮。','',
            '离线复现使用rl_drone：先运行analyze_fast.py runs.json，再运行compose_report.py。原始点云真值只用于评测，不进入导航。关闭bag/视频，保留CSV、事件和Gate。']
    (D/'REPORT.md').write_text('\n'.join(lines)+'\n')
    cards=''.join(f'<figure><figcaption>{html.escape(i)}</figcaption><a href="{i}"><img loading="lazy" src="{i}"></a></figure>' for i in ['paired_routes.png','time_comparison.png']+images)
    (D/'index.html').write_text('<!doctype html><html lang="zh"><meta charset="utf-8"><title>提速四轮完整图表</title><style>body{font:16px sans-serif;max-width:1300px;margin:2em auto;background:#fafafa}img{width:100%}figure{margin:2em 0;padding:1em;background:white}a{color:#175b90}</style><h1>全随机seed32/34提速四轮对照</h1><p><a href="REPORT.md">完整报告与失败分析</a> · <a href="summary.json">逐轮指标</a></p>'+cards+'</html>')

if __name__=='__main__':main()

"""Compare repaired trials to the preserved failed high trials and old coverage."""
import json,runpy,html
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent;R=D.parents[2]
ns=runpy.run_path(str(R/'docs/verification/fast_full_random_20260914/analyze_fast.py'));base=ns['base']
def fmt(x):return '—' if x is None else f'{x:.2f}'
def main():
    batch=json.loads((R/'logs/high_fallback_repair_20260915_batch/matrix.json').read_text())
    assert batch['status']=='COMPLETE' and len(batch['results'])==3
    cases=json.loads((D/'cases.json').read_text());metrics=[];pairs=[]
    for case in cases:
        result=next(v for v in batch['results'] if v['seed']==case['seed']);assert result['cleanup_pass']
        worlds=[];targets=[];cameras=[]
        for label,run in [('old_low',case['baseline_run']),('old_high',case['previous_high_run']),('repaired_high',result['run'])]:
            run=Path(run);item=dict(seed=case['seed'],label=label,run=str(run),world=case['world'])
            mfile=D/f'{case["seed"]}_{label}/metrics.json'
            if mfile.exists():
                m=json.loads(mfile.read_text());assert Path(m['run']).resolve()==run.resolve()
            else:m=ns['analyze'](item,D)
            metrics.append(m)
            truth=yaml.safe_load((run/'random_field_truth.yaml').read_text())
            targets.append(sorted((t['class'],t['world_x'],t['world_y'],t.get('yaw')) for t in truth['targets']))
            cam=json.loads((run/'actual_camera_info.json').read_text());cameras.append([cam[k] for k in ('width','height','K','D')])
        assert targets[0]==targets[1]==targets[2] and cameras[0]==cameras[1]==cameras[2]
        assert (Path(result['run'])/'scenario_inputs/field.world').read_bytes()==Path(case['world']).read_bytes()
        pairs.append(dict(seed=case['seed'],same_world=True,same_targets=True,same_camera=True))
    (D/'matrix.json').write_text(json.dumps(batch,indent=2));(D/'metrics.json').write_text(json.dumps(metrics,indent=2));(D/'pairing.json').write_text(json.dumps(pairs,indent=2))
    labels=('old_low','old_high','repaired_high');fig,axes=plt.subplots(3,3,figsize=(17,18))
    for col,case in enumerate(cases):
        for row,label in enumerate(labels):
            m=next(v for v in metrics if v['seed']==case['seed'] and v['label']==label);run=Path(m['run']);a=base.csv4(run/'truth_pose.csv');ax=axes[row,col]
            base.scene(ax,Path(case['world']),yaml.safe_load((run/'random_field_truth.yaml').read_text()))
            ax.plot(a[:,1],a[:,2],lw=1,color=('#477bb0','#a887b8','#139778')[row])
            for e in m['commit_times']:ax.scatter(np.interp(e['t'],a[:,0],a[:,1]),np.interp(e['t'],a[:,0],a[:,2]),marker='*',s=80,color='red')
            if m['status']!='PASS':ax.scatter(a[-1,1],a[-1,2],marker='x',s=65,color='red')
            ax.set_title(f"Seed {case['seed']} {label}\n{m['status']} | {len(m['commit_times'])}/3 | contacts {m['collisions']}")
    fig.tight_layout();fig.savefig(D/'paired_routes.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(16,5))
    for ax,key,title in zip(axes,('third_commit_s','completed_mission_s','xy_distance_m'),('Third delivery seconds','Successful full mission seconds','Observed XY distance (failures included)')):
        for j,label in enumerate(labels):
            for i,case in enumerate(cases):
                m=next(v for v in metrics if v['seed']==case['seed'] and v['label']==label);x=i+(j-1)*.25;v=m.get(key)
                if v is not None:ax.bar(x,v,.24,color=('#477bb0','#a887b8','#139778')[j],label=label if i==0 else None)
                else:ax.text(x,.02,'N/A',rotation=90,ha='center',transform=ax.get_xaxis_transform(),fontsize=8)
        ax.set_xticks(range(3),[c['seed'] for c in cases]);ax.set_title(title);ax.grid(axis='y',alpha=.2)
    from matplotlib.patches import Patch
    axes[0].legend(handles=[Patch(color=c,label=l) for c,l in zip(('#477bb0','#a887b8','#139778'),labels)],fontsize=8)
    fig.tight_layout();fig.savefig(D/'time_comparison.png',dpi=170);plt.close(fig)
    new=[m for m in metrics if m['label']=='repaired_high']
    lines=['# 高位修复31/32/34重跑报告','',f"冻结源码 `{batch['source'][:8]}`，三轮各一次，无自动重试。完整PASS {sum(m['status']=='PASS' for m in new)}/3，完成三投 {sum(len(m['commit_times'])==3 for m in new)}/3。旧失败保留，不以此次重跑替换。",'',
           '| seed | 方案 | Gate | 投递 | 碰撞 | 三投(s) | 完赛(s) |','|---|---|---|---:|---:|---:|---:|']
    for m in metrics:lines.append(f"| {m['seed']} | {m['label']} | {m['status']} | {len(m['commit_times'])}/3 | {m['collisions']} | {fmt(m['third_commit_s'])} | {fmt(m['completed_mission_s'])} |")
    lines+=['','相对历史低速遍历的同阶段节时：','', '| seed | 三投节时 | 双侧成功完赛节时 |','|---|---:|---:|']
    for case in cases:
        old=next(m for m in metrics if m['seed']==case['seed'] and m['label']=='old_low')
        repaired=next(m for m in metrics if m['seed']==case['seed'] and m['label']=='repaired_high')
        values=[]
        for key in ('third_commit_s','completed_mission_s'):
            a,b=old.get(key),repaired.get(key)
            values.append(f'{100*(a-b)/a:.2f}%' if a is not None and b is not None else '不可计算')
        lines.append(f"| {case['seed']} | {values[0]} | {values[1]} |")
    lines+=['','只有两侧均完成相同阶段才能计算节时；旧高位三轮均失败，不能以其较短终止时间评价优化。历史低速与本轮不是单变量消融。','',
            '## 理想高位视场','', '![理想视场](ideal_full_route_coverage.png)','',
            '完整名义航线、水平姿态、航向0、FC AGL2.6m和相机低16cm，几何覆盖约96.8%；未计遮挡、转弯/倾斜、实际避障改道与中断。中央窄缝与左边缘漏区不能忽略；不能把该比例当实际识别召回。','',
            '## 前后航迹及时间','', '![九条轨迹](paired_routes.png)','', '![时间](time_comparison.png)','',
            '## 修复触发及逐轮结果','']
    for m in new:
        status=m.get('high_view_final',{});events=status.get('events',[])
        selected=[e for e in events if e.get('stage') in ('SURVEY_NO_PROGRESS','SURVEY_ALTERNATIVE','SURVEY_SKIPPED','PARTIAL_HINT_FALLBACK','LOW_COVERAGE_HANDOFF','SURVEY_INTERRUPTED_TOP3')]
        lines += [f"### seed{m['seed']}",'',f"终态 `{status.get('stage')}`；失败 `{status.get('failure','')}`；原Gate原因 `{m.get('reason')}`。",'',
                  f"首次合格线索：{', '.join(status.get('first_hint_ready',{})) or '无'}；最终持有：{', '.join(status.get('top_hints',{})) or '无'}。",'',
                  '```json',json.dumps(selected,ensure_ascii=False,indent=2),'```','',f"原始数据：`{m['run']}`。",'']
    lines+=['## 实现和验证范围','',
            '持久导航记忆不刷新last_seen，不替代新鲜投递证据。高位8s无进展走原超时事务，最多一次0.6m横向候选替代再跳过，实际三维规划与全高障碍柱保留。部分线索优先复访，耗尽或重捕失败接原低位覆盖，保留已投槽位和原截止时间。',
            '本轮未新增弱线索高位补视角、局部30s补扫或新动态尾段储备估计，先复用原覆盖。181项离线测试及两个工作区构建PASS；三场world/实际靶位/相机匹配检查通过。运行日志与图表仅笔记本SITL，不替代规则完整投影或板端实飞验收。','',
            '[所有图表](index.html) · [指标](metrics.json) · [配对检查](pairing.json) · [实现范围](STATUS.md)']
    (D/'REPORT.md').write_text('\n'.join(lines)+'\n')
    images=[('理想完整高位视场','ideal_full_route_coverage.png'),('九条航迹对照','paired_routes.png'),('耗时对照','time_comparison.png')]
    for m in metrics:
        for name in ('flight_charts','route_stages','route_3d','speed_profile','phase_target_timeline'):
            images.append((f"seed{m['seed']} {m['label']} {name}",f"{m['seed']}_{m['label']}/{name}.png"))
    content='<!doctype html><html lang="zh"><meta charset="utf-8"><title>高位修复对照</title><style>body{max-width:1500px;margin:24px auto;font-family:sans-serif;background:#f4f6f8}img{width:100%;background:white}section{margin:32px 0}h1,p{margin:16px}</style><h1>高位修复31/32/34：完整航迹与图表</h1><p><a href="REPORT.md">报告</a> · 几何视场不等于识别覆盖；原失败保留。</p>'
    for title,path in images:
        assert (D/path).is_file();content+=f'<section><h2>{html.escape(title)}</h2><img loading="lazy" src="{path}" alt="{html.escape(title)}"></section>'
    (D/'index.html').write_text(content+'</html>')
    print(json.dumps([dict(seed=m['seed'],status=m['status'],slots=len(m['commit_times']),third=m['third_commit_s'],full=m['completed_mission_s'],reason=m['reason']) for m in new],indent=2))
if __name__=='__main__':main()

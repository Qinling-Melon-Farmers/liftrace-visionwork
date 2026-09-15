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
    followup=json.loads((R/'logs/high_fallback_descent32_20260915_batch/matrix.json').read_text())
    assert followup['status']=='COMPLETE' and len(followup['results'])==1
    extra=followup['results'][0];assert extra['seed']==32 and extra['cleanup_pass']
    case=next(c for c in cases if c['seed']==32);run=Path(extra['run'])
    item=dict(seed=32,label='repaired_high_v2',run=str(run),world=case['world'])
    f=D/'32_repaired_high_v2/metrics.json'
    extra_m=json.loads(f.read_text()) if f.exists() else ns['analyze'](item,D)
    assert Path(extra_m['run']).resolve()==run.resolve()
    prev=Path(case['previous_high_run'])
    target_key=lambda p:sorted((t['class'],t['world_x'],t['world_y'],t.get('yaw')) for t in yaml.safe_load((p/'random_field_truth.yaml').read_text())['targets'])
    assert target_key(prev)==target_key(run)
    a=json.loads((prev/'actual_camera_info.json').read_text());b=json.loads((run/'actual_camera_info.json').read_text())
    assert all(a[k]==b[k] for k in ('width','height','K','D'))
    assert (run/'scenario_inputs/field.world').read_bytes()==Path(case['world']).read_bytes()
    metrics.append(extra_m)
    for m in metrics:
        decisions=[]
        with (Path(m['run'])/'key_events.jsonl').open() as stream:
            for line in stream:
                e=json.loads(line)
                if e.get('kind')=='decision':decisions.append(e)
        m['last_decisions']=decisions[-2:]
        (D/f"{m['seed']}_{m['label']}/metrics.json").write_text(json.dumps(m,indent=2))
    (D/'followup32_matrix.json').write_text(json.dumps(followup,indent=2))
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
    lines=['# 高位修复31/32/34重跑报告','',f"首版冻结源码 `{batch['source'][:8]}`，三轮各一次，无自动重试。完整PASS {sum(m['status']=='PASS' for m in new)}/3，完成三投 {sum(len(m['commit_times'])==3 for m in new)}/3。旧失败保留，不以此次重跑替换。",'',
           f"32暴露下降选择问题后另作相关补修，以 `{followup['source'][:8]}` 单独补验一次，结果 **{extra_m['status']}**、{len(extra_m['commit_times'])}/3投递。共四次新运行；补验不合并成同版本三轮成功率，不覆盖32首版失败。",'',
           '**结论：修复部分奏效，整场尚未通过。** 31的缺靶兜底与34的高位换点都实际完成三投，但随后卡在第二门；32补验两投后复访红十字时发生保护圈与Wall_11接触。按用户意见保留a12750b下降解耦和逐点排序，同时保留持久导航记忆、有界换点与低位兜底。两投后靠墙复访碰撞单列待修，不把整场FAIL归因于下降解耦，也不宣称通过部署验收。', '',
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
    old32=next(m for m in metrics if m['seed']==32 and m['label']=='old_low')
    gains=[]
    for key in ('third_commit_s','completed_mission_s'):
        a,b=old32.get(key),extra_m.get(key);gains.append(f'{100*(a-b)/a:.2f}%' if a is not None and b is not None else '不可计算')
    lines+=['',f"32补版相对历史低速：三投节时{gains[0]}，双侧成功完赛节时{gains[1]}。",'',
            '只有两侧均完成相同阶段才能计算节时；旧高位三轮均失败，不能以其较短终止时间评价优化。历史低速与本轮不是单变量消融。','',
            '## 理想高位视场','', '![理想视场](ideal_full_route_coverage.png)','',
            '完整名义航线、水平姿态、航向0、FC AGL2.6m和相机低16cm，几何覆盖约96.8%；未计遮挡、转弯/倾斜、实际避障改道与中断。中央窄缝与左边缘漏区不能忽略；不能把该比例当实际识别召回。','',
            '## 前后航迹及时间','', '![九条轨迹](paired_routes.png)','', '![时间](time_comparison.png)','',
            '## 修复触发及逐轮结果','']
    for m in new+[extra_m]:
        status=m.get('high_view_final',{});events=status.get('events',[])
        selected=[e for e in events if e.get('stage') in ('SURVEY_NO_PROGRESS','SURVEY_ALTERNATIVE','SURVEY_SKIPPED','PARTIAL_HINT_FALLBACK','LOW_COVERAGE_HANDOFF','SURVEY_INTERRUPTED_TOP3','DESCENT_COLUMN_WITHOUT_FULL_TOUR')]
        lines += [f"### seed{m['seed']} {m['label']}",'',f"终态 `{status.get('stage')}`；状态摘要失败 `{status.get('failure','')}`；原Gate原因 `{m.get('reason')}`。",'',
                  f"首次合格线索：{', '.join(status.get('first_hint_ready',{})) or '无'}；最终持有：{', '.join(status.get('top_hints',{})) or '无'}。",'',
                  '```json',json.dumps(selected,ensure_ascii=False,indent=2),'```','',
                  '最终任务命令：`'+str(m['last_decisions'][-1]['data'].get('reason'))+'`。',
                  '下降诊断：`'+json.dumps(status.get('descent_debug'),ensure_ascii=False)+'`。',
                  f"原始数据：`{m['run']}`。",'']
    contact=json.loads((Path(extra_m['run'])/'gazebo_contact_status.json').read_text())
    a=base.csv4(Path(extra_m['run'])/'truth_pose.csv');ct=contact['events'][0]['ros_stamp']
    mask=(a[:,0]>=ct-8)&(a[:,0]<=ct+.1)
    truth=yaml.safe_load((Path(extra_m['run'])/'random_field_truth.yaml').read_text())
    target=next(t for t in truth['targets'] if t['class']=='red_cross')
    hint=extra_m['high_view_final']['top_hints']['red_cross']['xy']
    fig,ax=plt.subplots(figsize=(9,6));case=next(c for c in cases if c['seed']==32)
    base.scene(ax,Path(case['world']),truth)
    ax.plot(a[mask,1],a[mask,2],color='#139778',lw=2,label='Actual last 8s trajectory')
    ax.scatter(*hint,marker='D',color='#8c509f',s=65,label='High navigation hint')
    ax.scatter(target['world_x'],target['world_y'],marker='+',color='black',s=100,label='Ground-truth target (evaluation only)')
    ax.scatter(np.interp(ct,a[:,0],a[:,1]),np.interp(ct,a[:,0],a[:,2]),marker='x',s=140,color='red',label=f'Guard contact at {ct:.3f}s')
    ax.set(xlim=(2.,5.1),ylim=(.2,1.7),title='Seed32 follow-up: red-cross revisit and Wall_11 contact')
    ax.legend(fontsize=8,loc='upper left');fig.tight_layout();fig.savefig(D/'collision32_closeup.png',dpi=180);plt.close(fig)
    lines+=['## 32补验碰撞及保留解耦的依据','',
            '补验在38.1s记录：感知图时间37.946s，当前位置下降柱未占据、走廊入口未占据，red_cross端点被粗栅格占据。它确实展示了下降可行与完整目标排序不应混为一谈，但不能据此倒推首版32的None具有完全相同原因。',
            '补验已完成panzer和bridge投递，复访red_cross时保护圈接触Wall_11，Gate实际碰撞计数1。这里存在Gazebo接触事件，仍按本批原Gate记FAIL，未用此前约1mm事后投影容差改写结果。','',
            '![补验碰撞局部](collision32_closeup.png)','',
            '```json',json.dumps(contact.get('events',[]),ensure_ascii=False,indent=2),'```','',
            '原先按整场FAIL回退整个补版的范围过大，现已撤销该本地回退。下降已完成且随后两投成功，碰撞发生于最后一次靠墙复访，不能据此否定下降解耦；恢复旧耦合反而会重新引入已知退出条件。后续应先处理靠墙目标的复访终点、到点减速和规划跟踪余量，再重新评价解耦方案；本次不继续补跑。31/34应另查投后第二门的规划停滞，不扩大修改本轮速度/地图。','',
            '## 实现和验证范围','',
            '持久导航记忆不刷新last_seen，不替代新鲜投递证据。高位8s无进展走原超时事务，最多一次0.6m横向候选替代再跳过，实际三维规划与全高障碍柱保留。部分线索优先复访，耗尽或重捕失败接原低位覆盖，保留已投槽位和原截止时间。',
            '保留补版解耦下降柱与完整排序，完整排序失败时可先访问当前可达目标。首版181项、补版183项离线测试及两工作区构建通过；按用户意见恢复解耦后183项再次通过，三份飞行/测试文件与a12750b无差异。8s无进展可能误判长绕行，未对原成功33/35重跑，不宣称泛化稳定。',
            '本轮未新增弱线索高位补视角、局部30s补扫或新动态尾段储备估计，先复用原覆盖。三场及32补验world/实际靶位/相机匹配检查通过。运行日志与图表仅笔记本SITL，不替代规则完整投影或板端实飞验收。','',
            '[所有图表](index.html) · [指标](metrics.json) · [配对检查](pairing.json) · [实现范围](STATUS.md)']
    (D/'REPORT.md').write_text('\n'.join(lines)+'\n')
    images=[('理想完整高位视场','ideal_full_route_coverage.png'),('九条航迹对照','paired_routes.png'),('耗时对照','time_comparison.png'),('32补验碰撞局部','collision32_closeup.png')]
    for m in metrics:
        for name in ('flight_charts','route_stages','route_3d','speed_profile','phase_target_timeline'):
            images.append((f"seed{m['seed']} {m['label']} {name}",f"{m['seed']}_{m['label']}/{name}.png"))
    content='<!doctype html><html lang="zh"><meta charset="utf-8"><title>高位修复对照</title><style>body{max-width:1500px;margin:24px auto;font-family:sans-serif;background:#f4f6f8}img{width:100%;background:white}section{margin:32px 0}h1,p{margin:16px}</style><h1>高位修复31/32/34：完整航迹与图表</h1><p><strong>原三轮整场0/3通过，32补验亦失败；保留a12750b下降解耦，靠墙复访碰撞待修。</strong></p><p><a href="REPORT.md">报告</a> · 几何视场不等于识别覆盖；原失败保留。</p>'
    for title,path in images:
        assert (D/path).is_file();content+=f'<section><h2>{html.escape(title)}</h2><img loading="lazy" src="{path}" alt="{html.escape(title)}"></section>'
    (D/'index.html').write_text(content+'</html>')
    print(json.dumps([dict(seed=m['seed'],label=m['label'],status=m['status'],slots=len(m['commit_times']),third=m['third_commit_s'],full=m['completed_mission_s'],reason=m['reason']) for m in new+[extra_m]],indent=2))
if __name__=='__main__':main()

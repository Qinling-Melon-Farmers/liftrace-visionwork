"""Post-run paired five-seed analysis. Never starts ROS or alters raw results."""
import json,os,runpy
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import yaml

D=Path(__file__).resolve().parent;R=D.parents[2]
NS=runpy.run_path(str(R/'docs/verification/fast_full_random_20260914/analyze_fast.py'));base=NS['base']

def fmt(x):return '—' if x is None else f'{x:.2f}'
def main():
    batch=json.loads((R/'logs/high_fast_five_20260915_batch/matrix.json').read_text())
    assert batch['status']=='COMPLETE' and len(batch['results'])==5
    cases=json.loads((D/'cases.json').read_text());records=[];pairs=[];all_items=[]
    for case in cases:
        result=next(v for v in batch['results'] if v['seed']==case['seed'])
        assert result['cleanup_pass']
        newrun=Path(result['run']);oldrun=Path(case['baseline_run'])
        positions=lambda run:sorted((v['class'],v['world_x'],v['world_y'],v.get('yaw',v.get('world_yaw'))) for v in yaml.safe_load((run/'random_field_truth.yaml').read_text())['targets'])
        same_targets=positions(oldrun)==positions(newrun)
        assert same_targets, f"seed {case['seed']} target mismatch; do not compute paired gain"
        oldworld=R/f'docs/verification/full_random_five_20260910/seed_{case["seed"]}/scenario_inputs/field.world'
        assert oldworld.read_bytes()==Path(case['world']).read_bytes()
        cameras=[json.loads((p/'actual_camera_info.json').read_text()) for p in (oldrun,newrun)]
        same_camera=all(cameras[0][key]==cameras[1][key] for key in ('width','height','K','D'))
        pair=[]
        for label,run,source in [('slow_coverage',oldrun,case['baseline_source']),('fast_high',newrun,batch['source'])]:
            item=dict(seed=case['seed'],label=label,run=str(run),world=case['world'],source=source);all_items.append(item)
            dest=D/f'{case["seed"]}_{label}';metrics_file=dest/'metrics.json'
            if metrics_file.exists():
                m=json.loads(metrics_file.read_text());assert Path(m['run']).resolve()==run.resolve()
            else:m=NS['analyze'](item,D)
            m['source']=source;records.append(m);pair.append(m)
        old,new=pair
        def gain(key):
            a,b=old.get(key),new.get(key)
            return None if a is None or b is None else 100*(a-b)/a
        pairs.append(dict(seed=case['seed'],same_world=True,same_targets=True,same_camera_intrinsics=same_camera,old_status=old['status'],new_status=new['status'],full_gain_pct=gain('completed_mission_s'),third_gain_pct=gain('third_commit_s')))
    (D/'matrix.json').write_text(json.dumps(batch,indent=2));(D/'runs.json').write_text(json.dumps(all_items,indent=2))
    (D/'metrics.json').write_text(json.dumps(records,indent=2));(D/'paired_comparison.json').write_text(json.dumps(pairs,indent=2))
    fig,axes=plt.subplots(2,5,figsize=(25,11))
    for col,case in enumerate(cases):
        for row,label in enumerate(('slow_coverage','fast_high')):
            m=next(v for v in records if v['seed']==case['seed'] and v['label']==label)
            run=Path(m['run']);a=base.csv4(run/'truth_pose.csv');ax=axes[row,col]
            base.scene(ax,Path(case['world']),yaml.safe_load((run/'random_field_truth.yaml').read_text()))
            ax.plot(a[:,1],a[:,2],lw=1,color='#3979ab' if row==0 else '#dd8b25')
            for event in m['commit_times']:ax.scatter(*[np.interp(event['t'],a[:,0],a[:,k]) for k in (1,2)],marker='*',s=65,color='red')
            if m['status']!='PASS':ax.scatter(a[-1,1],a[-1,2],marker='x',s=70,color='red')
            ax.set_title(f"{case['seed']} {label}\n{m['status']} | {len(m['commit_times'])}/3 | contacts {m['collisions']}",fontsize=10)
    fig.tight_layout();fig.savefig(D/'all_routes.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(17,5))
    for ax,key,title in zip(axes,('third_commit_s','completed_mission_s','xy_distance_m'),('Third delivery (s)','Successful full mission (s)','Observed XY distance incl. failures (m)')):
        for j,label in enumerate(('slow_coverage','fast_high')):
            for i,case in enumerate(cases):
                m=next(v for v in records if v['seed']==case['seed'] and v['label']==label);x=i+(j-.5)*.35;value=m.get(key)
                color='#3979ab' if j==0 else '#dd8b25'
                if value is not None:ax.bar(x,value,.32,color=color,label=label if i==0 else None,hatch='//' if m['status']!='PASS' else None)
                else:ax.text(x,.03,'N/A',rotation=90,ha='center',color=color,transform=ax.get_xaxis_transform(),fontsize=8)
        ax.set_xticks(range(5),[str(v['seed']) for v in cases]);ax.set_title(title);ax.grid(axis='y',alpha=.2);ax.legend(handles=[Patch(color='#3979ab',label='slow_coverage'),Patch(color='#dd8b25',label='fast_high')],fontsize=8)
    fig.tight_layout();fig.savefig(D/'time_comparison.png',dpi=170);plt.close(fig)
    old=[m for m in records if m['label']=='slow_coverage'];new=[m for m in records if m['label']=='fast_high']
    summary=dict(old_pass=sum(m['status']=='PASS' for m in old),new_pass=sum(m['status']=='PASS' for m in new),
                 old_three=sum(len(m['commit_times'])==3 for m in old),new_three=sum(len(m['commit_times'])==3 for m in new),
                 old_colliding_runs=sum(m['collisions']>0 for m in old),new_colliding_runs=sum(m['collisions']>0 for m in new),
                 comparable_full_pairs=sum(p['full_gain_pct'] is not None for p in pairs))
    (D/'summary.json').write_text(json.dumps(summary,indent=2))
    lines=['# 高空快速先搜五seed验证报告','',f"本轮高位快速先搜完整通过 **{summary['new_pass']}/5**，历史低速遍历为 **{summary['old_pass']}/5**；三投完成分别{summary['new_three']}/5、{summary['old_three']}/5。全部预定seed31–35均保留，没有重跑替换失败。",'',
           '**结论：当前版本尚不能稳定替代低速遍历。** 本批完整成功率没有提升，三投完成率下降。高位方案在部分可用场景有明显节时，但此前固定树两布局的成功不能外推到本批五套历史随机场景。', '',
           '地图、树、门及实际靶位逐seed核对一致。历史低速源码7eb9446，新高位使用本批冻结源码；这是综合优化版本与历史数据的对照，不是严格单变量或统计显著性证明。失败终止策略/观察时长差异须与成功完赛指标分开理解。','',
           '| Seed | 方案 | Gate | 投递 | 碰撞 | 三投(s) | 成功完赛(s) | 原因 |','|---|---|---|---:|---:|---:|---:|---|']
    for m in records:lines.append(f"| {m['seed']} | {m['label']} | {m['status']} | {len(m['commit_times'])}/3 | {m['collisions']} | {fmt(m['third_commit_s'])} | {fmt(m['completed_mission_s'])} | {m['reason']} |")
    lines+=['','## 同场景提升效果','', '| Seed | 成功整场节时 | 三投节时 |','|---|---:|---:|']
    for p in pairs:lines.append(f"| {p['seed']} | {fmt(p['full_gain_pct'])+'%' if p['full_gain_pct'] is not None else '不可计算'} | {fmt(p['third_gain_pct'])+'%' if p['third_gain_pct'] is not None else '不可计算'} |")
    lines+=['','只在两轮均完成相应阶段时计算节时；不把高位漏检或提前失败的短运行时间当作提效。单一平均成功用时不能代替成功率与失败机制。','',
            '## 工程基线及坐标修复继承','',
            '高位分支以竞赛综合86e382d为祖先。核查发现板端71827d9此前未自动继承，现已移植动态camera_init→map对齐、双向反馈/设定点适配和硬件launch接线；四个来源文件完全一致，CMake保留新研究内容并补安装/测试注册。7项适配器与4项对齐回归、实际构建和硬件XML接线检查通过。详见frame_inheritance.json。',
            'SITL沿用综合仿真定位链，未启动真实设备或板端控制。非单位旋转/平移转换由离线回归验证，不能把仿真当作板端动态TF或实飞验收；没有覆盖正赛机载部署。','',
            '## 清理结果','',
            '删除36个本地及26个远端冗余引用，退休2个旧worktree，保留3个实际工作树。旧日志、交付资产与报告已归档，旧路径保留兼容链接；原始脏基线及独有分支保留。删除引用的提交仍由保留分支持有，详见CLEANUP.md及cleanup系列记录。','',
            '## 完整航迹与耗时','', '![十轮同场景航迹](all_routes.png)','', '![时长对照](time_comparison.png)','',
            '## 新高位逐轮诊断','']
    images=[('十轮同场景航迹','all_routes.png'),('耗时与航程','time_comparison.png'),('几何观测机会与线索保持，非识别召回率','visibility_opportunities.png')]
    for m in new:
        last=m.get('high_view_final',{});lines += [f"### seed{m['seed']}",'',f"高位终态：`{last.get('stage')}`，失败原因：`{last.get('failure','')}`；齐备线索：{', '.join(last.get('first_hint_ready',{})) or '无'}。原始目录：`{m['run']}`。",'',
               f"实测最大FC离地高度{fmt(m['max_fc_agl_m'])}m，采样断档{m['sample_gap_count']}次。失败项：{', '.join(m['failed_checks']) or '无'}。",'']
        if last.get('observation_counts'):lines+=['观测统计：`'+json.dumps(last['observation_counts'],ensure_ascii=False)+'`。','']
        lines+=['退出时保存的可用线索集：'+(', '.join(last.get('top_hints',{})) or '无/未执行退出评估')+'。','']
    lines+=['## 失败机制与下一步','',
            '- seed31：高位路线结束仍未形成panzer合格导航线索，退出集合仅bridge/red_cross。几何估计Panzer中心约6.0s在像面内，不能简单认定完全没覆盖；遮挡、实际识别和质量过滤仍需原图/逐条候选记录区分。',
            '- seed32：曾分别形成三类提示，但高位退出时red_cross不再可用，最终零投递。这不是“从未发现第三类”；当前实现按现时Catalog.hints及唯一类别集合判断终止，观测窗重置、投票/不确定度和歧义过滤均可能改变集合。现有记录不能确定是哪一项使red_cross失效。',
            '- seed34：起飞上升后，第一个高位平面目标(-3.5,1,2.38)处于地图内但被膨胀占据，0.30m邻域没有可用点；约90s后运动失败，未开始投递。应处理不可达高位航点，不能靠从障碍顶上穿越解决。',
            '- seed33/35：完成三投、两门和降落，0碰撞。seed33把旧未完成场景跑通；只有seed35两方案都完整成功，可给整场节时38.08%。不能用不同成功子集的均值声称整批节时。','',
            '后续优先级：先分离持久导航线索与短时观测窗，并保留低空新鲜重捕/释放门控；再增加高位未齐目标后的有界低空补搜或回退，以及不可达高位航点的横向替代/跳过。最后补同步原图和候选拒绝原因，再检验视场/遮挡与参数。上述改进本批未实施，不能算作已验证效果。','',
            '![观测机会与提示状态](visibility_opportunities.png)','',
            '该图由10Hz真值位姿、相机K/D和既有下视安装关系作几何估计，只计算目标中心是否进入像面，不含遮挡或实际神经网络检出。EXIT READY为高位退出时可用；LOST AT EXIT表示曾形成提示但退出时不可用。不能将几何时间解释为检测召回率。','']
    for m in records:
        folder=f"{m['seed']}_{m['label']}"
        for name,title in [('flight_charts','航迹/高度/速度'),('route_stages','阶段航线'),('route_3d','三维航迹'),('speed_profile','速度阶段'),('phase_target_timeline','阶段和投递事件')]:
            path=f'{folder}/{name}.png';images.append((folder+' '+title,path))
    lines+=['## 规则与解释边界','',
            '保留全高障碍柱，不采用已撤回的柱顶截断；高位仅允许0.30m水平目标调整，低位恢复0.15m。原飞行Gate仍含0.7m走廊工程阈值；不能直接把规则墙高1.5m代替本批原判据。零碰撞不等于整机在障碍水平投影外已有完备证明，此前保守包络投影疑点仍保留；本报告不据飞行Gate宣称最终规则/实机验收。','',
            '[全部图表浏览](index.html) · [原始运行索引](runs.json) · [对照指标](metrics.json) · [同场景校验及节时](paired_comparison.json)。所有分析用rl_drone，未记录全场bag/视频。']
    (D/'REPORT.md').write_text('\n'.join(lines)+'\n')
    gallery=''.join(f'<details open><summary>{title}</summary><a href="{path}"><img loading="lazy" src="{path}"></a></details>' for title,path in images)
    (D/'index.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>高空快速先搜五seed验证</title><style>body{max-width:1500px;margin:24px auto;padding:0 18px;font:16px/1.7 system-ui;background:#f5f7fa}details{background:white;padding:16px;margin:20px 0}summary{font-weight:bold;cursor:pointer}img{width:100%;height:auto}</style><h1>高空快速先搜五seed验证</h1><p><a href="REPORT.md">完整结论与失败分析</a> · <a href="CLEANUP.md">清理记录</a> · <a href="frame_inheritance.json">坐标修复继承</a></p>'+gallery+'</html>')
    print(json.dumps(summary,indent=2))
if __name__=='__main__':main()

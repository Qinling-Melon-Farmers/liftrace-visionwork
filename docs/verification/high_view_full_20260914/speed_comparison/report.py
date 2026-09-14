"""Eight fixed-layout observations, four methods; never rank failed times."""
import json,os,runpy,html
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml

D=Path(__file__).resolve().parent;O=D.parent;R=D.parents[3]
P=R/'docs/verification/fast_full_random_20260914/pilot'
NS=runpy.run_path(str(P.parent/'analyze_fast.py'));base=NS['base']
ORDER=['slow_coverage','fast_coverage','slow_high','fast_high']
NAMES={'slow_coverage':'提速前覆盖','fast_coverage':'提速覆盖','slow_high':'提速前高位先搜','fast_high':'提速高位先搜'}
EN={'slow_coverage':'Slow coverage','fast_coverage':'Fast coverage','slow_high':'Slow high survey','fast_high':'Fast high survey'}
COLORS=dict(zip(ORDER,['#777777','#2171b5','#31a354','#e67e22']))
def fmt(v):return '—' if v is None else f'{v:.3f}'
def gain(a,b):return None if a is None or b is None else 100*(a-b)/a
def main():
    items=json.loads((D/'runs.json').read_text())
    assert len(items)==8 and {(i['seed'],i['method']) for i in items}=={(s,m) for s in (32,34) for m in ORDER}
    records=[]
    for item in sorted(items,key=lambda v:(v['seed'],ORDER.index(v['method']))):
        method=item['method'];seed=item['seed']
        if method.startswith('slow_'):folder=O/f'{seed}_{item["label"]}'
        elif seed==32:folder=P/'results'/f'32_{item["label"]}'
        else:folder=D/f'{seed}_{method}'
        m=json.loads((folder/'metrics.json').read_text())
        assert Path(m['run']).resolve()==Path(item['run']).resolve()
        m.update(method=method,source=item['source'],world=item['world'],image_dir=os.path.relpath(folder,D))
        if 'following_speed_stats' not in m:m['following_speed_stats']={}
        records.append(m)
    validations=[]
    for seed in (32,34):
        cases=[i for i in items if i['seed']==seed]
        worlds=[Path(i['world']).read_text().strip() for i in cases]
        assert all(w==worlds[0] for w in worlds),'world differs within seed'
        positions=[]
        for item in cases:
            truth=yaml.safe_load((Path(item['run'])/'random_field_truth.yaml').read_text())
            positions.append(sorted((t['class'],t['world_x'],t['world_y'],t.get('world_z'),t.get('world_yaw',t.get('yaw'))) for t in truth['targets']))
        assert all(p==positions[0] for p in positions),'target layout differs within seed'
        validations.append(dict(seed=seed,same_world=True,same_target_layout=True))
    lookup={(m['seed'],m['method']):m for m in records};effects=[]
    for seed in (32,34):
        for left,right in [('slow_coverage','fast_coverage'),('slow_coverage','slow_high'),('slow_high','fast_high'),('fast_coverage','fast_high'),('slow_coverage','fast_high')]:
            a,b=lookup[seed,left],lookup[seed,right]
            effects.append(dict(seed=seed,from_method=left,to_method=right,
                                full_gain_pct=gain(a['completed_mission_s'],b['completed_mission_s']),
                                third_gain_pct=gain(a['third_commit_s'],b['third_commit_s'])))
    (D/'summary.json').write_text(json.dumps(records,indent=2))
    (D/'comparisons.json').write_text(json.dumps(dict(layout_validation=validations,effects=effects),indent=2))
    fig,axes=plt.subplots(2,4,figsize=(22,12))
    for row,seed in enumerate((32,34)):
        for col,method in enumerate(ORDER):
            m=lookup[seed,method];run=Path(m['run']);a=base.csv4(run/'truth_pose.csv');ax=axes[row,col]
            base.scene(ax,Path(m['world']),yaml.safe_load((run/'random_field_truth.yaml').read_text()))
            ax.plot(a[:,1],a[:,2],lw=1,color=COLORS[method])
            for commit in m['commit_times']:
                ax.scatter(*[np.interp(commit['t'],a[:,0],a[:,k]) for k in (1,2)],marker='*',s=70,color='red',zorder=5)
            if m['status']!='PASS':ax.scatter(a[-1,1],a[-1,2],marker='x',color='red',s=70)
            ax.set_title(f"{seed} {EN[method]} | {m['status']}",fontsize=10)
    fig.suptitle('Same-layout trajectories | stars: committed deliveries | red X: failed end')
    fig.tight_layout();fig.savefig(D/'four_method_routes.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(15,9))
    for col,seed in enumerate((32,34)):
        for method in ORDER:
            m=lookup[seed,method];run=Path(m['run']);a=base.csv4(run/'truth_pose.csv');start=m['start_ros_s']
            p=yaml.safe_load((run/'rosparams.yaml').read_text());offset=p['competition_key_recorder']['truth_world_offset'][2]
            axes[0,col].plot(a[:,0]-start,a[:,3]+offset,lw=.8,color=COLORS[method],label=EN[method])
            dt=np.diff(a[:,0]);ok=(dt>0)&(dt<=.5)
            axes[1,col].plot(a[1:,0][ok]-start,np.linalg.norm(np.diff(a[:,1:3],axis=0)[ok],axis=1)/dt[ok],lw=.6,color=COLORS[method],alpha=.8)
        axes[0,col].set_title(f'Seed {seed}');axes[0,col].set_ylabel('True FC AGL (m)');axes[0,col].legend(fontsize=8)
        axes[1,col].set(xlabel='Simulation seconds after mission start',ylabel='True horizontal speed (m/s)')
    for ax in axes.flat:ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(D/'four_method_height_speed.png',dpi=170);plt.close(fig)
    fig,axes=plt.subplots(2,3,figsize=(16,8))
    for row,seed in enumerate((32,34)):
        for col,key in enumerate(['third_commit_s','completed_mission_s','xy_distance_m']):
            ax=axes[row,col]
            for i,method in enumerate(ORDER):
                m=lookup[seed,method];v=m[key]
                if v is not None:
                    failed_path=key=='xy_distance_m' and m['status']!='PASS'
                    ax.bar(i,v,color=COLORS[method],alpha=.9,hatch='//' if failed_path else None,edgecolor='firebrick' if failed_path else 'none');ax.text(i,v,f'{v:.1f}'+(' FAIL' if failed_path else ''),ha='center',va='bottom',fontsize=8)
                else:ax.text(i,.03,'FAIL\nN/A',ha='center',color='firebrick',transform=ax.get_xaxis_transform(),fontsize=8)
            ax.set_xticks(range(4),[EN[m] for m in ORDER],rotation=20,ha='right',fontsize=8);ax.set_xlim(-.6,3.6);ax.grid(axis='y',alpha=.2);ax.margins(y=.17)
            ax.set_title(f"{seed}: {['Third delivery (s)','Successful finish (s)','Observed XY path (m), including failed runs'][col]}",fontsize=10)
    fig.tight_layout();fig.savefig(D/'four_method_times.png',dpi=170);plt.close(fig)
    eligible=[method for method in ORDER if all(lookup[s,method]['status']=='PASS' for s in (32,34))]
    best=min(eligible,key=lambda method:sum(lookup[s,method]['completed_mission_s'] for s in (32,34))) if eligible else None
    method_summary=[]
    for method in ORDER:
        pair=[lookup[s,method] for s in (32,34)]
        method_summary.append(dict(method=method,successful_runs=sum(v['status']=='PASS' for v in pair),total_runs=2,
                                   paired_mean_finish_s=float(np.mean([v['completed_mission_s'] for v in pair])) if method in eligible else None))
    (D/'selection.json').write_text(json.dumps(dict(best_by_flight_gate=best,final_rule_qualified_selection=None,
        unresolved='seed34 fast-high conservative body projection overlap about151mm; not covered by accepted seed32~1mm tolerance',methods=method_summary),indent=2))
    lines=['# 固定布局seed32/34：速度与搜索策略效率对照','',
           ('两布局均通过飞行Gate的已测方案中，平均完赛时间最短的是 **'+NAMES[best]+'**。但seed34较大的保守包络投影交叠尚未解决，当前只能称为最快候选，不能称为禁止整机越树条件下已最终验收的方案。' if best else '本批没有一个方案在两布局均完成，不能选出两布局共同验收的最优方案。'),'',
           '本次复用凌晨四轮和seed32先导，补seed34两轮。每个seed四种方案的世界文件及实际靶位一致。全随机计划按用户指令暂不执行；不能将这些结果推广为全随机或实机已验收。','',
           '| Seed | 方案 | Gate | 已投/3 | 碰撞 | 三投(s) | 成功完赛(s) |',
           '|---|---|---|---:|---:|---:|---:|']
    for m in records:lines.append(f"| {m['seed']} | {NAMES[m['method']]} | {m['status']} | {len(m['commit_times'])} | {m['collisions']} | {fmt(m['third_commit_s'])} | {fmt(m['completed_mission_s'])} |")
    lines+=['','## 提升效果','', '| Seed | 变化 | 完赛节时 | 三投节时 |','|---|---|---:|---:|']
    for e in effects:lines.append(f"| {e['seed']} | {NAMES[e['from_method']]} → {NAMES[e['to_method']]} | {fmt(e['full_gain_pct'])+'%' if e['full_gain_pct'] is not None else '无法计算'} | {fmt(e['third_gain_pct'])+'%' if e['third_gain_pct'] is not None else '无法计算'} |")
    lines+=['','只有双方成功完赛才计算整场节时。seed34提速前覆盖在降落失败，提速覆盖在首投恢复时碰Wall_12；不能把失败结束时间当成更快的完赛时间，也不补跑替代这两个失败。','',
            '## 实施组合与适用边界','',
            '- 提速配置：巡航前视1.0m、规划上限1.2m/s；精细阶段0.4m；末投后保持巡航，到走廊入口首个航点完成后切0.15m。参数上限不等于实测速度。',
            '- 高位先搜：FC离地约2.6m；累计获得三类高权重坐标线索后中断高位路线，就近下降，按感知地图代价排序，逐个低位新鲜重捕与投递。',
            '- 禁止越树：保留全高障碍柱；高位只允许0.30m水平航点调整，不改变Z、不降低净空，低位恢复0.15m，走廊保持原航点条件。柱顶截断的试验已撤回且不计入这八条有效对照记录。',
            '- 横移兜底的验证边界：新增0.30m范围通过了单元测试和旧失败快照检查；本批新高位轮没有记录到requested/effective目标偏移，不能凭成功轮认定先前地图波动已被完全排除，也不能把节时归功于该兜底。',
            '- 保守包络：seed32新高位中心航迹未进入障碍投影，55×55×40cm保守包络存在约1mm单次凸包边界重叠；用户确认其属于扩大的包络余量提示，保留记录，不按真实碰撞处理，不为此追加膨胀或重跑。',
            '- 新发现：seed34新高位中心航迹同样未进入障碍投影，但旋转保守包络与树/箱体凸包有约15.1cm的SAT投影交叠指示。这不是碰撞引擎的接触深度，也不能直接等同真实机体越树；它明显不同于先前约1mm的容差，不能未经确认就沿用该忽略口径。',
            '- 几何复核：低速高位seed32的最小分离轴间隙约+74mm，seed34约-9mm；提速高位seed32约-1mm，seed34约-151mm。负值为保守投影重叠。旧低速方案也不是“零保守投影交叠”证书；实际机体轮廓和规则解释尚需核对，当前不升级正赛部署。',
            '- 结论是这两布局、这些实际试验中的选择，不是全局最优或统计稳定性证明。seed34快速覆盖的真实仿真接触说明不能把提速参数直接当作所有路线均已稳定的默认部署。','',
            '下一步如继续优化，应优先处理高位转弯/中断下降时的贴边与轨迹跟踪，同时保留直线、低位复访和末投转场提速。该局部限速方案尚未实跑，不写成已验证最优组合。','',
            '![保守投影复核](projection_review.png)','', '[投影指标](projection_review.json)；该检查与原飞行Gate分别报告，不追改旧Gate。','',
            '## 完整图表','', '![四方案完整航迹](four_method_routes.png)','', '![高度与速度](four_method_height_speed.png)','', '![耗时与已飞航程](four_method_times.png)','',
            '## 各轮原始记录与图表','']
    cards=[]
    for m in records:
        lines += [f"### seed{m['seed']} {NAMES[m['method']]}",'',f"源码记录：`{m['source']}`；原始目录：`{m['run']}`。Gate原因：`{m['reason']}`。",'']
        if m['status']!='PASS':lines+=['失败项：'+', '.join(m['failed_checks'])+'。','']
        for filename,title in [('flight_charts.png','完整航迹/高度/速度'),('route_stages.png','阶段航线'),('phase_target_timeline.png','阶段和投递事件'),('speed_profile.png','速度参数切换'),('route_3d.png','三维航迹')]:
            image=Path(m['image_dir'])/filename
            if (D/image).exists():
                link=image.as_posix();lines.append(f'- [{title}]({link})');cards.append((f"seed{m['seed']} {NAMES[m['method']]}：{title}",link))
        lines+=['']
    lines+=['## 复现与口径','',
            '使用rl_drone运行本目录report.py；runs.json为八条输入，summary.json为逐轮指标，comparisons.json记录同布局检查和节时计算。新增轮次原始数据均经sim_run.sh单实例运行及零残留收尾。','',
            '耗时使用原Gate的mission_ros_sec，统一任务开始口径，不含Gazebo/模型加载和人工准备；电脑墙钟速度不用于比赛效率排名。轨迹速度从真值采样差分获得，断档剔除。旧轮源码5667bed，新提速轮包含速度阶段和高位横移修复，比较的是实际方案组合，不声称只隔离了一个数值参数。']
    (D/'REPORT.md').write_text('\n'.join(lines)+'\n')
    intro=''.join(f'<figure><figcaption>{title}</figcaption><img src="{file}"></figure>' for title,file in [('四方案完整航迹','four_method_routes.png'),('实际高度和速度','four_method_height_speed.png'),('耗时对照','four_method_times.png'),('保守包络投影复核，独立于原Gate','projection_review.png')])
    gallery=''.join(f'<details><summary>{html.escape(title)}</summary><img loading="lazy" src="{link}"></details>' for title,link in cards)
    (D/'index.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>固定布局效率对照</title><style>body{font:16px/1.7 system-ui,sans-serif;max-width:1500px;margin:24px auto;padding:0 18px;color:#25364a;background:#f5f7fa}img{width:100%;height:auto}figure,details{padding:16px;background:white;margin:20px 0}summary{cursor:pointer;font-weight:600}a{color:#1767a4}</style><h1>固定布局seed32/34：四方案效率对照</h1><p>'+html.escape(lines[2])+'</p><p><a href="REPORT.md">完整结论与失败分析</a> · <a href="summary.json">指标</a> · <a href="comparisons.json">同布局与节时检查</a></p>'+intro+gallery+'</html>')
    print(json.dumps(dict(best_observed_flight_gate=best,rule_qualified_final=None,validation=validations,effects=effects),indent=2))
if __name__=='__main__':main()

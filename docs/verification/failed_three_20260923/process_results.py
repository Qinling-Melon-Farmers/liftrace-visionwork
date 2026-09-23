"""Offline analysis only, after all six finish. Preserve historical results."""
from pathlib import Path
import collections,html,importlib.util,json,os,subprocess,sys
import numpy as np,yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent;R=D.parents[2]
OLD=D.parent/'history_31_40_20260920'

def f(v):return '—' if v is None else f'{v:.2f}'
def table(head,rows):
    return '\n'.join(['| '+' | '.join(head)+' |','|'+'|'.join(['---']*len(head))+'|']+['| '+' | '.join(map(str,row))+' |' for row in rows])+'\n'

def main():
    state=json.loads((D/'selected_matrix.json').read_text())
    if state['status']!='COMPLETE' or [v['seed'] for v in state['results']]!=[34,38,40]:raise RuntimeError('Wait for all three runs')
    oldmetrics={v['seed']:v for v in json.loads((OLD/'metrics.json').read_text())}
    oldruns={v['seed']:v for v in json.loads((OLD/'runs.json').read_text())}
    spec=importlib.util.spec_from_file_location('six_analysis',OLD/'analyze_current.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.D=D
    metrics=[];comparison=[];items=[]
    for row in state['results']:
        seed=row['seed'];run=Path(row['run']);oldrun=Path(oldruns[seed]['run'])
        manifest=yaml.safe_load((run/'manifest.yaml').read_text())
        assert manifest['git_head']==state['source']
        assert manifest['resolved_uav_mission']==str(R/'patrol_uav_ws-patrol_planner/src/uav_mission')
        assert manifest['resolved_uav_vision']==str(R/'vision_ws/src/uav_vision')
        before=yaml.safe_load((oldrun/'random_field_truth.yaml').read_text())['targets']
        after=yaml.safe_load((run/'random_field_truth.yaml').read_text())['targets']
        errors=[]
        for t in before:
            q=next(v for v in after if v['class']==t['class'])
            errors.append(float(np.linalg.norm(np.array([t['world_x'],t['world_y']])-np.array([q['world_x'],q['world_y']]))))
        assert max(errors)<1e-4,(seed,errors)
        item=dict(seed=seed,label='rerun',run=str(run),world=str(OLD/f'seed_{seed}/field.world'),source=state['source']);items.append(item)
        m=mod.one(item)
        progress=mod.prior.base.events(run/'trajectory_progress.jsonl')
        m['explicit_recovery_reasons']=dict(collections.Counter(v['data']['reason'] for v in progress if v['data']['reason'] in ['server_tracking_hold_replan','no_physical_progress_replan','server_hold_budget_exhausted','liveness_budget_exhausted']))
        metrics.append(m);old=oldmetrics[seed]
        comparison.append(dict(seed=seed,old_status=old['status'],new_status=m['status'],old_reason=old['reason'],new_reason=m['reason'],old_complete=old['completed_mission_s'],new_complete=m['completed_mission_s'],old_third=old['third_commit_s'],new_third=m['third_commit_s'],old_drops=old['gate_metrics'].get('release_commit_count'),new_drops=m['gate_metrics'].get('release_commit_count'),layout_max_error_m=max(errors),first_failure_analysis=row['first_failure_analysis']))
        output=run/('presentation_review.mp4' if seed in (38,40) else 'presentation.mp4')
        if not output.exists():
            composer=D.parent/'failed_six_20260921/compose_review.py' if seed in (38,40) else R/'vision_ws/src/uav_high_view/scripts/presentation_compose.py'
            cmd=[sys.executable,str(composer),str(run),'--output',str(output),'--flight-limit','4','--corridor-limit','1.2','--wall-height','4','--corridor-bounds','7.6','9.1','-4.8','4.8','--case-label',f'Seed {seed} | 历史失败回归 | 2.6m / 共享边界']
            with (D/f'compose_seed{seed}.txt').open('w') as log:subprocess.run(cmd,check=True,stdout=log,stderr=subprocess.STDOUT)
        subprocess.run(['ffmpeg','-v','error','-i',str(output),'-f','null','-'],check=True,stdout=subprocess.DEVNULL)
        print(f'Seed {seed}: metrics, figures, composed video verified',flush=True)
    for name,obj in [('metrics.json',metrics),('comparison.json',comparison),('runs.json',items),('matrix.json',state)]:
        (D/name).write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n')
    fig,axes=plt.subplots(1,3,figsize=(18,6))
    for ax,item,m in zip(axes.flat,items,metrics):
        seed=item['seed'];mod.scene(ax,Path(item['world']),yaml.safe_load((Path(item['run'])/'random_field_truth.yaml').read_text()))
        for path,label,style,color in [(Path(oldruns[seed]['run']),'old FAIL','--','gray'),(Path(item['run']),'rerun '+m['status'],'-','tab:blue')]:
            a=mod.prior.base.csv4(path/'truth_pose.csv');ax.plot(a[:,1],a[:,2],style,color=color,lw=1,label=label)
        ax.set_title(f'Seed {seed}');ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(D/'paired_paths.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(13,5));xs=np.arange(3)
    for i,(oldkey,newkey,title) in enumerate([('old_third','new_third','Third successful mock ACK'),('old_complete','new_complete','Completed mission only')]):
        for offset,key,label,color in [(-.18,oldkey,'old','gray'),(.18,newkey,'rerun','tab:blue')]:
            for x,row in zip(xs,comparison):
                if row[key] is None:axes[i].text(x+offset,3,'not reached',rotation=90,ha='center',fontsize=7)
                else:axes[i].bar(x+offset,row[key],width=.34,color=color,label=label if x==0 else None)
        axes[i].set_xticks(xs,[str(r['seed']) for r in comparison]);axes[i].set(title=title,xlabel='Seed',ylabel='Mission ROS seconds');axes[i].grid(axis='y',alpha=.2)
    fig.tight_layout();fig.savefig(D/'time_comparison.png',dpi=150);plt.close(fig)
    ok=sum(m['status']=='PASS' for m in metrics)
    lines=['# 失败三seed修复验收统一回归报告\n',f'本批仅重跑历史失败的34、38、40，各一次。结果为 **{ok}/3 PASS**。原十轮其他4个通过seed未复测，不能将此推算为当前版本十轮成功率。\n',
      '## 版本、场地和计时\n',f'统一HEAD `{state["source"]}`，实际导航/视觉包路径均核对为本工作树。三轮未调参、未补跑替换失败。与原十轮直接比对实际生成的五类靶位，最大XY差均小于0.1mm；树/门复用同一冻结world。2.6m高位、既有快走廊和录像配置一致。宿主墙钟改为6000秒，ROS任务600秒不变，避免旧seed40墙钟截尾；因此该项属于评测环境变化。\n',
      '本批是历史失败子集的可靠性复测，原三轮均未完整完赛，不计算所谓完整完赛节时百分比；只有双方确实达到的同一阶段才比较耗时。视频按ROS图像时间校正，宿主运行时长不等于飞行时间。\n',
      '## 配对结果\n',table(['seed','原结果','本轮','原投递数','本轮投递数','原三投/s','本轮三投/s','本轮完赛/s','本轮原因'],[[p['seed'],p['old_status'],p['new_status'],p['old_drops'],p['new_drops'],f(p['old_third']),f(p['new_third']),f(p['new_complete']),p['new_reason']] for p in comparison]),
      '![新旧完整记录航迹](paired_paths.png)\n','![三投及完赛时间](time_comparison.png)\n',
      '## 单轮指标和材料\n']
    for m in metrics:
        seed=m['seed'];folder=f'{seed}_rerun';run=Path(m['run']);rel=os.path.relpath(run/('presentation_review.mp4' if seed in (38,40) else 'presentation.mp4'),D)
        lines.extend([f'### Seed {seed}\n',f'结果{m["status"]}，原因`{m["reason"]}`；记录XY航程{m["xy_distance_m"]:.2f}m，实际碰撞{m["collisions"]}，样本间隔缺口{m["sample_gap_count"]}。失败轮航程/时长只是已执行部分。\n',
          f'[完整指标]({folder}/metrics.json) · [跟随+俯视视频]({rel})\n',f'![速度与阶段]({folder}/speed_profile.png)\n',f'![高度与姿态]({folder}/height_tilt.png)\n',
          '恢复事件：`'+json.dumps(m['explicit_recovery_reasons'],ensure_ascii=False)+'`。未出现事件不等于该故障分支已经动态验证。\n'])
    lines.extend(['## 统计边界\n','投递误差按mock成功ACK时真实飞控中心到靶心距离统计，不是包裹落点；真实机构耗时及释放口偏置仍未标定。原始与新Gate都保留，另有保守机体/树投影结果，不把保守包络重叠自动称为实际碰撞。失败原因和下一步处置需结合逐轮首个异常分析，不能只看最终ABORT。\n'])
    (D/'REPORT.md').write_text('\n'.join(lines))
    sections=[]
    for m in metrics:
        seed=m['seed'];folder=D/f'{seed}_rerun';video=os.path.relpath(Path(m['run'])/('presentation_review.mp4' if seed in (38,40) else 'presentation.mp4'),D)
        imgs=''.join(f'<a href="{seed}_rerun/{p.name}"><img loading="lazy" src="{seed}_rerun/{p.name}" alt="{html.escape(p.stem)}"></a>' for p in sorted(folder.glob('*.png')))
        sections.append(f'<section><h2>Seed {seed} — {m["status"]}</h2><video controls preload="none" src="{video}"></video><div class="plots">{imgs}</div></section>')
    (D/'index.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>失败三seed修复验收回归</title><style>body{max-width:1400px;margin:auto;font:16px sans-serif;padding:24px;background:#f7f8fa;color:#18212b}video{width:100%;max-width:960px}.plots{display:grid;grid-template-columns:repeat(2,1fr);gap:12px}img{width:100%}section{margin:32px 0;padding:20px;background:white}</style><h1>失败三seed修复验收回归</h1><p><a href="REPORT.md">统一报告</a> · <a href="comparison.json">配对数据</a></p>'+''.join(sections)+'</html>')
    (D/'validation.json').write_text(json.dumps(dict(three_complete=True,source_consistent=True,actual_targets_matched=True,all_videos_decoded=True),indent=2)+'\n')
    print('Unified report draft and all figures complete',flush=True)

if __name__=='__main__':main()

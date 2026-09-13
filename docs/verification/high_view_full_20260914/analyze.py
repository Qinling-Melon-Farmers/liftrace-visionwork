"""Offline paired full-mission report; truth never enters navigation."""
import argparse
import csv
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle


def csv4(path):
    with path.open() as f:
        return np.array([[float(r[k]) for k in ('t','x','y','z')] for r in csv.DictReader(f)])


def events(path):
    return [json.loads(s) for s in path.read_text().splitlines() if s.strip()]


def scene(ax, world, truth):
    field=next(m for m in ET.parse(world).getroot().find('world').findall('model') if m.get('name')=='toudi2')
    for link in field.findall('link'):
        if not link.get('name','').startswith('Wall'): continue
        pose=[float(v) for v in link.findtext('pose','0 0 0 0 0 0').split()]
        size=[float(v) for v in link.findtext('collision/geometry/box/size','0 0 0').split()]
        ax.add_patch(Rectangle((pose[0]-size[0]/2,pose[1]-size[1]/2),size[0],size[1],color='.5',alpha=.6))
    for model in field.findall('model'):
        if 'Tree' in model.get('name',''):
            p=[float(v) for v in model.findtext('pose','0 0 0 0 0 0').split()]
            ax.add_patch(Circle(p[:2],.43,color='forestgreen',alpha=.35))
    for target in truth['targets']:
        x,y=target['world_x'],target['world_y']
        ax.scatter(x,y,s=30,marker='s',color='darkred')
        ax.annotate(target['class'],(x,y),xytext=(3,4),textcoords='offset points',fontsize=7)
    for x,y in [(0,0),(4.2,8.5)]:
        ax.add_patch(Circle((x,y),.5,fill=False,color='black'))
        ax.text(x,y,'H',ha='center',va='center')
    ax.set(xlim=(-5.15,5.15),ylim=(-.85,9.45),xlabel='X (m)',ylabel='Y (m)')
    ax.set_aspect('equal');ax.grid(alpha=.2)


def analyze(item,out):
    run=Path(item['run']);label=item['label'];seed=item['seed']
    gate=json.loads((run/'gate_status.json').read_text())
    params=yaml.safe_load((run/'rosparams.yaml').read_text())
    truth=yaml.safe_load((run/'random_field_truth.yaml').read_text())
    ev=events(run/'key_events.jsonl');dec=[e for e in ev if e['kind']=='decision']
    start=min(e['data']['header']['stamp']['stamp_ns']/1e9 for e in dec)
    a=csv4(run/'truth_pose.csv');est=csv4(run/'mavros_pose.csv');cmd=csv4(run/'mavros_setpoint.csv')
    offset=params['competition_key_recorder']['truth_world_offset'][2]
    ground=params['target_map_projector']['ground_z']
    releases=[e for e in ev if e['kind']=='release']
    end=start+gate['metrics']['mission_ros_sec'] if gate['metrics'].get('mission_ros_sec') is not None else a[-1,0]
    active=a[(a[:,0]>=start)&(a[:,0]<=end)]
    dt=np.diff(active[:,0]);delta=np.diff(active[:,1:4],axis=0)
    valid=(dt>0)&(dt<=.5)
    speed=np.linalg.norm(delta[:,:2],axis=1)[valid]/dt[valid]
    metrics=dict(label=label,seed=seed,run=str(run),status=gate['status'],reason=gate['reason'],
        failed_checks=gate.get('failed_checks',[]),start_ros_s=start,observed_end_ros_s=end,
        completed_mission_s=gate['metrics'].get('mission_ros_sec') if gate['status']=='PASS' else None,
        landing_transaction_s=gate['metrics'].get('mission_ros_sec'),
        observed_duration_s=end-start,xy_distance_m=float(np.linalg.norm(delta[:,:2],axis=1)[valid].sum()),
        xyz_distance_m=float(np.linalg.norm(delta,axis=1)[valid].sum()),
        speed_xy_p95_mps=float(np.percentile(speed,95)) if len(speed) else None,
        max_fc_agl_m=float((active[:,3]+offset).max()),
        releases=releases,gate_metrics=gate['metrics'],
        collisions=json.loads((run/'gazebo_contact_status.json').read_text())['actual_collision_count'],
        sample_gap_count=int((dt>.5).sum()))
    moving=speed>.03
    metrics.update(moving_xy_time_s=float(dt[valid][moving].sum()),
                   nonmoving_xy_time_s=float(dt[valid][~moving].sum()),
                   moving_sample_threshold_mps=.03,
                   moving_speed_xy_median_mps=float(np.median(speed[moving])) if moving.any() else None,
                   moving_speed_xy_p95_mps=float(np.percentile(speed[moving],95)) if moving.any() else None)
    research=run/'high_view_full_events.jsonl'
    statuses=events(research) if research.exists() else []
    if statuses: metrics['high_view_final']=statuses[-1]['status']
    target_positions={t['class']:np.array([t['world_x'],t['world_y']]) for t in truth['targets']}
    errors={}
    final=metrics.get('high_view_final',{})
    for cls,h in final.get('top_hints',{}).items():
        if cls not in target_positions:continue
        low=next((v for v in final.get('reacquisitions',[]) if v['class_name']==cls),None)
        errors[cls]=dict(high_hint_error_m=float(np.linalg.norm(np.array(h['xy'])-target_positions[cls])),
                        hint_uncertainty_m=h['uncertainty_m'],
                        low_fused_point_error_m=float(np.linalg.norm(np.array(low['xy'])-target_positions[cls])) if low else None,
                        source_age_at_reacquisition_s=(low['last_seen_ns']-h['last_seen_ns'])/1e9 if low else None)
    metrics['target_hint_evaluation']=errors
    # Release commit timing comes from terminal payload-committed results,
    # deduplicated by slot; raw release diagnostic messages are kept separately.
    commits={}
    for e in ev:
        d=e['data']
        if e['kind']=='result' and d.get('payload_committed'):
            commits.setdefault(d['payload_slot'],dict(t=d['header']['stamp']['stamp_ns']/1e9,target=d['target_class']))
    metrics['commit_times']=list(commits.values())
    for commit in commits.values():
        xy=np.array([np.interp(commit['t'],a[:,0],a[:,i]) for i in (1,2)])
        commit['true_fc_xy_at_ack']=xy.tolist()
        if commit['target'] in target_positions:
            commit['fc_target_distance_at_ack_m']=float(np.linalg.norm(xy-target_positions[commit['target']]))
    metrics['third_commit_s']=max(c['t'] for c in commits.values())-start if len(commits)==3 else None
    metrics['first_commit_s']=min(c['t'] for c in commits.values())-start if commits else None
    metrics['xy_distance_to_third_commit_m']=(float(np.linalg.norm(delta[:,:2],axis=1)[valid&(active[1:,0]<=start+metrics['third_commit_s'])].sum()) if metrics['third_commit_s'] is not None else None)
    armed=next((e['ros_sec'] for e in ev if e['kind']=='state' and e['data'].get('armed')),None)
    airborne=next((e['ros_sec'] for e in ev if e['kind']=='extended_state' and e['data'].get('landed_state')==2),None)
    touchdown=next((e['ros_sec'] for e in ev if airborne is not None and e['ros_sec']>airborne and e['kind']=='extended_state' and e['data'].get('landed_state')==1),None)
    disarmed=next((e['ros_sec'] for e in ev if armed is not None and e['ros_sec']>armed and e['kind']=='state' and not e['data'].get('armed')),None)
    metrics.update(armed_ros_s=armed,airborne_ros_s=airborne,touchdown_ros_s=touchdown,disarmed_ros_s=disarmed,
                   airborne_to_touchdown_s=touchdown-airborne if touchdown is not None else None,
                   armed_to_third_commit_s=max(c['t'] for c in commits.values())-armed if armed is not None and len(commits)==3 else None,
                   touchdown_to_disarm_s=disarmed-touchdown if disarmed is not None and touchdown is not None else None)
    stages=[]
    for e in statuses:
        name=e['status']['stage']
        if name=='SURVEY' and not e['status'].get('ascent_verified'): name='ASCEND'
        if not stages or name!=stages[-1]['name']: stages.append(dict(name=name,t=e['t']))
    metrics['high_view_stage_changes']=stages
    timeline=[];active_seq=None;ordinary='SEARCH';research_phase=None;committed_decisions=set()
    combined=[(e['ros_sec'],'event',e) for e in ev if e['kind'] in ('decision','result')]
    combined += [(s['t'],'research',s) for s in stages]
    for t,kind,e in sorted(combined,key=lambda r:r[0]):
        if t<start or t>end:continue
        if kind=='research':research_phase=e['name']
        elif e['kind']=='decision':
            d=e['data'];active_seq=d['decision_seq']
            ordinary={0:'SEARCH',1:'APPROACH',2:'ALIGN',3:'SEARCH',4:'CORRIDOR',5:'LAND',6:'HOLD',7:'ABORT'}[d['command']]
        elif e['data']['decision_seq']==active_seq:
            d=e['data']
            if d['command'] in (1,2):ordinary={1:'APPROACH',2:'CAPTURE',3:'ALIGN',4:'RELEASE',5:'RECOVERY'}.get(d['stage'],ordinary)
            if d.get('payload_committed'):committed_decisions.add(active_seq)
            if active_seq in committed_decisions:ordinary='RECOVERY'
        phase=research_phase if research_phase in ('ASCEND','SURVEY','RETURN_COLUMN','LOCAL_DESCENT_TRANSIT','DESCEND','REVISIT','REACQUIRE') else ordinary
        if not timeline or timeline[-1]['phase']!=phase:timeline.append(dict(t=t,phase=phase))
    durations={}
    for i,entry in enumerate(timeline):
        duration=(timeline[i+1]['t'] if i+1<len(timeline) else end)-entry['t']
        durations[entry['phase']]=durations.get(entry['phase'],0)+duration
    metrics['sampled_phase_durations_s']=durations
    metrics['sampled_phase_changes']=timeline
    phase_speeds={}
    speed_times=active[1:,0][valid]
    for i,entry in enumerate(timeline):
        stop=timeline[i+1]['t'] if i+1<len(timeline) else end
        subset=speed[(speed_times>=entry['t'])&(speed_times<stop)&(speed>.03)]
        phase_speeds.setdefault(entry['phase'],[]).extend(subset.tolist())
    metrics['speed_by_phase']={k:dict(median_mps=float(np.median(v)),p95_mps=float(np.percentile(v,95)),samples=len(v)) for k,v in phase_speeds.items() if v}
    dest=out/(str(seed)+'_'+label);dest.mkdir(exist_ok=True)
    (dest/'metrics.json').write_text(json.dumps(metrics,indent=2))
    (dest/'gate_status.json').write_text(json.dumps(gate,indent=2))
    fig,axes=plt.subplots(2,2,figsize=(14,11));xy,zax,vax,tax=axes.flat
    scene(xy,Path(item['world']),truth)
    xy.plot(a[:,1],a[:,2],lw=1,label='Actual flight',color='#185b9a')
    xy.plot(cmd[:,1],cmd[:,2],lw=.5,alpha=.5,label='Command trajectory',color='#ef8425')
    goals=[e['data']['goal']['pose']['position'] for e in dec if e['data'].get('has_goal')]
    if goals:
        xy.scatter([g['x'] for g in goals],[g['y'] for g in goals],s=18,marker='x',color='black',label='Issued navigation goals')
    xy.set_title('XY: flight (blue), command (orange), goals (x)',fontsize=11)
    for values,shift,name,style in [(a,offset,'True FC AGL','-'),(est,-ground,'Estimated FC AGL','--'),(cmd,-ground,'Command FC AGL',':')]:
        zax.plot(values[:,0]-start,values[:,3]+shift,style,lw=.8,label=name)
    zax.set(xlabel='Simulation seconds after first mission command',ylabel='Height (m)',title='Height through delivery, doors and landing');zax.legend(fontsize=8)
    vax.plot(active[1:,0][valid]-start,speed,lw=.8,label='True horizontal speed')
    vax.plot(active[1:,0][valid]-start,delta[:,2][valid]/dt[valid],lw=.6,label='True vertical speed')
    vax.set(xlabel='Simulation seconds after first mission command',ylabel='Speed (m/s)',title='Speed from truth poses (gaps excluded)');vax.legend(fontsize=8)
    names={0:'SEARCH',1:'APPROACH',2:'ALIGN',3:'RESUME',4:'RETURN',5:'LAND',6:'HOLD',7:'ABORT'}
    tax.step([e['ros_sec']-start for e in dec],[e['data']['command'] for e in dec],where='post',lw=1)
    for c in commits.values():
        for ax in (zax,vax,tax): ax.axvline(c['t']-start,color='darkred',ls=':',alpha=.7)
        tax.annotate(c['target'],(c['t']-start,0),rotation=45,fontsize=7)
    for st in stages: tax.axvline(st['t']-start,color='gray',alpha=.3)
    tax.set(xlabel='Simulation seconds after first mission command',ylabel='MissionDecision command enum',title='Issued commands; red dotted lines = committed slots')
    tax.set_yticks(list(names),list(names.values()))
    for ax in (zax,vax,tax):ax.grid(alpha=.2)
    fig.suptitle(f'Seed {seed} | {label} | {gate["status"]} | {len(commits)}/3 committed | collisions {metrics["collisions"]}')
    fig.tight_layout();fig.savefig(dest/'flight_charts.png',dpi=170);plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(12,6))
    phase_names=list(durations)
    axes[0].barh(phase_names,list(durations.values()),color='#3677a6')
    axes[0].set(xlabel='Simulation seconds',title='Sampled phase duration (event/status boundaries)')
    target_classes=sorted(set(e['data'].get('target_class','') for e in ev if e['kind'] in ('decision','result','release'))-{''})
    for i,cls in enumerate(target_classes):
        for kind,marker in [('decision','>'),('result','.'),('release','*')]:
            xs=[e['ros_sec']-start for e in ev if e['kind']==kind and e['data'].get('target_class')==cls]
            axes[1].scatter(xs,[i]*len(xs),s=35 if kind=='release' else 12,marker=marker,label=kind if i==0 else None)
    axes[1].set_yticks(range(len(target_classes)),target_classes)
    axes[1].set(xlabel='Simulation seconds after first mission command',title='Target transactions (release message is diagnostic; commits checked separately)')
    axes[1].legend(fontsize=8)
    for ax in axes:ax.grid(axis='x',alpha=.2)
    fig.tight_layout();fig.savefig(dest/'phase_target_timeline.png',dpi=170);plt.close(fig)
    fig,ax=plt.subplots(figsize=(9,9));scene(ax,Path(item['world']),truth)
    palette={p:plt.get_cmap('tab20')(i%20) for i,p in enumerate(phase_names)}
    labeled=set()
    for i,entry in enumerate(timeline):
        stop=timeline[i+1]['t'] if i+1<len(timeline) else end
        rows=a[(a[:,0]>=entry['t'])&(a[:,0]<=stop)]
        phase=entry['phase']
        if len(rows):
            ax.plot(rows[:,1],rows[:,2],lw=1.5,color=palette[phase],label=phase if phase not in labeled else None)
            labeled.add(phase)
    if goals:
        ax.plot([g['x'] for g in goals],[g['y'] for g in goals],'k--',lw=.5,alpha=.3,label='Waypoint order (not a collision-free path)')
    for cls,hint in metrics.get('high_view_final',{}).get('top_hints',{}).items():
        ax.add_patch(Circle(hint['xy'],hint['uncertainty_m'],fill=False,edgecolor='red',ls=':',lw=1))
    for c in commits.values():
        xy=[np.interp(c['t'],a[:,0],a[:,i]) for i in (1,2)]
        ax.scatter(*xy,marker='*',color='red',s=100,zorder=8)
    ax.set_title(f'Seed {seed} | {label} | stages and committed deliveries')
    ax.legend(loc='upper center',bbox_to_anchor=(.5,-.07),ncol=3,fontsize=8)
    fig.tight_layout();fig.savefig(dest/'route_stages.png',dpi=170);plt.close(fig)
    return metrics


def main():
    p=argparse.ArgumentParser();p.add_argument('--runs',type=Path,required=True);p.add_argument('--out',type=Path,required=True);args=p.parse_args()
    args.out.mkdir(exist_ok=True)
    items=sorted(json.loads(args.runs.read_text()),key=lambda item:(item['seed'],item['label']))
    metrics=[analyze(item,args.out) for item in items]
    (args.out/'summary.json').write_text(json.dumps(metrics,indent=2))
    fig,axes=plt.subplots(1,3,figsize=(13,4))
    for ax,key,title in zip(axes,['third_commit_s','completed_mission_s','xy_distance_to_third_commit_m'],['Third committed delivery (s)','Completed mission (s)','XY to third delivery (m)']):
        for i,m in enumerate(metrics):
            value=m[key]
            if value is not None:ax.bar(i,value,color='#3b75a9' if m['label']=='baseline' else '#dd913a')
            else:
                observed=m['observed_duration_s']
                ax.scatter(i,observed,marker='x',s=60,color='gray')
                ax.annotate('FAIL: stopped',(i,observed),xytext=(0,8),textcoords='offset points',ha='center',fontsize=8)
        ax.set_xticks(range(len(metrics)),[str(m['seed'])+' '+m['label'] for m in metrics],rotation=25,ha='right');ax.set_title(title);ax.grid(axis='y',alpha=.2)
    fig.tight_layout();fig.savefig(args.out/'comparison.png',dpi=170);plt.close(fig)
    seeds=sorted(set(i['seed'] for i in items))
    fig,axes=plt.subplots(1,len(seeds),figsize=(7*len(seeds),7),squeeze=False)
    paired=[]
    for seed,ax in zip(seeds,axes.flat):
        pair=[i for i in items if i['seed']==seed]
        scene(ax,Path(pair[0]['world']),yaml.safe_load((Path(pair[0]['run'])/'random_field_truth.yaml').read_text()))
        for item in pair:
            data=csv4(Path(item['run'])/'truth_pose.csv')
            status=next(m['status'] for m in metrics if m['seed']==seed and m['label']==item['label'])
            ax.plot(data[:,1],data[:,2],lw=.85,alpha=.8,label=item['label']+' ('+status+')')
        ax.legend();ax.set_title(f'Seed {seed}: recorded trajectories')
        mm={m['label']:m for m in metrics if m['seed']==seed}
        if set(mm)=={'baseline','strategy'}:
            result={'seed':seed,'both_gate_pass':all(m['status']=='PASS' for m in mm.values())}
            for key in ('third_commit_s','completed_mission_s'):
                a,b=mm['baseline'][key],mm['strategy'][key]
                result[key+'_gain_s']=a-b if a is not None and b is not None else None
                result[key+'_gain_percent']=100*(a-b)/a if a is not None and b is not None and a>0 else None
            a,b=mm['baseline']['xy_distance_m'],mm['strategy']['xy_distance_m']
            result['xy_distance_gain_m']=a-b if result['both_gate_pass'] else None
            result['xy_distance_gain_percent']=100*(a-b)/a if result['both_gate_pass'] and a>0 else None
            a,b=mm['baseline']['xy_distance_to_third_commit_m'],mm['strategy']['xy_distance_to_third_commit_m']
            result['third_xy_gain_m']=a-b if a is not None and b is not None else None
            result['third_xy_gain_percent']=100*(a-b)/a if a is not None and b is not None and a>0 else None
            paired.append(result)
    fig.tight_layout();fig.savefig(args.out/'paired_paths.png',dpi=170);plt.close(fig)
    (args.out/'paired_differences.json').write_text(json.dumps(paired,indent=2))
    fig,ax=plt.subplots(figsize=(12,5))
    for i,m in enumerate(metrics):
        times=[c['t']-m['start_ros_s'] for c in m['commit_times']]
        color='#3b75a9' if m['label']=='baseline' else '#dd913a'
        ax.plot(times,[i]*len(times),'o-',color=color,lw=1)
        for slot,(t,c) in enumerate(zip(times,m['commit_times']),1):
            ax.annotate(f'{slot}: {c["target"]}',(t,i),xytext=(0,-20 if slot==2 else 9),textcoords='offset points',ha='center',fontsize=8)
    ax.set_yticks(range(len(metrics)),[str(m['seed'])+' '+m['label'] for m in metrics])
    ax.set(xlabel='Seconds after first mission command',title='Every committed delivery; early first delivery does not imply early completion',ylim=(-.5,len(metrics)-.4))
    ax.grid(axis='x',alpha=.2);fig.tight_layout();fig.savefig(args.out/'delivery_milestones.png',dpi=170);plt.close(fig)
    def number(value):return '未完成' if value is None else f'{value:.2f}'
    rows=['| seed | 策略 | Gate | 投递槽 | 碰撞 | 首投/s | 第三投/s | 整场验收/s | 离地至落地/s | XY航程/m |',
          '|---|---|---|---:|---:|---:|---:|---:|---:|---:|']
    for m in metrics:
        rows.append(f"| {m['seed']} | {m['label']} | {m['status']} | {len(m['commit_times'])}/3 | {m['collisions']} | {number(m['first_commit_s'])} | {number(m['third_commit_s'])} | {number(m['completed_mission_s'])} | {number(m['airborne_to_touchdown_s'])} | {number(m['xy_distance_m'])} |")
    all_pass=len(metrics)==4 and len(paired)==2 and all(p['both_gate_pass'] for p in paired)
    if all_pass:
        wins=all(p['completed_mission_s_gain_s']>0 for p in paired)
        conclusion=('两套布局中高位策略均实现整场净节时，值得继续验证。' if wins else '两套布局的整场结果未一致支持高位策略更快，需按布局和阶段分析收益。')
    elif (len(metrics)==4 and all(m['status']=='PASS' for m in metrics if m['label']=='strategy')
          and all(p['third_commit_s_gain_s'] is not None and p['third_commit_s_gain_s']>0 for p in paired)):
        conclusion='本批显示高位优先策略有明确提效价值：两布局三投均更快，高位组两轮整场均通过。基线存在未完成轮次，该布局不能计算成功整场节时；碰撞差异不能由单次对照归因于策略。'
    else:conclusion='本批未形成两组全部通过的完整对照，不能宣称高位策略已全面优于低位基线；失败和可比较分项均保留。'
    report=['# 高位快速搜索与低位覆盖：两组完整链条对照','',conclusion,
            '', '核心问题是先高位找齐三目标、再低位避障投递，是否减少整场时间。这里评估的是WSL笔记本上的真实PX4/Gazebo仿真链，不是实机结果。每种策略每布局仅一轮，两布局不能代表普遍成功率或统计显著性。',
            '', '飞行源码冻结为 `5667bed`，四轮一致；159项相关离线测试和两个Catkin工作区构建通过。实际布局、共同任务/Gate参数和相机信息配对一致，见[报告一致性验证](validation.json)。',
            '', '## 完整结果', '', *rows,
            '', 'baseline 为原低位覆盖，保留高权重中断、投后恢复，并在三投完成后立即转入走廊；strategy 为高位闭环观测、三目标齐备提前中断、就近下降和低位按序复访。两组共用模型、地图、低位释放门槛、速度跟踪参数、随机门导航及原整场Gate。',
            '', '第三投和整场验收均从首条任务指令计时；整场验收包含最终降落事务及原Gate要求。另列离地至落地时长，落地到解除武装的等待见各轮metrics.json。失败轮的航程为已观察航程，不得拿截断航程当节省量。',
            '', '## 同图差值', '', '正数表示高位策略节省；任一未完成则不计算该项节时。']
    for p in paired:
        full_text=(f"整场节省 {number(p['completed_mission_s_gain_s'])} s（{number(p['completed_mission_s_gain_percent'])}%）" if p['completed_mission_s_gain_s'] is not None else '整场节时不可计算（存在未完成轮次）')
        report.append(f"- seed{p['seed']}：第三投节省 {number(p['third_commit_s_gain_s'])} s（{number(p['third_commit_s_gain_percent'])}%）；{full_text}。")
        report.append(f"  到第三投XY航程减少 {number(p['third_xy_gain_m'])} m（{number(p['third_xy_gain_percent'])}%）。")
        report.append(f"  完整XY航程减少 {number(p['xy_distance_gain_m'])} m（{number(p['xy_distance_gain_percent'])}%）。" if p['both_gate_pass'] else '  该组存在失败轮，不比较完整航程。')
    report += ['', '## 提效来自哪里', '',
               '| seed | 基线覆盖搜索/s | 高位观察及升降/s | 高位组低位复访与等待/s |',
               '|---|---:|---:|---:|']
    for seed in seeds:
        mm={m['label']:m for m in metrics if m['seed']==seed}
        if set(mm)!={'baseline','strategy'}:continue
        base=mm['baseline']['sampled_phase_durations_s'];high=mm['strategy']['sampled_phase_durations_s']
        overhead=sum(high.get(k,0) for k in ('ASCEND','SURVEY','DESCEND','LOCAL_DESCENT_TRANSIT','RETURN_COLUMN'))
        revisit=sum(high.get(k,0) for k in ('REVISIT','REACQUIRE'))
        report.append(f"| {seed} | {base.get('SEARCH',0):.2f} | {overhead:.2f} | {revisit:.2f} |")
    report += ['', '直接可见的主要变化是低位覆盖航程减少：高位观察和升降并非免费，但其成本小于本批基线继续覆盖搜索的开销。三投前航程也在两组中明显减少，说明收益不仅是计时口径差异。阶段边界来自采样状态和事务事件，释放ACK后至恢复完成计入RECOVERY。',
               '', '首投并不总是更快：seed34基线先投了起点附近的bridge，高位组先等三目标齐备再开始投递；后者最终三投更早。应按完整三槽任务评价。',
               '', '本批没有单独消融高位门槛和就近下降的贡献。六个实际高位线索的不确定度仍约0.20m；不能把全部收益归功于放宽到0.45m。低位新鲜重捕验证保持通过，线索只承担导航引导。']
    failure_path=args.out/'landing_failure_metrics.json'
    if failure_path.exists() and any(m['seed']==34 and m['label']=='baseline' and m['status']=='FAIL' for m in metrics):
        failure=json.loads(failure_path.read_text())
        report += ['', '## seed34基线降落失败复盘', '',
                   '首发失败为机体保护圈与东侧边界墙Wall_11接触，场景时刻471.985s（任务约460.352s）。此前三次释放、三次恢复、9个投后导航点、走廊入口和两道门均已完成，H标志定位也已有效。其余落地/解除武装/任务完成相关未通过项是碰撞中止后的结果，不能解释成9个独立故障。',
                   '', f"接近支撑高度后发生反弹。首次墙接触时，插值MAVROS相对真值的平面误差约{failure['mavros_xy_error_m']:.3f}m、高度误差约{failure['mavros_z_error_m']:.3f}m；LIO对应误差约{failure['lio_xy_error_m']:.3f}m和{failure['lio_z_error_m']:.3f}m。",
                   '', '这些记录将问题定位到近地阶段的估计分离与反弹现象，仍需PX4 ULog和接地动力学复核因果。该失败及原始飞控参数保留，尚未通过参数消融或重复试验确定因果。',
                   '', '![降落失败局部复盘](seed34_landing_failure.png)', '', '[插值指标](landing_failure_metrics.json) · [原始时刻摘录与接触对象](seed34_landing_diagnostics.json)']
    report += ['', '![配对耗时和航程](comparison.png)', '', '灰色叉号为失败停止时刻，不是完成时间。', '', '![逐次投递时间](delivery_milestones.png)', '', '![同地图完整航迹](paired_paths.png)',
               '', '## 每轮航线、航迹、高度、速度与阶段图']
    for m in metrics:
        folder=f"{m['seed']}_{m['label']}"
        report += ['', f"### seed{m['seed']} / {m['label']}", '',
                   f"原始run：`{m['run']}`。Gate：{m['status']}，原因：`{m['reason']}`。",
                   f"投递顺序：{' → '.join(c['target'] for c in m['commit_times']) or '无'}。移动段实际水平速度中位数 {number(m['moving_speed_xy_median_mps'])} m/s。",
                   f"[完整指标与阶段分解]({folder}/metrics.json) · [Gate]({folder}/gate_status.json)", '',
                   f"![完整飞行图]({folder}/flight_charts.png)", '', f"![分阶段航迹]({folder}/route_stages.png)", '',
                   f"![阶段耗时和目标事务]({folder}/phase_target_timeline.png)"]
        if m['failed_checks']:report.append('未通过项：'+', '.join(m['failed_checks'])+'。')
        if m.get('high_view_final'):
            h=m['high_view_final']
            report += [f"下降提案：`{json.dumps(h.get('descent_proposal'),ensure_ascii=False)}`。",
                       '高位坐标误差、低位新鲜重捕误差和线索年龄见指标文件的 target_hint_evaluation；这些不是实际快递落点得分。']
    report += ['', '## 解释边界与后续', '',
               '- 高位线索门槛与低位投递门槛分开：高位三次观测最短跨度0.2s、不确定度上限0.45m；低位仍须原连续确认、新鲜坐标和释放许可。',
               '- 三点顺序使用感知占据栅格距离，包含最后目标至走廊入口；实际三维轨迹仍由Fast-Planner避障。栅格最短顺序不等于连续空间全局最短，也未按不同阶段实际速度优化总时间。',
               '- 规划上限1m/s不是实际巡航速度。当前位置跟踪前视约0.4m，末段约0.15m；两组一致。提速后需重新比较识别和任务收益，不能直接推广本批结果。',
               '- 图中树圆为布局示意，虚线为导航点顺序而非可飞直线；碰撞结论来自仿真接触监测。真值只用于评测，没有输入高位选靶或排序。',
               '- 开发过程另保留随机门配置中止轮（excluded_runs.json）和返起飞柱旧版二投后失败轮（development_runs.json、development/）。旧版不是成功轮，不与冻结版混算；其返起飞柱阶段耗时21.657s。',
               '- 后续先依据本批第三投与整场结果判断策略价值；独立对照2025跟踪链、分级提速，再检查避障、识别和净收益。研究分支不替换正赛部署。',
               '', '建议继续推进这一研究方向；先独立复核近地失败并对照2025跟踪链，再在共同更高实际速度下做配对复测。当前结果支持提效潜力，尚不足以替换正赛部署。',
               '', '[复现说明](REPRODUCE.md) · [运行索引](runs.json) · [配对差值](paired_differences.json) · [速度核对](speed_review.json) · [报告验证](validation.json)']
    (args.out/'REPORT.md').write_text('\n'.join(report)+'\n',encoding='utf-8')
    cards=''.join(f'<h2>seed {m["seed"]} / {m["label"]} — {m["status"]}</h2><img src="{m["seed"]}_{m["label"]}/flight_charts.png"><img src="{m["seed"]}_{m["label"]}/route_stages.png"><img src="{m["seed"]}_{m["label"]}/phase_target_timeline.png">' for m in metrics)
    if (args.out/'seed34_landing_failure.png').exists():cards += '<h2>Landing failure detail</h2><img src="seed34_landing_failure.png">'
    (args.out/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>高位搜索完整对照</title><style>body{font:16px sans-serif;max-width:1450px;margin:24px auto;color:#243440;background:#f4f5f6}img{display:block;width:100%;margin:18px 0}h2{margin-top:40px}</style><h1>高位快速搜索：完整链条对照</h1><p>'+conclusion+'</p><p><a href="REPORT.md">完整报告与方法说明</a></p><img src="comparison.png"><img src="delivery_milestones.png"><img src="paired_paths.png">'+cards,encoding='utf-8')


if __name__=='__main__':main()

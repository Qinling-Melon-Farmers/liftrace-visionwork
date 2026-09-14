"""Two requested pilot runs only; preserves failed full-mission outcome."""
import json, html, importlib.util
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml

D=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('flight_report',D.parent.parent/'high_view_full_20260914/analyze.py')
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)

def main():
    labels=['baseline','strategy_repaired'];names=['Low coverage (PASS)','High survey repair (FAIL at landing)'];colors=['#2166ac','#d97706']
    items={v['label']:v for v in json.loads((D/'runs.json').read_text())}
    ms=[json.loads((D/'results'/f'32_{k}'/'metrics.json').read_text()) for k in labels]
    trajectories=[];truths=[];params=[]
    for k in labels:
        run=Path(items[k]['run']);trajectories.append(base.csv4(run/'truth_pose.csv'))
        truths.append(yaml.safe_load((run/'random_field_truth.yaml').read_text()))
        params.append(yaml.safe_load((run/'rosparams.yaml').read_text()))
    points=lambda t:sorted((v['class'],v['world_x'],v['world_y']) for v in t['targets'])
    assert points(truths[0])==points(truths[1])
    fig,axes=plt.subplots(1,3,figsize=(18,7))
    for ax in axes:base.scene(ax,Path(items['baseline']['world']),truths[0])
    for i,(a,m,k,name,c) in enumerate(zip(trajectories,ms,labels,names,colors)):
        for ax in (axes[i],axes[2]):
            ax.plot(a[:,1],a[:,2],lw=1.2,color=c,label=name)
            for commit in m['commit_times']:
                xy=[np.interp(commit['t'],a[:,0],a[:,j]) for j in (1,2)]
                ax.scatter(*xy,marker='*',s=120,color=c,edgecolors='black',linewidths=.5,zorder=5)
            if m['status']!='PASS':ax.scatter(a[-1,1],a[-1,2],marker='x',s=90,color='red',zorder=6)
        axes[i].set_title(name,fontsize=11)
    axes[2].set_title('Same scene: route overlay',fontsize=11)
    for ax in axes:ax.legend(loc='lower left',fontsize=7)
    fig.suptitle('Fixed-tree seed32 pilots | stars: committed deliveries | red X: failed run end')
    fig.tight_layout();fig.savefig(D/'two_routes.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(3,1,figsize=(13,10),sharex=True)
    region_stats=[]
    for a,m,p,name,c in zip(trajectories,ms,params,names,colors):
        start=m['start_ros_s'];h=a[:,3]+p['competition_key_recorder']['truth_world_offset'][2]
        axes[0].plot(a[:,0]-start,h,color=c,lw=.85,label=name)
        dt=np.diff(a[:,0]);ok=(dt>0)&(dt<=.5);delta=np.diff(a[:,1:4],axis=0)
        axes[1].plot(a[1:,0][ok]-start,np.linalg.norm(delta[:,:2][ok],axis=1)/dt[ok],color=c,lw=.7)
        axes[2].plot(a[1:,0][ok]-start,delta[ok,2]/dt[ok],color=c,lw=.7)
        for commit in m['commit_times']:
            for ax in axes:ax.axvline(commit['t']-start,color=c,alpha=.25,ls=':')
        # Same XY rectangle as the archived gate, evaluated on recorded truth.
        region=(a[:,1]>=-4.8)&(a[:,1]<=4.8)&(a[:,2]>=7.6)&(a[:,2]<=9.1)
        region_stats.append(dict(label=m['label'],region_max_fc_agl_m=float(h[region].max()) if region.any() else None,
                                 recorded_region_samples_above_1_5=int(np.count_nonzero(region&(h>1.5))),
                                 scope='Recorded 10Hz truth, until the original gate stopped the run'))
    axes[0].axhline(1.5,color='gray',ls='--',lw=.8,label='1.5m reference (not the original gate)')
    axes[0].axhline(.7,color='gray',ls=':',lw=.8,label='Original corridor gate: 0.7m')
    axes[0].set_ylabel('True FC AGL (m)');axes[0].legend(fontsize=8,ncol=2)
    axes[1].set_ylabel('Horizontal speed (m/s)');axes[2].set_ylabel('Vertical speed (m/s)')
    axes[2].set_xlabel('Simulation seconds after first mission command')
    for ax in axes:ax.grid(alpha=.2)
    fig.suptitle('Height and actual speeds | dotted colored lines: delivery commits')
    fig.tight_layout();fig.savefig(D/'two_height_speed.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(13,4))
    keys=['third_commit_s','third_commit_to_corridor_entry_s','completed_mission_s']
    titles=['Third delivery (s)','Third delivery to corridor entry (s)','Successful full mission (s)']
    for ax,key,title in zip(axes,keys,titles):
        for i,m in enumerate(ms):
            value=m[key]
            if value is not None:
                ax.bar(i,value,color=colors[i]);ax.text(i,value,f'{value:.2f}',ha='center',va='bottom')
            else:ax.text(i,.1,'FAIL\nNo completion time',ha='center',transform=ax.get_xaxis_transform(),color='firebrick')
        ax.set_xticks([0,1],['Low coverage','High repair']);ax.set_xlim(-.6,1.6);ax.set_title(title);ax.grid(axis='y',alpha=.2);ax.margins(y=.15)
    fig.tight_layout();fig.savefig(D/'two_times.png',dpi=160);plt.close(fig)
    (D/'two_pilots.json').write_text(json.dumps(dict(runs=ms,corridor_height_check=region_stats,same_targets=True),indent=2))
    photos=[('两轮完整航迹与叠加','two_routes.png'),('两轮实际高度与速度','two_height_speed.png'),('三投、转场和完赛耗时','two_times.png')]
    for k,title in zip(labels,['原覆盖提速先导','最新高位修复先导']):
        for f,caption in [('flight_charts','航迹／高度／速度／指令'),('route_stages','按阶段着色的航线'),('route_3d','三维航迹'),('speed_profile','实际速度与前视参数切换'),('phase_target_timeline','阶段耗时与投递事件')]:
            photos.append((f'{title}：{caption}',f'results/32_{k}/{f}.png'))
    header='原覆盖：PASS，三投195.716s，整场348.954s。最新高位修复：三投86.080s、两门通过、0碰撞；H降落阶段触发原0.7m走廊高度门槛，整场FAIL。'
    notes='高位缩短虚拟障碍柱的方案不能作为“禁止从障碍上方飞越”规则下的合规验收。本页保留实际试验图，不据此批准该方案或追改原Gate。两轮都是固定树seed32先导，不是全随机矩阵。'
    stats='；'.join(f"{v['label']}在走廊矩形内已记录的FC最高离地高度{v['region_max_fc_agl_m']:.3f}m" for v in region_stats)
    body=''.join(f'<figure><figcaption>{html.escape(title)}</figcaption><a href="{link}"><img loading="lazy" src="{link}" alt="{html.escape(title)}"></a></figure>' for title,link in photos)
    (D/'index.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>先导两轮完整图表</title><style>body{font:16px/1.7 system-ui,sans-serif;max-width:1400px;margin:24px auto;padding:0 18px;color:#243447;background:#f3f5f7}figure{background:white;padding:16px;margin:24px 0;border-radius:8px}img{width:100%;height:auto}figcaption{font-weight:600}a{color:#1767a4}.note{padding:16px;background:#fff0d5}</style><h1>先导两轮完整图表</h1><p>'+header+'</p><p class="note">'+notes+'</p><p>'+stats+'。只覆盖原Gate停止前的数据，不能推断继续飞行会成功。</p><p><a href="TWO_PILOTS.md">图表索引与口径</a> · <a href="two_pilots.json">完整指标</a></p>'+body+'</html>')
    lines=['# 先导两轮图表','',header,'',notes,'',stats+'。若改按1.5m判断，已记录区间未见越限；但本轮已在原0.7m门槛停止，不能据此推定后续落地成功或改判PASS。','',f"原始目录：`{items['baseline']['run']}`；`{items['strategy_repaired']['run']}`。",'', '[统一图表浏览入口](index.html)','']
    for title,link in photos:lines += [f'## {title}','',f'![{title}]({link})','']
    (D/'TWO_PILOTS.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(region_stats,indent=2));print('13 charts indexed; 3 new comparison figures + 10 existing full-flight figures.')

if __name__=='__main__':main()

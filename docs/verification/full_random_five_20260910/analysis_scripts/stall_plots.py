from pathlib import Path
import json,sys,re
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=next(p for p in Path(__file__).resolve().parents if (p/'AGENTS.md').exists() and (p/'patrol_uav_ws-patrol_planner').is_dir())
O=R/'logs/_artifacts/full_random_five_20260910';D=R/'docs/verification/full_random_five_20260910'
sys.path.insert(0,str(Path(__file__).resolve().parent));from analyze import scene,csv,events
for seed in [33,34]:
    dd=D/('seed_%02d'%seed);row=json.loads((dd/'metrics.json').read_text());run=Path(row['run_dir']);ev=events(run);a=csv(run/'truth_pose.csv');cmd=csv(run/'mavros_setpoint.csv')
    fig,axs=plt.subplots(1,3,figsize=(16,5.4));scene(axs[0],run)
    low,hi=(330,490) if seed==33 else (20,420);v=a[(a[:,0]>=low)&(a[:,0]<=hi)]
    axs[0].plot(v[:,1],v[:,2],c='#1467a3',lw=1.5,label='Actual path in interval')
    targets=[(2.3,8.35)] if seed==33 else [(4.3,0),(4.3,.7)]
    for x,y in targets:axs[0].scatter(x,y,c='#ab183b',marker='X',s=80,zorder=8)
    axs[0].set_title('Failed requested goals (X), actual path');axs[0].legend(fontsize=7)
    if seed==33:
        axs[0].set(xlim=(-2.7,3.1),ylim=(7.3,9.4))
        for col,label in [(1,'Actual X'),(2,'Actual Y')]:axs[1].plot(v[:,0],v[:,col],label=label)
        axs[1].axvspan(393.986,483.996,color='#d07757',alpha=.16,label='90 s failed route goal')
        axs[1].set(title='No progress through the second door',ylabel='Position (m)')
    else:
        axs[1].plot(v[:,0],v[:,1],label='Actual X',c='#1467a3');axs[1].axhline(4.3,c='#ab183b',ls='--',label='Requested X = 4.3')
        for i,(st,en) in enumerate([(42.63,132.634),(132.634,222.651),(222.651,312.657),(312.657,402.68)]):
            axs[1].axvspan(st,en,color=('#d07757' if i%2==0 else '#788ca3'),alpha=.18)
            axs[1].text((st+en)/2,2.8,'90 s',ha='center',fontsize=8)
        axs[1].set(title='Four deadlines consumed ~360 s',ylabel='X position (m)')
    failures=[e['ros_sec'] for e in ev if e['kind']=='planner' and e['data'].get('reason')=='new_trajectory_attempt_failed' and low<=e['ros_sec']<=hi]
    axs[2].step(failures,range(1,len(failures)+1),where='post',color='#a83d46');axs[2].set(title='Failed new-trajectory attempts',ylabel='Cumulative recorded attempts')
    for ax in axs[1:]:ax.set_xlabel('ROS simulation time (s)');ax.grid(alpha=.2)
    axs[1].legend(fontsize=7);fig.suptitle('Seed %d | critical planning interval, %.0f–%.0f ROS s'%(seed,low,hi),fontsize=13);fig.tight_layout(rect=(0,0,1,.96));fig.savefig(dd/'planning_stall.png',dpi=150);plt.close(fig)
    if seed==34:
        lines=(run/'run.log').read_text(errors='replace').splitlines();i=next(i for i,s in enumerate(lines) if 'kinodynamic search fail!' in s)
        (dd/'planner_log_excerpt.txt').write_text('\n'.join(re.sub(r'\x1b\[[0-9;]*m','',l) for l in lines[i-13:i+16]))

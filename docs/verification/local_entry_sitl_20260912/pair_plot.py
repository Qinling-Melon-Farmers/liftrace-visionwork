"""Plot completed paired trials without running ROS."""
from pathlib import Path
import json, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[3]
OUT=Path(__file__).resolve().parent
COLORS=['#276b96','#c26a24']

def pair(seed):
    records=[json.loads((OUT/f'local_entry_seed{seed}_{mode}_metrics.json').read_text()) for mode in ['baseline','active']]
    from base_analysis import field
    fig,ax=plt.subplots(1,3,figsize=(16,6),gridspec_kw={'width_ratios':[1.15,1.2,.75]})
    field(ax[0],ROOT/records[0]['run_dir'])
    for i,r in enumerate(records):
        pose=np.loadtxt(ROOT/r['run_dir']/'truth_pose.csv',delimiter=',',skiprows=1)
        end=r['mission_start_ros']+r['third_release_s'] if r['third_release_s'] is not None else r['last_ros']
        p=pose[(pose[:,0]>=r['mission_start_ros'])&(pose[:,0]<=end)]
        ax[0].plot(p[:,1],p[:,2],lw=.9,color=COLORS[i],label=['Baseline','Local entry'][i])
        for j,entry in enumerate(r['local_entries']):
            xy=entry['entry'][:2];ax[0].scatter(*xy,color=COLORS[i],marker='D',s=25,zorder=5)
            ax[0].annotate(f'E{j+1}',xy,xytext=(3,4),textcoords='offset points',fontsize=8)
    ax[0].set_title('Actual flight up to third release\nDiamonds: commanded local entries');ax[0].legend(fontsize=8)
    labels=['Third release','SEARCH + RESUME','Initial-plan wait']
    for i,r in enumerate(records):
        values=[r['third_release_s'],r['command_motion']['search_resume_s'],r['command_motion']['initial_wait_s']]
        x=np.arange(3)+(i-.5)*.36
        bars=ax[1].bar(x,[v if v is not None else 0 for v in values],width=.36,color=COLORS[i],label=['Baseline','Local entry'][i])
        for bar,v in zip(bars,values):ax[1].text(bar.get_x()+bar.get_width()/2,bar.get_height()+4,'N/A' if v is None else f'{v:.1f}',ha='center',fontsize=8)
    ax[1].set(xticks=range(3),xticklabels=labels,ylabel='ROS seconds',title='Timing: lower is better');ax[1].tick_params(axis='x',labelsize=8)
    ax[1].set_ylim(0,max(r['command_motion']['search_resume_s'] for r in records)*1.3);ax[1].legend(fontsize=8)
    values=[r['command_motion']['search_resume_xy_m'] for r in records]
    bars=ax[2].bar(['Baseline','Local entry'],values,color=COLORS,width=.55)
    for bar,v in zip(bars,values):ax[2].text(bar.get_x()+bar.get_width()/2,v+1,f'{v:.2f}',ha='center',fontsize=9)
    ax[2].set(ylabel='Actual XY distance (m)',title='SEARCH + RESUME distance');ax[2].set_ylim(0,max(values)*1.15)
    for axis in ax[1:]:axis.grid(axis='y',alpha=.2);axis.set_axisbelow(True)
    fig.suptitle(f'Seed {seed}, paired 0.70 m lanes | baseline {records[0]["status"]}, local entry {records[1]["status"]}\nSame layout and configuration; one trial per treatment, not a robustness estimate')
    fig.tight_layout(rect=(0,0,1,.9));fig.savefig(OUT/f'seed{seed}_comparison.png',dpi=150);plt.close(fig)
    if all(r.get('alignment_intervals') and r.get('third_release_s') is not None for r in records):
        fig,axis=plt.subplots(figsize=(10,3.8))
        for i,r in enumerate(records):
            last=r['alignment_intervals'][-1]
            prior=last['alignment_accepted_ros']-r['mission_start_ros'];align=last['alignment_to_release_s']
            axis.barh(i,prior,color='#6e8ba0',label='Before third alignment accepted' if i==0 else None)
            axis.barh(i,align,left=prior,color='#df9a42',label='Third alignment to release' if i==0 else None)
            axis.text(prior/2,i,f'{prior:.2f}s',ha='center',va='center',color='white')
            axis.text(prior+align+3,i,f'total {r["third_release_s"]:.2f}s; align {align:.2f}s',va='center',fontsize=9)
        axis.set(yticks=[0,1],yticklabels=['Baseline','Local entry'],xlabel='ROS seconds since first recorded decision',
            title=f'Seed {seed}: third-release time decomposition',xlim=(0,max(r['third_release_s'] for r in records)*1.48))
        axis.invert_yaxis();axis.legend(loc='lower right',fontsize=8);axis.grid(axis='x',alpha=.15)
        fig.tight_layout();fig.savefig(OUT/f'seed{seed}_release_breakdown.png',dpi=150);plt.close(fig)

if __name__=='__main__':
    for seed in sys.argv[1:] or [32,34]:pair(int(seed))

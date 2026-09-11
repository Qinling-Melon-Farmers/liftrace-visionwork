#!/usr/bin/env python3
"""Standalone report plots from measured JSON and a labelled pacing fixture."""
import json
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[2]/'vision_ws/src/uav_coverage_memory/src'))
from uav_coverage_memory.resources import WorkBudget


def main():
    data=json.loads((HERE/'benchmark_ros.json').read_text())['modes']
    colors=['#607d8b','#007f87']
    fig,axes=plt.subplots(1,3,figsize=(14,4.6))
    for ax,mode,title in zip(axes[:2],['historical','stress'],['252 recorded samples','500k-point projection stress']):
        x=np.arange(2)
        for offset,version,color in zip([-.18,.18],['baseline','current'],colors):
            values=[data[mode]['metrics'][version]['wall_ms'][k] for k in ['p50','p95']]
            bars=ax.bar(x+offset,values,.36,label=version,color=color)
            ax.bar_label(bars,fmt='%.1f',padding=3,fontsize=9)
        ax.set_xticks(x,['Median','P95']);ax.set_ylabel('Core wall time (ms)');ax.set_title(title)
        ax.set_ylim(0,ax.get_ylim()[1]*1.15);ax.grid(axis='y',alpha=.2);ax.legend(frameon=False)
    for offset,version,color in zip([-.18,.18],['baseline','current'],colors):
        values=[data[m]['metrics'][version]['peak_process_rss_mib'] for m in ['historical','stress']]
        bars=axes[2].bar(np.arange(2)+offset,values,.36,label=version,color=color)
        axes[2].bar_label(bars,fmt='%.1f',padding=3,fontsize=9)
    axes[2].set_xticks([0,1],['Recorded replay','Stress fixture'])
    axes[2].set_ylabel('Peak worker RSS (MiB)');axes[2].set_title('Separate worker processes')
    axes[2].grid(axis='y',alpha=.2);axes[2].set_ylim(0,520)
    fig.suptitle('Resource comparison | x86 WSL, Python 3.8 / OpenCV 4.2, one native thread',fontsize=13)
    fig.text(.5,.025,'Core-only timing; RSS includes interpreter and benchmark buffers. No onboard or flight-speed claim.',ha='center',fontsize=10)
    fig.tight_layout(rect=(0,.07,1,.94));fig.savefig(HERE/'resources.png',dpi=160);plt.close(fig)

    budget=WorkBudget(3,.15,150)
    accepted=[];skipped=[];work=[]
    for tick in np.arange(0,12,1/3):
        if not budget.ready(tick):
            skipped.append(tick);continue
        cost=.18 if 3<=tick<8 else .025
        budget.finish(tick,tick+cost,cost)
        accepted.append(tick);work.append((tick,cost))
    fig,axes=plt.subplots(2,1,figsize=(11,5),sharex=True,gridspec_kw={'height_ratios':[1,1.2]})
    axes[0].broken_barh(work,(.2,.5),facecolors=colors[1])
    axes[0].scatter(skipped,np.full(len(skipped),.08),marker='x',color='#ad5535',label='Skipped tick')
    axes[0].set_yticks([]);axes[0].set_ylim(0,1);axes[0].legend(loc='upper right',frameon=False)
    axes[0].set_title('Synthetic pacing example: 25 ms CPU/frame, then 180 ms, then 25 ms | duty target 0.15 core')
    axes[1].plot(accepted[1:],np.diff(accepted),'-o',color=colors[1],markersize=4,label='Processed-image interval')
    axes[1].axhline(.8,color='#ad5535',ls='--',label='Unchanged maximum observation gap: 0.8 s')
    for ax in axes:
        ax.axvspan(3,8,color='#eddcc2',alpha=.4,zorder=-1);ax.grid(alpha=.2)
    axes[1].set_xlabel('Wall time (s)');axes[1].set_ylabel('Source-time gap (s)');axes[1].legend(frameon=False)
    fig.text(.5,.02,'Illustrative CPU-bound fixture, not board measurements. Excessive gaps break new multi-frame confirmation.',ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.05,1,1));fig.savefig(HERE/'budget_pacing.png',dpi=160);plt.close(fig)


if __name__=='__main__':main()

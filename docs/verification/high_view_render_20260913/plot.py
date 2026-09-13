"""Render evaluation figures, not flight trajectory claims."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import yaml

root=Path(__file__).resolve().parent
metrics=json.loads((root/'results/metrics.json').read_text())
segments=json.loads((root/'results/segments.json').read_text())
rows=[json.loads(s) for s in (root/'results/predictions.jsonl').read_text().splitlines()]
sample_run=Path(rows[0]['image']).parents[1]
camera=json.loads((sample_run/'captures.jsonl').read_text().splitlines()[0])
heights=[1.4,2.4,2.6,2.8];classes=['tent','pillbox','bridge','panzer','red_cross']
fig,axs=plt.subplots(1,2,figsize=(12,4.8))
for ax,seed in zip(axs,[32,34]):
    values=np.array([[int(any(s['confirmed_three'] for s in segments if s['seed']==seed and s['fc_agl']==h and s['class_name']==c)) for c in classes] for h in heights])
    ax.imshow(values,vmin=0,vmax=1,cmap='Blues',aspect='auto')
    for i,h in enumerate(heights):
        for j,c in enumerate(classes):ax.text(j,i,'seen' if values[i,j] else 'not seen',ha='center',va='center',color='white' if values[i,j] else 'black',fontsize=9)
    ax.set(xticks=range(5),xticklabels=classes,yticks=range(4),yticklabels=heights,ylabel='FC AGL (m)',title='seed%d: at least one 3-frame view'%seed)
fig.suptitle('Same sparse observation grid; not a comparison with full low-altitude search')
fig.tight_layout();fig.savefig(root/'discovery.png',dpi=160);plt.close(fig)

fig,ax=plt.subplots(figsize=(8,4.5));bottom=np.zeros(4)
for cls in classes:
    count=np.array([sum(s['confirmed_three'] for s in segments if s['fc_agl']==h and s['class_name']==cls) for h in heights])
    ax.bar(range(4),count,bottom=bottom,label=cls);bottom+=count
for i,n in enumerate(bottom):ax.text(i,n+.3,str(int(n)),ha='center')
ax.set(xticks=range(4),xticklabels=heights,xlabel='FC AGL (m)',ylabel='Object-view observations (three correct frames)',
       title='40 stationary view positions per height; counts are not recall or time savings',ylim=(0,30))
ax.legend(ncol=3,fontsize=8);fig.tight_layout();fig.savefig(root/'opportunities.png',dpi=160);plt.close(fig)

fig,axs=plt.subplots(1,2,figsize=(12,4.5))
errors=[[np.median(s['errors']) for s in segments if s['fc_agl']==h and s['eligible'] and s['errors']] for h in heights]
axs[0].boxplot(errors,tick_labels=heights)
axs[0].set(xlabel='FC AGL (m)',ylabel='Projected box-center error (m)',title='Exact rig pose; one median per eligible view')
q=json.loads((root/'results/projection_check.json').read_text())
for axis,label in [(0,'u'),(1,'v')]:
    axs[1].plot([s['fc_agl'] for s in q['summary']],[s['median_delta_px'][axis] for s in q['summary']],'-o',label=label+' observed ring residual')
axs[1].axhline(camera['width']/2-camera['k'][2],ls='--',color='C0',alpha=.5,label='image center - reported cx')
axs[1].axhline(camera['height']/2-camera['k'][5],ls='--',color='C1',alpha=.5,label='image center - reported cy')
axs[1].set(xlabel='FC AGL (m)',ylabel='Pixel residual',title='Camera projection consistency diagnostic');axs[1].legend(fontsize=7)
for ax in axs:ax.grid(alpha=.2)
fig.tight_layout();fig.savefig(root/'projection_errors.png',dpi=160);plt.close(fig)

fig,axs=plt.subplots(2,4,figsize=(16,5.7))
for i,view in enumerate([19,17]):
    for j,h in enumerate(heights):
        row=next(r for r in rows if r['seed']==32 and r['fc_agl']==h and r['view']==view and r['frame']==0)
        ax=axs[i,j];ax.imshow(plt.imread(row['image']))
        for p in row['detections']:
            x0,y0,x1,y1=p['bbox'];ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,fill=False,color='lime',lw=1))
            ax.text(x0,max(15,y0-5),p['class_name']+' %.2f'%p['confidence'],color='lime',fontsize=8,backgroundcolor='black')
        ax.set_title('seed32 view%d / %.1fm'%(view,h),fontsize=10);ax.axis('off')
fig.suptitle('Actual rendered frames and unchanged PT detector output; fixed positions across heights')
fig.tight_layout();fig.savefig(root/'examples.png',dpi=150);plt.close(fig)

fig,axs=plt.subplots(1,2,figsize=(10,6))
for ax,seed in zip(axs,[32,34]):
    row=next(r for r in rows if r['seed']==seed)
    run=Path(row['image']).parents[1]
    truth=yaml.safe_load((run/'random_field_truth.yaml').read_text())
    cfg=json.loads((run/'capture_config.json').read_text())
    field=yaml.safe_load((root/('seed_%d'%seed)/'field_config.yaml').read_text())
    for t in field['static_exclusions']:ax.add_patch(Circle((t['world_x'],t['world_y']),t['radius'],color='forestgreen',alpha=.3))
    for t in truth['targets']:
        ax.scatter(t['world_x'],t['world_y'],marker='s',s=60)
        ax.annotate(t['class'],(t['world_x'],t['world_y']),xytext=(4,5),textcoords='offset points',fontsize=8)
    ax.scatter(*zip(*cfg['view_xy']),marker='+',color='black',label='stationary views x 4 heights')
    ax.set(xlim=(-5,5),ylim=(-.6,7.5),xlabel='world X (m)',ylabel='world Y (m)',title='seed%d fixed trees + random targets'%seed)
    ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=7)
fig.suptitle('Evaluation view grid: cameras repositioned, not an aircraft route')
fig.tight_layout();fig.savefig(root/'layouts.png',dpi=160)

fig,ax=plt.subplots(figsize=(8,3.8))
for seed in [32,34]:
    rs=[r for r in rows if r['seed']==seed]
    ax.plot([r['stamp']-rs[0]['stamp'] for r in rs],[r['fc_agl'] for r in rs],'.-',ms=2,label='seed%d'%seed)
ax.set(xlabel='Sim seconds after first capture',ylabel='Rig FC-equivalent AGL (m)',
       title='Camera rig height sequence; repositioning, no aircraft dynamics')
ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(root/'capture_heights.png',dpi=160)

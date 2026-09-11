"""Offline XY passage markers around fixed trees; NOT a visibility/clearance test."""
from pathlib import Path
import json,sys,xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
OUT=Path(__file__).resolve().parent

def search_mask(pose,events):
    decisions={x['data']['decision_seq']:x for x in events if x['kind']=='decision'}
    ordered=sorted(decisions.values(),key=lambda x:x['ros_sec'])
    terminal={}
    for x in events:
        if x['kind']=='result' and x['data'].get('terminal'):terminal.setdefault(x['data']['decision_seq'],x)
    mid=(pose[:-1,0]+pose[1:,0])/2
    mask=np.zeros(len(mid),dtype=bool)
    for i,x in enumerate(ordered):
        d=x['data']
        if d['command'] not in [0,3]:continue
        end=min(ordered[i+1]['ros_sec'] if i+1<len(ordered) else pose[-1,0],terminal.get(d['decision_seq'],{}).get('ros_sec',pose[-1,0]))
        mask|=(mid>=x['ros_sec'])&(mid<end)
    return mask

def passages(pose,mask,tree_xy,half_x=.5,half_y=1.5,deadband=.2):
    """Require motion from one X edge to the other inside the Y window.

    Reset on non-search, a >0.5 s sampling gap, or leaving the Y window.
    A center-line crossing without reaching the opposite X edge is not counted.
    Center-line Y is interpolated from consecutive truth poses.
    """
    cx,cy=tree_xy;anchor=0;candidate=None;result=[]
    for i,valid in enumerate(mask):
        a,b=pose[i],pose[i+1];dt=b[0]-a[0]
        if not valid or not 0<dt<=.5 or max(abs(a[2]-cy),abs(b[2]-cy))>half_y:
            anchor=0;candidate=None;continue
        edge=lambda x:-1 if x<=cx-half_x else (1 if x>=cx+half_x else 0)
        if not anchor:anchor=edge(a[1]) or edge(b[1])
        crossed=(a[1]<cx<=b[1]) or (b[1]<cx<=a[1])
        if anchor and crossed:
            fraction=(cx-a[1])/(b[1]-a[1]);p=a+fraction*(b-a);dy=float(p[2]-cy)
            candidate=dict(ros_sec=float(p[0]),x=cx,y=float(p[2]),offset_y=dy,fc_agl=float(p[3]+.22),
                direction='E' if b[1]>a[1] else 'W',side='N' if dy>=deadband else ('S' if dy<=-deadband else 'CENTER'))
        current=edge(b[1])
        if anchor and current and current!=anchor:
            if candidate and candidate['direction']==('E' if current>0 else 'W'):result.append(candidate)
            anchor=current;candidate=None
    return result

def sanity():
    def line(start,end,duration=4):
        t=np.arange(0,duration+.001,.1);x=np.linspace(start,end,len(t));return np.column_stack([t,x,np.full(len(t),.8),np.ones(len(t))])
    full=line(-2,2);valid=np.ones(len(full)-1,dtype=bool)
    assert len(passages(full,valid,(0,0)))==1 and passages(full,valid,(0,0))[0]['side']=='N'
    partial=line(-2,.25);assert not passages(partial,np.ones(len(partial)-1,dtype=bool),(0,0))
    interrupted=valid.copy();interrupted[(full[:-1,0]>=1.8)&(full[:-1,0]<=2.2)]=False
    assert not passages(full,interrupted,(0,0))
    reversed_path=np.vstack([full,np.column_stack([full[1:,0]+4,full[-2::-1,1],np.full(len(full)-1,.8),np.ones(len(full)-1)])])
    assert [x['direction'] for x in passages(reversed_path,np.ones(len(reversed_path)-1,dtype=bool),(0,0))]==['E','W']

def analyze(run):
    pose=np.loadtxt(run/'truth_pose.csv',delimiter=',',skiprows=1)
    events=[json.loads(x) for x in (run/'key_events.jsonl').read_text().splitlines()]
    mask=search_mask(pose,events)
    t0=next(x['ros_sec'] for x in events if x['kind']=='decision')
    result=[]
    for m in ET.parse(run/'scenario_inputs/field.world').iter('model'):
        if 'tree' not in m.get('name','').lower():continue
        xyz=[float(x) for x in m.findtext('pose').split()];p=passages(pose,mask,xyz[:2])
        for x in p:x['since_first_decision_s']=x['ros_sec']-t0
        result.append(dict(name=m.get('name'),tree_xy=xyz[:2],passages=p,sequence=' '.join(x['side'] for x in p)))
    record=dict(run=run.name,scope='SEARCH/RESUME XY passages across tree X +/-0.5m, within Y +/-1.5m; north is +Y. A same-side passage is not automatically redundant or a coverage proof.',center_deadband_m=.2,max_pose_gap_s=.5,trees=result)
    OUT.mkdir(exist_ok=True)
    name=run.name.split('_20260912_')[0]
    (OUT/(name+'_tree_passages.json')).write_text(json.dumps(record,indent=2)+'\n')
    print(run.name,[(x['name'],x['sequence']) for x in result])

def plot_pair(seed):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    records=[json.loads((OUT/f'fixed_tree_seed{seed}_{mode}_tree_passages.json').read_text()) for mode in ['baseline','active']]
    fig,axes=plt.subplots(2,2,figsize=(12,8))
    for i,ax in enumerate(axes.flat):
        reference=records[0]['trees'][i]
        for j,(label,color,marker) in enumerate([('Baseline','#276b96','o'),('Local entry','#c26a24','x')]):
            data=records[j]['trees'][i]
            assert data['tree_xy']==reference['tree_xy']
            p=data['passages']
            ax.scatter([x['since_first_decision_s'] for x in p],[x['offset_y'] for x in p],label=label,color=color,marker=marker,s=40)
        ax.axhline(0,color='gray',lw=.8);ax.axhspan(-.2,.2,color='gray',alpha=.08)
        xy=reference['tree_xy']
        ax.set(title=f'T{i+1} ({xy[0]:.1f}, {xy[1]:.1f})',xlabel='ROS s since first recorded decision',ylabel='Y offset from tree (m); north is +',ylim=(-1.6,1.6))
        ax.grid(alpha=.15)
    handles,labels=axes.flat[0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='upper center',bbox_to_anchor=(.5,.90),ncol=2,fontsize=9)
    fig.suptitle(f'Seed {seed}: fixed-tree XY passage records\nComplete traversals of the X window; same-side passages are not automatically redundant or proof of coverage.')
    fig.tight_layout(rect=(0,0,1,.86));fig.savefig(OUT/f'seed{seed}_tree_sides.png',dpi=150);plt.close(fig)

if __name__=='__main__':
    sanity()
    if len(sys.argv)>1 and sys.argv[1]=='--pair':plot_pair(int(sys.argv[2]))
    else:
        paths=sys.argv[1:] or [ROOT/r['run'] for r in json.loads((OUT/'runs.json').read_text())['runs']]
        for path in paths:analyze(Path(path).resolve())
        if not sys.argv[1:]:
            for seed in [32,34]:plot_pair(seed)

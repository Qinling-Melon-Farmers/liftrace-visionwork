"""Offline ledger review of recorded observer snapshots; never feeds a mission."""
from pathlib import Path
import sys,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

ROOT=Path(__file__).resolve().parents[3]
OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'vision_ws/src/uav_coverage_memory/src'))
from uav_coverage_memory.memory import Config
from uav_coverage_memory.queries import Snapshot,MissionLedger,Region


def review(run):
    events=[json.loads(x) for x in (run/'key_events.jsonl').read_text().splitlines() if x]
    releases=[e['ros_sec'] for e in events if e['kind']=='release' and e['data'].get('success')]
    end=releases[2] if len(releases)>=3 else float('inf')
    ledger=MissionLedger();rows=[];last=None;rejected=[]
    for path in sorted((run/'coverage').glob('*.json')):
        meta=json.loads(path.read_text());r=meta['result']
        if not r['epoch'] or r['image_stamp']>end:continue
        with np.load(path.with_suffix('.npz')) as arrays:
            snapshot=Snapshot(Config(**meta['config']),arrays['state_grid'],r['image_stamp'],r['epoch'],r['generation'])
        try:
            view=ledger.update(snapshot,r['image_stamp']+r.get('image_age',.01))
        except ValueError as error:
            rejected.append({'sample':path.name,'reason':str(error)});continue
        area=np.diff(view.ye)[:,None]*np.diff(view.xe)[None,:]
        rows.append({'stamp':view.stamp,'generation':view.generation,
                     'history_m2':float(area[view.historical_seen].sum()),
                     'active_m2':float(area[view.states==100].sum())})
        last=view
    if last is None:return None
    fig,axes=plt.subplots(1,2,figsize=(12,5))
    axes[0].plot([r['stamp'] for r in rows],[r['history_m2'] for r in rows],label='Recorded historical estimates',color='#357bad')
    axes[0].plot([r['stamp'] for r in rows],[r['active_m2'] for r in rows],label='Active estimates, TTL60s',color='#348352')
    axes[0].set(xlabel='ROS time (s)',ylabel='Estimated area (m²)',title='Before third release; 6s snapshots only');axes[0].legend(fontsize=8);axes[0].grid(alpha=.2)
    colors=last.historical_seen.astype(int);colors[last.states==100]=2
    c=last.config
    axes[1].imshow(colors,origin='lower',extent=(c.min_x,c.max_x,c.min_y,c.max_y),vmin=0,vmax=2,
                   cmap=ListedColormap(['#e9ecef','#80add0','#4c9861']),interpolation='nearest')
    axes[1].set(xlabel='X (m)',ylabel='Y (m)',title='Blue: history only; green: active estimate\nGray: no retained estimate; not a free-space map')
    fig.suptitle(run.name);fig.tight_layout();fig.savefig(OUT/(run.name+'_history.png'),dpi=145);plt.close(fig)
    # Exercise real snapshot query plumbing without proposing waypoints. Equal
    # placeholder costs are deliberately not claimed to be obstacle-aware costs.
    candidates=[Region('south',(-4.3,4.3,0.,2.1),10.),Region('middle',(-4.3,4.3,2.1,4.2),10.),Region('north',(-4.3,4.3,4.2,7.1),10.)]
    ranking=last.rank(candidates,now=last.stamp+.01,frame_id=c.frame_id,epoch=last.epoch,generation=last.generation)
    return {'run_dir':str(run),'samples_before_third_release':len(rows),'rejected':rejected,
            'scope':'Offline sampled estimate history, not actual coverage/recall and not used for control. Sampling may omit transient observations. Three generic regions use equal placeholder travel costs only to exercise the API.',
            'last':rows[-1],'history':rows,'demonstration_queries':ranking}


if __name__=='__main__':
    results=[]
    for run in sorted((ROOT/'logs').glob('coverage_seed*reference*')):
        if (run/'gate_status.json').exists():
            result=review(run)
            if result:results.append(result)
    (OUT/'history_review.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps([{'run':r['run_dir'],'samples':r['samples_before_third_release'],'last':r['last']} for r in results],indent=2))

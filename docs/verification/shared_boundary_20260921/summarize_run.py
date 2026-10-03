"""Offline only: summarize one completed run without ROS publishers."""
from pathlib import Path
import argparse,collections,csv,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def rows(path):
    if not path.exists():return []
    result=[]
    for line in path.open():
        try:result.append(json.loads(line))
        except json.JSONDecodeError:continue
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args()
    dest=Path(__file__).resolve().parent
    events=rows(a.run/'key_events.jsonl');high=rows(a.run/'high_view_full_events.jsonl')
    planners=[e for e in events if e.get('kind')=='planner']
    reasons=collections.Counter(e.get('data',{}).get('reason','') for e in planners)
    first={}
    for e in planners:
        d=e['data'];key=str(d['goal_seq'])
        if key not in first:first[key]={'first_event_s':e['ros_sec'],'goal':d.get('requested_goal',{}).get('pose',{}).get('position')}
        if d.get('status')==2 and 'ready_s' not in first[key]:
            first[key]['ready_s']=e['ros_sec'];first[key]['latency_s']=e['ros_sec']-first[key]['first_event_s']
    progress=rows(a.run/'trajectory_progress.jsonl')
    recovery=[e for e in progress if any(s in json.dumps(e) for s in ['server_tracking_hold_replan','no_physical_progress_replan','budget_exhausted'])]
    gate=json.loads((a.run/'gate_status.json').read_text()) if (a.run/'gate_status.json').exists() else None
    final=high[-1] if high else None
    result=dict(run=str(a.run),gate=gate,final_high_view=final,planner_reason_counts=dict(reasons),goals=first,recovery_rows=recovery,
                cleanup_pass='SITL cleanup verification: PASS' in (a.run/'run.log').read_text(errors='replace'))
    (dest/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    with (a.run/'truth_pose.csv').open() as f:xyzt=np.array([[float(r[k]) for k in ['t','x','y','z']] for r in csv.DictReader(f)])
    if not len(xyzt):return
    fig,ax=plt.subplots(1,3,figsize=(16,5))
    t,x,y,z=xyzt.T
    sc=ax[0].scatter(x,y,c=z,s=4,cmap='viridis');ax[0].plot(x[0],y[0],'r^',label='Start (+X forward)');ax[0].plot(x[-1],y[-1],'kx',label='End');ax[0].set_aspect('equal');ax[0].set(xlabel='X (m)',ylabel='Y (m)',title='Actual flight / world coordinates');ax[0].legend(fontsize=8);fig.colorbar(sc,ax=ax[0],label='World Z (m)')
    ax[1].plot(t,z);ax[1].set(xlabel='ROS time (s)',ylabel='World Z (m)',title='Altitude (not uncalibrated local Z)')
    dt=np.diff(t);valid=dt>0
    v=np.hypot(np.diff(x),np.diff(y))[valid]/dt[valid]
    ax[2].plot(t[1:][valid],v,lw=.8);ax[2].set(xlabel='ROS time (s)',ylabel='Horizontal speed (m/s)',title='Actual speed from truth positions')
    if final:
        for e in final['status'].get('events',[]):
            if e.get('stage') in ['SURVEY_INTERRUPTED_TOP3','DELIVERY','TAIL']:
                for chart in ax[1:]:chart.axvline(e['time'],color='gray',alpha=.4,ls='--')
    for chart in ax:chart.grid(alpha=.25)
    fig.suptitle('seed32 reliability regression; no timing comparison with diagnostic runs')
    fig.tight_layout();fig.savefig(dest/'flight.png',dpi=150)
    print(json.dumps({'gate':gate,'planner_reasons':dict(reasons),'first_goals':dict(list(first.items())[:3]),'recovery_rows':len(recovery)},ensure_ascii=False))

if __name__=='__main__':main()

"""Read existing simulation artifacts only; never launches ROS or changes flight configs."""
import argparse,csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

D=Path(__file__).resolve().parent
SOURCES={
 '2672_A':'docs/verification/fov_landing_inner_20260919/2672_A_baseline/metrics.json',
 '2672_B':'docs/verification/fov_landing_inner_20260919/2672_B_corridor/metrics.json',
 '2672_C':'docs/verification/fov_landing_inner_20260919/2672_C_high3/metrics.json',
 'seed31':'docs/verification/failed_six_20260921/31_rerun/metrics.json',
 'seed32':'docs/verification/failed_six_20260921/32_rerun/metrics.json',
 'seed37':'docs/verification/failed_six_20260921/37_rerun/metrics.json',
}
def read_pose(path):
    with path.open() as f:
        return np.array([[float(row[k]) for k in ('t','x','y','z')] for row in csv.DictReader(f)])
def stat(values):
    return dict(n=int(len(values)),p50=float(np.median(values)) if len(values) else None,
                p95=float(np.percentile(values,95)) if len(values) else None)
def summarize(m,root):
    # Prefer this checkout's retained run; fall back to recorded absolute path.
    run=root/'logs'/Path(m['run']).name
    if not run.exists():run=Path(m['run'])
    a=read_pose(run/'truth_pose.csv');est=read_pose(run/'mavros_pose.csv')
    dt=np.diff(a[:,0]);valid=(dt>0)&(dt<=.5)
    t=a[1:,0][valid];v=np.diff(a[:,1:],axis=0)[valid]/dt[valid,None]
    xy=np.linalg.norm(v[:,:2],axis=1);vz=v[:,2]
    changes=m['sampled_phase_changes']
    times=np.array([x['t'] for x in changes]);names=np.array([x['phase'] for x in changes],dtype=object)
    idx=np.searchsorted(times,t,side='right')-1
    phases=np.full(len(t),'PRE_MISSION',dtype=object);ok=idx>=0;phases[ok]=names[idx[ok]]
    active=(t>=m['start_ros_s'])&(t<m['observed_end_ros_s'])
    rows={}
    for phase in dict.fromkeys(names):
        mask=active&(phases==phase)
        rows[phase]=dict(xy_moving=stat(xy[mask&(xy>.03)]),
                        climb_moving=stat(vz[mask&(vz>.03)]),
                        descent_moving=stat(-vz[mask&(vz<-.03)]))
    pre=(t>=m['airborne_ros_s'])&(t<m['start_ros_s'])
    rows['INITIAL_TAKEOFF']=dict(climb_moving=stat(vz[pre&(vz>.03)]),
                                window_s=m['start_ros_s']-m['airborne_ros_s'])
    releases=[]
    for event in m['releases']:
        tm=event['ros_sec'];data=event['data']
        releases.append(dict(target=data['target_class'],slot=data['payload_slot'],
          true_fc_agl_m=float(np.interp(tm,a[:,0],a[:,3])+.22),
          mavros_local_z_m=float(np.interp(tm,est[:,0],est[:,3]))))
    return dict(source_run=str(run),status=m['status'],mission_s=m['completed_mission_s'],
        milestone_s=m['milestone_durations_s'],speed_by_phase=m['speed_by_phase'],
        following_speed_stats=m['following_speed_stats'],component_speed=rows,
        ack_heights=releases,stage_duration_s=m['sampled_phase_durations_s'])
def plot(b,out):
    speed=b['following_speed_stats']
    stages=['CRUISE','PRECISION','TRANSIT_TO_CORRIDOR','CORRIDOR_DESCENT','CORRIDOR_OPEN','DOOR','H_APPROACH','TERMINAL']
    labels=['Cruise (high + low)','Precision / delivery','Transfer before corridor',
            'Corridor entry descent','Open corridor','Door slow segments','H approach','H align + land']
    colors=['#2980b9','#b57613','#218c74','#7f8c8d','#45aaf2','#c0392b','#8854d0','#596275']
    fig,axes=plt.subplots(1,2,figsize=(13,5.3),gridspec_kw={'width_ratios':[1.2,1.0]})
    ys=np.arange(len(stages))
    vals=[speed[s]['median_mps'] or 0 for s in stages]
    axes[0].barh(ys,vals,color=colors)
    for y,s,x in zip(ys,stages,vals):
        p95=speed[s]['p95_mps']
        if p95 is not None:axes[0].plot([x,p95],[y,y],color='#333333',lw=1.1);axes[0].plot(p95,y,'|',color='#333333')
        axes[0].text(max(x,p95 or 0)+.018,y,'n/a' if p95 is None else f'{x:.3f}',va='center',fontsize=9)
    axes[0].set_yticks(ys,labels);axes[0].set_xlim(0,1.06);axes[0].invert_yaxis()
    axes[0].set_xlabel('Horizontal speed, m/s (bar=P50, line to P95)')
    ds=[speed[s]['duration_s'] for s in stages]
    axes[1].barh(ys,ds,color=colors)
    for y,x in zip(ys,ds):axes[1].text(x+.6,y,f'{x:.1f}s',va='center',fontsize=9)
    axes[1].set_yticks(ys,[]);axes[1].set_xlim(0,72);axes[1].invert_yaxis()
    axes[1].set_xlabel('Full phase duration, including stops / vertical motion')
    for ax in axes:ax.grid(axis='x',alpha=.2);ax.set_axisbelow(True)
    fig.suptitle('Historical seed2672 B: 187.73s completed mission (2026-09-19)')
    fig.text(.5,.014,'Speed excludes horizontal samples <=0.03m/s. Door includes approach/exit legs. No new simulation.',ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.045,1,.96))
    fig.savefig(out/'speed_time_budget.png',dpi=160);plt.close(fig)
def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=D.parents[2]);p.add_argument('--out',type=Path,default=D)
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    results={key:summarize(json.loads((a.root/path).read_text()),a.root) for key,path in SOURCES.items()}
    data=dict(method='Existing PASS runs. Truth-pose first difference, 0<dt<=0.5s. Component moving threshold0.03m/s. End-of-interval phase label. Historical FC AGL=truth Z+0.22m.',
              sources=SOURCES,runs=results)
    (a.out/'measurements.json').write_text(json.dumps(data,indent=2)+'\n')
    plot(results['2672_B'],a.out)
    b=results['2672_B']
    for phase,row in b['component_speed'].items():
        print(phase, {k:round(v['p50'],3) if isinstance(v,dict) and v.get('p50') is not None else v for k,v in row.items()})
    print('B ACK heights',b['ack_heights'])
    for phase in ['SURVEY','REVISIT','ALIGN']:
        print(phase,[(k,round(v['speed_by_phase'][phase]['median_mps'],3)) for k,v in results.items() if phase in v['speed_by_phase']])
    print('outputs',str(a.out))
if __name__=='__main__':main()

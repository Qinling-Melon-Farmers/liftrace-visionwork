"""Small, read-only report supplements for the frozen SITL comparison."""
from pathlib import Path
import json, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent

def rows(path):
    return [json.loads(x) for x in path.read_text().splitlines() if x]

def supplemental(run):
    e = rows(run/'key_events.jsonl')
    d = [x for x in e if x['kind']=='decision']
    releases = [x for x in e if x['kind']=='release' and x['data'].get('success')]
    t0 = d[0]['ros_sec']
    stop = releases[2]['ros_sec'] if len(releases)>=3 else e[-1]['ros_sec']
    unions = {}; samples = []; config = None
    for path in sorted((run/'coverage').glob('[0-9]*.json')):
        meta = json.loads(path.read_text()); result = meta['result']; stamp = result.get('image_stamp',0)
        if not t0<=stamp<=stop: continue
        with np.load(path.with_suffix('.npz')) as arrays: state = arrays['state_grid']
        key = str(result['generation'])
        if key not in unions: unions[key] = np.zeros_like(state, dtype=bool)
        unions[key] |= state==100
        config = meta['config']
        samples.append(dict(t=stamp-t0, generation=key, union_m2=float(unions[key].sum()*config['resolution']**2)))
    contact = json.loads((run/'gazebo_contact_status.json').read_text())
    pose = np.loadtxt(run/'truth_pose.csv',delimiter=',',skiprows=1)
    mav = np.loadtxt(run/'mavros_pose.csv',delimiter=',',skiprows=1)
    cmd = np.loadtxt(run/'mavros_setpoint.csv',delimiter=',',skiprows=1)
    detail = []
    if contact['events']:
        hit = contact['events'][0]['ros_stamp']
        for delta in [3,1,.1]:
            actual = pose[np.argmin(abs(pose[:,0]-(hit-delta)))]; estimate = mav[np.argmin(abs(mav[:,0]-actual[0]))]
            detail.append(dict(seconds_before_first_counted_collision=delta,actual_t_xyz=actual[:4].tolist(),
                estimated_t_xyz=estimate[:4].tolist(),truth_minus_estimated_xyz=(actual[1:4]-estimate[1:4]).tolist()))
    record = dict(run=run.name, sampled_union_scope='Union of saved SEEN_ESTIMATE cells, per generation, from first decision through third release; sparse ~6 ROS s snapshots, not all observer frames, target recall or free space',
        sampled_union_samples=samples,sampled_union_final_m2={k:float(v.sum()*config['resolution']**2) for k,v in unions.items()},
        collision_preceding_samples=detail,
        terminal_vehicle_events=[x for x in e if x['kind'] in ['state','extended_state'] and x['ros_sec']>pose[-1,0]-20])
    name=run.name.split('_20260912_')[0]
    (OUT/(name+'_supplement.json')).write_text(json.dumps(record,indent=2)+'\n')
    if unions:
        fig,ax=plt.subplots(figsize=(6,5))
        grid=unions[list(unions)[-1]]
        ax.imshow(grid,origin='lower',extent=(config['min_x'],config['max_x'],config['min_y'],config['max_y']),vmin=0,vmax=1,cmap='Greens',interpolation='nearest')
        ax.set(title=name+'\nSampled cumulative observation estimate',xlabel='X (m)',ylabel='Y (m)')
        fig.tight_layout();fig.savefig(OUT/(name+'_sampled_union.png'),dpi=140);plt.close(fig)
    if len(pose):
        hit=contact['events'][0]['ros_stamp'] if contact['events'] else pose[-1,0]
        reference_label='First counted collision' if contact['events'] else 'End of recording'
        start=hit-15; finish=pose[-1,0]
        fig,ax=plt.subplots(2,2,figsize=(12,8))
        mask=(pose[:,0]>=start)&(pose[:,0]<=finish)
        actual=pose[mask]
        est=mav[(mav[:,0]>=start)&(mav[:,0]<=finish)]
        command=cmd[(cmd[:,0]>=start)&(cmd[:,0]<=finish)]
        ax[0,0].plot(actual[:,1],actual[:,2],label='Actual path')
        ax[0,0].plot(est[:,1],est[:,2],label='Estimated path',lw=.8)
        ax[0,0].scatter([4.2],[8.5],marker='x',color='black',label='H anchor')
        ax[0,0].set(xlabel='X (m)',ylabel='Y (m)',title='Last 15 s before '+reference_label.lower());ax[0,0].axis('equal')
        ax[0,1].plot(actual[:,0]-hit,actual[:,3]+.22,label='Actual FC AGL')
        ax[0,1].plot(est[:,0]-hit,est[:,3]+.22,label='Estimated z +0.22')
        ax[0,1].plot(command[:,0]-hit,command[:,3]+.22,label='Command z +0.22',lw=.8)
        ax[0,1].axhline(.22,ls=':',color='gray',label='Nominal support height')
        ax[0,1].set(ylabel='Height (m)',title='Height; ground-plane contacts are excluded')
        qx,qy,qz,qw=actual[:,4:].T
        roll=np.arctan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy))*180/np.pi
        pitch=np.arcsin(np.clip(2*(qw*qy-qz*qx),-1,1))*180/np.pi
        ax[1,0].plot(actual[:,0]-hit,roll,label='Actual roll');ax[1,0].plot(actual[:,0]-hit,pitch,label='Actual pitch')
        ax[1,0].set(ylabel='Angle (deg)',title='Actual body attitude')
        interp=np.stack([np.interp(actual[:,0],mav[:,0],mav[:,k]) for k in [1,2,3]],axis=1)
        for i,label in enumerate(['X','Y','Z']):ax[1,1].plot(actual[:,0]-hit,actual[:,i+1]-interp[:,i],label=label)
        ax[1,1].set(ylabel='Truth - estimate (m)',title='Position difference; estimate interpolated')
        for axis in [ax[0,1],ax[1,0],ax[1,1]]:
            axis.axvline(0,color='#c94444',ls=':',label=reference_label)
            for event in e:
                if event['kind']=='state' and event['data'].get('mode')=='AUTO.LAND' and event['ros_sec']>=start:
                    axis.axvline(event['ros_sec']-hit,color='#777777',ls='--',label='AUTO.LAND')
            axis.set_xlabel('Time relative to '+reference_label.lower()+' (s)')
        for axis in ax.flat:axis.legend(fontsize=7);axis.grid(alpha=.2)
        fig.suptitle(name+' | terminal segment, not a causal diagnosis')
        fig.tight_layout(rect=(0,0,1,.95));fig.savefig(OUT/(name+'_terminal.png'),dpi=145);plt.close(fig)
    print(name,record['sampled_union_final_m2'])

if __name__=='__main__':
    paths=sys.argv[1:] or [ROOT/r['run'] for r in json.loads((OUT/'runs.json').read_text())['runs']]
    for name in paths:supplemental(Path(name).resolve())

"""Read existing run logs for a planning document; no ROS or route generation."""
from pathlib import Path
import json
import math
import collections
import xml.etree.ElementTree as ET
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Circle, Rectangle

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
VERIFY = ROOT / 'docs/verification'
plt.rcParams.update({'font.size': 9, 'axes.spines.top': False,
                     'axes.spines.right': False, 'savefig.facecolor': 'white'})


def read(path):
    return json.loads(path.read_text())


def repeated(points, times, radius, lag=15.0):
    """Nearby earlier search movement; temporal lag excludes adjacent samples."""
    buckets = collections.defaultdict(list)
    result = np.zeros(len(points), dtype=bool)
    previous = 0
    for i, (point, stamp) in enumerate(zip(points, times)):
        while previous < i and times[previous] <= stamp - lag:
            old = points[previous]
            buckets[tuple(np.floor(old / radius).astype(int))].append(old)
            previous += 1
        cell = np.floor(point / radius).astype(int)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if any(np.linalg.norm(point - old) <= radius
                       for old in buckets[(cell[0] + dx, cell[1] + dy)]):
                    result[i] = True
    return result


def manifest():
    rows = []
    for group, folder in [('R60', 'r60_full_matrix'), ('R64', 'r64_matrix'),
                          ('Random', 'full_random_five_20260910')]:
        for row in read(VERIFY / folder / 'matrix_status.json')['results']:
            rows.append(dict(row, group=group))
    rows.append(dict(read(VERIFY / 'r60_full_matrix/pilot/summary.json'), group='R60'))
    rows.append(dict(read(VERIFY / 'r64_matrix/flight_metrics.json')[0], group='R64'))
    row = read(VERIFY / 'search_wait_candidate_20260911/rerun/run_status.json')
    rows.append(dict(row, group='Wait20', status=row['gate']))
    return sorted(rows, key=lambda r: (['R60', 'R64', 'Random', 'Wait20'].index(r['group']), r['seed']))


def analyze(row):
    run = Path(row['run_dir'])
    pose = np.loadtxt(run / 'truth_pose.csv', delimiter=',', skiprows=1)
    events = [json.loads(line) for line in (run / 'key_events.jsonl').read_text().splitlines()]
    phases = [e for e in events if e['kind'] == 'mission']
    phase_times = np.array([e['ros_sec'] for e in phases])
    phase_names = np.array([e['data']['phase'] for e in phases])
    starts = [e for e in events if e['kind'] == 'decision']
    mission_start = min(e['ros_sec'] for e in starts)
    mid_t = (pose[:-1, 0] + pose[1:, 0]) / 2
    dt = np.diff(pose[:, 0])
    ds = np.linalg.norm(np.diff(pose[:, 1:3], axis=0), axis=1)
    indices = np.searchsorted(phase_times, mid_t, side='right') - 1
    mask = (indices >= 0) & (phase_names[np.maximum(indices, 0)] == 'SEARCH') & (dt > 0) & (dt <= .5)
    speed = ds / np.maximum(dt, 1e-9)
    moving = mask & (speed >= .1)
    points = (pose[:-1, 1:3] + pose[1:, 1:3]) / 2
    repeats = {str(rad): repeated(points[moving], mid_t[moving], rad) for rad in [.2, .3, .4]}
    total = float(ds[moving].sum())
    durations = collections.defaultdict(float)
    for i, phase in enumerate(phases):
        end = phases[i+1]['ros_sec'] if i+1 < len(phases) else pose[-1, 0]
        durations[phase['data']['phase']] += max(0, end - phase['ros_sec'])
    releases = [e for e in events if e['kind'] == 'release' and e['data'].get('success')]
    rt = yaml.safe_load((run / 'scenario_inputs/runtime.yaml').read_text())
    tree_models = [e for e in ET.parse(run / 'scenario_inputs/field.world').getroot().iter('model')
                   if 'tree' in e.get('name', '').lower()]
    trees = [dict(name=e.get('name'), xyz=[float(v) for v in e.findtext('pose').split()[:3]])
             for e in tree_models]
    terminals = {e['data']['decision_seq']: e['ros_sec'] for e in events
                 if e['kind'] == 'result' and e['data'].get('terminal')}
    crossings = []
    for i, dec in enumerate(starts):
        data = dec['data']
        if data['command'] not in (0, 3):
            continue
        begin = dec['ros_sec']
        end = min(starts[i+1]['ros_sec'] if i+1 < len(starts) else pose[-1, 0],
                  terminals.get(data['decision_seq'], pose[-1, 0]))
        segment = pose[(pose[:, 0] >= begin) & (pose[:, 0] <= end)]
        if len(segment) < 2:
            continue
        goal = data['goal']['pose']['position']
        for tree in trees:
            tx, ty = tree['xyz'][:2]
            # Opposite sides in X by >=0.6m excludes local jitter and endpoint turns.
            if not ((segment[0, 1] < tx-.6 and goal['x'] > tx+.6) or
                    (segment[0, 1] > tx+.6 and goal['x'] < tx-.6)):
                continue
            hits = np.where((segment[:-1, 1]-tx)*(segment[1:, 1]-tx) < 0)[0]
            for hit in hits[:1]:
                a, b = segment[hit:hit+2]
                fraction = (tx-a[1])/(b[1]-a[1])
                stamp = a[0]+fraction*(b[0]-a[0])
                y = a[2]+fraction*(b[2]-a[2])
                if abs(y-ty) > 1.5 or abs(y-ty) < .3:
                    continue
                crossings.append(dict(tree=tree['name'], tree_xy=[tx, ty], t=float(stamp),
                    y=float(y), side='N' if y > ty else 'S', command=data['command'],
                    decision_seq=data['decision_seq'], goal=goal, begin=begin, end=end,
                    goal_side='N' if goal['y'] > ty else 'S'))
    item = dict(group=row['group'], seed=row['seed'], status=row['status'], run_dir=str(run),
                invalid_r64_layout=row['group']=='R64' and row['seed'] in [5, 7, 8],
                mission_start_ros=mission_start, mission_start_basis='first recorded decision; can postdate actual task start', observed_end_ros=float(pose[-1, 0]),
                drops=len(releases), third_release_s=(releases[2]['ros_sec']-mission_start) if len(releases)>=3 else None,
                phase_seconds=dict(durations), search_sampled_s=float(dt[mask].sum()),
                search_slow_s=float(dt[mask & (speed < .1)].sum()),
                search_moving_path_m=total, search_total_path_m=float(ds[mask].sum()),
                revisit_m={key:float(ds[moving][v].sum()) for key,v in repeats.items()},
                revisit_fraction={key:float(ds[moving][v].sum())/total if total else None for key,v in repeats.items()},
                search_config=rt['search'], tree_crossings=crossings)
    return item, (pose, mask, moving, repeats['0.3'], trees, points)


def route_plot(ax, item, arrays):
    pose, mask, moving, rep, trees, points = arrays
    ax.plot(pose[:, 1], pose[:, 2], color='#ccd2d9', lw=.55)
    segs = np.stack([pose[:-1, 1:3], pose[1:, 1:3]], axis=1)[moving]
    ax.add_collection(LineCollection(segs, colors=np.where(rep, '#d95f28', '#1776a4'), linewidths=1.0))
    for tree in trees:
        ax.add_patch(Circle(tree['xyz'][:2], .43, color='#488763', alpha=.3))
    sc = item['search_config']
    ax.add_patch(Rectangle((sc['min_x'], sc['min_y']), sc['max_x']-sc['min_x'],
                          sc['max_y']-sc['min_y'], fill=False, ls='--', color='#707070', lw=.5))
    frac = item['revisit_fraction']['0.3']
    ax.set_title(f"{item['group']} seed{item['seed']:02} | {item['status']}"+
                 (' [layout invalid]' if item['invalid_r64_layout'] else '')+
                 f"\nSEARCH {item['search_sampled_s']:.0f}s | revisit {100*frac:.1f}%", fontsize=9)
    ax.set_aspect('equal'); ax.grid(alpha=.15)
    ax.set(xlim=(-5.1, 5.1), ylim=(-1.7, 9.5), xlabel='X (m)', ylabel='Y (m)')


def main():
    assert repeated(np.array([[0, 0], [0, 0], [0, 0]]), np.array([0, 1, 16]), .3).tolist()==[False,False,True]
    assert not repeated(np.array([[0, 0], [1, 0]]), np.array([0, 20]), .3).any()
    results = [analyze(row) for row in manifest()]
    items = [row[0] for row in results]
    for group in ['R60', 'R64', 'Random']:
        chosen = [row for row in results if row[0]['group']==group or (group=='Random' and row[0]['group']=='Wait20')]
        cols=4 if len(chosen)>6 else 3; rows=math.ceil(len(chosen)/cols)
        fig, axes=plt.subplots(rows, cols, figsize=(cols*3.35, rows*3.65), squeeze=False)
        for ax,(item,arrays) in zip(axes.flat,chosen):route_plot(ax,item,arrays)
        for ax in list(axes.flat)[len(chosen):]:ax.axis('off')
        fig.suptitle('Recorded SEARCH movement: blue=new vicinity, orange=revisited vicinity\n0.30m distance / >=15s separation; gray=full flight reference; circles=tree-location proxy', fontsize=11)
        fig.tight_layout(rect=(0,0,1,.94));fig.savefig(OUT/f'routes_{group.lower()}.png',dpi=140);plt.close(fig)
    labels=[f"{dict(Random='FR', Wait20='W20').get(x['group'], x['group'])}\n{x['seed']:02}" for x in items]
    fig,axes=plt.subplots(3,1,figsize=(15,10),sharex=True)
    x=np.arange(len(items))
    axes[0].bar(x,[v['search_sampled_s']-v['search_slow_s'] for v in items],label='XY speed >=0.1m/s',color='#1776a4')
    axes[0].bar(x,[v['search_slow_s'] for v in items],bottom=[v['search_sampled_s']-v['search_slow_s'] for v in items],label='Slow / holding (not necessarily planner wait)',color='#c6d2dd')
    axes[0].set(ylabel='SEARCH seconds');axes[0].legend(fontsize=8)
    axes[1].bar(x,[v['search_moving_path_m']-v['revisit_m']['0.3'] for v in items],color='#1776a4',label='New vicinity')
    axes[1].bar(x,[v['revisit_m']['0.3'] for v in items],bottom=[v['search_moving_path_m']-v['revisit_m']['0.3'] for v in items],color='#d95f28',label='Revisited vicinity')
    axes[1].set(ylabel='SEARCH moving XY (m)');axes[1].legend(fontsize=8)
    for radius,color in [('0.2','#4a8b60'),('0.3','#d95f28'),('0.4','#81589e')]:
        axes[2].plot(x,[v['revisit_fraction'][radius]*100 for v in items],'.-',color=color,label=f'{radius}m radius')
    axes[2].set(ylabel='Revisit proxy (%)',xticks=x,xticklabels=labels);axes[2].legend(fontsize=8)
    for ax in axes:ax.grid(axis='y',alpha=.15)
    fig.suptitle('Existing runs: repeat movement and waiting are separate; failed runs are censored')
    fig.tight_layout();fig.savefig(OUT/'search_metrics.png',dpi=150);plt.close(fig)
    candidates=[]
    for item,arrays in results:
        if item['group'] not in ['Random','Wait20']:continue
        for tree in arrays[4]:
            hits=[e for e in item['tree_crossings'] if e['tree']==tree['name']]
            for side in ['N','S']:
                same=[e for e in hits if e['side']==side]
                if len(same)>=2:candidates.append((len(same),sum(e['goal_side']!=side for e in same),item,arrays,same))
    candidates.sort(key=lambda c:(c[4][0]['side']=='N', c[1]>0, c[0], -np.ptp([e['y'] for e in c[4]])),reverse=True)
    if candidates:
        _,_,item,arrays,hits=candidates[0]
        fig,axes=plt.subplots(1,2,figsize=(12,5))
        route_plot(axes[0],item,arrays)
        pose=arrays[0]; tx,ty=hits[0]['tree_xy']
        for e,color in zip(hits,plt.get_cmap('tab10').colors):
            segment=pose[(pose[:,0]>=e['begin']) & (pose[:,0]<=e['end'])]
            axes[1].plot(segment[:,1],segment[:,2],color=color,lw=1.5,label=f"seq{e['decision_seq']} / goal y={e['goal']['y']:.2f}")
            axes[1].scatter([tx],[e['y']],color=color,s=25)
        for tree in arrays[4]:
            axes[1].add_patch(Circle(tree['xyz'][:2],.43,color='#488763',alpha=.3))
        axes[1].axhline(ty,ls=':',color='#555555',lw=.8)
        axes[1].set(xlim=(tx-2.1,tx+2.1),ylim=(ty-1.8,ty+1.8),xlabel='X (m)',ylabel='Y (m)',title=f"{item['group']} seed{item['seed']} / {hits[0]['tree']}\nRepeated {hits[0]['side']} crossings; opposite-side feasibility not established")
        axes[1].set_aspect('equal');axes[1].grid(alpha=.15);axes[1].legend(fontsize=8)
        fig.tight_layout();fig.savefig(OUT/'repeated_tree_side.png',dpi=160);plt.close(fig)
        example=dict(group=item['group'],seed=item['seed'],crossings=hits)
    else:example=None
    summary={}
    for group in ['R60','R64','Random','Wait20']:
        rows=[i for i in items if i['group']==group]
        summary[group]=dict(n=len(rows),full_pass=sum(i['status']=='PASS' for i in rows),
            three_drops=sum(i['drops']>=3 for i in rows),
            search_s_median=float(np.median([i['search_sampled_s'] for i in rows])),
            revisit_fraction_median=float(np.median([i['revisit_fraction']['0.3'] for i in rows])),
            search_moving_path_m_median=float(np.median([i['search_moving_path_m'] for i in rows])))
    result=dict(scope='Existing logs only; no simulated candidate or flight execution',
        method='SEARCH phase, dt<=0.5s, XY speed>=0.1m/s; revisit within radius of earlier moving SEARCH sample at least 15s old. Not observed area, target recall, avoidable path or attainable savings.',
        lag_s=15,radii_m=[.2,.3,.4],runs=items,summary=summary,example=example)
    (OUT/'recorded_metrics.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(summary=summary,example=example),indent=2))


if __name__=='__main__':main()

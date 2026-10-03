"""Analyze one exported high-view flight; source times and receipt times stay separate."""
import argparse,json,csv,collections,sys,bisect
from pathlib import Path
import numpy as np,cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def plainpos(p):return [p[k] for k in ('x','y','z')]
def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--report',type=Path,required=True);a=p.parse_args();a.report.mkdir(parents=True,exist_ok=True)
    d=json.loads((a.run/'replay/data.json').read_text());r=d['rows'];sup=json.loads((a.run/'supplement.json').read_text())
    motion=np.array([[v['t'],*plainpos(v['m']['pose']['pose']['position'])] for v in r['odom']]);mt=motion[:,0]
    velocity=np.column_stack([(np.interp(mt+.25,mt,motion[:,i])-np.interp(mt-.25,mt,motion[:,i]))/.5 for i in (1,2,3)]);speed=np.linalg.norm(velocity[:,:2],axis=1)
    ground_fc=float(np.median(motion[mt<.9,3]));ground_z=ground_fc-.22
    cmds=[];names={0:'SEARCH',1:'APPROACH',2:'DROP',3:'RESUME',4:'RETURN_HOME',5:'LAND'}
    for row in r['command']:
        m=row['m'];cmds.append(dict(t=row['t'],source_t=row['stamp'],seq=m['decision_seq'],command=names.get(m['command'],str(m['command'])),target=m['target_class'],slot=m['payload_slot'],goal=plainpos(m['goal']['pose']['position']) if m['has_goal'] else None,reason=m['reason']))
    results=[dict(t=v['t'],seq=v['m']['decision_seq'],reason=v['m']['reason'],terminal=v['m']['terminal'],status=v['m']['status']) for v in r['result']]
    releases=[dict(t=v['t'],**{k:v['m'][k] for k in ('payload_slot','target_class','success','reason')}) for v in r['release']]
    fc=[];last=None
    for row in r['fc']:
        m=row['m'];state=(m['mode'],m['armed'])
        if state!=last:fc.append(dict(t=row['t'],mode=state[0],armed=state[1]));last=state
    landed=[];last=None
    for row in sup['/mavros/extended_state']:
        state=row['m']['landed_state']
        if state!=last:landed.append(dict(t=row['t'],state=state));last=state
    detector=[x for x in r['raw'] if x['m']['source']=='target_detector'];dts=np.array([x['stamp'] for x in detector]);lags=np.array([x['t']-x['stamp'] for x in detector])
    counts=collections.Counter(q['class_name'] for x in detector for q in x['m']['detections']);stages={}
    for key in ('raw','resolved','refined','mapped'):
        stages[key]=dict(collections.Counter(q['class_name'] for x in r[key] for q in x['m']['detections']))
    panzer=[dict(t=x['t'],source_t=x['stamp'],**q) for x in r['mapped'] for q in x['m']['detections'] if q['class_name']=='panzer']
    center=np.mean([plainpos(x['map_point']) for x in panzer],axis=0)
    circles=[dict(t=x['t'],stamp=x['stamp'],q=q) for x in r['mapped'] for q in x['m']['detections'] if q['class_name']=='circle' and q['map_valid'] and np.linalg.norm(np.array(plainpos(q['map_point']))[:2]-center[:2])<.45]
    phase=[]
    for i,c in enumerate(cmds):
        start=max(0,c['t']);end=cmds[i+1]['t'] if i+1<len(cmds) else next((v['t'] for v in landed if v['state']==1 and v['t']>start),d['duration'])
        mask=(mt>=start)&(mt<end)
        if not mask.any():continue
        phase.append(dict(seq=c['seq'],name=c['command']+(' '+c['target'] if c['target'] else ''),start=start,end=end,duration=end-start,z_min=float(motion[mask,3].min()),z_max=float(motion[mask,3].max()),z_p50=float(np.median(motion[mask,3])),xy_p50=float(np.median(speed[mask])),xy_p95=float(np.percentile(speed[mask],95))))
    target_states={};pan_memory=[]
    for row in r['targets']:
        for q in row['m']['targets']:
            key=(q['id'],q['class_name']);info=target_states.setdefault(key,dict(id=q['id'],cls=q['class_name'],states=set(),max_streak=0,first=row['t'],last=row['t']))
            info['states'].add(q['state']);info['max_streak']=max(info['max_streak'],q['consecutive_observe_count']);info['last']=row['t']
            if q['class_name']=='panzer':pan_memory.append(dict(t=row['t'],state=q['state'],count=q['observe_count'],streak=q['consecutive_observe_count'],map_valid=q['map_valid']))
    for v in target_states.values():v['states']=sorted(v['states'])
    stats=dict(duration=d['duration'],images=len(d['frames']),ground_fc_local_z=ground_fc,estimated_ground_z=ground_z,max_local_z=float(motion[:,3].max()),estimated_max_agl=float(motion[:,3].max()-ground_z),commands=cmds,results=results,releases=releases,fc_changes=fc,landed_changes=landed,phase_stats=phase,detector_messages=len(detector),detector_median_interval=float(np.median(np.diff(dts))),detector_lag_p50=float(np.median(lags)),detector_lag_p95=float(np.percentile(lags,95)),detector_counts=dict(counts),stage_counts=stages,panzer=panzer,panzer_memory=pan_memory,target_states=list(target_states.values()),selected_counts=dict(collections.Counter(v['m']['class_name'] for v in r['selected'])),panzer_circle_count=len(circles),panzer_circle_range=[min(x['stamp'] for x in circles),max(x['stamp'] for x in circles)],final_status=json.loads(r['mission'][-1]['m']['data']))
    (a.report/'metrics.json').write_text(json.dumps(stats,indent=2,ensure_ascii=False)+'\n')
    with (a.report/'phase_stats.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(phase[0]));w.writeheader();w.writerows(phase)
    fig,axs=plt.subplots(3,1,figsize=(13,9),sharex=True)
    axs[0].plot(mt,motion[:,3],label='FC local Z',color='tab:blue');axs[0].axhline(1.8,color='gray',ls='--',label='recorded search goal Z=1.8m')
    axs[1].plot(mt,speed,label='XY speed: 0.5s pose difference',color='tab:green')
    colors={'red_cross':'tab:red','bridge':'tab:blue','panzer':'tab:orange','tent':'tab:purple'}
    for cls,color in colors.items():
        tt=[];cc=[]
        for row in detector:
            for q in row['m']['detections']:
                if q['class_name']==cls:tt.append(row['stamp']);cc.append(q['class_confidence'])
        axs[2].scatter(tt,cc,s=12,label=cls,color=color)
    for ax in axs:
        for rel in releases:ax.axvline(rel['t'],color='black',ls=':',alpha=.7)
        ax.axvline(130.135,color='gray',ls='--');ax.legend(loc='upper right');ax.grid(alpha=.2)
    axs[0].set_ylabel('local Z / m');axs[1].set_ylabel('XY / m/s');axs[2].set_ylabel('YOLO confidence');axs[2].set_xlabel('Bag seconds (detection: source time; events: receipt time)');fig.suptitle('2026-09-27 board flight: two releases, panzer not confirmed');fig.tight_layout();fig.savefig(a.report/'height_speed_vision.png',dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(9,7));ax.plot(motion[:,1],motion[:,2],lw=1.4,label='recorded FC pose')
    goals=np.array([c['goal'] for c in cmds if c['command']=='SEARCH']);ax.plot(goals[:,0],goals[:,1],'--o',label='issued search goals')
    ax.scatter([center[0]],[center[1]],marker='x',s=100,c='orange',label='panzer mapped (~4.06,1.25)')
    for c in cmds:
        if c['command']=='APPROACH':ax.scatter(*c['goal'][:2],marker='x',s=80,label=c['target'])
    ax.annotate('initial head +X',xy=(.9,0),xytext=(0,0),arrowprops=dict(arrowstyle='->',color='red'),color='red');ax.set_aspect('equal');ax.set_xlabel('X forward / m');ax.set_ylabel('Y left / m');ax.grid(alpha=.2);ax.legend();fig.tight_layout();fig.savefig(a.report/'trajectory.png',dpi=160);plt.close(fig)
    frames=sorted(d['frames'],key=lambda v:v['stamp']);ft=np.array([v['stamp'] for v in frames]);ct=np.array([v['stamp'] for v in circles]);cells=[];quality=[]
    for t in [118.0,119.5,121.,122.461,122.752,124.,125.5,127.]:
        j=int(np.argmin(abs(ft-t)));fr=frames[j];im=cv2.imread(str(a.run/'replay'/fr['file']));original=im.copy();circle=circles[int(np.argmin(abs(ct-fr['stamp'])))]
        note='circle not matched'
        if abs(circle['stamp']-fr['stamp'])<.12:
            q=circle['q'];cx,cy,rad=q['center_px']['x'],q['center_px']['y'],q['center_px']['z'];rad=max(rad*.9,20)
            x1,y1,x2,y2=map(int,[max(0,cx-rad),max(0,cy-rad),min(im.shape[1],cx+rad),min(im.shape[0],cy+rad)])
            roi=cv2.cvtColor(original[y1:y2,x1:x2],cv2.COLOR_BGR2GRAY);norm=cv2.resize(roi,(200,200));quality.append(dict(t=fr['stamp'],mean=float(roi.mean()),median=float(np.median(roi)),p10=float(np.percentile(roi,10)),p90=float(np.percentile(roi,90)),dark_lt40=float(np.mean(roi<40)),laplacian_200=float(cv2.Laplacian(norm,cv2.CV_64F).var()),box=[x1,y1,x2,y2]));cv2.rectangle(im,(x1,y1),(x2,y2),(255,220,0),2);note=f'ROI gray median={np.median(roi):.0f}'
        matched=[q for row in detector if abs(row['stamp']-fr['stamp'])<.03 for q in row['m']['detections'] if q['class_name']=='panzer']
        for q in matched:
            b=q['roi'];cv2.rectangle(im,(b['x_offset'],b['y_offset']),(b['x_offset']+b['width'],b['y_offset']+b['height']),(0,230,0),3)
        im=cv2.resize(im,(640,360));strip=np.zeros((60,640,3),np.uint8)
        cv2.putText(strip,f't={fr["stamp"]:.3f}s  panzer='+','.join(f'{q["class_confidence"]:.3f}' for q in matched),(12,22),0,.55,(240,240,240),1);cv2.putText(strip,note,(12,47),0,.5,(240,240,240),1);cells.append(np.vstack([im,strip]))
    cv2.imwrite(str(a.report/'panzer_contact_sheet.jpg'),np.vstack([np.hstack(cells[i:i+2]) for i in range(0,len(cells),2)]))
    (a.report/'sample_quality.json').write_text(json.dumps(quality,indent=2)+'\n')
    print(json.dumps({k:v for k,v in stats.items() if k not in ('panzer_memory','stage_counts','final_status')},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
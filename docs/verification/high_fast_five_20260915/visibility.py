"""Approximate geometric camera opportunities, not detector recall/occlusion proof."""
from pathlib import Path
import json,csv
import numpy as np
import yaml,cv2
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent;R=D.parents[2]
CLASSES=['bridge','panzer','red_cross']
def main():
    batch=json.loads((R/'logs/high_fast_five_20260915_batch/matrix.json').read_text());assert batch['status']=='COMPLETE'
    results=[];values=[]
    for result in batch['results']:
        run=Path(result['run'])
        with (run/'truth_pose.csv').open() as f:
            reader=csv.reader(f);next(reader);a=np.array([[float(v) for v in row] for row in reader])
        params=yaml.safe_load((run/'rosparams.yaml').read_text());camera=json.loads((run/'actual_camera_info.json').read_text())
        truth=yaml.safe_load((run/'random_field_truth.yaml').read_text())
        events=[json.loads(l) for l in (run/'high_view_full_events.jsonl').read_text().splitlines()]
        stamps=np.array([e['t'] for e in events]);indices=np.clip(np.searchsorted(stamps,a[:,0],side='right')-1,0,len(events)-1)
        survey=np.array([events[i]['status'].get('stage')=='SURVEY' and events[i]['status'].get('ascent_verified',False) and not events[i]['status'].get('done',False) for i in indices])
        body=Rotation.from_quat(a[:,4:8]).as_matrix()
        fc=a[:,1:4]+np.array(params['competition_key_recorder']['truth_world_offset'])
        centre=fc+np.einsum('nij,j->ni',body,np.array([0.,0.,-.16]))
        optical=np.array([[0.,-1.,0.],[-1.,0.,0.],[0.,0.,-1.]])
        camera_world=body@optical
        dt=np.append(np.diff(a[:,0]),0);dt[(dt<0)|(dt>.5)]=0
        last=events[-1]['status'];mask=survey&(fc[:,2]>=2.4);ready=last.get('first_hint_ready',{})
        exit_hints=last.get('top_hints',{});exit_evaluated=bool(exit_hints) or last.get('failure')=='survey_complete_missing_top3'
        k=np.array(camera['K']).reshape(3,3);dist=np.array(camera['D']);w,h=camera['width'],camera['height'];row=[]
        for cls in CLASSES:
            target=next(t for t in truth['targets'] if t['class']==cls)
            point=np.array([target['world_x'],target['world_y'],target.get('world_z',0.)])
            xyz=np.einsum('nji,nj->ni',camera_world,point-centre)
            uv=cv2.projectPoints(xyz,np.zeros(3),np.zeros(3),k,dist)[0].reshape(-1,2)
            inside=mask&(xyz[:,2]>0)&(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h)
            central=inside&(uv[:,0]>=.1*w)&(uv[:,0]<.9*w)&(uv[:,1]>=.1*h)&(uv[:,1]<.9*h)
            item=dict(seed=result['seed'],target=cls,estimated_center_in_image_s=float(dt[inside].sum()),estimated_center_in_central80pct_s=float(dt[central].sum()),ever_accepted_hint=cls in ready,usable_at_survey_exit=(cls in exit_hints) if exit_evaluated else None)
            results.append(item);row.append(item['estimated_center_in_image_s'])
        values.append(row)
    data=dict(scope='Geometric approximation from 10Hz truth pose, recorded K/D, installed downward optical transform and camera16cm below FC; no occlusion or actual image detections evaluated.',rows=results)
    (D/'visibility_opportunities.json').write_text(json.dumps(data,indent=2))
    fig,ax=plt.subplots(figsize=(8,6));im=ax.imshow(values,cmap='Blues',aspect='auto');fig.colorbar(im,ax=ax,label='Estimated target-center time in image (s)')
    ax.set_xticks(range(3),CLASSES);ax.set_yticks(range(5),[str(v['seed']) for v in batch['results']]);ax.set_ylabel('Seed')
    for i in range(5):
        for j in range(3):
            item=results[i*3+j]
            label='EXIT READY' if item['usable_at_survey_exit'] else ('LOST AT EXIT' if item['ever_accepted_hint'] and item['usable_at_survey_exit'] is False else ('EVER READY' if item['ever_accepted_hint'] else 'NO HINT'))
            ax.text(j,i,f"{values[i][j]:.1f}s\n"+label,ha='center',va='center',fontsize=9,color='white' if values[i][j]>np.max(values)*.55 else 'black')
    ax.set_title('Geometric viewing opportunity versus high hint state\nNot detection recall: occlusion and full-image evidence unavailable')
    fig.tight_layout();fig.savefig(D/'visibility_opportunities.png',dpi=170);plt.close(fig)
if __name__=='__main__':main()

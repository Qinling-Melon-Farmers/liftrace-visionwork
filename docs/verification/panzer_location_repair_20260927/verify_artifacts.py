"""Decode exported recordings and summarize runtime parameters without ROS."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import json,subprocess,yaml
D=Path(__file__).resolve().parent
items=json.loads((D/'runs.json').read_text())
def check(task):
 seed,p=task
 q=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','format=duration,size:stream=width,height,avg_frame_rate','-of','json',str(p)]))
 r=subprocess.run(['ffmpeg','-v','error','-threads','1','-i',str(p),'-f','null','-'],capture_output=True,text=True)
 return dict(seed=seed,file=str(p),probe=q,decode_pass=r.returncode==0 and not r.stderr.strip(),errors=r.stderr[-1000:])
tasks=[(i['seed'],Path(i['run'])/f'{name}.mp4') for i in items for name in ('overview','follow','presentation_review')]
with ThreadPoolExecutor(max_workers=2) as pool:videos=list(pool.map(check,tasks))
params=[]
for i in items:
 run=Path(i['run']);p=yaml.safe_load((run/'rosparams.yaml').read_text());s=p['fast_planner_node']['sdf_map']
 assert s['obstacles_inflation']==.275 and s['horizontal_avoidance']['enabled']
 assert p['navigation']['mission_manager']['high_view_full']['policy']['interrupt_refined_classes']==['panzer']
 params.append(dict(seed=i['seed'],inflation_xy=s['obstacles_inflation'],up=s['obstacles_inflation_up'],down=s['obstacles_inflation_down'],resolution=s['resolution'],columns=s['horizontal_avoidance'],composition=json.loads((run/'presentation_review.json').read_text())))
 subprocess.run(['ffmpeg','-y','-v','error','-ss',str(120),'-i',str(run/'presentation_review.mp4'),'-frames:v','1',str(D/f"video_sample_{i['seed']}.jpg")],check=True)
result=dict(videos=videos,runtime_params=params,status='PASS' if all(v['decode_pass'] for v in videos) else 'FAIL')
(D/'artifact_validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if result['status']!='PASS':raise SystemExit(1)
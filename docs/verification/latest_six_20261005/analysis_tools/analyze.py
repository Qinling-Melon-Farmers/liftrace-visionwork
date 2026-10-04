from pathlib import Path
import json,importlib.util,subprocess,sys,yaml,argparse,csv,math
R=Path(__file__).resolve().parents[4]
D=R/'docs/verification/latest_six_20261005';D.mkdir(exist_ok=True)
parser=argparse.ArgumentParser();parser.add_argument('--seed',type=int);parser.add_argument('--skip-video',action='store_true');parser.add_argument('--refresh',action='store_true');a=parser.parse_args()
batch=json.loads((R/'logs/latest_six_20261005_batch/matrix.json').read_text())
items=[dict(seed=r['seed'],label=r['variant'],run=r['run'],world=str(R/f"docs/verification/latest_six_20261005/generated/{r['variant']}_{r['seed']}/{r['variant']}_seed{r['seed']}/field.world"),source=batch['source']) for r in batch['results'] if a.seed is None or r['seed']==a.seed]
(D/'runs.json').write_text(json.dumps(items,indent=2))
spec=importlib.util.spec_from_file_location('hist',R/'docs/verification/history_31_40_20260920/analyze_current.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);m.D=D
# Preserve historical analysis; draw this run's explicitly sized H assets.
scene_original=m.scene
def scene_current(ax,world,truth):
 scene_original(ax,world,truth)
 for obj in m.ET.parse(world).getroot().find('world').findall('include'):
  if obj.findtext('uri')=='model://landing_h_80cm':
   p=list(map(float,obj.findtext('pose').split()))
   ax.add_patch(m.Rectangle((p[0]-.4,p[1]-.4),.8,.8,fill=False,color='black'))
   ax.text(p[0],p[1],'H',ha='center',va='center')
m.scene=scene_current
m.prior.base.scene=scene_current

metrics=[]
for i in items:
 cache=D/(str(i['seed'])+'_'+i['label'])/'metrics.json'
 metrics.append(json.loads(cache.read_text()) if cache.exists() and not a.refresh else m.one(i))
for item,metric in zip(items,metrics):
 # The legacy tool reads mutable final hints: never label these as initial high views.
 for entry in metric.get('target_hint_evaluation',{}).values():
  if 'high_hint_error_m' in entry:
   entry['final_retained_hint_error_m']=entry.pop('high_hint_error_m')
 metric['hint_metric_note']='Final retained hints may already include low revisits. Initial high-view errors are reported separately by the center pipeline from event snapshots.'
 truth_rows=list(csv.DictReader((Path(item['run'])/'truth_pose.csv').open()))
 touchdown=metric.get('touchdown_ros_s')
 if touchdown is not None:
  row=min(truth_rows,key=lambda row:abs(float(row['t'])-touchdown))
  anchor=metric['gate_metrics']['landing_xy']
  metric['touchdown_truth_xy']=[float(row['x']),float(row['y'])]
  metric['touchdown_truth_sample_age_s']=abs(float(row['t'])-touchdown)
  metric['touchdown_truth_center_error_m']=math.hypot(float(row['x'])-anchor[0],float(row['y'])-anchor[1])
 (D/(str(item['seed'])+'_'+item['label'])/'metrics.json').write_text(json.dumps(metric,indent=2))
(D/'metrics.json').write_text(json.dumps(metrics,indent=2))
for i in ([] if a.skip_video else items):
 output=Path(i['run'])/'presentation_review.mp4'
 if output.exists():
  if not output.with_suffix('.json').exists():raise RuntimeError('Existing video lacks completion metadata: '+str(output))
  subprocess.run(['ffmpeg','-nostdin','-v','error','-threads','1','-i',str(output),'-f','null','-'],check=True)
  continue
 subprocess.run([sys.executable,str(R/'docs/verification/failed_six_20260921/compose_review.py'),i['run'],'--output',i['run']+'/presentation_review.mp4','--flight-limit','4','--corridor-limit','1.2','--wall-height','4','--corridor-bounds','8','9.5','-5','5','--case-label',f"{i['label']} Seed{i['seed']} columns ON | XY 0.25m | H 80cm"],check=True)
print('ANALYSIS COMPLETE' if a.skip_video else 'ANALYSIS AND VIDEOS COMPLETE')
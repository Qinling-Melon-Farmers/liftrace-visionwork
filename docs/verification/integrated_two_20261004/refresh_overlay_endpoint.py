#!/usr/bin/env python3
"""Preserve the previous derived overlay, regenerate its inclusive endpoint, validate."""
import argparse,csv,json,subprocess,sys
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--seed',type=int,choices=(31,38),required=True)
a=p.parse_args();D=Path(__file__).resolve().parent
video=D/f'{a.seed}_mapped_overlay.mp4'
meta=json.loads(video.with_suffix('.json').read_text())
if meta.get('final_source_frame_shown'):
    raise ValueError('Endpoint already inclusive')
archive=D/f'{a.seed}_centers/inspection/prior_endpoint_overlay'
archive.mkdir(exist_ok=False)
for suffix in ['.mp4','.json','.frames.csv','.validation.json']:
    source=video.with_suffix(suffix)
    source.rename(archive/source.name)
cmd=[sys.executable,'-B',str(D/'mapped_video_overlay.py'),
     '--video',meta['source_video'],'--timestamps',meta['source_csv'],
     '--data',meta['mapped_data'],'--centers',meta['center_csv'],
     '--output',str(video),'--fps',str(meta['fps']),'--match-ms','30','--threads','1',
     '--label',meta['label']]
subprocess.run(cmd,check=True)
subprocess.run([sys.executable,'-B',str(D/'verify_overlay.py'),str(video)],check=True)
inspection=D/f'{a.seed}_centers/inspection/inspection.json'
obj=json.loads(inspection.read_text())
obj['previous_endpoint_coverage']=dict(obj['coverage'])
rows=list(csv.DictReader(video.with_suffix('.frames.csv').open()))
c=obj['coverage'];last=rows[-1]
c.update(last_output_playback_ros_s=float(last['playback_ros_s']),
    last_output_source_ros_s=float(last['image_stamp_ros_s']),
    last_output_source_frame=int(last['source_frame']),
    final_source_frames_not_selected_by_output_grid=c['source_frames']-int(last['source_frame'])-1,
    final_source_to_last_output_tick_s=c['last_source_ros_s']-float(last['playback_ros_s']))
obj['endpoint_correction']='Inclusive last output tick; previous derived media retained under prior_endpoint_overlay. Raw media unchanged.'
inspection.write_text(json.dumps(obj,indent=2)+'\n')

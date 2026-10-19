from pathlib import Path
import json,sys,shutil,subprocess,html
from finish_trial import summarize
out=Path(sys.argv[1]);links=[];result=summarize(out);committed=result['committed_deliveries']
for name in ('camera_raw','camera_annotated'):
    source=out/(name+'.mp4');target=out/(name+'_h264.mp4')
    if not source.exists():continue
    if shutil.which('ffmpeg') and not target.exists():
        completed=subprocess.run(['ffmpeg','-nostdin','-v','error','-i',str(source),'-an','-c:v','libx264','-threads','2','-preset','veryfast','-crf','23','-movflags','+faststart',str(target)])
        if completed.returncode:target.unlink(missing_ok=True)
    video=target if target.exists() else source;links.append(f'<h2>{name}</h2><video controls src="{video.name}" style="max-width:100%"></video>')
bag_report=out/'bag_recording.json'
if bag_report.exists():
    record=json.loads(bag_report.read_text())
    links.append('<h2>ROS bag</h2><p>'+html.escape(record['status'])+'; <a href="bag_recording.json">topic counts / missing data</a></p>')
    links.extend('<p><a href="'+html.escape(name,quote=True)+'">'+html.escape(name)+'</a></p>' for name in record.get('bags',[]))
(out/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Modular board camera review</title><p>Actuator: '+html.escape(result['actuator_mode'])+'</p><h1>相机与视觉链回看</h1><p>结果：'+result['status']+'；投递确认 '+str(committed)+' 次。<a href="result.json">结果详情</a></p><p>橙色：YOLO；绿色：几何精修/地图投影。时间未匹配的框不画在当前图像上。底栏为任务/记忆/对准状态，详见 vision_events.jsonl 与 camera_frames.csv。INCOMPLETE保留中途停止、预览、缺靶与失败，不伪报成功。</p>'+''.join(links))
print('Camera review:',out/'index.html')

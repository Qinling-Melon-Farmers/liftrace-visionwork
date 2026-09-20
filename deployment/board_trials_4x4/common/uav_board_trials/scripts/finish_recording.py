from pathlib import Path
import json,sys,shutil,subprocess,html
out=Path(sys.argv[1]);links=[];latest={};mock_calls=0
if (out/'vision_events.jsonl').exists():
    for line in (out/'vision_events.jsonl').read_text().splitlines():
        try:e=json.loads(line)
        except ValueError:continue
        latest[e['kind']]=e['data'];mock_calls+=int(e['kind']=='mock')
supervisor=json.loads((out/'supervisor_result.json').read_text()) if (out/'supervisor_result.json').exists() else {}
mission=latest.get('mission',{});high=latest.get('high',{});trial=supervisor.get('trial');committed=mission.get('committed_slots',0);expected=1 if trial=='visual_interrupt' else high.get('trial_memory_count',0)
landed=supervisor.get('end_reason')=='landed_after_flight'
success=(landed and ((trial in ('landing','corridor_landing') and mission.get('phase')=='COMPLETE') or (trial not in ('landing','corridor_landing') and 1<=expected<=3 and committed==expected and latest.get('land_handoff',{}).get('mode_sent') is True)))
result=dict(status='PASS' if success else 'INCOMPLETE',trial=trial,expected_mock_deliveries=expected if trial not in ('landing','corridor_landing') else 0,committed_mock_deliveries=committed,mock_service_calls=mock_calls,supervisor=supervisor,final_mission=mission,final_high_view=high,auto_land_handoff=latest.get('land_handoff'))
(out/'result.json').write_text(json.dumps(result,indent=2))
for name in ('camera_raw','camera_annotated'):
    source=out/(name+'.mp4');target=out/(name+'_h264.mp4')
    if not source.exists():continue
    if shutil.which('ffmpeg') and not target.exists():
        completed=subprocess.run(['ffmpeg','-nostdin','-v','error','-i',str(source),'-an','-c:v','libx264','-threads','2','-preset','veryfast','-crf','23','-movflags','+faststart',str(target)])
        if completed.returncode:target.unlink(missing_ok=True)
    video=target if target.exists() else source;links.append(f'<h2>{name}</h2><video controls src="{video.name}" style="max-width:100%"></video>')
(out/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>4x4 board camera review</title><h1>相机与视觉链回看（模拟投递）</h1><p>结果：'+result['status']+'；模拟投递确认 '+str(committed)+' 次。<a href="result.json">结果详情</a></p><p>橙色：YOLO；绿色：几何精修/地图投影。时间未匹配的框不画在当前图像上。底栏为任务/记忆/对准状态，详见 vision_events.jsonl 与 camera_frames.csv。INCOMPLETE保留中途停止、预览、缺靶与失败，不伪报成功。</p>'+''.join(links))
print('Camera review:',out/'index.html')

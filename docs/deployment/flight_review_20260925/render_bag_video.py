"""Offline recorded-image/recorded-evidence replay. No ROS publication or inference."""
import ast
import bisect
import csv
import json
import math
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
OUT=ROOT/'logs/flight_review_20260925/video'

def plain(m):
    if hasattr(m,'__slots__'):return {k:plain(getattr(m,k)) for k in m.__slots__}
    if isinstance(m,(tuple,list)):return [plain(v) for v in m]
    return m

def export():
    import rosbag
    OUT.mkdir(parents=True,exist_ok=True);(OUT/'frames').mkdir(exist_ok=True)
    frames=[]; records={}
    topics=['/camera/image_raw/compressed','/uav_vision/detections','/uav_vision/detections_resolved','/uav_vision/detections_refined','/uav_vision/detections_mapped','/uav_vision/targets','/uav_vision/drop_offset','/uav_vision/release_evidence','/uav_vision/align_mode','/mission/release_permission_active','/mission/release_result','/navigation/mission_status','/mavros/state']
    with rosbag.Bag(str(next((ROOT/'试飞产物').glob('flight_debug_2026-09-25*.bag')))) as b:
        start=b.get_start_time();end=b.get_end_time()
        for topic,m,t in b.read_messages(topics=topics):
            if topic==topics[0]:
                name=f'frames/{len(frames):05d}.jpg';(OUT/name).write_bytes(bytes(m.data))
                frames.append(dict(file=name,t=t.to_sec()-start,stamp=m.header.stamp.to_sec()-start))
            else:
                stamp=m.header.stamp.to_sec()-start if hasattr(m,'header') else t.to_sec()-start
                records.setdefault(topic,[]).append(dict(t=t.to_sec()-start,stamp=stamp,m=plain(m)))
    (OUT/'records.json').write_text(json.dumps(dict(start=start,duration=end-start,frames=frames,records=records)))
    print('Exported',len(frames),'camera frames',flush=True)

def ns(x):
    if isinstance(x,dict):return SimpleNamespace(**{k:ns(v) for k,v in x.items()})
    if isinstance(x,list):return [ns(v) for v in x]
    return x

def render():
    import cv2
    import numpy as np
    cv2.setNumThreads(2)
    d=json.loads((OUT/'records.json').read_text());records=d['records'];frames=d['frames']
    # Reuse the existing recorder's pure pixel drawing functions without ROS imports.
    source=ast.parse((ROOT/'vision_ws/src/uav_vision/scripts/video_annotation_recorder.py').read_text())
    names={'_roi_points','_center','_put_label','_draw_raw','_draw_refined','_draw_fused'}
    nodes=[n for n in source.body if isinstance(n,ast.FunctionDef) and n.name in names]
    ctx=dict(cv2=cv2,np=np,YOLO_COLOR=(0,220,0),GEOMETRY_COLOR=(255,220,0),REFINED_COLOR=(0,140,255),REJECTED_COLOR=(0,255,255))
    exec(compile(ast.Module(body=nodes,type_ignores=[]),'existing_video_annotation_recorder','exec'),ctx)
    groups={}
    for topic,rows in records.items():
        if '/detections' in topic:
            for r in rows:groups.setdefault((topic,r['m']['source']),[]).append(r)
    for rows in groups.values():rows.sort(key=lambda r:r['stamp'])
    gt={k:[r['stamp'] for r in v] for k,v in groups.items()}
    rt={k:[r['t'] for r in v] for k,v in records.items()}
    ft=[r['t'] for r in frames]
    def latest(topic,t,age=1.0):
        rows=records.get(topic,[]);i=bisect.bisect_right(rt.get(topic,[]),t)-1
        return rows[i]['m'] if i>=0 and t-rows[i]['t']<=age else None
    def matched(key,t):
        ts=gt[key];i=bisect.bisect_left(ts,t);ids=[j for j in (i-1,i) if 0<=j<len(ts)]
        j=min(ids,key=lambda j:abs(ts[j]-t))
        return groups[key][j] if abs(ts[j]-t)<=.03 else None
    motion=json.loads((OUT.parent/'extracted.json').read_text())['motion'];mt=np.array([r['t'] for r in motion])
    xy=np.array([[r['x'],r['y']] for r in motion]);z=np.array([r['z'] for r in motion])
    speed=np.linalg.norm(np.column_stack([(np.interp(mt+.25,mt,xy[:,k])-np.interp(mt-.25,mt,xy[:,k]))/.5 for k in (0,1)]),axis=1)
    fps=15;writers=[]
    for name,height in [('camera_raw',720),('camera_annotated',1080)]:
        writers.append(subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','bgr24','-s',f'1280x{height}','-r',str(fps),'-i','-','-an','-c:v','libx264','-preset','fast','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(OUT/(name+'.mp4'))],stdin=subprocess.PIPE))
    count=math.ceil(d['duration']*fps);last_index=-1;matches=0
    with (OUT/'video_frames.csv').open('w') as f:
        csvout=csv.writer(f);csvout.writerow(['video_frame','bag_seconds','image_stamp_seconds','image_age_seconds','matched_messages'])
        for index in range(count):
            t=index/fps;fi=max(0,bisect.bisect_right(ft,t)-1);frame=frames[fi]
            if fi!=last_index:raw=cv2.imread(str(OUT/frame['file']));last_index=fi
            image=raw.copy();raw_msgs={};mapped=[];refined=[];delays=[]
            for key in groups:
                r=matched(key,frame['stamp'])
                if r is None:continue
                delays.append(r['t']-r['stamp'])
                if key[0]=='/uav_vision/detections':raw_msgs[key[1]]=ns(r['m'])
                elif key[0]=='/uav_vision/detections_refined':refined.extend(r['m']['detections'])
                elif key[0]=='/uav_vision/detections_mapped':mapped.extend(r['m']['detections'])
            ctx['_draw_raw'](image,raw_msgs);ctx['_draw_refined'](image,ns(dict(detections=refined)))
            for item in mapped:
                if item['map_valid']:
                    p=item['center_px'];cv2.drawMarker(image,(round(p['x']),round(p['y'])),(255,0,255),cv2.MARKER_CROSS,18,2)
            mission=latest('/navigation/mission_status',t);mission=json.loads(mission['data']) if mission else {}
            mode=latest('/uav_vision/align_mode',t);fc=latest('/mavros/state',t,2) or {}
            targets=latest('/uav_vision/targets',t) or {'targets':[]}
            ev=latest('/uav_vision/release_evidence',t) or {};permit=latest('/mission/release_permission_active',t) or {}
            rel=latest('/mission/release_result',t,999) or {}
            offset=latest('/uav_vision/drop_offset',t) or {}
            mem=', '.join(f"{r['class_name']}#{r['id']} state={r['state']} map={int(r['map_valid'])}" for r in targets['targets']) or 'none (fresh topic)'
            mr='; '.join(f"{r['class_name']} map={int(r['map_valid'])} {r['reject_reason']}" for r in mapped if r['class_name']!='circle') or 'no matched actionable map result'
            lines=[f"RECORDED BAG REPLAY  1x | t={t:6.2f}s / {d['duration']:.2f}s | camera image t={frame['stamp']:.2f}s",
                f"FC local Z={np.interp(t,mt,z):.2f}m | XY speed~{np.interp(t,mt,speed):.2f}m/s | {fc.get('mode','?')} armed={fc.get('armed','?')}",
                f"Mission {mission.get('phase','?')} / {mission.get('active_command','')} | committed={mission.get('committed_slots','?')} | {mission.get('slot_status',[])}",
                f"Vision mode={(mode or {}).get('data','?')} | memory: {mem}",
                'Map: '+mr,
                f"Align={ev.get('aligned','?')} evidence={ev.get('evidence_valid','?')} permit={permit.get('data','?')} | dx={offset.get('dx_px','?')} dy={offset.get('dy_px','?')}",
                f"Last release: {rel.get('target_class','none')} success={rel.get('success','?')} {rel.get('reason','')}",
                'Green=YOLO  Cyan=geometry  Orange=refined  Yellow=rejected  Magenta=map-valid',
                f"Pixel results matched by image stamp <=30ms (offline); max recorded processing lag={max(delays,default=0):.2f}s",
                'No detector rerun. Task states follow recorded receipt time. Missing/stale evidence is not a negative detection.']
            panel=np.full((360,1280,3),20,np.uint8)
            for i,line in enumerate(lines):cv2.putText(panel,line[:150],(12,27+i*34),cv2.FONT_HERSHEY_SIMPLEX,.54,(235,235,235),1,cv2.LINE_AA)
            full=np.vstack((image,panel));writers[0].stdin.write(raw.tobytes());writers[1].stdin.write(full.tobytes())
            csvout.writerow([index,t,frame['stamp'],t-frame['stamp'],len(delays)]);matches+=bool(delays)
            if index in (round(22.7*fps),round(41.8*fps),round(48.7*fps)):cv2.imwrite(str(OUT/f'preview_{index}.jpg'),full)
            if index%450==0:print('Rendered',index,'/',count,flush=True)
    for p in writers:
        p.stdin.close()
        if p.wait()!=0:raise RuntimeError('ffmpeg failed')
    (HERE/'video_metadata.json').write_text(json.dumps(dict(fps=fps,frames=count,duration=count/fps,source_images=len(frames),matched_frames=matches,box_tolerance_seconds=.03,mode='offline recorded results; no inference rerun',outputs=str(OUT)),indent=2))

if __name__=='__main__':
    {'export':export,'render':render}[sys.argv[1]]()

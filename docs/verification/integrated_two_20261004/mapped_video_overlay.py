#!/usr/bin/env python3
"""Overlay recorded mapped detections on recorded video, using source-image stamps."""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import subprocess

def stamp_ns(value):
    return int(value.get('secs',0))*1000000000+int(value.get('nsecs',0))

def message_stamp(message):
    return stamp_ns(message.get('header',{}).get('stamp',{}))

def nearest(stamps, now, tolerance):
    i=bisect.bisect_left(stamps,now)
    choices=[j for j in (i-1,i) if 0<=j<len(stamps)]
    if not choices:return None
    j=min(choices,key=lambda k:abs(stamps[k]-now))
    return stamps[j] if abs(stamps[j]-now)<=tolerance else None

def metric_key(stamp,cls,frame,point):
    def value(x):
        try:
            x=float(x)
            return round(x,6) if math.isfinite(x) else None
        except (ValueError,TypeError):
            return None
    return (stamp,cls,frame,*[value(x) for x in point])

def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('video','timestamps','data','centers','output'):
        p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--fps',type=float,default=10)
    p.add_argument('--match-ms',type=float,default=30)
    p.add_argument('--threads',type=int,choices=(1,2),default=2)
    p.add_argument('--label',default='Recorded mapped detections')
    p.add_argument('--max-seconds',type=float,default=900)
    a=p.parse_args()
    if a.output.exists():raise ValueError('Output exists; choose a new file')
    if not 1<=a.fps<=30 or not 0<a.match_ms<=30:raise ValueError('fps 1..30; match tolerance at most 30 ms')
    import cv2
    import numpy as np
    cv2.setNumThreads(1)
    timestamps=list(csv.DictReader(a.timestamps.open()))
    times=[float(row['image_stamp_ros_sec']) for row in timestamps]
    if not times or any(b<=x for x,b in zip(times,times[1:])):
        raise ValueError('Image stamps must be nonempty and strictly increasing')
    if 'frame' in timestamps[0] and [int(r['frame']) for r in timestamps]!=list(range(len(times))):
        raise ValueError('CSV frame numbers do not match sequential source video')
    duration=times[-1]-times[0]
    if not 0<duration<=a.max_seconds:raise ValueError('Invalid or over-budget source ROS duration')
    groups={}
    # Include empty arrays so an empty result is not confused with missing detection input.
    with a.data.open() as source:
        for line in source:
            event=json.loads(line)
            if event.get('kind')!='mapped':continue
            msg=event['m'];array_stamp=message_stamp(msg)
            if array_stamp:groups.setdefault(array_stamp,[])
            for det in msg.get('detections',[]):
                s=message_stamp(det) or array_stamp
                if s:groups.setdefault(s,[]).append(det)
    for s,items in groups.items():
        unique={json.dumps(item,sort_keys=True):item for item in items}
        groups[s]=list(unique.values())
    detection_stamps=sorted(groups)
    metrics={}
    with a.centers.open() as source:
        for row in csv.DictReader(source):
            if row['stream']!='mapped':continue
            s=int(round(float(row['source_ros_s'])*1e9))
            key=metric_key(s,row['class_name'],row['map_frame'],[row[k] for k in ('raw_x','raw_y','raw_z')])
            metrics.setdefault(key,row)
    if not hasattr(cv2,'CAP_PROP_N_THREADS'):
        raise RuntimeError('Use the existing rl_drone OpenCV with CAP_PROP_N_THREADS support')
    cap=cv2.VideoCapture(str(a.video),cv2.CAP_FFMPEG,[cv2.CAP_PROP_N_THREADS,1])
    if not cap.isOpened():raise ValueError('Cannot decode source video')
    frame_count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if frame_count!=len(times):raise ValueError('Source MP4 frame count differs from timestamp CSV')
    width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH));height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    panel_width=560;top=94
    output_width=(width+panel_width+1)//2*2
    output_height=(max(height,640)+top+1)//2*2
    a.output.parent.mkdir(parents=True,exist_ok=True)
    metadata_path=a.output.with_suffix('.json')
    mapping_path=a.output.with_suffix('.frames.csv')
    if metadata_path.exists() or mapping_path.exists():raise ValueError('Output sidecars exist; choose a new stem')
    encoder=subprocess.Popen(['ffmpeg','-nostdin','-hide_banner','-loglevel','error','-n',
        '-f','rawvideo','-pixel_format','bgr24','-video_size',f'{output_width}x{output_height}',
        '-framerate',str(a.fps),'-i','pipe:0','-an','-c:v','libx264','-threads',str(a.threads),
        '-preset','veryfast','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(a.output)],
        stdin=subprocess.PIPE)
    source_index=-1;previous_output_index=-1;frame=None;cached=None;matched_stamp=None;delta_ms=None;matched_count=0
    empty_matches=0;missing_matches=0;processed=0;max_image_age=0.;max_match_delta=0.;written=0
    def text(image,line,x,y,color=(230,230,230),scale=.48):
        cv2.putText(image,str(line),(int(x),int(y)),cv2.FONT_HERSHEY_SIMPLEX,scale,color,1,cv2.LINE_AA)
    def number(value):
        try:return f'{float(value):.3f}'
        except (ValueError,TypeError):return 'NA'
    try:
        with mapping_path.open('x',newline='') as out:
            writer=csv.writer(out)
            writer.writerow(['output_frame','playback_ros_s','source_frame','image_stamp_ros_s',
                'image_age_s','mapped_source_stamp_ros_s','match_delta_ms','recorded_detections'])
            # Include the final recorded frame on the next output tick, even
            # when the source endpoint is between two ticks of the 10 Hz grid.
            count=int(math.ceil(duration*a.fps-1e-7))+1
            for k in range(count):
                now=times[0]+k/a.fps
                wanted=max(0,min(len(times)-1,bisect.bisect_right(times,now)-1))
                while source_index<wanted:
                    ok,frame=cap.read()
                    if not ok:raise RuntimeError('Source decoder ended before timestamp CSV')
                    source_index+=1
                if cached is None or cached[0]!=source_index:
                    canvas=np.full((output_height,output_width,3),24,np.uint8)
                    canvas[top:top+height,:width]=frame
                    stamp=int(round(times[source_index]*1e9))
                    matched_stamp=nearest(detection_stamps,stamp,int(round(a.match_ms*1e6)))
                    dets=groups[matched_stamp] if matched_stamp is not None else []
                    delta_ms=(matched_stamp-stamp)/1e6 if matched_stamp is not None else None
                    processed+=1
                    if matched_stamp is None:missing_matches+=1
                    elif not dets:empty_matches+=1
                    else:matched_count+=1
                    if delta_ms is not None:max_match_delta=max(max_match_delta,abs(delta_ms))
                    text(canvas,'Recorded ROI + center_px only',width+12,top+23)
                    text(canvas,'No detector rerun; no reconstructed H contour',width+12,top+46,scale=.43)
                    text(canvas,'Distances: offline truth association, not release error',width+12,top+68,scale=.42)
                    if matched_stamp is None:
                        text(canvas,'NO MAPPED MESSAGE WITHIN 30ms',width+12,top+105,(50,190,255),.52)
                    elif not dets:
                        text(canvas,'MATCHED EMPTY DETECTION ARRAY',width+12,top+105,(140,210,200),.50)
                    for j,det in enumerate(dets):
                        cls=det.get('class_name','?');valid=bool(det.get('map_valid'))
                        point=det.get('map_point',{});frame_name=det.get('map_frame','')
                        row=metrics.get(metric_key(matched_stamp,cls,frame_name,[point.get(c) for c in ('x','y','z')]),{})
                        confusion=str(row.get('category_confusion_suspected','')).lower()=='true'
                        color=(80,85,255) if confusion else (75,220,95) if valid else (45,185,255)
                        roi=det.get('roi',{})
                        x=int(roi.get('x_offset',0));y=int(roi.get('y_offset',0))
                        rw=int(roi.get('width',0));rh=int(roi.get('height',0))
                        x0=max(0,min(width-1,x));y0=max(0,min(height-1,y))
                        x1=max(0,min(width-1,x+rw));y1=max(0,min(height-1,y+rh))
                        if rw>0 and rh>0 and x1>x0 and y1>y0:
                            cv2.rectangle(canvas,(x0,y0+top),(x1,y1+top),color,2)
                            text(canvas,f'{j+1}:{cls}',x0,max(top+16,y0+top-5),color,.56)
                        center=det.get('center_px',{})
                        cx=float(center.get('x',float('nan')));cy=float(center.get('y',float('nan')))
                        if math.isfinite(cx) and math.isfinite(cy) and 0<=cx<width and 0<=cy<height:
                            cv2.drawMarker(canvas,(round(cx),round(cy)+top),color,cv2.MARKER_CROSS,24,2)
                            text(canvas,f'{j+1}',round(cx)+8,round(cy)+top-8,color,.52)
                        yy=top+106+j*122
                        if yy+113>=output_height:
                            text(canvas,'More recorded detections: see center_samples.csv',width+12,output_height-16,(100,190,250),.43)
                            break
                        tag='CLASS/INSTANCE AMBIGUITY' if confusion else ('MAP VALID' if valid else 'MAP INVALID')
                        text(canvas,f'{j+1} {cls} | {tag}',width+12,yy,color,.48)
                        text(canvas,f'XY=({number(point.get("x"))},{number(point.get("y"))}) [{frame_name}]',width+12,yy+22,scale=.43)
                        text(canvas,'nearest: '+row.get('nearest_any_instance','NA')+' / '+row.get('nearest_any_class','NA'),width+12,yy+44,scale=.43)
                        text(canvas,'nearestDist='+number(row.get('nearest_any_distance_m'))+'m   sameClassDist='+number(row.get('same_class_distance_m'))+'m',width+12,yy+66,scale=.42)
                        text(canvas,'center='+det.get('center_source','?')+' refined='+str(bool(det.get('center_refined'))),width+12,yy+88,scale=.42)
                        reason=det.get('reject_reason','') or row.get('exclusion','')
                        if reason:text(canvas,'reject: '+reason[:68],width+12,yy+109,(100,190,250),.40)
                    cached=(source_index,canvas,len(dets))
                display=cached[1].copy()
                age=max(0.,now-times[source_index]);max_image_age=max(max_image_age,age)
                held='HELD RECORDED FRAME' if source_index==previous_output_index else 'recorded frame'
                previous_output_index=source_index
                text(display,a.label[:140],12,22,scale=.58)
                text(display,f'Playback ROS {now:.3f}s | image stamp {times[source_index]:.3f}s | age {age:.3f}s | {held}',12,49,
                     (65,190,255) if age>.3 else (230,230,230),.51)
                match_text='NONE' if matched_stamp is None else f'{matched_stamp/1e9:.3f}s delta={delta_ms:+.2f}ms'
                text(display,'Mapped image stamp: '+match_text+' | 1x source ROS clock | center = recorded point',12,76,scale=.48)
                encoder.stdin.write(display.tobytes());written+=1
                writer.writerow([k,f'{now:.9f}',source_index,f'{times[source_index]:.9f}',f'{age:.9f}',
                    f'{matched_stamp/1e9:.9f}' if matched_stamp is not None else '',delta_ms,cached[2]])
        encoder.stdin.close()
        if encoder.wait()!=0:raise RuntimeError('FFmpeg encode failed')
    finally:
        cap.release()
        if encoder.poll() is None:
            encoder.terminate();encoder.wait()
    metadata=dict(source_video=str(a.video),source_csv=str(a.timestamps),mapped_data=str(a.data),
        center_csv=str(a.centers),label=a.label,output=str(a.output),output_frames=written,fps=a.fps,
        source_ros_span_s=duration,source_frames=len(times),visited_source_frames=processed,
        frames_with_nonempty_match=matched_count,frames_with_empty_match=empty_matches,
        frames_without_match=missing_matches,max_match_abs_delta_ms=max_match_delta,max_held_image_age_s=max_image_age,
        match_tolerance_ms=a.match_ms,ffmpeg_threads=a.threads,
        final_source_frame_shown=source_index==len(times)-1,
        last_source_ros_s=times[-1],last_output_ros_s=now,
        note='Source-image stamp match only. Held images labelled. Recorded ROI/center only; no H shape fitting or model inference.',
        interpretation='Nearest-any and same-class distances are different; class ambiguity is not pure localization error.',
        validation='Encoding completed; run ffmpeg full decode separately.')
    metadata_path.write_text(json.dumps(metadata,indent=2))
    print(json.dumps(metadata,indent=2))
if __name__=='__main__':main()

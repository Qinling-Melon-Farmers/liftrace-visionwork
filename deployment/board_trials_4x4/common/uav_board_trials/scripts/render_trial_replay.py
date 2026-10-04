#!/usr/bin/env python3
"""Recorded camera + telemetry animation. No inference, synthetic flight, or ROS."""
import argparse,bisect,csv,json,subprocess
from pathlib import Path
import cv2,numpy as np,yaml

def read_csv(path,keys):
    return np.array([[float(r[k]) for k in keys] for r in csv.DictReader(path.open())])

def main():
    ap=argparse.ArgumentParser();ap.add_argument('directory',type=Path);args=ap.parse_args();d=args.directory
    ref=json.loads((d/'ground_reference.json').read_text());scene=json.loads((d/'scene.json').read_text());runtime=yaml.safe_load((d/'runtime.yaml').read_text())
    stamps=read_csv(d/'camera_frames.csv',('record_ros_sec','image_ros_sec'));pose=read_csv(d/'navigation_pose.csv',('t','x','y','z'))
    truth=read_csv(d/'truth_pose.csv',('t','x','y','z')) if (d/'truth_pose.csv').exists() else None
    events=[];observations={'mapped':[],'coarse':[]}
    for line in (d/'vision_events.jsonl').read_text().splitlines():
        e=json.loads(line)
        if e['kind']=='mission':events.append((e['t'],e['data']))
        if e['kind'] in observations:observations[e['kind']].append((e['t'],e['data']))
    events.sort(key=lambda e:e[0]);times=[e[0] for e in events];t0=stamps[0,0];duration=stamps[-1,0]-t0
    cap=cv2.VideoCapture(str(d/'camera_annotated.mp4'));fps=cap.get(cv2.CAP_PROP_FPS);n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));assert n==len(stamps),(n,len(stamps))
    output=d/'report_replay.mp4'
    ff=subprocess.Popen(['ffmpeg','-nostdin','-y','-v','error','-f','rawvideo','-pixel_format','bgr24','-video_size','1280x720','-framerate',str(fps),'-i','pipe:0','-an','-c:v','libx264','-threads','2','-preset','veryfast','-crf','23','-pix_fmt','yuv420p','-movflags','+faststart',str(output)],stdin=subprocess.PIPE)
    base=np.full((720,1280,3),(245,245,245),np.uint8)
    def text(im,s,xy,color=(35,35,35),size=.48):cv2.putText(im,str(s),xy,cv2.FONT_HERSHEY_SIMPLEX,size,color,1,cv2.LINE_AA)
    def xy(x,y):return (round(832+(x+.8)/5.2*422),round(457-(y+2.4)/4.8*390))
    def path(im,pts,color,thick=1):
        if len(pts)>1:cv2.polylines(im,[np.array([xy(*v) for v in pts],np.int32)],False,color,thick,cv2.LINE_AA)
    cv2.rectangle(base,xy(0,2),xy(4,-2),(70,70,70),1)
    for ob in scene['obstacles']:
        x,y=ob['xy']
        if 'size' in ob:
            dx,dy,_=ob['size'];cv2.rectangle(base,xy(x-dx/2,y+dy/2),xy(x+dx/2,y-dy/2),(140,140,140),-1)
        else:cv2.circle(base,xy(x,y),round(.4/5.2*422),(125,180,120),-1)
    for target in scene['targets']:
        pos=xy(target['x'],target['y']);cv2.drawMarker(base,pos,(35,35,220),cv2.MARKER_CROSS,10,1);text(base,target['class_name'],(pos[0]+5,pos[1]-6),size=.36)
    pts=runtime['trial']['waypoints']
    if ref['mode'].startswith('high') or ref['mode']=='memory_only':pts=runtime['high_view_probe']['config']['survey_xy']
    if ref['mode']=='landing':pts=runtime['mission']['post_delivery_route']
    path(base,[p[:2] for p in pts],(180,180,180))
    if ref['mode']=='high_view_full' or scene['trial']=='corridor_landing':path(base,[p[:2] for p in runtime['mission']['post_delivery_route']],(180,180,180))
    cv2.circle(base,xy(0,0),8,(65,65,65),1)
    land=ref['settings'].get('landing_xy')
    if land:
        cv2.circle(base,xy(*land),8,(65,65,65),1);text(base,'H',xy(land[0]+.1,land[1]+.1),size=.42)
    text(base,'red +: scene targets; green/yellow: live map/hints',(824,35),size=.37)
    cv2.arrowedLine(base,xy(0,0),xy(.7,0),(20,20,20),1,tipLength=.25);text(base,'nose +X',xy(-.45,-.35),size=.34)
    text(base,'X forward > ; +Y left is up',(839,480),size=.42)
    text(base,'Telemetry reconstruction (not Gazebo video)',(824,51),size=.43)
    text(base,'blue: estimate | orange: truth when recorded',(825,505),size=.40)
    def chart(t,z):return (round(845+(t-t0)/max(duration,1)*405),round(680-z/3.3*140))
    for z in (0,1,2,3):
        cv2.line(base,chart(t0,z),chart(t0+duration,z),(215,215,215),1);text(base,str(z), (823,chart(t0,z)[1]+4),size=.38)
    text(base,'FC height / m; time follows camera CSV',(830,533),size=.42)
    text(base,'Elapsed ROS time / s',(916,709),size=.4)
    for sec in range(0,int(duration)+1,20):text(base,str(sec),(chart(t0+sec,0)[0]-5,698),size=.34)
    for k,(now,_) in enumerate(stamps):
        ok,camera=cap.read()
        if not ok:raise RuntimeError(f'camera frame {k} missing')
        im=base.copy();im[40:595,:800]=cv2.resize(camera,(800,555))
        idx=bisect.bisect_right(times,now)-1;m=events[idx][1] if idx>=0 else {}
        text(im,f'{scene["trial"]} | recorded ROS +{now-t0:.1f}s / {duration:.1f}s',(12,25),size=.65)
        text(im,f'MOCK release | phase {m.get("phase","WAIT")} | committed {m.get("committed_slots",0)}',(14,625),size=.55)
        text(im,m.get('last_reason',''),(14,653),size=.5)
        text(im,'Original camera annotations; no detector rerun',(14,681),size=.5)
        text(im,'Guide lines are task waypoints, not optimized trajectories',(14,707),size=.42)
        for arr,color,zoffset in ((pose,(200,100,20),-ref['ground_z']),(truth,(0,150,240),0)):
            if arr is None:continue
            j=np.searchsorted(arr[:,0],now,side='right');part=arr[:j]
            if not len(part):continue
            path(im,part[:,1:3],color,2);cv2.circle(im,xy(*part[-1,1:3]),4,color,-1)
            pts=[chart(t,z+zoffset) for t,_,_,z in part]
            if len(pts)>1:cv2.polylines(im,[np.array(pts,np.int32)],False,color,1,cv2.LINE_AA)
        for kind,color in (('mapped',(35,170,35)),('coarse',(0,160,185))):
            buf=observations[kind];j=bisect.bisect_right([e[0] for e in buf],now)-1
            if j<0 or now-buf[j][0]>.6:continue
            for det in buf[j][1]:
                point=det.get('xy',[])
                if not det.get('map_valid') or len(point)!=2 or not np.isfinite(point).all() or now-det.get('stamp',0)>.6:continue
                a,b=xy(*point);cv2.circle(im,(a,b),5,color,1)
                text(im,f'{det["class_name"]} ({point[0]:.2f},{point[1]:.2f})',(min(a+6,1090),max(62,b-8)),color,size=.31)
        ff.stdin.write(im.tobytes())
    cap.release();ff.stdin.close();code=ff.wait()
    if code:raise RuntimeError('ffmpeg export failed')
    (d/'report_replay.json').write_text(json.dumps(dict(source='recorded camera + CSV telemetry; no new flight or inference',frames=n,fps=fps,recorded_span_s=duration,video_s=n/fps,truth_available=truth is not None),indent=2))
    print(output)
if __name__=='__main__':main()

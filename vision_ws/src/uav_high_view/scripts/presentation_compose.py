#!/usr/bin/env python3
"""Offline paired-camera report video, synchronized by ROS image timestamps."""
import argparse,csv,json,math,subprocess
from pathlib import Path
import cv2,numpy as np,yaml
from PIL import Image,ImageDraw,ImageFont

PHASES={'IDLE':'待启动','STARTUP':'系统准备','SEARCH':'搜索','SURVEY':'高位快速搜索','ASCEND':'起飞 / 升高','DESCEND':'回降低空',
        'LOCAL_DESCENT_TRANSIT':'转移到下降位置','REVISIT':'低空目标复访','REACQUIRE':'视觉重新捕获',
        'DELIVERY':'对准 / 投递 / 恢复','LOW_COVERAGE':'低空补充搜索','TAIL':'投后航段',
        'POST_DELIVERY_ROUTE':'走廊穿行','LAND':'H标志降落','COMPLETE':'任务完成'}

class Stream:
    def __init__(self,path):
        self.cap=cv2.VideoCapture(str(path));self.index=-1;self.frame=None
        with path.with_suffix('.csv').open() as f:self.times=np.array([float(r['image_stamp_ros_sec']) for r in csv.DictReader(f)])
        if len(self.times)<2 or np.any(np.diff(self.times)<=0):raise ValueError('invalid camera time series')
    def at(self,t):
        desired=max(0,min(len(self.times)-1,int(np.searchsorted(self.times,t,side='right')-1)))
        while self.index<desired:
            ok,self.frame=self.cap.read()
            if not ok:raise RuntimeError('video shorter than timestamp file')
            self.index+=1
        return self.frame,t-self.times[desired]

def jsonl(path):return [json.loads(l) for l in path.read_text().splitlines() if l.strip()] if path.exists() else []

def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--source-run',type=Path);p.add_argument('--font',type=Path)
    p.add_argument('--test-pattern',action='store_true',help='label synthetic input self-tests, not flight footage')
    p.add_argument('--max-seconds',type=float,default=900.)
    p.add_argument('--flight-limit',type=float,required=True);p.add_argument('--corridor-limit',type=float,required=True);p.add_argument('--wall-height',type=float,required=True)
    p.add_argument('--corridor-bounds',type=float,nargs=4,default=[-4.8,4.8,7.6,9.1],metavar=('XMIN','XMAX','YMIN','YMAX'))
    a=p.parse_args()
    if a.output.exists():raise ValueError('output exists; choose a new artifact path')
    if not all(math.isfinite(v) and v>0 for v in (a.flight_limit,a.corridor_limit,a.wall_height)):raise ValueError('invalid display limits')
    if not all(math.isfinite(v) for v in a.corridor_bounds) or a.corridor_bounds[0]>=a.corridor_bounds[1] or a.corridor_bounds[2]>=a.corridor_bounds[3]:raise ValueError('invalid corridor display region')
    replay=json.loads((a.run/'replay_metadata.json').read_text()) if (a.run/'replay_metadata.json').exists() else None
    source=a.source_run or (Path(replay['source_run']) if replay else a.run)
    with (source/'truth_pose.csv').open() as f:poses=np.array([[float(r[k]) for k in ('t','x','y','z')] for r in csv.DictReader(f)])
    params=yaml.safe_load((source/'rosparams.yaml').read_text());offset=params['competition_key_recorder']['truth_world_offset'][2]
    events=jsonl(source/'key_events.jsonl');mission=[(e['ros_sec'],e['data']) for e in events if e['kind']=='mission']
    high=[(e['t'],e['status']) for e in jsonl(source/'high_view_full_events.jsonl')]
    decisions=[e for e in events if e['kind']=='decision']
    start=min((e['data']['header']['stamp']['stamp_ns']/1e9 for e in decisions),default=poses[0,0])
    overview,follow=Stream(a.run/'overview.mp4'),Stream(a.run/'follow.mp4')
    first=max(overview.times[0],follow.times[0]);last=min(overview.times[-1],follow.times[-1])
    if last<=first:raise ValueError('cameras have no common recording interval')
    if not 0<a.max_seconds<=3600 or last-first>a.max_seconds:raise ValueError('recording exceeds composition duration budget')
    font=a.font
    if font is None:font=next(iter(Path('/usr/share/fonts').rglob('NotoSansCJK-Regular.ttc')),None)
    if font is None:raise ValueError('supply --font with a CJK font file')
    large=ImageFont.truetype(str(font),40);small=ImageFont.truetype(str(font),28)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    command=['ffmpeg','-hide_banner','-loglevel','error','-f','rawvideo','-pix_fmt','bgr24','-s','1920x1080','-r','10','-i','-',
             '-an','-c:v','libx264','-threads','2','-preset','veryfast','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(a.output)]
    process=subprocess.Popen(command,stdin=subprocess.PIPE);count=0;max_age=[0.,0.];mi=hi=0;state={};high_state={};limit_counts={};over_limit=0
    try:
        for t in np.arange(first,last+1e-8,.1):
            source_t=t if replay is None else replay['source_start']+t-replay['replay_start_ros']
            while mi<len(mission) and mission[mi][0]<=source_t:state=mission[mi][1];mi+=1
            while hi<len(high) and high[hi][0]<=source_t:high_state=high[hi][1];hi+=1
            phase=high_state.get('stage',state.get('phase','STARTUP'))
            if phase=='SURVEY' and not high_state.get('ascent_verified',True):phase='ASCEND'
            if phase=='TAIL':phase=state.get('phase','POST_DELIVERY_ROUTE')
            z=float(np.interp(source_t,poses[:,0],poses[:,3])+offset)
            x,y=(float(np.interp(source_t,poses[:,0],poses[:,i])) for i in (1,2))
            x0,x1,y0,y1=a.corridor_bounds
            in_corridor=x0<=x<=x1 and y0<=y<=y1
            limit=a.corridor_limit if in_corridor else a.flight_limit
            limit_counts[str(limit)]=limit_counts.get(str(limit),0)+1;over_limit+=int(z>limit)
            over,age0=overview.at(t);chase,age1=follow.at(t);max_age=[max(max_age[0],age0),max(max_age[1],age1)]
            canvas=np.full((1080,1920,3),(28,23,20),np.uint8)
            canvas[120:840,:1280]=cv2.resize(chase,(1280,720))
            canvas[120:760,1280:]=cv2.resize(over,(640,640))
            img=Image.fromarray(cv2.cvtColor(canvas,cv2.COLOR_BGR2RGB));draw=ImageDraw.Draw(img)
            label='合成素材自检（不是飞行视频）' if a.test_pattern else ('记录轨迹回放' if replay else '仿真实录')
            draw.text((35,25),'全场自主任务  |  '+label,font=large,fill='white')
            phase_caption=PHASES.get(phase,phase)
            if phase=='POST_DELIVERY_ROUTE' and not in_corridor:phase_caption='投后转场 / 进廊准备'
            draw.text((35,870),'阶段：'+phase_caption,font=large,fill='#83d1ff')
            draw.text((35,940),f'任务时间 {max(0,source_t-start):06.1f} s     投递确认 {state.get("committed_slots",0)} / 3',font=large,fill='white')
            draw.text((1310,790),f'飞控中心离地（真值） {z:.2f} m',font=small,fill='#ff7373' if z>limit else 'white')
            draw.text((1310,842),f'本区测试限高 {limit:.2f} m',font=small,fill='white')
            draw.text((1310,894),f'边界墙参考高度 {a.wall_height:.1f} m',font=small,fill='#bdc7cf')
            draw.text((1310,946),'右上：正上方 / 固定+Y向上',font=small,fill='#bdc7cf')
            draw.text((35,1020),f'曾形成高位线索 {len(high_state.get("first_hint_ready",{}))} / 3  |  观察相机不参与导航控制',font=small,fill='#bdc7cf')
            process.stdin.write(cv2.cvtColor(np.array(img),cv2.COLOR_RGB2BGR).tobytes());count+=1
        process.stdin.close()
        if process.wait()!=0:raise RuntimeError('ffmpeg composition failed')
    finally:
        overview.cap.release();follow.cap.release()
        if process.poll() is None:process.terminate();process.wait()
    summary=dict(frames=count,fps=10,common_ros_start=float(first),common_ros_end=float(last),max_image_age_s=max_age,
                 source_run=str(source),replay=bool(replay),synthetic_test=a.test_pattern,display_limit_counts=limit_counts,display_over_limit_frames=over_limit,display_limits=dict(flight=a.flight_limit,corridor=a.corridor_limit,wall=a.wall_height,corridor_bounds=a.corridor_bounds),
                 note='Source-clock paired views; missing camera frames held. Display limit labels are user inputs, not new rule claims.')
    a.output.with_suffix('.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))

if __name__=='__main__':main()

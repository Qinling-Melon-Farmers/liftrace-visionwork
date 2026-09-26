"""ROS1 bag -> recorded-evidence videos/reports. Never publishes or reruns inference."""
import argparse
import bisect
import csv
import json
import math
import subprocess
from pathlib import Path

TOPICS=dict(camera='/camera/image_raw/compressed',odom='/mavros/local_position/odom',pose='/mavros/local_position/pose',setpoint='/mavros/setpoint_position/local',goal='/fastplanner/goal',trajectory='/planning_vis/trajectory',cloud='/sdf_map/occupancy_inflate',raw='/uav_vision/detections',resolved='/uav_vision/detections_resolved',refined='/uav_vision/detections_refined',mapped='/uav_vision/detections_mapped',targets='/uav_vision/targets',selected='/uav_vision/selected_target',mission='/navigation/mission_status',mode='/uav_vision/align_mode',fc='/mavros/state',release='/mission/release_result',result='/navigation/mission_result',command='/navigation/mission_command_raw',evidence='/uav_vision/release_evidence',permission='/mission/release_permission_active',offset='/uav_vision/drop_offset',tf='/tf',tf_static='/tf_static')

def plain(m):
    if hasattr(m,'__slots__'):return {k:plain(getattr(m,k)) for k in m.__slots__}
    if isinstance(m,(tuple,list)):return [plain(v) for v in m]
    return m

def stamp(h):
    s=h.get('stamp',{});return s.get('secs',0)+s.get('nsecs',0)*1e-9

def xyz(p):return [p.get(k,0.) for k in ('x','y','z')]

def export(a):
    import rosbag
    import numpy as np
    topics=dict(TOPICS)
    if a.topics:topics.update(json.loads(Path(a.topics).read_text()))
    reverse={v:k for k,v in topics.items()};out=Path(a.out).resolve();out.mkdir(parents=True,exist_ok=True)
    frames_dir=out/'frames';frames_dir.mkdir(exist_ok=True)
    rows={k:[] for k in topics};frames=[]
    with rosbag.Bag(str(Path(a.bag).resolve())) as b:
        begin=b.get_start_time();end=b.get_end_time();available=b.get_type_and_topic_info().topics
        for topic,m,bt in b.read_messages(topics=list(topics.values())):
            key=reverse[topic];t=bt.to_sec()-begin
            if key=='camera':
                if m._type!='sensor_msgs/CompressedImage':raise ValueError('camera must be sensor_msgs/CompressedImage; remap --topics')
                ext='png' if 'png' in m.format.lower() else 'jpg'
                file=f'frames/{len(frames):06d}.{ext}';(out/file).write_bytes(bytes(m.data))
                frames.append(dict(t=t,stamp=m.header.stamp.to_sec()-begin,file=file));continue
            if key=='cloud':
                fields={f.name:f for f in m.fields}
                if not all(k in fields and fields[k].datatype==7 for k in ('x','y','z')):continue
                endian='>' if m.is_bigendian else '<'
                dtype=np.dtype(dict(names=['x','y','z'],formats=[endian+'f4']*3,offsets=[fields[k].offset for k in ('x','y','z')],itemsize=m.point_step))
                cloud=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step)).reshape(-1)
                cloud=cloud[::max(1,math.ceil(len(cloud)/2000))]
                points=np.column_stack([cloud[k] for k in ('x','y','z')]);points=points[np.isfinite(points).all(axis=1)]
                data=dict(header=plain(m.header),points=points.tolist())
            else:data=plain(m)
            rows[key].append(dict(t=t,stamp=m.header.stamp.to_sec()-begin if hasattr(m,'header') else t,m=data))
    meta=dict(bag=str(Path(a.bag).resolve()),start=begin,duration=end-begin,topics=topics,missing=[k for k,v in topics.items() if v not in available],frames=frames,rows=rows)
    (out/'data.json').write_text(json.dumps(meta,ensure_ascii=False))
    print(f'Exported {len(frames)} camera images, duration {end-begin:.2f}s. Missing: {meta["missing"]}',flush=True)

class Timeline:
    def __init__(self,rows):self.rows=rows;self.times=[r['t'] for r in rows]
    def row(self,t):
        i=bisect.bisect_right(self.times,t)-1
        return self.rows[i] if i>=0 else None
    def msg(self,t,max_age=None):
        r=self.row(t)
        return r['m'] if r and (max_age is None or t-r['t']<=max_age) else None

class Frames:
    def __init__(self,rows):
        self.rows=sorted(rows,key=lambda r:r['stamp']);self.times=[r['stamp'] for r in self.rows]
    def match(self,t):
        i=bisect.bisect_left(self.times,t);ids=[k for k in (i-1,i) if 0<=k<len(self.rows)]
        if not ids:return None
        j=min(ids,key=lambda k:abs(self.times[k]-t))
        return self.rows[j] if abs(self.times[j]-t)<=.03 else None

class TransformTree:
    def __init__(self,rows,begin):
        self.edges={};self.begin=begin
        for key in ('tf_static','tf'):
            for row in rows[key]:
                for tr in row['m']['transforms']:
                    edge=(tr['child_frame_id'].lstrip('/'),tr['header']['frame_id'].lstrip('/'))
                    self.edges.setdefault(edge,[]).append(dict(t=-1e20 if key=='tf_static' else stamp(tr['header'])-begin,m=tr['transform'],static=key=='tf_static'))
        self.edges={k:Timeline(sorted(v,key=lambda r:r['t'])) for k,v in self.edges.items()}
    def matrix(self,source,target,t):
        import numpy as np
        from scipy.spatial.transform import Rotation
        source=source.lstrip('/');target=target.lstrip('/')
        if not source:return None
        if source==target:return np.eye(4)
        adj={}
        for (child,parent),line in self.edges.items():
            row=line.row(t)
            if not row or (not row['static'] and t-row['t']>.5):continue
            tr=row['m'];q=tr['rotation'];mat=np.eye(4)
            mat[:3,:3]=Rotation.from_quat([q[k] for k in ('x','y','z','w')]).as_matrix();mat[:3,3]=xyz(tr['translation'])
            adj.setdefault(child,[]).append((parent,mat));adj.setdefault(parent,[]).append((child,np.linalg.inv(mat)))
        queue=[(source,np.eye(4))];seen={source}
        for node,mat in queue:
            for nxt,step in adj.get(node,[]):
                if nxt==target:return step@mat
                if nxt not in seen:seen.add(nxt);queue.append((nxt,step@mat))
        return None

def render(a):
    import cv2
    import numpy as np
    out=Path(a.out).resolve();d=json.loads((out/'data.json').read_text());rows=d['rows'];lines={k:Timeline(v) for k,v in rows.items()}
    cv2.setNumThreads(2);tree=TransformTree(rows,d['start']);warnings=set()
    def convert(points,frame,t):
        mat=tree.matrix(frame,a.frame,t)
        if mat is None:warnings.add(f'Cannot transform {frame!r} -> {a.frame}; omitted from map');return None
        points=np.asarray(points,dtype=float).reshape(-1,3)
        return points@mat[:3,:3].T+mat[:3,3]
    motion=[]
    for r in rows['odom'] or rows['pose']:
        m=r['m'];p=m['pose'].get('pose',m['pose']);v=convert([xyz(p['position'])],m['header']['frame_id'],r['t'])
        if v is not None:motion.append([r['t'],*v[0]])
    motion=np.array(motion) if motion else np.empty((0,4));mt=motion[:,0]
    speeds=np.zeros(len(motion))
    if len(motion)>1:
        speeds=np.linalg.norm(np.column_stack([(np.interp(mt+.25,mt,motion[:,k])-np.interp(mt-.25,mt,motion[:,k]))/.5 for k in (1,2)]),axis=1)
    sp=[]
    for r in rows['setpoint']:
        p=convert([xyz(r['m']['pose']['position'])],r['m']['header']['frame_id'],r['t'])
        if p is not None:sp.append([r['t'],*p[0]])
    sp=np.asarray(sp).reshape(-1,4)
    low=motion[:,1:3].min(axis=0)-1 if len(motion) else np.array([-2.,-2.]);high=motion[:,1:3].max(axis=0)+1 if len(motion) else np.array([2.,2.])
    for key in ('goal','setpoint'):
        for r in rows[key]:
            p=convert([xyz(r['m']['pose']['position'])],r['m']['header']['frame_id'],r['t'])
            if p is not None:low=np.minimum(low,p[0,:2]-.5);high=np.maximum(high,p[0,:2]+.5)
    groups={}
    for key in ('raw','resolved','refined','mapped'):
        for r in rows[key]:groups.setdefault((key,r['m']['source']),[]).append(r)
    groups={k:Frames(v) for k,v in groups.items()}
    frame_line=Timeline(d['frames']);history={};target_events=[];last_target_t=-1
    def txt(img,text,x,y,color=(235,235,235),scale=.53):cv2.putText(img,str(text),(x,y),cv2.FONT_HERSHEY_SIMPLEX,scale,color,1,cv2.LINE_AA)
    def canvas(w,h):return np.full((h,w,3),24,np.uint8)
    def map_panel(t,w,h):
        img=canvas(w,h);pad=45;scale=min((w-2*pad)/(high-low)[0],(h-2*pad)/(high-low)[1]);center=(high+low)/2
        def uv(p):return (int(w/2+(p[0]-center[0])*scale),int(h/2-(p[1]-center[1])*scale))
        def path(points,color,width=2):
            if points is not None and len(points)>1:cv2.polylines(img,[np.array([uv(p) for p in points],np.int32)],False,color,width,cv2.LINE_AA)
        for gx in range(math.ceil(low[0]),math.floor(high[0])+1):
            cv2.line(img,uv([gx,low[1]]),uv([gx,high[1]]),(45,45,45),1);txt(img,str(gx),uv([gx,low[1]])[0],h-60,scale=.35)
        for gy in range(math.ceil(low[1]),math.floor(high[1])+1):
            cv2.line(img,uv([low[0],gy]),uv([high[0],gy]),(45,45,45),1);txt(img,str(gy),12,uv([low[0],gy])[1],scale=.35)
        zi=float(np.interp(t,mt,motion[:,3])) if len(mt) else 0.
        cloud=lines['cloud'].row(t)
        if cloud and t-cloud['t']<=2:
            p=convert(cloud['m']['points'],cloud['m']['header']['frame_id'],cloud['t'])
            if p is not None:
                for point in p[abs(p[:,2]-zi)<.3]:cv2.circle(img,uv(point),1,(90,90,90),-1)
        tr=lines['trajectory'].msg(t)
        if tr and tr.get('points') and tr.get('action',0)==0:
            from scipy.spatial.transform import Rotation
            pp=np.array([xyz(p) for p in tr['points']]);pose=tr['pose'];q=pose['orientation'];qq=[q[k] for k in ('x','y','z','w')]
            if np.linalg.norm(qq)>0:pp=Rotation.from_quat(qq).apply(pp)
            pp+=xyz(pose['position']);pp=convert(pp,tr['header']['frame_id'],t)
            if tr.get('type')==5 and pp is not None:
                for j in range(0,len(pp)-1,2):path(pp[j:j+2],(30,180,255))
            else:path(pp,(30,180,255))
        past=motion[mt<=t];path(past[:,1:] if len(past) else None,(255,220,40),3)
        path(sp[sp[:,0]<=t,1:],(210,80,190),1)
        for key,color,mark in [('setpoint',(255,50,220),cv2.MARKER_CROSS),('goal',(40,220,255),cv2.MARKER_DIAMOND)]:
            r=lines[key].row(t)
            if r:
                p=convert([xyz(r['m']['pose']['position'])],r['m']['header']['frame_id'],r['t'])
                if p is not None:cv2.drawMarker(img,uv(p[0]),color,mark,18,2)
        for label_index,(ident,r) in enumerate(history.items()):
            q=r['m'];p=convert([xyz(q['map_point'])],q['map_frame'],r['t'])
            if p is not None:cv2.circle(img,uv(p[0]),7,(90,210,90),1);txt(img,f"{ident} last",uv(p[0])[0]+9,uv(p[0])[1]+18*label_index,(90,210,90),.42)
        if len(past):cv2.circle(img,uv(past[-1,1:]),6,(255,255,255),-1)
        txt(img,f'TOP VIEW [{a.frame}]   X right / Y up (m)',12,24)
        txt(img,'cyan=actual orange=last plan pink=setpoint yellow=goal',12,h-38,scale=.39)
        txt(img,'gray=inflated cloud slice +/-0.3m; green=last valid target',12,h-16,scale=.39)
        return img
    def curves(t,w,h):
        img=canvas(w,h)
        for j,(values,label) in enumerate([(motion[:,3] if len(mt) else [],'FC local height (m)'),(speeds,'XY speed (m/s), pose difference')]):
            y0=j*h//2;bottom=y0+h//2-25;top=y0+28;left=50;right=w-15
            txt(img,label,12,y0+20,scale=.43)
            if len(mt):
                vmax=max(float(np.max(values)),.1);vmin=min(float(np.min(values)),0);span=max(vmax-vmin,.1)
                pts=np.column_stack((left+mt/d['duration']*(right-left),bottom-(np.asarray(values)-vmin)/span*(bottom-top))).astype(np.int32)
                cv2.polylines(img,[pts],False,(120,120,120),1);cv2.polylines(img,[pts[mt<=t]],False,(255,220,40),2)
                x=int(left+t/d['duration']*(right-left));cv2.line(img,(x,top),(x,bottom),(230,230,230),1)
                txt(img,f'{np.interp(t,mt,values):.2f}',w-65,y0+20,(255,220,40),.46)
        return img
    fps=a.fps;count=math.ceil(d['duration']*fps);writers={}
    for name,size in [('camera_raw',(1280,720)),('camera_annotated',(1280,1080)),('trajectory',(1280,1080)),('dashboard',(1920,1080))]:
        if name=='camera_raw' and not d['frames']:continue
        writers[name]=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','bgr24','-s',f'{size[0]}x{size[1]}','-r',str(fps),'-i','-','-an','-c:v','libx264','-threads','2','-preset','veryfast','-crf','23','-pix_fmt','yuv420p','-movflags','+faststart',str(out/(name+'.mp4'))],stdin=subprocess.PIPE)
    last_file=None;raw_image=None
    with (out/'video_frames.csv').open('w') as f:
        csvout=csv.writer(f);csvout.writerow(['frame','bag_seconds','image_seconds','matched_results'])
        try:
            for i in range(count):
                t=i/fps;fr=frame_line.row(t)
                if fr:
                    if last_file!=fr['file']:raw_image=cv2.imread(str(out/fr['file']));last_file=fr['file']
                    if raw_image is None:raise ValueError('Cannot decode '+fr['file'])
                    raw=cv2.resize(raw_image,(1280,720));sx=1280/raw_image.shape[1];sy=720/raw_image.shape[0]
                else:raw=canvas(1280,720);txt(raw,'Camera not recorded / not yet available',100,300);sx=sy=1
                annotated=raw.copy();matched=[]
                if fr:
                    for key,line in groups.items():
                        r=line.match(fr['stamp'])
                        if r:matched.append((key,r))
                for (key,source),r in matched:
                    if key not in ('raw','mapped'):continue
                    for q in r['m']['detections']:
                        box=q['roi'];p=q['center_px'];x=int(box['x_offset']*sx);y=int(box['y_offset']*sy);cx=int(p['x']*sx);cy=int(p['y']*sy)
                        color=(0,220,0) if source=='target_detector' else (255,220,0)
                        label=f"{q['class_name']} {q['class_confidence']:.2f}"
                        if key=='mapped':
                            color=(255,0,255) if q['map_valid'] else (0,255,255)
                            label=f"{q['class_name']} "+('MAP VALID' if q['map_valid'] else q['reject_reason'])
                            cv2.drawMarker(annotated,(cx,cy),color,cv2.MARKER_CROSS,16,2)
                        else:
                            cv2.rectangle(annotated,(x,y),(int((box['x_offset']+box['width'])*sx),int((box['y_offset']+box['height'])*sy)),color,2)
                            if p['z']>0:cv2.circle(annotated,(cx,cy),int(p['z']*sx),color,2)
                        txt(annotated,label,max(0,min(x,900)),max(20,y-5)+(18 if key=='mapped' else 0),color,.48)
                target_row=lines['targets'].row(t)
                if target_row and target_row['t']!=last_target_t:
                    last_target_t=target_row['t']
                    for q in target_row['m']['targets']:
                        if q['map_valid']:
                            ident=f"{q['class_name']}#{q['id']}";history[ident]=dict(t=target_row['t'],m=q)
                mission=lines['mission'].msg(t,1) or {};mission=json.loads(mission.get('data','{}'))
                mode=(lines['mode'].msg(t,1) or {}).get('data','?');fc=lines['fc'].msg(t,2) or {};release=lines['release'].msg(t) or {};ev=lines['evidence'].msg(t,1) or {};permit=lines['permission'].msg(t,1) or {}
                coord=[]
                live={f"{q['class_name']}#{q['id']}" for q in (target_row['m']['targets'] if target_row and t-target_row['t']<=.5 else []) if q['map_valid']}
                for ident,r in list(history.items())[:3]:
                    q=r['m'];p=q['map_point'];coord.append(f"{ident}: ({p['x']:.3f},{p['y']:.3f},{p['z']:.3f}) [{q['map_frame']}] {'CURRENT' if ident in live else 'HISTORY'} age={t-r['t']:.1f}s")
                mapped=[q for (key,_),r in matched if key=='mapped' for q in r['m']['detections'] if q['class_name']!='circle']
                map_text='; '.join(q['class_name']+': '+('valid' if q['map_valid'] else q['reject_reason']) for q in mapped) or 'no matched map result'
                panel=canvas(1280,360)
                texts=[f"RECORDED BAG | 1x t={t:.2f}/{d['duration']:.2f}s | image={fr['stamp'] if fr else -1:.2f}s | {fc.get('mode','?')}",f"Mission={mission.get('phase','?')} / {mission.get('active_command','')} | vision={mode} | committed={mission.get('committed_slots','?')}",f"Map: {map_text}",f"aligned={ev.get('aligned','?')} evidence={ev.get('evidence_valid','?')} permit={permit.get('data','?')} | release={release.get('reason','none')}",'LAST VALID RECORDED TARGET POSITIONS (historical; not current release permission):',*coord]
                while len(texts)<8:texts.append('')
                texts+=['Circle IDs are auxiliary geometry; map-valid / selected does not mean task accepted.','Pixel overlay: offline image-stamp match <=30ms; states use receipt time. No inference rerun.']
                for n,line in enumerate(texts[:10]):txt(panel,line[:155],12,25+n*34,scale=.52)
                ann=np.vstack((annotated,panel));mp=map_panel(t,640,720);cv=curves(t,640,360)
                dashboard=np.hstack((ann,np.vstack((mp,cv))));trajectory=np.vstack((map_panel(t,1280,720),curves(t,1280,360)))
                for name,img in [('camera_raw',raw),('camera_annotated',ann),('trajectory',trajectory),('dashboard',dashboard)]:
                    if name in writers:writers[name].stdin.write(img.tobytes())
                csvout.writerow([i,t,fr['stamp'] if fr else '',len(matched)])
                if i in (int(22.7*fps),int(50.3*fps)):cv2.imwrite(str(out/f'preview_{i}.jpg'),dashboard)
                if i%300==0:print(f'Render {i}/{count}',flush=True)
        finally:
            codes=[]
            for p in writers.values():p.stdin.close();codes.append(p.wait())
            if any(codes):raise RuntimeError('Video encoding failed')
    classes={}
    for key in ('raw','mapped','targets'):
        for r in rows[key]:
            for q in r['m'].get('detections',r['m'].get('targets',[])):
                k=key+':'+q['class_name'];v=classes.setdefault(k,dict(count=0,first=r['t'],last=r['t'],valid=0));v['count']+=1;v['last']=r['t'];v['valid']+=bool(q['map_valid'])
    events=[]
    for key in ('command','result','release'):
        for r in rows[key]:events.append(dict(t=r['t'],topic=key,reason=r['m'].get('reason',''),target=r['m'].get('target_class','')))
    events.sort(key=lambda r:r['t'])
    with (out/'events.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=['t','topic','reason','target']);w.writeheader();w.writerows(events)
    summary=dict(bag=d['bag'],duration=d['duration'],fps=fps,frames=count,display_frame=a.frame,missing=d['missing'],warnings=sorted(warnings),classes=classes,video_files=[n+'.mp4' for n in writers])
    (out/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))
    np.savetxt(out/'motion.csv',np.column_stack((motion,speeds)),delimiter=',',header='bag_seconds,x,y,z,xy_speed',comments='')
    with (out/'target_coordinates.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['bag_seconds','class','id','frame','map_valid','x','y','z','state'])
        for r in rows['targets']:
            for q in r['m']['targets']:w.writerow([r['t'],q['class_name'],q['id'],q['map_frame'],q['map_valid'],*xyz(q['map_point']),q['state']])
    cv2.imwrite(str(out/'flight_summary.png'),np.vstack((map_panel(d['duration'],1280,720),curves(d['duration'],1280,360))))
    report=['# Bag离线回放报告','',f"源文件：`{d['bag']}`；记录{d['duration']:.2f}秒；输出{fps}fps、1倍速。",'', '视频为记录数据可视化，不是重新飞行或新算法验收。目标坐标标记为最后有效历史值；不是实时投递许可。', '', '## 话题完整性',str(d['missing']), '', '## 坐标系限制',*sorted(warnings),'', '## 检测/候选统计','|链路与类别|记录数|首次秒|末次秒|map_valid数|','|---|---:|---:|---:|---:|']
    report.extend(f"|{k}|{v['count']}|{v['first']:.3f}|{v['last']:.3f}|{v['valid']}|" for k,v in sorted(classes.items()))
    report+=['','时序见events.csv，完整指标见summary.json。灰色地图仅显示当前位置高度±0.3m的稀疏膨胀点云；规划曲线是最近记录Marker，缺失时不猜测。高度为位姿坐标系Z，非独立离地测距。','']
    (out/'REPORT.md').write_text('\n'.join(report))
    html='<!doctype html><meta charset="utf-8"><title>Bag replay</title><style>body{background:#16191e;color:white;max-width:1500px;margin:20px auto;font:18px sans-serif}video{width:100%}a{color:#8cf}button{margin:5px;padding:8px}</style><h1>Bag 多画面离线回放</h1><p>1倍速。仅展示记录证据；目标坐标为最后有效历史位置。</p><video id="v" controls src="dashboard.mp4"></video>'
    for name in writers:html+=f'<button onclick="v.src=\'{name}.mp4\';v.play()">{name}</button>'
    html+='<p><a href="REPORT.md">报告</a> · <a href="events.csv">事件时间线</a> · <a href="summary.json">指标/缺失话题</a></p>'
    (out/'index.html').write_text(html)

def verify(a):
    out=Path(a.out).resolve();s=json.loads((out/'summary.json').read_text());results={}
    for name in s['video_files']:
        p=out/name
        subprocess.run(['ffmpeg','-v','error','-i',str(p),'-f','null','-'],check=True)
        info=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','format=duration,size','-of','json',str(p)]))['format']
        if abs(float(info['duration'])-s['duration'])>1/s['fps']+.03:raise ValueError('Video duration mismatch')
        results[name]=info
    (out/'validation.json').write_text(json.dumps(results,indent=2))
    if not a.keep_frames:
        data=json.loads((out/'data.json').read_text());folder=(out/'frames').resolve()
        if folder!=out/'frames':raise ValueError('Refusing cleanup of redirected frames directory')
        for frame in data['frames']:
            p=(out/frame['file']).resolve()
            if p.parent!=folder:raise ValueError('Unexpected frame path')
            if p.is_file():p.unlink()
        if folder.exists() and not any(folder.iterdir()):folder.rmdir()
    print('All videos decoded; duration checks passed.',flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('mode',choices=['export','render','verify']);p.add_argument('--bag');p.add_argument('--out',required=True);p.add_argument('--fps',type=int,default=10);p.add_argument('--frame',default='map');p.add_argument('--topics')
    p.add_argument('--keep-frames',action='store_true');a=p.parse_args()
    if not 1<=a.fps<=30:p.error('fps must be 1..30')
    if a.mode=='export' and not a.bag:p.error('export requires --bag')
    {'export':export,'render':render,'verify':verify}[a.mode](a)

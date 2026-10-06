#!/usr/bin/env python3
"""Offline annotation adapter for the existing source-clock review compositor.

Original recordings and compositor remain unchanged. Run only after SITL stops.
The center cross is a recorded image observation, never parcel impact or truth.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
BASE = ROOT / 'docs/verification/failed_six_20260921/compose_review.py'
code = BASE.read_text()


def replace_once(old, new):
    global code
    if code.count(old) != 1:
        raise RuntimeError('Existing compositor changed: ' + old[:70])
    code = code.replace(old, new, 1)


replace_once("p.add_argument('--case-label',default='')", """p.add_argument('--case-label',default='')
    p.add_argument('--precision-json',type=Path,required=True)
    p.add_argument('--exact-offset-data',type=Path,required=True)
    p.add_argument('--keys-only',action='store_true')""")
replace_once("overview,follow=Stream(a.run/'overview.mp4'),Stream(a.run/'follow.mp4')", """precision=json.loads(a.precision_json.read_text())
    drops=precision['drops']; target_truth={v['class_name']:v['csv_xy'] for v in precision['truth']}
    if precision.get('diagnostic_status'):
        gate={'status':'DIAGNOSTIC FAIL','reason':'第三槽阻断；两槽完成'}
    camera=json.loads((source/'actual_camera_info.json').read_text())
    principal=(camera['K'][2],camera['K'][5])
    offset_events=[e for e in jsonl(a.exact_offset_data) if e.get('kind')=='drop_offset']
    offset_events.sort(key=lambda e:e['m']['header']['stamp']['secs']+e['m']['header']['stamp']['nsecs']/1e9)
    offset_times=np.array([e['m']['header']['stamp']['secs']+e['m']['header']['stamp']['nsecs']/1e9 for e in offset_events])
    overview,follow=Stream(a.run/'overview.mp4'),Stream(a.run/'follow.mp4')
    downward=Stream(a.run/'downward.mp4')""")
replace_once("first=max(overview.times[0],follow.times[0]);last=min(overview.times[-1],follow.times[-1])", "first=max(overview.times[0],follow.times[0],downward.times[0]);last=min(overview.times[-1],follow.times[-1],downward.times[-1])")
replace_once("for t in np.arange(first,last+1e-8,.1):", """ranges=[(float(first),float(last))]
        if a.keys_only:
            ranges=[(max(first,d['completed']['source_ros_s']-10),min(last,d['completed']['source_ros_s']+5)) for d in drops]
            # Include the third approach, freeze loss and the final blocked state.
            ranges.extend([(max(first,110.),min(last,127.)),(max(first,last-10),last)])
            merged=[]
            for segment_lo,segment_hi in sorted(ranges):
                if segment_hi<=segment_lo:continue
                if merged and segment_lo<=merged[-1][1]:merged[-1]=(merged[-1][0],max(segment_hi,merged[-1][1]))
                else:merged.append((segment_lo,segment_hi))
            ranges=merged
        review_times=np.concatenate([np.arange(lo,hi+1e-8,.1) for lo,hi in ranges])
        if not a.keys_only:
            # Include the exact final source timestamp and hold its recorded
            # frame for two seconds so final COMPLETE can be read.
            review_times=np.concatenate([review_times,np.full(20,last)])
        for t in review_times:""")
replace_once("canvas[120:760,1280:]=cv2.resize(over,(640,640))", """canvas[120:480,1280:]=cv2.resize(over,(640,360))
            down,down_age=downward.at(t); down=down.copy()
            image_t=float(downward.times[downward.index]); observed=None
            oi=int(np.searchsorted(offset_times,image_t,side='right')-1)
            if oi>=0 and image_t-offset_times[oi]<=.12:
                observed=offset_events[oi]['m']
                point=(int(round(principal[0]+observed['dx_px'])),int(round(principal[1]+observed['dy_px'])))
                cv2.drawMarker(down,point,(0,255,255),cv2.MARKER_CROSS,35,3)
            canvas[480:840,1280:]=cv2.resize(down,(640,360))""")
replace_once("draw.text((1310,790)", "draw.text((1310,870)")
replace_once("draw.text((1310,842)", "draw.text((1310,915)")
replace_once("draw.text((1310,894)", "draw.text((1310,960)")
replace_once("draw.text((1310,946),'右上：正上方 / 固定+Y向上',font=small,fill='#bdc7cf')", """draw.text((1310,1005),'右：俯视 / 下视；黄色十字为观测中心',font=small,fill='#bdc7cf')
            draw.text((1300,490),f'下视曝光 ROS {image_t:.3f} s',font=small,fill='white')
            if observed is None:
                draw.text((1300,530),'无同帧精修中心；不补画旧中心',font=small,fill='white')""")
replace_once("draw.text((35,1020),f'曾形成高位线索 {len(high_state.get(\"first_hint_ready\",{}))} / 3  |  观察相机不参与导航控制',font=small,fill='#bdc7cf')", """target=decision.get('target_class',''); own=target_truth.get(target)
            current_error=math.hypot(x-own[0],y-own[1])*100 if own else None
            ack=next((d for d in drops if d['class_name']==target),None)
            slot=ack['slot'] if ack else (3 if target=='panzer' and precision.get('diagnostic_status') else None)
            center_caption=f'槽位 {slot if slot else "-"} / {target or "航段"}'
            if own:
                center_caption+=f'  靶心 ({own[0]:.4f}, {own[1]:.4f})  当前机体误差 {current_error:.2f} cm'
            if ack and source_t>=ack['completed']['source_ros_s']:
                center_caption+=f'  ACK同历史口径 {ack["legacy_ack_receipt_nearest"]["error_m"]*100:.3f} cm'
            if target=='panzer' and precision.get('diagnostic_status') and source_t>=121.301:
                center_caption+='  未释放：许可拒绝 / 机体漂移'
            draw.text((35,1020),center_caption,font=small,fill='#bdc7cf')
            draw.text((35,1060),'机体中心 / mock ACK，不是实物包裹落点；剪辑跳时以源ROS时间为准',font=ImageFont.truetype(str(font),20),fill='#bdc7cf')""")
replace_once("overview.cap.release();follow.cap.release()", "overview.cap.release();follow.cap.release();downward.cap.release()")
replace_once("source_run=str(source),replay=bool(replay)", "source_run=str(source),source_segments_ros_s=ranges,final_recorded_frame_hold_s=0 if a.keys_only else 2,precision_status=precision.get('diagnostic_status',precision.get('raw_gate')),center_annotation='Recorded image offsets within 120ms of image exposure only; no truth pixel projection or parcel impact',replay=bool(replay)")
exec(compile(code, str(BASE), 'exec'), {'__name__': '__main__', '__file__': str(BASE)})

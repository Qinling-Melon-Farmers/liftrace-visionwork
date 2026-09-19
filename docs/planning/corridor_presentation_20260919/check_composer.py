"""Tiny synthetic-input video check; never launches any simulator."""
from pathlib import Path
import csv,json,subprocess,sys
import cv2,numpy as np,yaml

D=Path(__file__).resolve().parent;R=D.parents[2]
def main():
    root=R/'logs/presentation_composer_selftest_20260919'
    root.mkdir(exist_ok=True)
    for kind,color in [('overview',(70,110,70)),('follow',(120,75,35))]:
        width,height=(400,400) if kind=='overview' else (640,360)
        out=cv2.VideoWriter(str(root/(kind+'.mp4')),cv2.VideoWriter_fourcc(*'mp4v'),10,(width,height));assert out.isOpened()
        times=np.arange(0,1.21,.1)
        if kind=='follow':times=np.delete(times,5)  # explicit image gap, source timestamps remain authoritative
        with (root/(kind+'.csv')).open('w') as f:
            writer=csv.writer(f);writer.writerow(['frame','image_stamp_ros_sec','receipt_ros_sec'])
            for i,t in enumerate(times):
                frame=np.full((height,width,3),color,np.uint8);cv2.circle(frame,(int(width*.25+t*60),height//2),16,(255,255,255),-1)
                cv2.putText(frame,'SYNTHETIC '+kind,(15,30),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
                out.write(frame);writer.writerow([i,float(t),float(t)])
        out.release()
    with (root/'truth_pose.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['t','x','y','z'])
        for t in np.arange(0,1.21,.1):writer.writerow([t,0,6.7 if t<.6 else 8.0,1.18 if t<.6 else .68])
    (root/'rosparams.yaml').write_text(yaml.safe_dump({'competition_key_recorder':{'truth_world_offset':[0,0,.22]}}))
    ev=[dict(kind='decision',ros_sec=0.,data={'header':{'stamp':{'stamp_ns':0}}}),dict(kind='mission',ros_sec=0.,data={'phase':'POST_DELIVERY_ROUTE','committed_slots':3})]
    (root/'key_events.jsonl').write_text('\n'.join(json.dumps(e) for e in ev)+'\n')
    (root/'high_view_full_events.jsonl').write_text(json.dumps({'t':0.,'status':{'stage':'TAIL','first_hint_ready':{'bridge':{},'panzer':{},'red_cross':{}}}})+'\n')
    target=root/'synthetic_presentation.mp4'
    if target.exists():target.unlink()  # replace only this regenerable test artifact
    subprocess.run([sys.executable,str(R/'vision_ws/src/uav_high_view/scripts/presentation_compose.py'),str(root),'--output',str(target),
                    '--flight-limit','3.0','--corridor-limit','1.2','--wall-height','4.0','--test-pattern'],check=True)
    result=json.loads(target.with_suffix('.json').read_text());assert result['frames']==13 and result['synthetic_test']
    assert .09<result['max_image_age_s'][1]<.11  # missing 0.5s frame holds 0.4s image once
    assert result['display_limit_counts']=={'3.0':6,'1.2':7} and result['display_over_limit_frames']==0
    cap=cv2.VideoCapture(str(target));cap.set(cv2.CAP_PROP_POS_FRAMES,8);ok,image=cap.read();cap.release();assert ok and image.shape==(1080,1920,3)
    cv2.imwrite(str(D/'presentation_layout_selftest.jpg'),image)
    (D/'composer_check.json').write_text(json.dumps(dict(status='PASS',scope='SYNTHETIC_INPUT_NOT_FLIGHT',**result),indent=2))
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-i',str(target),'-f','null','-'],check=True)
    print('Composer synthetic decode/timing PASS')

if __name__=='__main__':main()

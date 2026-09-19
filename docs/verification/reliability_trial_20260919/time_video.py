"""Restore source-clock timing by holding frames across camera drops (no new flight)."""
import csv,json,subprocess,sys
from pathlib import Path
import cv2,numpy as np

def main():
    replay=Path(sys.argv[1]);meta=json.loads((replay/'replay_metadata.json').read_text())
    times=np.array([float(row['image_stamp_ros_sec']) for row in csv.DictReader((replay/'overview_replay.csv').open())])
    cap=cv2.VideoCapture(str(replay/'overview_replay.mp4'))
    width,height=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    target=replay/'overview_replay_timed.mp4';fps=10
    source_span=meta['source_end']-meta['source_start']
    count=int(np.ceil(source_span*fps))+1
    command=['ffmpeg','-hide_banner','-loglevel','error','-f','rawvideo','-pix_fmt','bgr24',
             '-s',f'{width}x{height}','-r',str(fps),'-i','-','-an','-c:v','libx264','-threads','2',
             '-preset','veryfast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(target)]
    process=subprocess.Popen(command,stdin=subprocess.PIPE)
    current=-1;frame=None;used=set()
    for i in range(count):
        stamp=meta['replay_start_ros']+min(i/fps,source_span)
        desired=max(0,min(len(times)-1,int(np.searchsorted(times,stamp,side='right')-1)))
        while current<desired:
            ok,frame=cap.read()
            if not ok:raise RuntimeError('raw video ended before its timestamp CSV')
            current+=1
        used.add(desired);process.stdin.write(frame.tobytes())
    process.stdin.close();assert process.wait()==0;cap.release()
    info=dict(file=str(target),frames=count,fps=fps,duration_s=count/fps,source_span_s=source_span,
              raw_frames_used=len(used),held_duplicate_frames=count-len(used),
              max_original_frame_gap_s=float(np.diff(times).max()),
              method='10fps source-clock grid; hold previous captured frame through gaps, no generated motion',
              source_start_ros=meta['source_start'],source_end_ros=meta['source_end'])
    (replay/'timed_video.json').write_text(json.dumps(info,indent=2));print(json.dumps(info,indent=2))

if __name__=='__main__':main()

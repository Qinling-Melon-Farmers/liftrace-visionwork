"""Render the offline appearance comparison separately from recorded-flight videos."""
import argparse,json,bisect,subprocess
from pathlib import Path
import cv2,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from appearance_probe import variants

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--report',type=Path,required=True);a=p.parse_args()
    cv2.setNumThreads(2);d=json.loads((a.run/'replay/data.json').read_text());probe=json.loads((a.run/'appearance_probe.json').read_text());frames=sorted(d['frames'],key=lambda r:r['stamp']);ft=np.array([r['stamp'] for r in frames]);samples=sorted(set(r['t'] for r in probe['rows']));modes=['raw','gamma065','clahe','mean3_registered'];lookup={(r['t'],r['mode']):r for r in probe['rows']}
    def generate(t):
        j=int(np.argmin(abs(ft-t)));im=cv2.imread(str(a.run/'replay'/frames[j]['file']));past=[cv2.imread(str(a.run/'replay'/frames[k]['file'])) for k in (j-1,j-2) if k>=0 and 0<t-ft[k]<=.12];modified,_=variants(im,past);tiles=[]
        for mode in modes:
            img=modified[mode].copy();boxes=lookup[(t,mode)]['detections']
            for q in boxes:
                if q['cls']=='panzer' and q['conf']>=.05:
                    x1,y1,x2,y2=map(int,q['box']);color=(0,220,0) if q['conf']>=.5 else (0,200,255);cv2.rectangle(img,(x1,y1),(x2,y2),color,2);cv2.putText(img,f'panzer {q["conf"]:.3f}',(x1,max(25,y1-10)),0,.7,color,2)
            img=cv2.resize(img,(640,360));band=np.full((50,640,3),22,np.uint8);cv2.putText(band,f'{mode} | source t={t:.3f}s',(12,21),0,.55,(235,235,235),1);cv2.putText(band,'OFFLINE PT DIAGNOSIS; green >=0.50; not flight output',(12,42),0,.40,(235,235,235),1);tiles.append(np.vstack([img,band]))
        return np.vstack([np.hstack(tiles[:2]),np.hstack(tiles[2:])])
    selected=min(samples,key=lambda x:abs(x-122.63));cv2.imwrite(str(a.report/'appearance_comparison.jpg'),generate(selected))
    proc=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','bgr24','-s','1280x820','-r','10','-i','-','-an','-c:v','libx264','-threads','2','-preset','veryfast','-crf','23','-pix_fmt','yuv420p','-movflags','+faststart',str(a.run/'appearance_comparison.mp4')],stdin=subprocess.PIPE)
    cache={}
    for now in np.arange(118,129,.1):
        index=bisect.bisect_right(samples,now)-1
        if index<0:im=np.zeros((820,1280,3),np.uint8);cv2.putText(im,'Waiting for recorded detector source image',(100,200),0,1.,(255,255,255),2)
        else:
            t=samples[index]
            if t not in cache:cache={t:generate(t)}
            im=cache[t]
        proc.stdin.write(im.tobytes())
    proc.stdin.close();assert proc.wait()==0
    fig,axs=plt.subplots(2,1,figsize=(11,7),sharex=True)
    for mode in probe['summary']:
        confidence=[max((q['conf'] for q in lookup.get((t,mode),{}).get('detections',[]) if q['cls']=='panzer'),default=0.) for t in samples]
        axs[0 if mode in ('raw','gamma065','clahe') else 1].plot(samples,confidence,label=mode,lw=1.2)
    for ax in axs:ax.axhline(.5,color='gray',ls='--');ax.axhline(.7,color='gray',ls=':');ax.legend();ax.grid(alpha=.2);ax.set_ylabel('panzer confidence')
    axs[1].set_xlabel('Source image time / bag seconds');fig.suptitle('92 exact recorded YOLO samples: PT appearance diagnosis, not recall');fig.tight_layout();fig.savefig(a.report/'appearance_confidence.png',dpi=160);plt.close(fig)
    subprocess.run(['ffmpeg','-v','error','-i',str(a.run/'appearance_comparison.mp4'),'-f','null','-'],check=True)
    info=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','format=duration','-of','json',str(a.run/'appearance_comparison.mp4')]))
    assert abs(float(info['format']['duration'])-11.)<.11
    (a.report/'appearance_video_validation.json').write_text(json.dumps(dict(duration=float(info['format']['duration']),decode_pass=True,rate=10,source_start=118,source_end=129),indent=2)+'\n')
    print('Appearance comparison image/video verified')
if __name__=='__main__':main()
from pathlib import Path
import json,re,subprocess,concurrent.futures
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path('/home/xhj/liftrace-worktrees/r2026-high-view-search');D=R/'docs/verification/history_31_40_20260920';items=json.loads((D/'runs.json').read_text())
def check(item):
    video=Path(item['run'])/'presentation.mp4'
    probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','format=duration,size:stream=width,height,codec_name,nb_frames','-of','json',str(video)],text=True))
    result=subprocess.run(['ffmpeg','-v','error','-threads','1','-i',str(video),'-f','null','-'],capture_output=True,text=True)
    return dict(seed=item['seed'],path=str(video),probe=probe,decode_exit=result.returncode,error=result.stderr)
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:videos=list(pool.map(check,items))
for chunk in range(2):
    fig,axes=plt.subplots(5,2,figsize=(14,18))
    for item,meta,row in zip(items[chunk*5:(chunk+1)*5],videos[chunk*5:(chunk+1)*5],axes):
        cap=cv2.VideoCapture(meta['path']);duration=float(meta['probe']['format']['duration'])
        for ax,t in zip(row,[duration*.5,max(0,duration-.5)]):
            cap.set(cv2.CAP_PROP_POS_MSEC,t*1000);ok,frame=cap.read();assert ok
            ax.imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB));ax.set_title(f'Seed{item["seed"]} / video {t:.1f}s');ax.set_axis_off()
        cap.release()
    fig.tight_layout();fig.savefig(D/f'video_contact_sheet_{chunk+1}.jpg',dpi=120);plt.close(fig)
missing=[]
for file in (D/'REPORT.md',D/'index.html'):
    text=file.read_text();links=re.findall(r'(?:src|href)="([^"]+)"',text)+re.findall(r'\]\(([^)]+)\)',text)
    for link in links:
        if '://' in link or link.startswith('#') or link=='validation.json':continue
        if not (file.parent/link.split('#')[0]).exists():missing.append(dict(source=file.name,target=link))
result=dict(all_videos_decode=all(v['decode_exit']==0 for v in videos),videos=videos,missing_links=missing,png_count=len(list(D.rglob('*.png'))))
(D/'validation.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='videos'},indent=2))
assert result['all_videos_decode'] and not missing

from pathlib import Path
import json,subprocess,re
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent
result={'videos':[],'missing_links':[]}
fig,axes=plt.subplots(3,3,figsize=(18,11))
for item,row in zip(json.loads((D/'runs.json').read_text()),axes):
    path=Path(item['run'])/'presentation.mp4'
    probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','format=duration,size:stream=codec_name,width,height,nb_frames','-of','json',str(path)],text=True))
    decode=subprocess.run(['ffmpeg','-v','error','-threads','1','-i',str(path),'-f','null','-'],capture_output=True,text=True)
    result['videos'].append(dict(label=item['label'],path=str(path),probe=probe,decode_exit=decode.returncode,decode_error=decode.stderr))
    cap=cv2.VideoCapture(str(path));duration=float(probe['format']['duration'])
    for ax,t in zip(row,[20.,duration*.55,duration-.5]):
        cap.set(cv2.CAP_PROP_POS_MSEC,t*1000);ok,image=cap.read();assert ok
        ax.imshow(cv2.cvtColor(image,cv2.COLOR_BGR2RGB));ax.set_title(f'{item["label"]} / movie {t:.1f}s');ax.set_axis_off()
    cap.release()
fig.tight_layout();fig.savefig(D/'video_contact_sheet.jpg',dpi=130);plt.close(fig)
for p in [D/'REPORT.md',D/'index.html']:
    text=p.read_text();links=re.findall(r'(?:src|href)="([^"]+)"',text)+re.findall(r'\]\(([^)]+)\)',text)
    for link in links:
        if '://' in link or link.startswith('#') or link=='validation.json':continue
        if not (p.parent/link.split('#')[0]).exists():result['missing_links'].append(dict(source=p.name,target=link))
result['png_count']=len(list(D.rglob('*.png')))
result['all_videos_decode']=all(v['decode_exit']==0 for v in result['videos'])
(D/'validation.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='videos'},indent=2))

from pathlib import Path
import json,collections,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import cv2
import argparse
parser=argparse.ArgumentParser();parser.add_argument('analysis_dir',type=Path);out=parser.parse_args().analysis_dir
d=collections.defaultdict(list)
for line in (out/'records.jsonl').open():
    r=json.loads(line);d[r['topic']].append(r)
meta=json.loads((out/'metadata.json').read_text())[-1]['data']; ground=meta['ground_reference']['ground_z']
for f in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc','/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc']:
    if Path(f).exists():font_manager.fontManager.addfont(f);plt.rcParams['font.family']=font_manager.FontProperties(fname=f).get_name();break
plt.rcParams['axes.unicode_minus']=False

def pos(topic):
    return np.array([[r['t'],*[r['msg']['pose']['position'][k] for k in 'xyz']] for r in d[topic]])
p=pos('/navigation/local_pose');sp=pos('/navigation/setpoint_mission');ev=pos('/mavros/vision_pose/pose')
fig,axes=plt.subplots(3,1,figsize=(13,11),sharex=True)
axes[0].plot(p[:,0],p[:,3]-ground,label='任务坐标反馈 FC AGL');axes[0].plot(sp[:,0],sp[:,3]-ground,label='实际发送目标 AGL');axes[0].plot(ev[:,0],ev[:,3]-ground,alpha=.6,label='LIO外部位姿（独立原点，仅供对照）')
axes[0].set_ylabel('高度 / m');axes[0].legend(loc='upper right');axes[0].set_title('10月15日15:02实飞：实际加载旧场地，高位停滞后首次红十字下降')
axes[0].axhline(.22,c='gray',ls=':',label='静置FC参考')
for idx,axis in enumerate(axes[1:]):
    axis.plot(p[:,0],p[:,idx+1],label='反馈');axis.plot(sp[:,0],sp[:,idx+1],label='目标');axis.set_ylabel(('X' if idx==0 else 'Y')+' / m');axis.legend(loc='upper right')
for ax in axes:
    ax.grid(alpha=.25)
    for t,label in [(58.15,'首搜索点'),(69.39,'停滞换点'),(78.90,'跳过区域'),(84.13,'中断转下降'),(110.73,'红十字对准'),(118.93,'投递下降'),(120.31,'POSCTL')]:
        ax.axvline(t,ls=':',alpha=.45,color='black')
    ax.set_xlim(43,131)
axes[-1].set_xlabel('相对bag起点 / s（起点15:02:01）');fig.tight_layout();fig.savefig(out/'timeline.png',dpi=140);plt.close(fig)
fig,ax=plt.subplots(figsize=(11,7));ax.plot(p[:,1],p[:,2],label='实际任务系轨迹',lw=1.7)
route=np.array(meta['settings']['flight_area']['survey_xy']);fc=meta['ground_reference']['fc_xyz'];route+=np.array(fc[:2]);ax.plot(route[:,0],route[:,1],'--o',label='bag实际搜索航点')
for t,txt in [(58.15,'首点开始'),(69.39,'换点'),(84.13,'搜索中断'),(110.73,'红十字'),(120.31,'POSCTL')]:
    q=p[np.argmin(abs(p[:,0]-t))];ax.scatter(q[1],q[2],s=30);ax.annotate(txt,(q[1],q[2]),xytext=(7,7),textcoords='offset points')
ax.set_aspect('equal');ax.grid(alpha=.25);ax.legend();ax.set_xlabel('任务 X / m');ax.set_ylabel('任务 Y / m');ax.set_title('本轮没有进入走廊/H；轨迹来自定位反馈');fig.tight_layout();fig.savefig(out/'trajectory.png',dpi=140);plt.close(fig)
cap=cv2.VideoCapture(str(out/'replay/camera_raw.mp4'));frames=[]
for t in [58,62,80,84,103,110,117,119,120.2,121.5,124,129]:
    cap.set(cv2.CAP_PROP_POS_MSEC,t*1000);ok,img=cap.read()
    if not ok:continue
    img=cv2.resize(img,(480,270));cv2.putText(img,f'bag +{t:.1f}s',(12,26),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,255),2);frames.append(img)
cap.release()
while len(frames)%3:frames.append(np.zeros((270,480,3),np.uint8))
cv2.imwrite(str(out/'camera_contact.jpg'),np.vstack([np.hstack(frames[i:i+3]) for i in range(0,len(frames),3)]))
print(out)
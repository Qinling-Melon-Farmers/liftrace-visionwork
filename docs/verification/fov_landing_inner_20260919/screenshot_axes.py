"""Coordinate arrows on an archived observer image; no synthetic flight imagery."""
from pathlib import Path
import json,cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
D=Path(__file__).resolve().parent
run=Path(json.loads((D/'runs.json').read_text())[1]['run'])
cap=cv2.VideoCapture(str(run/'overview.mp4'));cap.set(cv2.CAP_PROP_POS_MSEC,10000);ok,image=cap.read();cap.release();assert ok
cv2.imwrite(str(D/'overview_actual.jpg'),image)
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
fig,ax=plt.subplots(figsize=(11,11));ax.imshow(cv2.cvtColor(image,cv2.COLOR_BGR2RGB));ax.set_axis_off()
# The frozen presentation camera is identical to the previous frame review.
# Scale calibrated image coordinates in case the encoding resolution changes.
sx=image.shape[1]/960.;sy=image.shape[0]/960.;origin=(257*sx,480*sy)
for end,color,label,labelxy in [((392,480),'#ff3838','+X 朝场内\n初始机头方向',(398,480)),((257,345),'#17d9ff','+Y 起飞时左侧',(265,332))]:
    ax.annotate('',xy=(end[0]*sx,end[1]*sy),xytext=origin,arrowprops=dict(arrowstyle='-|>',color=color,lw=3,mutation_scale=23))
    ax.text(labelxy[0]*sx,labelxy[1]*sy,label,color=color,fontproperties=font,fontsize=13,va='center',bbox=dict(facecolor='#101820',alpha=.9,pad=5,edgecolor='none'))
ax.text(235*sx,510*sy,'起飞点 (0,0)',fontproperties=font,color='white',fontsize=12,bbox=dict(facecolor='#101820',alpha=.85,edgecolor='none'))
ax.set_title('修复后B轮实际俯视截图｜固定起飞坐标系',fontproperties=font,fontsize=19,pad=15)
fig.text(.5,.04,'新 X＝旧 Y；新 Y＝−旧 X；+Z 垂直向上（朝屏幕外）\n箭头表示起飞点和初始机头，不是截图时空中飞机的当前航向。',fontproperties=font,fontsize=12,ha='center')
fig.subplots_adjust(left=.02,right=.98,bottom=.10,top=.94)
fig.savefig(D/'frame_axes.png',dpi=150);fig.savefig(D/'frame_axes.svg');plt.close(fig)
p=D/'frame_axes.svg';p.write_text('\n'.join(line.rstrip() for line in p.read_text().splitlines())+'\n')

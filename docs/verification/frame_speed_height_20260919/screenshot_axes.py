"""Scientific coordinate overlay on an actual Gazebo observer frame."""
from pathlib import Path
import shutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

D=Path(__file__).resolve().parent
shutil.copyfile('/home/xhj/presentation_overview_preview.jpg',D/'overview_actual.jpg')
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
fig,ax=plt.subplots(figsize=(11,11));ax.imshow(plt.imread(D/'overview_actual.jpg'));ax.set_axis_off()
# Ground pad centre from pinhole projection, verified against rendered H centre.
origin=(257,480)
for end,color,label,labelxy in [((392,480),'#ff3838','+X 朝场内\n初始机头方向',(398,480)),((257,345),'#17d9ff','+Y 起飞时左侧',(265,332))]:
    ax.annotate('',xy=end,xytext=origin,arrowprops=dict(arrowstyle='-|>',color=color,lw=3,mutation_scale=23))
    ax.text(*labelxy,label,color=color,fontproperties=font,fontsize=13,va='center',bbox=dict(facecolor='#101820',alpha=.9,pad=5,edgecolor='none'))
ax.text(235,510,'起飞点 (0,0)',fontproperties=font,color='white',fontsize=12,bbox=dict(facecolor='#101820',alpha=.85,edgecolor='none'))
ax.set_title('本轮仿真实际俯视截图｜固定起飞坐标系',fontproperties=font,fontsize=19,pad=15)
fig.text(.5,.04,'新 X＝旧 Y；新 Y＝−旧 X；+Z 垂直向上（朝屏幕外）\n箭头标在起飞 H 点，表示初始机头，不是截图时空中飞机的当前位置或航向。',fontproperties=font,fontsize=12,ha='center')
fig.subplots_adjust(left=.02,right=.98,bottom=.10,top=.94)
fig.savefig(D/'frame_axes.png',dpi=150);fig.savefig(D/'frame_axes.svg');plt.close(fig)

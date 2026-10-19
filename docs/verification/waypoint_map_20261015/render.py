"""Render the supplied map YAML without changing flight settings or guessing walls."""
from pathlib import Path
import argparse, json, math, shutil
import yaml
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Rectangle
from matplotlib.backends.backend_pdf import PdfPages

ap=argparse.ArgumentParser()
ap.add_argument('--input',type=Path,required=True)
ap.add_argument('--output',type=Path,required=True)
a=ap.parse_args(); a.output.mkdir(parents=True,exist_ok=True)
original=a.input.read_bytes(); cfg=yaml.safe_load(original)
font=font_manager.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
font_manager.fontManager.addfont(font.get_file())
plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':11,'savefig.facecolor':'white','svg.fonttype':'path'})
colors={'flight_bounds':'#344154','target_bounds':'#b179a3','search_center_bounds':'#2780bb','coverage_bounds':'#39a578','survey':'#d57416','corridor':'#267ca6'}
labels={'flight_bounds':'FC飞行边界','target_bounds':'目标许可范围','search_center_bounds':'搜索中心范围','coverage_bounds':'低空兜底范围'}
S=np.array(cfg['staging_xy'],float); survey=np.array(cfg['survey_xy'],float); corridor=np.array([[p['x'],p['y']] for p in cfg['corridor_waypoints']],float); H=np.array(cfg['landing_xy'],float)
points=[('O',0.,0.),('S',*S)]+[(f'P{i+1}',*p) for i,p in enumerate(survey[:-1])]+[(f'C{i+1}',*p) for i,p in enumerate(corridor)]
legs=lambda pts:[float(np.linalg.norm(b-a)) for a,b in zip(pts[:-1],pts[1:])]
survey_legs=legs(survey);corridor_legs=legs(corridor)
bounds={k:dict(x_min=v[0],x_max=v[1],y_min=v[2],y_max=v[3],size_x=v[1]-v[0],size_y=v[3]-v[2],area=(v[1]-v[0])*(v[3]-v[2])) for k,v in cfg.items() if k in colors and k.endswith('bounds')}
axis=cfg['corridor_speed_schedule']['axis'];walls=sorted(cfg['corridor_speed_schedule']['wall_coordinates'],reverse=True);crossings=[]
for wall in walls:
    for i,(p,q) in enumerate(zip(corridor[:-1],corridor[1:])):
        if (p[axis]-wall)*(q[axis]-wall)<0:
            t=(wall-p[axis])/(q[axis]-p[axis]);xy=p+t*(q-p)
            crossings.append(dict(wall_axis=axis,wall_coordinate=wall,segment=f'C{i+1}→C{i+2}',xy=xy.tolist(),distance_before=float(np.linalg.norm(xy-p)),distance_after=float(np.linalg.norm(q-xy))))
metrics={'source':str(a.input),'bounds':bounds,'points':[dict(label=n,x=float(x),y=float(y)) for n,x,y in points], 'landing_xy':H.tolist(),'survey_segments':survey_legs,'survey_total':sum(survey_legs),'corridor_segments':corridor_legs,'corridor_total':sum(corridor_legs),'origin_to_staging':float(np.linalg.norm(S)),'staging_to_P1':float(np.linalg.norm(S-survey[0])),'origin_to_H_direct':float(np.linalg.norm(H)),'wall_crossings_inferred_not_measured_doors':crossings,'scope':'Geometric distances from YAML; no real wall width/opening/obstacle positions supplied. Flight bounds are FC constraints, not surveyed physical walls.'}
(a.output/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(a.output/'waypoint_map.snapshot.yaml').write_bytes(original)

def dim(ax,p,q,text,color='#52616e',fontsize=10):
    ax.annotate('',xy=q,xytext=p,arrowprops=dict(arrowstyle='<->',lw=1,color=color,shrinkA=0,shrinkB=0))
    m=(np.array(p)+np.array(q))/2;vertical=abs(q[1]-p[1])>abs(q[0]-p[0])
    ax.text(m[0]-.08 if vertical else m[0],m[1] if vertical else m[1]+.05,text,rotation=90 if vertical else 0,ha='right' if vertical else 'center',va='center' if vertical else 'bottom',fontsize=fontsize,color=color,bbox=dict(facecolor='white',edgecolor='none',alpha=.92,pad=1))
def line_arrows(ax,pts,color,lw=2):
    ax.plot(pts[:,0],pts[:,1],color=color,lw=lw,zorder=6)
    for p,q in zip(pts[:-1],pts[1:]):
        ax.annotate('',xy=p+.60*(q-p),xytext=p+.43*(q-p),arrowprops=dict(arrowstyle='-|>',lw=lw,color=color),zorder=7)
def draw_box(ax,key,face=None):
    x0,x1,y0,y1=cfg[key]
    ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,facecolor=face or 'none',edgecolor=colors[key],lw=1.8,ls='-' if key=='flight_bounds' else '--',alpha=.95,zorder=1))
def table(ax,header,rows,widths=None,fontsize=11):
    ax.axis('off');t=ax.table(cellText=rows,colLabels=header,loc='center',cellLoc='center',colWidths=widths)
    t.auto_set_font_size(False);t.set_fontsize(fontsize);t.scale(1,1.65)
    for (r,c),cell in t.get_celld().items():
        cell.set_edgecolor('#d8e0e5')
        if r==0:cell.set_facecolor('#e9f0f5');cell.set_text_props(weight='bold')
        else:cell.set_facecolor('#f7f9fb' if r%2 else 'white')
    return t

fig=plt.figure(figsize=(18,12));gs=fig.add_gridspec(1,2,width_ratios=[1.7,1.],left=.06,right=.98,bottom=.18,top=.90,wspace=.12)
ax=fig.add_subplot(gs[0]);rs=gs[1].subgridspec(3,1,height_ratios=[1.,1.65,.7],hspace=.08)
for key,face in [('flight_bounds','#f9fafc'),('target_bounds',None),('search_center_bounds',None),('coverage_bounds','#edf8f0')]:draw_box(ax,key,face)
ax.axhline(0,color='#aeb9c4',lw=.8);ax.axvline(0,color='#aeb9c4',lw=.8)
ax.plot([0,S[0],survey[0,0]],[0,S[1],survey[0,1]],':',color='#818b94',lw=1.2,zorder=2)
line_arrows(ax,survey,colors['survey']);line_arrows(ax,corridor,colors['corridor'])
for i,(x,y) in enumerate(survey[:-1]):
    ax.scatter(x,y,color=colors['survey'],s=42,zorder=9)
    dx,dy=([-.15,-.28],[.08,-.28],[.08,.18],[-.15,.18])[i]
    ax.text(x+dx,y+dy,f'P{i+1}',color=colors['survey'],weight='bold',fontsize=11,zorder=10)
for i,(x,y) in enumerate(corridor):
    ax.scatter(x,y,color=colors['corridor'],s=42,zorder=9)
    dx,dy=(-.2,.23) if i<2 else (.15,.05)
    ax.text(x+dx,y+dy,f'C{i+1}' if i<4 else 'C5 / H',color=colors['corridor'],weight='bold',fontsize=11,zorder=10)
ax.scatter(*H,marker='*',s=180,color='#ba3d49',zorder=10)
ax.scatter(0,0,marker='^',s=90,color='#273641',zorder=10);ax.text(.08,-.37,'O 起飞原点',fontsize=10,weight='bold')
ax.scatter(*S,marker='D',s=45,color='#657382',zorder=10);ax.text(S[0]+.16,S[1]+.20,'S 准备点',fontsize=10)
for wall in walls:
    if axis==1:
        ax.plot([7.7,9.35],[wall,wall],ls=(0,(5,4)),color='#967b62',lw=1.8,zorder=3)
        ax.text(9.42,wall,f'Y={wall:+.1f}',ha='left',va='center',fontsize=9,color='#80634e')
for name,style in [('flight_bounds','-'),('target_bounds','--'),('search_center_bounds','--'),('coverage_bounds','--')]:ax.plot([],[],style,color=colors[name],label=labels[name],lw=1.6)
ax.plot([],[],color=colors['survey'],label='高位搜索：P1→P2→P3→P4→P1',lw=2)
ax.plot([],[],color=colors['corridor'],label='走廊：C1→C2→C3→C4→C5/H',lw=2)
fig.legend(*ax.get_legend_handles_labels(),loc='lower left',bbox_to_anchor=(.06,.068),ncol=3,frameon=False,fontsize=10)
dim(ax,(-.5,5.2),(9.5,5.2),'飞行边界 X跨度 10.00m')
dim(ax,(-1.1,-4.8),(-1.1,4.8),'飞行边界 Y跨度 9.60m')
dim(ax,(-.1,4.12),(7.05,4.12),'搜索中心 X跨度 7.15m',colors['search_center_bounds'],9)
ax.text(3.25,-3.28,'4.50m',ha='center',color=colors['survey'],bbox=dict(fc='white',ec='none',alpha=.85))
ax.text(3.25,3.23,'4.50m',ha='center',color=colors['survey'],bbox=dict(fc='white',ec='none',alpha=.85))
ax.text(5.68,0,'7.00m',rotation=90,va='center',color=colors['survey'])
ax.text(.82,1.5,'7.00m',rotation=90,va='center',ha='right',color=colors['survey'])
ax.set(xlim=(-1.45,10.2),ylim=(-5.15,5.55),aspect='equal',xlabel='X / m  → 起飞时机头朝向（场内）',ylabel='Y / m  ↑ 起飞时机身左侧')
ax.set_xticks(np.arange(0,10,1));ax.set_yticks(np.arange(-4,5,1));ax.grid(alpha=.15,zorder=0)
ax.set_title('坐标与工作台一致：屏幕向右 +X，向上 +Y',fontsize=13,pad=12)
r0=fig.add_subplot(rs[0]);table(r0,['配置范围','X最小～最大','Y最小～最大','X×Y跨度'],[[labels[k],f"{b['x_min']:g}～{b['x_max']:g}",f"{b['y_min']:g}～{b['y_max']:g}",f"{b['size_x']:g}×{b['size_y']:g}m"] for k,b in bounds.items()],fontsize=10)
r1=fig.add_subplot(rs[1]);table(r1,['点位','X / m','Y / m','作用'],[[n,f'{x:g}',f'{y:g}',('起飞原点' if n=='O' else '高位前准备' if n=='S' else '高位搜索' if n.startswith('P') else '走廊；末点=H' if n=='C5' else '走廊引导')] for n,x,y in points],fontsize=10)
r2=fig.add_subplot(rs[2]);r2.axis('off');r2.text(0,1,'当前高度（均为FC中心离地）\n高位 2.60m｜低位 1.40m｜走廊 0.90m\nH转场 1.00m｜H捕获 0.90m\n\n高位矩形一圈 23.00m；走廊折线 8.30m\nS→P1 3.57m；O→S 0.60m',va='top',fontsize=12,linespacing=1.65)
fig.suptitle('waypoint_map.yaml · 场地与计划航点反推图',fontsize=21,weight='bold',y=.965)
fig.text(.06,.025,'所有边框均为YAML软件范围，不代表实体围墙。棕色虚线仅示意门墙参考面；墙长、门洞宽度和真实门中心未给出。\n航线连线表示设计点间几何关系，不等于实际避障轨迹；本图未修改任何配置。',fontsize=11,color='#56626d')
fig.savefig(a.output/'waypoint_map_overview.png',dpi=170);fig.savefig(a.output/'waypoint_map_overview.svg')

fig2=plt.figure(figsize=(16,11));gs2=fig2.add_gridspec(1,2,width_ratios=[1.05,1.25],left=.07,right=.97,bottom=.10,top=.89,wspace=.17)
bx=fig2.add_subplot(gs2[0]);line_arrows(bx,corridor,colors['corridor'])
for i,(x,y) in enumerate(corridor):
    bx.scatter(x,y,color=colors['corridor'],s=65,zorder=9)
    bx.annotate(f'C{i+1}'+(' / H' if i==4 else '')+f' ({x:g}, {y:g})',xy=(x,y),xytext=(-10,12) if i<2 else (12,10),textcoords='offset points',ha='right' if i<2 else 'left',fontsize=11,weight='bold',color=colors['corridor'])
for i,c in enumerate(crossings):
    wall=c['wall_coordinate'];x,y=c['xy']
    bx.axhline(wall,color='#967b62',ls=(0,(5,4)),lw=1.6)
    bx.scatter(x,y,marker='s',s=80,facecolors='white',edgecolors='#9c6640',zorder=10)
    bx.text(6.55,wall+.13,f'参考墙{i+1}：Y={wall:+.1f}',color='#80634e',fontsize=11)
    bx.text(9.4,wall-.18,f'推导交点D{i+1}',color='#80634e',fontsize=10,ha='right')
dim(bx,tuple(corridor[0]+[0,.58]),tuple(corridor[1]+[0,.58]),'C1→C2：1.40m',fontsize=10)
for i in range(1,len(corridor)-1):dim(bx,(9.5,corridor[i,1]),(9.5,corridor[i+1,1]),f'{corridor_legs[i]:.2f}m',colors['corridor'],10)
for y0,y1,txt in [(3.4,1.6,'1.80m'),(1.6,-1.6,'两墙间3.20m'),(-1.6,-3.5,'至H 1.90m')]:dim(bx,(7.15,y0),(7.15,y1),txt,'#80634e',10)
bx.set(xlim=(6.45,9.85),ylim=(-4.05,4.45),xlabel='X / m',ylabel='Y / m',aspect='equal');bx.set_xticks([6.9,7.4,8.3,9.5]);bx.grid(alpha=.17);bx.set_title('走廊按给定顺序：向 +X 入场，再向 −Y',fontsize=12)
right=gs2[1].subgridspec(3,1,height_ratios=[1.1,1.1,1.4],hspace=.08)
cx=fig2.add_subplot(right[0]);table(cx,['设计航段','ΔX','ΔY','长度'],[[f'C{i+1}→C{i+2}',f'{q[0]-p[0]:+.2f}',f'{q[1]-p[1]:+.2f}',f'{corridor_legs[i]:.2f}m'] for i,(p,q) in enumerate(zip(corridor[:-1],corridor[1:]))],fontsize=11)
dx=fig2.add_subplot(right[1]);wall_table=table(dx,['参考面 / 推导交点','穿越航段','前点→墙','墙→后点'],[[f"Y={c['wall_coordinate']:+.1f}\n({c['xy'][0]:g}, {c['xy'][1]:g})",c['segment'],f"{c['distance_before']:.2f}m",f"{c['distance_after']:.2f}m"] for c in crossings],fontsize=10)
for (row,col),cell in wall_table.get_celld().items(): cell.set_height(cell.get_height()*(1.8 if row else 1.2))
ex=fig2.add_subplot(right[2]);ex.axis('off');ex.text(0,1,'读图要点\n\n• C5与H中心重合：(8.3, −3.5)。\n• 五个走廊点均为FC AGL 0.90m。\n• entry_waypoints=1：完成C1后进入走廊调度；\n  不是“第一道门”的编号。\n• C3在第一参考墙前0.30m，C4在其后2.00m。\n• 第二参考墙位于C4与C5之间，距两端1.20/1.90m。\n• D1/D2只是设计折线与参考面的交点，\n  不能代替现场测得的门洞中心。\n• 走廊两侧长墙、入口墙、门宽与障碍物未配置，\n  因而不能从1.40m入场航段反推走廊宽度。',va='top',fontsize=12,linespacing=1.5)
fig2.suptitle('走廊 / 门墙参考面 / H · 距离放大图',fontsize=21,weight='bold',y=.96)
fig2.text(.07,.035,'虚线仅标YAML中的常量坐标面，显示长度无物理含义；图中未绘制或假设门洞开口。所有距离为水平几何距离。',fontsize=11,color='#56626d')
fig2.savefig(a.output/'corridor_detail.png',dpi=170);fig2.savefig(a.output/'corridor_detail.svg')
with PdfPages(a.output/'waypoint_map_diagrams.pdf') as pdf:pdf.savefig(fig);pdf.savefig(fig2)
assert a.input.read_bytes()==original
print(json.dumps(metrics,ensure_ascii=False,indent=2));print('OUTPUT',a.output)
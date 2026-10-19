"""Model-derived continuous FOV sweep for this supplied rectangular map."""
from pathlib import Path
import argparse,json,math
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Rectangle,Patch
from matplotlib.lines import Line2D

p=argparse.ArgumentParser();p.add_argument('--map',type=Path,required=True);p.add_argument('--calibration-result',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
cfg=yaml.safe_load(a.map.read_text(encoding='utf-8'));cal=json.loads(a.calibration_result.read_text(encoding='utf-8'))
fc=float(cfg['high_agl']);lens=fc+cal['camera_z_from_fc'];source=cal['heights'][0]
bounds=np.array(source['D_inscribed_bounds'])/source['lens_agl']*lens
route=np.array(cfg['survey_xy'],float);target=np.array(cfg['target_bounds'],float);search=np.array(cfg['search_center_bounds'],float)
font=font_manager.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');font_manager.fontManager.addfont(font.get_file());plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':11,'svg.fonttype':'path'})
def sweeps(half):
    x0,x1,y0,y1=bounds+np.array([half,-half,half,-half])
    if x0>=x1 or y0>=y1:return []
    rects=[]
    for u,v in zip(route[:-1],route[1:]):
        if not (np.isclose(u[0],v[0]) or np.isclose(u[1],v[1])):raise ValueError('This report expects an axis-aligned rectangular route')
        rects.append([min(u[0],v[0])+x0,max(u[0],v[0])+x1,min(u[1],v[1])+y0,max(u[1],v[1])+y1])
    return rects

def clip(r,d):return [max(r[0],d[0]),min(r[1],d[1]),max(r[2],d[2]),min(r[3],d[3])]
def area(rects,domain):
    xs=sorted(set([domain[0],domain[1]]+[max(domain[0],min(domain[1],v)) for r in rects for v in r[:2]]))
    ys=sorted(set([domain[2],domain[3]]+[max(domain[2],min(domain[3],v)) for r in rects for v in r[2:]]))
    covered=0.
    for l,r in zip(xs[:-1],xs[1:]):
        for b,t in zip(ys[:-1],ys[1:]):
            x,y=(l+r)/2,(b+t)/2
            if any(q[0]<=x<=q[1] and q[2]<=y<=q[3] for q in rects):covered+=(r-l)*(t-b)
    total=(domain[1]-domain[0])*(domain[3]-domain[2])
    return dict(domain=domain.tolist(),area=total,covered=covered,uncovered=total-covered,coverage_percent=100*covered/total)
point=sweeps(0);whole=sweeps(.5);whole_domain=target+np.array([.5,-.5,.5,-.5]);rot_half=math.sqrt(2)/2;rot_domain=target+np.array([rot_half,-rot_half,rot_half,-rot_half])
metrics=dict(fc_agl=fc,lens_agl=lens,footprint_bounds=bounds.tolist(),footprint_xy=[bounds[1]-bounds[0],bounds[3]-bounds[2]],point_target=area(point,target),point_search_center=area(point,search),whole_1m_axis_aligned=area(whole,whole_domain),whole_1m_sqrt2_envelope=area(sweeps(rot_half),rot_domain),scope='Numerical inscribed rectangle from K/D; nominal level fixed yaw, flat floor, continuous travel along every line; no timing/FPS, occlusion, attitude, detector recall or actual flight guarantee.')
(a.output/'fov_coverage_metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
fig,axes=plt.subplots(1,2,figsize=(19,11));fig.subplots_adjust(left=.055,right=.98,top=.82,bottom=.20,wspace=.20)
for idx,(ax,rects,domain,result) in enumerate([(axes[0],point,target,metrics['point_target']),(axes[1],whole,whole_domain,metrics['whole_1m_axis_aligned'])]):
    ax.add_patch(Rectangle((domain[0],domain[2]),domain[1]-domain[0],domain[3]-domain[2],fc='#f9d8d4',ec='none',hatch='///',zorder=1))
    for r in rects:
        r=clip(r,domain)
        if r[1]>r[0] and r[3]>r[2]:ax.add_patch(Rectangle((r[0],r[2]),r[1]-r[0],r[3]-r[2],fc='#c8e7f2',ec='none',zorder=2))
    ax.add_patch(Rectangle((domain[0],domain[2]),domain[1]-domain[0],domain[3]-domain[2],fill=False,ec='#7e5584',lw=1.8,zorder=3))
    ax.add_patch(Rectangle((search[0],search[2]),search[1]-search[0],search[3]-search[2],fill=False,ec='#6d747e',lw=1,ls=':',zorder=4))
    ax.plot(route[:,0],route[:,1],'-o',color='#d57416',lw=2,zorder=7)
    for i,(x,y) in enumerate(route[:-1]):ax.annotate(f'P{i+1}',(x,y),xytext=(6,8),textcoords='offset points',color='#a1540d',fontsize=10,zorder=8)
    for u,v in zip(route[:-1],route[1:]):ax.annotate('',xy=u+.62*(v-u),xytext=u+.44*(v-u),arrowprops=dict(arrowstyle='-|>',lw=2,color='#d57416'),zorder=8)
    ax.scatter(0,0,color='#263744',marker='^',s=55,zorder=9);ax.text(.08,-.32,'O (0,0)',fontsize=9,zorder=9)
    ax.set(xlim=(-1,8.4),ylim=(-5.15,5.2),aspect='equal',xlabel='X / m → 机头朝向',ylabel='Y / m ↑ 机身左侧');ax.set_xticks(np.arange(0,9));ax.set_yticks(np.arange(-4,5));ax.grid(alpha=.13,zorder=0)
    ax.set_title(('地面点：整圈扫过范围' if idx==0 else '完整1m方板：可被完整看见的靶心范围\n（靶板边与XY轴平行）')+f"\n模型覆盖 {result['coverage_percent']:.2f}%｜未覆盖 {result['uncovered']:.2f}㎡",fontsize=13,pad=12)
x0,x1,y0,y1=bounds
axes[0].add_patch(Rectangle((5.5+x0,y0),x1-x0,y1-y0,fill=False,ec='#1e618b',ls='--',lw=2,zorder=6))
axes[0].scatter(5.5,0,color='#1e618b',marker='x',s=65,zorder=9)
axes[0].text(6.1,.25,'单帧示例\n相机在(5.5,0)',color='#1e618b',fontsize=10,zorder=9,bbox=dict(facecolor='white',edgecolor='none',alpha=.9))
xgap=[route[:,0].min()+x1,route[:,0].max()+x0];ygap=[route[:,1].min()+y1,route[:,1].max()+y0]
metrics['central_uncovered_rectangle']=dict(x=xgap,y=ygap,width=xgap[1]-xgap[0],height=ygap[1]-ygap[0])
axes[0].annotate(f"中心模型缺口\n{xgap[1]-xgap[0]:.2f}×{ygap[1]-ygap[0]:.2f}m",xy=((xgap[0]+xgap[1])/2,1.),xytext=(.2,2.4),arrowprops=dict(arrowstyle='->',color='#b25249'),color='#a74c45',fontsize=11,bbox=dict(facecolor='white',edgecolor='none',alpha=.9),zorder=10)
handles=[Patch(fc='#c8e7f2',label='本模型整圈可覆盖'),Patch(fc='#f9d8d4',hatch='///',label='本模型未覆盖'),Line2D([],[],color='#d57416',lw=2,label='当前高位四边形'),Line2D([],[],color='#1e618b',lw=2,ls='--',label='单帧内接视场'),Line2D([],[],color='#6d747e',ls=':',label='搜索中心范围（配置）')]
fig.legend(handles=handles,loc='lower left',bbox_to_anchor=(.055,.10),ncol=3,frameon=False,fontsize=11)
fig.suptitle(f"当前YAML的FOV覆盖：FC {fc:.2f}m / 镜头 {lens:.2f}m\n单帧内接视场 X×Y ≈ {x1-x0:.2f}×{y1-y0:.2f}m；两长边X间距4.50m",fontsize=20,weight='bold',y=.975)
fig.text(.055,.035,'左图按 target_bounds 内的地面点统计；右图仅统计整块1m板可放入该范围的靶心位置，两图分母不同。\n按已记录K/D、水平相机、固定航向、平地、沿完整矩形连续扫过计算。颜色不是实测识别率，也不包含姿态、障碍遮挡或提前中断。',fontsize=11,color='#56626d')
fig.savefig(a.output/'fov_coverage.png',dpi=170);fig.savefig(a.output/'fov_coverage.svg');fig.savefig(a.output/'fov_coverage.pdf')
(a.output/'fov_coverage_metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps(metrics,ensure_ascii=False,indent=2))
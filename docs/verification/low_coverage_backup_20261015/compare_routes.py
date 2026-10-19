"""Compare measured-map search candidates using the existing calibrated footprint.
No ROS runtime, inference recall, or obstacle-free guarantee is implied.
"""
import argparse,json,math,sys
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Rectangle

def swept_rectangles(route,footprint,inset):
    b=footprint+np.array([inset,-inset,inset,-inset])
    if b[0]>=b[1] or b[2]>=b[3]:return []
    rects=[]
    for u,v in zip(route[:-1],route[1:]):
        if abs(u[0]-v[0])>1e-7 and abs(u[1]-v[1])>1e-7:
            raise ValueError('Axis-aligned sweeps required; do not overestimate diagonal connectors')
        rects.append([min(u[0],v[0])+b[0],max(u[0],v[0])+b[1],min(u[1],v[1])+b[2],max(u[1],v[1])+b[3]])
    return rects

def union_area(rects,d):
    xs=sorted({d[0],d[1],*(max(d[0],min(d[1],x)) for r in rects for x in r[:2])})
    ys=sorted({d[2],d[3],*(max(d[2],min(d[3],y)) for r in rects for y in r[2:])})
    covered=0.
    for x0,x1 in zip(xs[:-1],xs[1:]):
        for y0,y1 in zip(ys[:-1],ys[1:]):
            if any(r[0]<=(x0+x1)/2<=r[1] and r[2]<=(y0+y1)/2<=r[3] for r in rects):covered+=(x1-x0)*(y1-y0)
    total=float((d[1]-d[0])*(d[3]-d[2]))
    return dict(domain=d.tolist(),area_m2=total,covered_m2=covered,uncovered_m2=total-covered,percent=100*covered/total)

def evaluate(route,fp,target):
    out={}
    for name,margin in [('point',0.)]:
        domain=target+np.array([margin,-margin,margin,-margin]);out[name]=union_area(swept_rectangles(route,fp,margin),domain)
    return out

def draw(ax,route,fp,target,title):
    margin=0.;d=target
    for r in swept_rectangles(route,fp,margin):
        x0,x1=max(d[0],r[0]),min(d[1],r[1]);y0,y1=max(d[2],r[2]),min(d[3],r[3])
        if x0<x1 and y0<y1:ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,fc='#cae4ed',ec='none'))
    ax.add_patch(Rectangle((d[0],d[2]),d[1]-d[0],d[3]-d[2],fill=False,ec='#7a5a89',lw=1.4))
    ax.plot(route[:,0],route[:,1],'-o',color='#d87920',lw=1.6,ms=3,zorder=3)
    for n,(x,y) in enumerate(route):
        if n==0 and np.allclose(route[0],route[-1]):ax.annotate(f'1/{len(route)}',(x,y),xytext=(4,4),textcoords='offset points',fontsize=8)
        elif n in (0,len(route)-1) and not (n==len(route)-1 and np.allclose(route[0],route[-1])):ax.annotate(f'{n+1}',(x,y),xytext=(4,4),textcoords='offset points',fontsize=8)
    ax.scatter(0,0,marker='^',color='#283e4f',s=25,zorder=5)
    ax.annotate('O',(0,0),xytext=(-12,0),textcoords='offset points',fontsize=9)
    ax.set(xlim=(-1,8.5),ylim=(-5,5),aspect='equal',xlabel='X / m',ylabel='Y / m',title=title);ax.grid(alpha=.17)

def main():
    p=argparse.ArgumentParser();p.add_argument('--root',required=True,type=Path);p.add_argument('--map',required=True,type=Path);p.add_argument('--fov',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
    sys.path[:0]=[str(a.root/'deployment/board_trials_4x4/common/uav_board_trials/scripts'),str(a.root/'patrol_uav_ws-patrol_planner/src/uav_mission/src'),str(a.root/'patrol_uav_ws-patrol_planner/src/uav_high_view/src')]
    from trial_config import load_trial_settings,apply_site_profile
    from trial_low_coverage import normalize_coverage_site,prepare_low_coverage,coverage_waypoints
    site=yaml.safe_load(a.map.read_text(encoding='utf-8'));cal=json.loads(a.fov.read_text());unit=np.array(cal['footprint_bounds'])/cal['lens_agl'];target=np.array(site['target_bounds'])
    font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc';font_manager.fontManager.addfont(font);plt.rcParams.update({'font.family':font_manager.FontProperties(fname=font).get_name(),'axes.unicode_minus':False,'font.size':10})
    routes=[('old_high_rectangle','原四角矩形（降高对照）',np.array(site['survey_xy'],float),[1.8,1.9,2.6])]
    for pattern,name in [('lawnmower','短牛耕'),('nested_rectangles','回字形')]:
        settings=load_trial_settings(a.root,'10_low_coverage_full');apply_site_profile(settings,normalize_coverage_site(site));prepare_low_coverage(settings,site_provided=True,pattern=pattern)
        points,source=coverage_waypoints(settings,settings['flight_area']);routes.append((pattern,name,np.array(points,float),[1.8,1.9]))
    metrics=dict(scope='Calibrated K/D inscribed rectangle. Level, fixed heading, flat plane, continuous completed route; no obstacle, frame-rate, recognition or actual-flight claim.',target_bounds=target.tolist(),routes=[])
    fig,axes=plt.subplots(1,3,figsize=(16,8));fig.subplots_adjust(left=.045,right=.985,bottom=.18,top=.82,wspace=.23)
    for ax,(key,name,route,heights) in zip(axes,routes):
        length=float(np.linalg.norm(np.diff(route,axis=0),axis=1).sum());dirs=np.diff(route,axis=0);turns=sum(abs(u[0]*v[1]-u[1]*v[0])>1e-6 or np.dot(u,v)<0 for u,v in zip(dirs[:-1],dirs[1:]))
        row=dict(pattern=key,waypoints=route.tolist(),length_m=length,entry_from_origin_m=float(np.linalg.norm(route[0])),heading_changes=int(turns),distance_only_seconds_at_0_5=length/.5,distance_only_seconds_at_1_2=length/1.2,heights=[])
        for h in heights:
            fp=unit*(h-.16);row['heights'].append(dict(fc_agl=h,lens_agl=h-.16,footprint_bounds=fp.tolist(),footprint_xy=[float(fp[1]-fp[0]),float(fp[3]-fp[2])],coverage=evaluate(route,fp,target)))
        metrics['routes'].append(row);first=row['heights'][0];c=first['coverage']['point']['percent']
        draw(ax,route,np.array(first['footprint_bounds']),target,f'{name}：FC {heights[0]:.1f}m\n路长{length:.1f}m · {len(route)}点 · 转向{turns}次\nFOV扫过地面 {c:.1f}%')
    fig.suptitle('同一现场范围、同FC 1.8m：三种路线的理论FOV覆盖',fontsize=18,weight='bold',y=.95)
    fig.text(.045,.07,'橙线：名义搜索航线；浅蓝：FOV沿路线扫过的地面；紫框：原地图target_bounds，7.9×9.6=75.84m²；白色：未覆盖。\n不扣靶板尺寸，不要求靶板完整入镜。使用标定K/D内接视场，固定航向、水平机体、连续扫描；不代表识别召回率。\n路长不含起飞接入、绕树、投递与尾程；模型未计树木遮挡、姿态、采样间隔与提前中断。',fontsize=10,color='#42535e')
    a.output.mkdir(parents=True,exist_ok=True);fig.savefig(a.output/'route_comparison.png',dpi=160);fig.savefig(a.output/'route_comparison.pdf');plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(16,8));fig.subplots_adjust(left=.045,right=.985,bottom=.16,top=.82,wspace=.23)
    for ax,row,name in zip(axes,metrics['routes'],['原四边形','短牛耕','回字形']):
        h=row['heights'][1];draw(ax,np.array(row['waypoints']),np.array(h['footprint_bounds']),target,f'{name}：FC1.9m / 镜头1.74m\nFOV扫过地面 {h["coverage"]["point"]["percent"]:.1f}%')
    fig.suptitle('同FC 1.9m：三种路线的理论FOV覆盖',fontsize=17,weight='bold',y=.94);fig.text(.07,.06,'同样以75.84m²原地图范围为分母，仅计算FOV扫地面积。1.9m不代表可越过约1.9m树冠；实体避障仍需动态验证。',fontsize=10)
    fig.savefig(a.output/'route_1p9.png',dpi=160);fig.savefig(a.output/'route_1p9.pdf');plt.close(fig)
    (a.output/'route_comparison.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(metrics,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
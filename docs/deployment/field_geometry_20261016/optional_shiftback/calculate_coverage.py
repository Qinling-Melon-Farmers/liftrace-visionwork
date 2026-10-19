from pathlib import Path
import ast,json,csv,math
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Patch
from matplotlib.lines import Line2D
from matplotlib import font_manager

ROOT=Path('/home/xhj/liftrace-worktrees/r2026-board-vision-tests')
OUT=Path(__file__).resolve().parent
model_path=ROOT/'docs/verification/competition_video_20261015/coverage.py'
archive_path=ROOT/'docs/verification/competition_video_20261015/fov_metrics.json'
sweep_path=ROOT/'docs/verification/low_coverage_backup_20261015/compare_routes.py'
cal_path=ROOT/'docs/verification/flight_1014_0425_20261014/bridge_metrics.json'
cal=json.loads(cal_path.read_text(encoding='utf-8'))
# Execute the original numerical K/D rectangle construction, without its plotting or writes.
src=model_path.read_text(encoding='utf-8')
exec(compile(src[src.index('K = np.array'):src.index('rows = []')],str(model_path),'exec'),globals())
unit=base*lo
arch=json.loads(archive_path.read_text(encoding='utf-8'))
np.testing.assert_allclose(unit*2,arch['heights'][0]['D_inscribed_bounds'],rtol=0,atol=1e-12)
# Reuse original axis-aligned sweep and exact rectangle-cell union calculation.
tree=ast.parse(sweep_path.read_text(encoding='utf-8'))
nodes=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ('swept_rectangles','union_area')]
exec(compile(ast.Module(body=nodes,type_ignores=[]),str(sweep_path),'exec'),globals())
domain=np.array([-1.34,6.56,-4.85,4.85])
routes={
'high_rectangle':('高位矩形',2.6,np.array([[1,-3.5],[5.5,-3.5],[5.5,3.5],[1,3.5],[1,-3.5]])),
'lawnmower':('三线牛耕',1.8,np.array([[1,-4.1],[1,4.1],[3.6,4.1],[3.6,-4.1],[6.2,-4.1],[6.2,4.1]])),
'nested_rectangles':('回字形',1.8,np.array([[1,-4.1],[1,4.1],[6.2,4.1],[6.2,-4.1],[1,-4.1],[2.8,-4.1],[2.8,-3.2],[2.8,3.2],[4.4,3.2],[4.4,-3.2],[2.8,-3.2]]))
}
# Translate every low-route point rigidly; preserve production traversal order.
for key in ('lawnmower','nested_rectangles'):
    name,fc,route=routes[key]
    routes[key]=(name,fc,np.round(route+np.array([-1.0,0.0]),10))
before_shift=json.loads((OUT.parent/'before_shift/metrics.json').read_text(encoding='utf-8'))
def missing(rects,d):
    xs=sorted({d[0],d[1],*(max(d[0],min(d[1],x)) for r in rects for x in r[:2])})
    ys=sorted({d[2],d[3],*(max(d[2],min(d[3],y)) for r in rects for y in r[2:])})
    strips=[]
    for left,right in zip(xs[:-1],xs[1:]):
        run=None
        for bottom,top in zip(ys[:-1],ys[1:]):
            covered=any(r[0]<=(left+right)/2<=r[1] and r[2]<=(bottom+top)/2<=r[3] for r in rects)
            if not covered:
                if run is None:run=[left,right,bottom,top]
                else:run[3]=top
            elif run is not None:strips.append(run);run=None
        if run is not None:strips.append(run)
    merged=[]
    for r in strips:
        match=next((q for q in merged if abs(q[1]-r[0])<1e-10 and abs(q[2]-r[2])<1e-10 and abs(q[3]-r[3])<1e-10),None)
        if match is None:merged.append(r.copy())
        else:match[1]=r[1]
    return [dict(bounds_xyxy=[r[0],r[2],r[1],r[3]],area_m2=(r[1]-r[0])*(r[3]-r[2])) for r in merged]
results=[]
for key,(name,fc,route) in routes.items():
    for fc in ([2.6] if key=='high_rectangle' else [1.8,1.9]):
        lens=fc-.16;fp=unit*lens
        rects=swept_rectangles(route,fp,0.)
        envelope=np.array([min(r[0] for r in rects),max(r[1] for r in rects),min(r[2] for r in rects),max(r[3] for r in rects)])
        total=union_area(rects,envelope)['covered_m2'];inside=union_area(rects,domain);gaps=missing(rects,domain)
        assert abs(sum(g['area_m2'] for g in gaps)-inside['uncovered_m2'])<1e-9
        row=dict(key=key,name=name,fc_agl_m=fc,lens_agl_m=lens,route=route.tolist(),route_length_m=float(np.linalg.norm(np.diff(route,axis=0),axis=1).sum()),footprint_bounds=fp.tolist(),footprint_xy_m=[fp[1]-fp[0],fp[3]-fp[2]],swept_union_area_m2=total,intersection_area_m2=inside['covered_m2'],coverage_percent=inside['percent'],uncovered_area_m2=inside['uncovered_m2'],outside_area_m2=total-inside['covered_m2'],missing_rectangles=gaps,sweep_rectangles=rects)
        row['sweep_envelope_bounds']=envelope.tolist()
        row['boundary_margin_m']=dict(x_negative=float(domain[0]-envelope[0]),x_positive=float(envelope[1]-domain[1]),y_negative=float(domain[2]-envelope[2]),y_positive=float(envelope[3]-domain[3]))
        row['translation_xy_m']=[0.,0.] if key=='high_rectangle' else [-1.,0.]
        previous=next(v for v in before_shift['results'] if v['key']==key and v['fc_agl_m']==fc)
        np.testing.assert_allclose(route,np.array(previous['route'])+np.array(row['translation_xy_m']),rtol=0,atol=1e-12)
        assert abs(row['route_length_m']-previous['route_length_m'])<1e-10
        assert abs(row['swept_union_area_m2']-previous['swept_union_area_m2'])<1e-10
        row['before_shift_coverage_percent']=previous['coverage_percent']
        row['before_shift_intersection_area_m2']=previous['intersection_area_m2']
        row['coverage_gain_percentage_points']=row['coverage_percent']-previous['coverage_percent']
        results.append(row)
font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(font)
plt.rcParams.update({'font.family':font_manager.FontProperties(fname=font).get_name(),'axes.unicode_minus':False,'font.size':10,'svg.fonttype':'path'})
def draw(ax,r):
    d=domain
    ax.add_patch(Rectangle((d[0],d[2]),d[1]-d[0],d[3]-d[2],fc='#ffe2df',ec='none',hatch='///'))
    for q in r['sweep_rectangles']:
        ax.add_patch(Rectangle((q[0],q[2]),q[1]-q[0],q[3]-q[2],fc='#dfe9f1',ec='none',alpha=.35))
        l,rr,b,t=max(q[0],d[0]),min(q[1],d[1]),max(q[2],d[2]),min(q[3],d[3])
        if l<rr and b<t:ax.add_patch(Rectangle((l,b),rr-l,t-b,fc='#a9d9e5',ec='none'))
    # Restore exact uncovered cells after drawing outside translucent sweeps.
    for g in r['missing_rectangles']:
        l,b,rr,t=g['bounds_xyxy']
        ax.add_patch(Rectangle((l,b),rr-l,t-b,fc='#ffe2df',ec='#d99b95',lw=.3,hatch='///'))
    ax.add_patch(Rectangle((d[0],d[2]),d[1]-d[0],d[3]-d[2],fill=False,ec='#283e50',lw=1.5))
    p=np.array(r['route']);ax.plot(p[:,0],p[:,1],'-o',color='#c56b14',ms=3,lw=1.6)
    for a,b in zip(p[:-1],p[1:]):
        ax.annotate('',xy=a+.64*(b-a),xytext=a+.45*(b-a),arrowprops={'arrowstyle':'->','color':'#9b4f09','lw':1.2})
    ax.scatter([p[0,0]],[p[0,1]],s=60,marker='s',facecolors='none',edgecolors='#8f4409',zorder=8)
    ax.scatter([0],[0],marker='+',color='black',s=35)
    ax.set(xlim=(-1.65,8.65),ylim=(-5.3,5.3),aspect='equal',xlabel='X / m（负侧：机体后方；正侧：机头）',ylabel='Y / m')
    ax.text(.015,.98,('可选 X−1 m（非默认）；后方在图左' if r['key']!='high_rectangle' else '高位不变；图左为机体后方 X 负侧'),transform=ax.transAxes,va='top',fontsize=8,color='#8a403b',bbox=dict(facecolor='white',edgecolor='none',alpha=.85),zorder=10)
    ax.set_xticks(np.arange(-1,9));ax.set_yticks(np.arange(-5,6));ax.grid(alpha=.16)
    ax.set_title(f"{r['name']}  FC {r['fc_agl_m']:.1f} / 镜头 {r['lens_agl_m']:.2f} m\n覆盖 {r['coverage_percent']:.2f}% = {r['intersection_area_m2']:.3f} m²\n漏区 {r['uncovered_area_m2']:.3f} m² · 路长 {r['route_length_m']:.1f} m",fontsize=11)
legend=[Patch(fc='#a9d9e5',label='区域内已扫过'),Patch(fc='#ffe2df',hatch='///',label='区域内漏区'),Patch(fc='#eaf0f5',label='区域外扫过'),Line2D([],[],color='#c56b14',marker='o',label='当前路线（方框为起点）')]
def finish(fig,name):
    fig.legend(handles=legend,loc='lower center',bbox_to_anchor=(.5,.045),ncol=4,frameon=False)
    fig.text(.5,.012,'区域 X[-1.34,6.56] / Y[-4.85,4.85]，面积76.63 m²；仅扫地，不扣靶板尺寸。\nK/D数值内接模型，平地、水平、固定航向、连续完整航线。',ha='center',fontsize=8)
    fig.savefig(OUT/(name+'.png'),dpi=170);fig.savefig(OUT/(name+'.pdf'));plt.close(fig)
for key in routes:
    rows=[r for r in results if r['key']==key]
    fig,axes=plt.subplots(1,len(rows),figsize=(7*len(rows),8),squeeze=False)
    for ax,r in zip(axes.ravel(),rows):draw(ax,r)
    fig.subplots_adjust(left=.08,right=.97,top=.86,bottom=.14,wspace=.20)
    fig.suptitle('可选后移1 m（非默认）：低位平移，高位不变',fontsize=16)
    finish(fig,key)
fig,axes=plt.subplots(2,3,figsize=(18,14))
for j,fc in enumerate([1.8,1.9]):
    for i,key in enumerate(routes):
        row=next(r for r in results if r['key']==key and r['fc_agl_m']==(2.6 if i==0 else fc))
        draw(axes[j,i],row)
fig.subplots_adjust(left=.045,right=.98,top=.88,bottom=.12,wspace=.20,hspace=.40)
fig.suptitle('可选后移1 m（非默认）｜上排低位1.8 m，下排1.9 m；高位不变',fontsize=18)
finish(fig,'comparison')
data=dict(before_shift_data='../before_shift/metrics.json',variant='optional_shiftback',enabled_by_default=False,route_update='Low routes translated rigidly by [-1,0] m; high route and corridor unchanged',search_bounds=domain.tolist(),search_area_m2=76.63,model_source=str(model_path),calibration_source=str(cal_path),sweep_source=str(sweep_path),K=cal['K'],D=cal['D'],fov_angles_deg=fovs,inscribed_scale=lo,unit_bounds=unit.tolist(),archived_model_reproduction_max_error=float(np.max(np.abs(unit*2-np.array(arch['heights'][0]['D_inscribed_bounds'])))),conditions=['K/D numerical inscribed rectangle, sampled boundary; not exact maximal rectangle','level downward camera, fixed original nominal yaw, flat floor','continuous translation along all route segments; connectors included','no full-target constraint; no obstacle occlusion, attitude, FPS, trajectory deviation or mission interruption model','no entry from origin or exit trajectory included; no measured coverage claim'],results=results)
(OUT/'metrics.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
fields=['name','fc_agl_m','lens_agl_m','route_length_m','swept_union_area_m2','intersection_area_m2','coverage_percent','uncovered_area_m2','outside_area_m2']
with (OUT/'comparison.csv').open('w',encoding='utf-8-sig',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(results)
lines=['2026-10-16 新搜索区域扫地覆盖计算','区域：X[-1.34,6.56], Y[-4.85,4.85]，7.9×9.7=76.63 m²。',
'沿用前次K/D数值畸变内接模型，非1.75/.985近似。它通过每边1001点、60轮二分求一致缩放的保守内接矩形，不是最大内接矩形解析解，也不是覆盖实测。',
'复现脚本：'+str(model_path),'内参来源：'+str(cal_path),'扫掠/并集函数：'+str(sweep_path),
'宽边沿X，固定原航向；保留主点不居中带来的非对称边界。覆盖按连续整条路线的矩形扫掠并集与区域相交计算，面积不是栅格估算。',
'可选方案optional_shiftback（非默认，仅选用时生效）：牛耕/回字全部点整体X减1.0m，Y、点序及连接方式不变，不做机动路径变形；高位不动，走廊不动。本任务不修改YAML。',
'回字方向保持生产11点顺序：平移后外圈先沿X=0向Y正侧，内圈先沿X=1.8向Y正侧；经(1.8,-4.1)连接。',
'方位说明：图左是机体后方X负侧，不是现场Y左墙；旧低位的图左漏带已被平移消除。图上方为Y正侧，1.8m剩余漏区在Y正边界。',
'原默认未平移结果在上级目录，历史对照在../before_shift/。余量为扫掠包络超出边界的距离：正数为富余，负数为欠覆盖；非机体安全净空。',
'起飞接入、离开路线不计；无靶板尺寸/完整靶心约束；不计遮挡、倾斜、帧率、航迹偏差和提前中断。',
'图：high_rectangle.png；lawnmower.png；nested_rectangles.png；comparison.png。另附PDF、CSV、JSON和本计算脚本。','']
for r in results:
    lines.append(f"{r['name']} FC={r['fc_agl_m']:.1f} 镜头={r['lens_agl_m']:.2f}：union={r['swept_union_area_m2']:.6f} m²；交集={r['intersection_area_m2']:.6f} m²；覆盖={r['coverage_percent']:.6f}%；漏区={r['uncovered_area_m2']:.6f} m²。")
    lines.append('路线：'+str(r['route']))
    lines.append('FOV边界余量(m)：'+str(r['boundary_margin_m']))
    lines.append(f"平移前覆盖 {r['before_shift_coverage_percent']:.6f}%；当前提升 {r['coverage_gain_percentage_points']:.6f} 个百分点。")
    for g in r['missing_rectangles']:
        l,b,rr,t=g['bounds_xyxy'];lines.append(f"  漏区无重叠矩形：X[{l:.6f},{rr:.6f}] Y[{b:.6f},{t:.6f}]，面积{g['area_m2']:.6f} m²")
lines+=['','校验：原归档镜头2m结果复现一致；所有漏区矩形面积之和等于76.63减交集面积。','检查：低位每一点均为原点+[-1,0]，高位逐点不变，路长和未裁剪union面积不变。仅更新交付脚本/图/报告，未修改生产源码或YAML，未提交、未仿真、未接板。']
(OUT/'report.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')
for r in results:print(json.dumps({k:r[k] for k in fields},ensure_ascii=False))
print('GAPS',json.dumps({r['key']+'_'+str(r['fc_agl_m']):r['missing_rectangles'] for r in results},ensure_ascii=False))

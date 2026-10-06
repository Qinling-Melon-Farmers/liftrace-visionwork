"""Axis-aligned survey planning and ideal pinhole coverage; no ROS dependency."""
import math

PATTERNS=('rectangle','snake2','snake3')
FIELDS={'bounds','inset','pattern','camera_agl','fc_to_camera_z','fov_x_deg','fov_y_deg'}

def plan(spec):
    if not isinstance(spec,dict) or set(spec)!=FIELDS:
        raise ValueError('搜索设计需bounds/inset/pattern/camera_agl/fc_to_camera_z/fov_x_deg/fov_y_deg')
    def number(v):return type(v) in (int,float) and math.isfinite(v)
    bounds=spec['bounds']
    if not isinstance(bounds,list) or len(bounds)!=4 or not all(number(v) for v in bounds):
        raise ValueError('搜索区需[xmin,xmax,ymin,ymax]四个有限数值')
    x0,x1,y0,y1=bounds
    for key in FIELDS-{'bounds','pattern'}:
        if not number(spec[key]):raise ValueError('无效数值: '+key)
    inset=spec['inset'];pattern=spec['pattern'];height=spec['camera_agl'];offset=spec['fc_to_camera_z']
    if pattern not in PATTERNS:raise ValueError('未知搜索路线')
    if not (x0<x1 and y0<y1 and 0<=inset<min(x1-x0,y1-y0)/2):raise ValueError('搜索区或内收距离无效')
    if not (0<height<=4 and abs(offset)<=1 and 0<height-offset<=4):raise ValueError('镜头/FC离地高度无效')
    if not all(1<spec[k]<179 for k in ('fov_x_deg','fov_y_deg')):raise ValueError('FOV必须为1至179度之间的有效角度')
    xa,xb=x0+inset,x1-inset;ya,yb=y0+inset,y1-inset
    lanes=[xa,xb] if pattern!='snake3' else [xa,(xa+xb)/2,xb]
    if pattern=='rectangle':route=[[xa,ya],[xb,ya],[xb,yb],[xa,yb],[xa,ya]]
    else:
        route=[]
        for i,x in enumerate(lanes):route.extend([[x,ya if i%2==0 else yb],[x,yb if i%2==0 else ya]])
    fx=2*height*math.tan(math.radians(spec['fov_x_deg'])/2)
    fy=2*height*math.tan(math.radians(spec['fov_y_deg'])/2)
    footprints=[]
    for a,b in zip(route,route[1:]):
        rect=[max(x0,min(a[0],b[0])-fx/2),min(x1,max(a[0],b[0])+fx/2),
              max(y0,min(a[1],b[1])-fy/2),min(y1,max(a[1],b[1])+fy/2)]
        if rect[0]<rect[1] and rect[2]<rect[3]:footprints.append(rect)
    # Exact union/complement of swept rectangles, no raster-resolution optimism.
    xs=sorted({x0,x1,*[v for r in footprints for v in r[:2]]})
    ys=sorted({y0,y1,*[v for r in footprints for v in r[2:]]})
    blind=[];uncovered=0.
    for left,right in zip(xs,xs[1:]):
        for bottom,top in zip(ys,ys[1:]):
            midx=(left+right)/2;midy=(bottom+top)/2
            if not any(r[0]<=midx<=r[1] and r[2]<=midy<=r[3] for r in footprints):
                blind.append([left,right,bottom,top]);uncovered+=(right-left)*(top-bottom)
    area=(x1-x0)*(y1-y0);spacing=(xb-xa)/(len(lanes)-1)
    warnings=['理想几何覆盖：水平下视、固定航向、平地、完整沿设计线飞完；未计遮挡、倾斜、畸变、模糊、提前中断或避障偏航。',
              'FOV按任务X/Y方向填写，不能不核对安装方向就直接套用图像宽/高视角；本图不评估完整1m靶板可见率。']
    if uncovered>1e-8:warnings.append('存在盲区；请结合靶板尺寸和摆位核对，不能按有航线即全覆盖。')
    if pattern!='rectangle' and spacing>fx:warnings.append('扫描线间距大于X方向视场，中间可能留出条带盲区。')
    return dict(pattern=pattern,bounds=bounds,route=route,search_bounds=bounds,
                high_agl=height-offset,camera_agl=height,fc_to_camera_z=offset,
                footprint_xy=[fx,fy],footprints=footprints,blind_rectangles=blind,
                area_m2=area,covered_m2=area-uncovered,coverage_percent=100*(area-uncovered)/area,
                lane_spacing_m=spacing,route_length_m=sum(math.dist(a,b) for a,b in zip(route,route[1:])),
                warnings=warnings)

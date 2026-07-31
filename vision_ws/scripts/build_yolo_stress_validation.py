#!/usr/bin/env python3
import argparse,csv,json,random,shutil
from pathlib import Path
import cv2,numpy as np,yaml
NAMES=["bridge","panzer","pillbox","tent","tank","red_cross"]
CONDS=["motion_blur","rotation","local_crop","small_target","strong_light","low_light","occlusion","multi_target"]
EXT={".jpg",".jpeg",".png",".bmp",".webp"}

def labels(p):
    out=[]
    for line in p.read_text(encoding="utf-8").splitlines():
        f=line.split()
        if not f: continue
        if len(f)!=5: raise ValueError(f"bad label: {p}: {line}")
        c=int(f[0]); v=list(map(float,f[1:]))
        if c<0 or c>=6 or not(0<=v[0]<=1 and 0<=v[1]<=1 and 0<v[2]<=1 and 0<v[3]<=1):
            raise ValueError(f"invalid label: {p}: {line}")
        out.append((c,*v))
    return out

def px(ls,w,h):
    out=[]
    for c,cx,cy,bw,bh in ls:
        x1=max(0,(cx-bw/2)*w); y1=max(0,(cy-bh/2)*h)
        x2=min(w,(cx+bw/2)*w); y2=min(h,(cy+bh/2)*h)
        if x2>x1 and y2>y1: out.append((c,x1,y1,x2,y2))
    return out

def norm(bs,w,h):
    out=[]
    for c,x1,y1,x2,y2 in bs:
        x1=max(0,min(w,x1)); y1=max(0,min(h,y1)); x2=max(0,min(w,x2)); y2=max(0,min(h,y2))
        if x2>x1 and y2>y1: out.append((c,(x1+x2)/(2*w),(y1+y2)/(2*h),(x2-x1)/w,(y2-y1)/h))
    return out

def write_labels(p,ls):
    p.write_text("".join(f"{c} {x:.7f} {y:.7f} {w:.7f} {h:.7f}\n" for c,x,y,w,h in ls),encoding="utf-8")

def blur(im,r):
    k=r.choice([9,13,17,21]); ker=np.zeros((k,k),np.float32); ker[k//2,:]=1/k
    return cv2.filter2D(im,-1,ker)

def rotate(im,ls,r):
    h,w=im.shape[:2]; a=r.choice([-28,-18,18,28]); m=cv2.getRotationMatrix2D((w/2,h/2),a,1)
    out=cv2.warpAffine(im,m,(w,h),borderMode=cv2.BORDER_REFLECT_101); bs=[]
    for c,x1,y1,x2,y2 in px(ls,w,h):
        q=np.array([[x1,y1,1],[x2,y1,1],[x2,y2,1],[x1,y2,1]],np.float32)@m.T
        bs.append((c,q[:,0].min(),q[:,1].min(),q[:,0].max(),q[:,1].max()))
    return out,norm(bs,w,h)

def crop(im,ls,r):
    h,w=im.shape[:2]; bs=px(ls,w,h); x1=min(q[1] for q in bs); y1=min(q[2] for q in bs); x2=max(q[3] for q in bs); y2=max(q[4] for q in bs)
    tw,th=max(2,x2-x1),max(2,y2-y1); cw=min(w,max(tw+2,tw*r.uniform(1.4,2))); ch=min(h,max(th+2,th*r.uniform(1.4,2)))
    cx=(x1+x2)/2+r.uniform(-.12,.12)*tw; cy=(y1+y2)/2+r.uniform(-.12,.12)*th
    left=int(max(0,min(w-cw,cx-cw/2))); top=int(max(0,min(h-ch,cy-ch/2))); right=int(left+cw); bottom=int(top+ch)
    out=cv2.resize(im[top:bottom,left:right],(w,h)); sx=w/(right-left); sy=h/(bottom-top)
    return out,norm([(c,(a-left)*sx,(b-top)*sy,(d-left)*sx,(e-top)*sy) for c,a,b,d,e in bs],w,h)

def small(im,ls,r):
    h,w=im.shape[:2]; s=r.uniform(.42,.62); q=cv2.resize(im,(int(w*s),int(h*s)),interpolation=cv2.INTER_AREA); out=cv2.GaussianBlur(im,(0,0),18)
    left=(w-q.shape[1])//2; top=(h-q.shape[0])//2; out[top:top+q.shape[0],left:left+q.shape[1]]=q
    return out,norm([(c,left+a*s,top+b*s,left+d*s,top+e*s) for c,a,b,d,e in px(ls,w,h)],w,h)

def light(im,f,g):
    lut=np.array([np.clip(((i/255)**g)*255*f,0,255) for i in range(256)],np.uint8); return cv2.LUT(im,lut)

def block(im,ls,r):
    out=im.copy(); h,w=im.shape[:2]
    for _,x1,y1,x2,y2 in px(ls,w,h):
        ow=max(2,(x2-x1)*r.uniform(.25,.45)); oh=max(2,(y2-y1)*r.uniform(.25,.45)); left=int(r.uniform(x1,max(x1,x2-ow))); top=int(r.uniform(y1,max(y1,y2-oh))); right=min(w,int(left+ow)); bottom=min(h,int(top+oh))
        roi=im[int(y1):max(int(y1)+1,int(y2)),int(x1):max(int(x1)+1,int(x2))]; col=tuple(map(int,roi.mean(axis=(0,1)))); cv2.rectangle(out,(left,top),(right,bottom),col,-1)
    return out

def multi(im,ls,extras):
    h,w=im.shape[:2]; tw,th=w//2,h//2; out=np.zeros_like(im); result=[]
    for i,(tile,tls) in enumerate([(im,ls)]+extras):
        q=cv2.resize(tile,(tw,th),interpolation=cv2.INTER_AREA); ox,oy=(i%2)*tw,(i//2)*th; out[oy:oy+th,ox:ox+tw]=q
        for c,x,y,bw,bh in tls: result.append((c,(ox+x*tw)/w,(oy+y*th)/h,bw*tw/w,bh*th/h))
    return out,result

def yml(p,root,val):
    p.write_text(yaml.safe_dump({"path":str(root),"train":"images/train","val":val,"nc":6,"names":{i:n for i,n in enumerate(NAMES)}},sort_keys=False),encoding="utf-8")

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--source-dataset",type=Path,required=True); ap.add_argument("--output-dataset",type=Path,required=True); ap.add_argument("--seed",type=int,default=20260714); ap.add_argument("--overwrite",action="store_true"); a=ap.parse_args()
    src=a.source_dataset.resolve(); out=a.output_dataset.resolve(); idir=src/"images"/"val"; ldir=src/"labels"/"val"
    if not idir.is_dir() or not ldir.is_dir(): raise SystemExit(f"source must contain images/val and labels/val: {src}")
    if out.exists():
        if not a.overwrite: raise SystemExit(f"output exists: {out}")
        shutil.rmtree(out)
    (out/"images"/"train").mkdir(parents=True); (out/"labels"/"train").mkdir(parents=True)
    for c in CONDS: (out/"images"/"val"/c).mkdir(parents=True); (out/"labels"/"val"/c).mkdir(parents=True)
    paths=sorted(p for p in idir.iterdir() if p.suffix.lower() in EXT); r=random.Random(a.seed); data=[]
    for p in paths:
        ls=labels(ldir/(p.stem+".txt"))
        if not ls: raise SystemExit(f"empty labels unsupported: {p}")
        im=cv2.imread(str(p)); 
        if im is None: raise SystemExit(f"cannot read: {p}")
        data.append((p,im,ls))
    rows=[]
    for i,(p,im,ls) in enumerate(data):
        extras=[(data[(i+j)%len(data)][1],data[(i+j)%len(data)][2]) for j in range(1,4)]
        ts={"motion_blur":(blur(im,r),ls),"rotation":rotate(im,ls,r),"local_crop":crop(im,ls,r),"small_target":small(im,ls,r),"strong_light":(light(im,1.18,.82),ls),"low_light":(light(im,.72,1.18),ls),"occlusion":(block(im,ls,r),ls),"multi_target":multi(im,ls,extras)}
        for c,(tim,tls) in ts.items():
            name=c+"__"+p.stem+".jpg"; ip=out/"images"/"val"/c/name; lp=out/"labels"/"val"/c/(Path(name).stem+".txt")
            if not cv2.imwrite(str(ip),tim,[int(cv2.IMWRITE_JPEG_QUALITY),95]): raise SystemExit(f"cannot write: {ip}")
            write_labels(lp,tls); rows.append({"condition":c,"source":p.name,"image":str(ip.relative_to(out)),"label":str(lp.relative_to(out)),"objects":len(tls)})
    yml(out/"data.yaml",out,"images/val")
    for c in CONDS: yml(out/f"condition_{c}.yaml",out,"images/val/"+c)
    with (out/"manifest.csv").open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=["condition","source","image","label","objects"]); w.writeheader(); w.writerows(rows)
    meta={"source_dataset":str(src),"source_images":len(data),"conditions":CONDS,"generated_images":len(rows),"seed":a.seed,"training_images":0,"purpose":"held-out synthetic stress validation; never use as training data"}
    (out/"metadata.json").write_text(json.dumps(meta,indent=2)+"\n",encoding="utf-8")
    (out/"README.md").write_text("# v5 stress validation set\n\nGenerated from the held-out v5 validation images. Eight conditions: motion blur, rotation, local crop, small target, strong light, low light, occlusion, and multi-target. Train directories are empty; this synthetic set is validation-only.\n",encoding="utf-8")
    print(json.dumps(meta,indent=2)); return 0
if __name__=="__main__": raise SystemExit(main())

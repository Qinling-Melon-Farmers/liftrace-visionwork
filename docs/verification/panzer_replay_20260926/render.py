"""Build a 1x recorded-flight / recomputed-perception comparison video."""
import argparse,bisect,json,math,subprocess
from pathlib import Path
import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

class Line:
    def __init__(self,rows,key="t"):
        self.rows=sorted(rows,key=lambda r:r[key]);self.ts=[r[key] for r in self.rows]
    def at(self,t):
        i=bisect.bisect_right(self.ts,t)-1
        return self.rows[i] if i>=0 else None
    def near(self,t,tolerance=.04):
        i=bisect.bisect_left(self.ts,t);ids=[j for j in (i-1,i) if 0<=j<len(self.ts)]
        if not ids:return None
        j=min(ids,key=lambda j:abs(self.ts[j]-t))
        return self.rows[j] if abs(self.ts[j]-t)<=tolerance else None

def main():
    ap=argparse.ArgumentParser();ap.add_argument("--source",type=Path,required=True)
    ap.add_argument("--recomputed",type=Path,required=True);ap.add_argument("--recorded",type=Path,required=True)
    ap.add_argument("--report",type=Path,required=True);a=ap.parse_args()
    data=json.loads((a.source/"data.json").read_text())
    recomputed=json.loads(a.recomputed.read_text());recorded=json.loads(a.recorded.read_text())
    frames=Line(data["frames"]);raw=Line(recomputed["yolo"],"stamp")
    mapped=Line(recomputed["mapped"],"stamp")
    memories={v:Line([r for r in recomputed["rows"] if r["variant"]==v]) for v in ("strict","gap1s")}
    phases=Line([dict(t=r["t"],m=json.loads(r["m"]["data"])) for r in data["rows"]["mission"]])
    odom=data["rows"]["odom"]
    motion=np.array([[r["t"],*[r["m"]["pose"]["pose"]["position"][k] for k in "xyz"]] for r in odom])
    speed=np.linalg.norm(np.column_stack([(np.interp(motion[:,0]+.25,motion[:,0],motion[:,k])-np.interp(motion[:,0]-.25,motion[:,0],motion[:,k]))/.5 for k in (1,2)]),axis=1)
    tracks=[]
    for source,title,variant in ((recorded,"Recorded RKNN + new gap","gap1s"),
                                (recomputed,"Image rerun / strict","strict"),
                                (recomputed,"Image rerun / gap 1s","gap1s")):
        ts=[r["t"] for r in source["rows"] if r["variant"]==variant and any(
            c["class_name"]=="panzer" and c["admitted"] for c in r["targets"])]
        tracks.append((title,ts))
    colors=[(190,180,140),(100,190,255),(140,240,110)]
    W,H,fps=1600,900,10
    video=a.source/"panzer_comparison.mp4"
    cmd=["ffmpeg","-y","-v","error","-f","rawvideo","-pixel_format","bgr24","-video_size",f"{W}x{H}",
         "-framerate",str(fps),"-i","-","-an","-c:v","libx264","-preset","fast","-crf","20",
         "-pix_fmt","yuv420p","-movflags","+faststart",str(video)]
    encoder=subprocess.Popen(cmd,stdin=subprocess.PIPE)
    cache={};last_path=None;last_image=None;snaps={}
    def text(canvas,s,x,y,scale=.62,color=(230,235,240),thick=1):
        cv2.putText(canvas,s,(x,y),cv2.FONT_HERSHEY_SIMPLEX,scale,color,thick,cv2.LINE_AA)
    def draw(t):
        nonlocal last_path,last_image
        f=frames.at(t) or data["frames"][0];path=a.source/f["file"]
        if path!=last_path:
            last_image=cv2.imread(str(path));last_path=path
        image=last_image.copy();rh,rw=image.shape[:2]
        info=mapped.near(f["stamp"])
        detections=info["m"]["detections"] if info else []
        # All overlay coordinates refer to the displayed source image, not receipt time.
        for det in detections:
            if det["class_name"] not in ("panzer","red_cross","circle"):continue
            roi=det["roi"];x,y=roi["x_offset"],roi["y_offset"];w,h=roi["width"],roi["height"]
            color=(255,210,30) if det["class_name"]=="circle" else (40,245,100)
            cv2.rectangle(image,(x,y),(x+w,y+h),color,3)
            label=det["class_name"]+f" {det['class_confidence']:.2f}"
            cv2.putText(image,label,(max(0,x),max(26,y-8)),cv2.FONT_HERSHEY_SIMPLEX,.85,color,2,cv2.LINE_AA)
            if det["center_refined"]:
                pt=det["center_px"];cv2.drawMarker(image,(int(pt["x"]),int(pt["y"])),(255,210,30),cv2.MARKER_CROSS,22,3)
        canvas=np.full((H,W,3),(27,23,20),np.uint8)
        text(canvas,"PANZER | recorded flight + offline vision rerun",22,38,.9,thick=2)
        text(canvas,f"1x  t={t:5.1f}s / {data['duration']:.1f}s",1175,38,.65)
        canvas[70:633,20:1020]=cv2.resize(image,(1000,563))
        text(canvas,f"Image source time {f['stamp']:.3f}s | PT YOLO + production ring/refiner + recorded TF",24,660,.55)
        for i,v in enumerate(("strict","gap1s")):
            y=85+i*195
            cv2.rectangle(canvas,(1040,y),(1580,y+178),(50,45,41),-1)
            text(canvas,"BEFORE: consecutive" if v=="strict" else "AFTER: gaps <= 1.0s",1055,y+29,.7,colors[i+1],2)
            r=memories[v].at(t);cs=[c for c in r["targets"] if c["class_name"]=="panzer"] if r else []
            if cs:
                c=max(cs,key=lambda c:(c["last_seen"],c["streak"]))
                fresh=bool(c["map_valid"] and 0<=t-c["last_seen"]<=.5)
                text(canvas,f"Panzer ID {c['id']} | hits {c['streak']} / 3",1055,y+61,.64)
                label="CONFIRMED" if c["state"]==2 else ("OBSERVING" if c["state"]==1 else "DETECTED")
                text(canvas,label+(" / current" if fresh else " / historical"),1055,y+91,.65,
                     (130,240,130) if fresh and c["state"]==2 else (160,175,195))
                text(canvas,f"map XY ({c['xy'][0]:.2f}, {c['xy'][1]:.2f}) m",1055,y+122,.62)
            else:text(canvas,"No panzer candidate",1055,y+77,.65)
            action=(r.get("suggested_class") or "") if r and r.get("suggested_action")=="APPROACH" and t-r["t"]<=.5 else "none"
            text(canvas,"Shadow APPROACH: "+action,1055,y+153,.59)
        state=phases.at(t)
        phase=state["m"] if state else {"phase":"not recorded yet"}
        z=np.interp(t,motion[:,0],motion[:,3]);v=np.interp(t,motion[:,0],speed)
        text(canvas,"Recorded phase: "+phase["phase"],1048,510,.66)
        text(canvas,f"FC local Z {z:.2f}m | XY speed {v:.2f}m/s",1048,545,.57)
        text(canvas,"37.53s: recorded return started",1048,580,.58,(130,180,255))
        text(canvas,"No new flight / no actuator commands",1048,615,.56,(160,190,240))
        text(canvas,"Only one outbound panzer hit; a longer confirmation gap cannot invent two more.",22,701,.64)
        text(canvas,"Green box=class  Cyan=ring/center | XY is a vision estimate, not surveyed target truth.",22,731,.57)
        # Valid candidate evidence on a common bag clock.
        x0,x1=360,1540
        for j,(label,ts) in enumerate(tracks):
            y=778+j*31
            text(canvas,label,22,y+5,.54,colors[j])
            cv2.line(canvas,(x0,y),(x1,y),(80,78,74),1)
            for sec in ts:
                x=x0+int(sec/data["duration"]*(x1-x0))
                cv2.line(canvas,(x,y-7),(x,y+7),colors[j],2)
            x=x0+int(t/data["duration"]*(x1-x0))
            cv2.circle(canvas,(x,y),4,(255,255,255),-1)
        text(canvas,"0s",x0,885,.5);text(canvas,"101.5s",x1-60,885,.5)
        return canvas
    try:
        for n in range(math.ceil(data["duration"]*fps)):
            t=n/fps;frame=draw(t);encoder.stdin.write(frame.tobytes())
            for sec,name in [(15.5,"outbound"),(52.4,"earlier_confirm"),(52.6,"return_confirm")]:
                if abs(t-sec)<.01:
                    cv2.imwrite(str(a.report/(name+".jpg")),frame,[cv2.IMWRITE_JPEG_QUALITY,92])
            if n%250==0:print("video",n,flush=True)
    finally:
        encoder.stdin.close()
    if encoder.wait()!=0:raise RuntimeError("video encode failed")
    fig,ax=plt.subplots(3,1,figsize=(12,7),sharex=True,layout="constrained")
    for i,(source,title) in enumerate(((recorded,"Recorded detections, recomputed downstream"),(recomputed,"Same camera samples re-inferred (PT)"))):
        for v,color in (("strict","#dd843d"),("gap1s","#268a7e")):
            rows=[r for r in source["rows"] if r["variant"]==v]
            xs=[];ys=[]
            for r in rows:
                cs=[c for c in r["targets"] if c["class_name"]=="panzer"]
                xs.append(r["t"]);ys.append(max((c["streak"] if c["map_valid"] else 0 for c in cs),default=0))
            ax[i].step(xs,ys,where="post",label=v,color=color)
        ax[i].axhline(3,color="#999",ls=":",label="3 hits")
        ax[i].set_ylabel("Current hit streak");ax[i].set_title(title);ax[i].legend(ncol=3)
    ax[2].plot(motion[:,0],motion[:,3],label="FC local Z (m)")
    ax[2].plot(motion[:,0],speed,label="XY speed (m/s)")
    ax[2].legend();ax[2].set_xlabel("Seconds from bag start (receipt time)")
    for axis in ax:
        axis.axvline(37.53,color="black",ls="--",lw=.8)
        axis.axvspan(37.53,57.67,alpha=.1,color="#456eaf")
        axis.set_xlim(10,60);axis.grid(alpha=.2)
    fig.savefig(a.report/"confirmation_timeline.png",dpi=160)
    plt.close(fig)
    print("video exported",video,flush=True)

if __name__=="__main__":main()

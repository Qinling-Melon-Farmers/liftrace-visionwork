import json,sys,cv2,numpy as np
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
h=Path(__file__).resolve().parents[3]
report=h/"docs/verification/panzer_replay_20260926"
sys.path.insert(0,str(report))
from threshold_probe import appearance
d=json.loads((h/"logs/panzer_threshold_probe/probe.json").read_text())
summary=json.loads((h/"logs/panzer_threshold_probe/summary.json").read_text())
raw=[r for r in d["rows"] if r["mode"]=="raw"]
r=min(raw,key=lambda r:abs(r["t"]-14.954))
panels=[]
for mode in ["raw","gamma065","clahe"]:
 row=next(a for a in d["rows"] if a["stamp_ns"]==r["stamp_ns"] and a["mode"]==mode)
 im=appearance(cv2.imread(row["image"]),mode)
 sx,sy=768/im.shape[1],432/im.shape[0]
 im=cv2.resize(im,(768,432))
 for det in row["detections"]:
  if det["class"]!="panzer" or det["confidence"]<.1:continue
  pts=[int(v*(sx if i%2==0 else sy)) for i,v in enumerate(det["xyxy"])]
  c=(0,230,70) if det["confidence"]>=.5 else (0,190,255)
  cv2.rectangle(im,(pts[0],pts[1]),(pts[2],pts[3]),c,2)
  cv2.putText(im,"panzer %.3f"%det["confidence"],(max(3,pts[0]),max(28,pts[1]-8)),
              cv2.FONT_HERSHEY_SIMPLEX,.65,c,2,cv2.LINE_AA)
 canvas=np.full((490,768,3),24,np.uint8);canvas[58:]=im
 label={"raw":"Original | below 0.50 cutoff","gamma065":"Gamma 0.65 | diagnosis only","clahe":"CLAHE | diagnosis only"}[mode]
 cv2.putText(canvas,label,(18,28),cv2.FONT_HERSHEY_SIMPLEX,.68,(255,255,255),1,cv2.LINE_AA)
 cv2.putText(canvas,"Same source image: t=%.3fs"%r["t"],(18,49),cv2.FONT_HERSHEY_SIMPLEX,.48,(200,200,200),1,cv2.LINE_AA)
 panels.append(canvas)
cv2.imwrite(str(report/"appearance_comparison.jpg"),np.hstack(panels),[cv2.IMWRITE_JPEG_QUALITY,90])
# Montage of all newly detected outbound boxes to permit visual inspection.
new=[]
for row in d["rows"]:
 if row["mode"] not in ("gamma065","clahe") or not 13.5<=row["t"]<=17.5:continue
 if not any(x["class"]=="panzer" and x["confidence"]>=.5 for x in row["detections"]):continue
 im=appearance(cv2.imread(row["image"]),row["mode"])
 sx,sy=480/im.shape[1],270/im.shape[0]
 im=cv2.resize(im,(480,270))
 for det in row["detections"]:
  if det["class"]!="panzer" or det["confidence"]<.5: continue
  x1,y1,x2,y2=[int(v*(sx if i%2==0 else sy)) for i,v in enumerate(det["xyxy"])]
  cv2.rectangle(im,(x1,y1),(x2,y2),(0,240,100),2)
  cv2.putText(im,"%.3f"%det["confidence"],(max(5,x1),max(y1-3,25)),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,240,100),2)
 canvas=np.full((310,480,3),24,np.uint8);canvas[40:]=im
 cv2.putText(canvas,"%s t=%.3f"%(row["mode"],row["t"]),(10,27),cv2.FONT_HERSHEY_SIMPLEX,.62,(255,255,255),1)
 new.append(canvas)
while len(new)%3:new.append(np.zeros_like(new[0]))
cv2.imwrite(str(report/"outbound_probe_boxes.jpg"),np.vstack([np.hstack(new[i:i+3]) for i in range(0,len(new),3)]))
fig,axes=plt.subplots(1,2,figsize=(12,4.7),layout="constrained")
for m,col in [("raw","#333333"),("gamma065","#bc6400"),("clahe","#007d8a")]:
 rows=sorted([r for r in d["rows"] if r["mode"]==m and 13.5<=r["t"]<=17.5],key=lambda r:r["t"])
 score=[max([v["confidence"] for v in r["detections"] if v["class"]=="panzer"]+[0]) for r in rows]
 axes[0].plot([r["t"] for r in rows],score,".-",label=m,color=col,lw=1.2)
 axes[0].axhline(.5,color="gray",linestyle="--",lw=.5) if m=="raw" else None
 axes[0].set(xlabel="Bag source-image time (s)",ylabel="Panzer confidence",title="Same 36 sampled outbound frames",ylim=(-.02,1.03))
 axes[0].legend()
 th=[.1,.2,.3,.4,.5,.6,.7]
 axes[1].plot(th,[summary["outbound"][m]["hits_by_threshold"][str(x)] for x in th],"o-",label=m,color=col)
axes[1].set(xlabel="YOLO confidence cutoff",ylabel="Frames with panzer output",title="Detector outputs, not confirmed targets",ylim=(0,12))
axes[1].legend()
for a in axes:a.grid(alpha=.2)
fig.savefig(report/"threshold_appearance.png",dpi=150)
(report/"threshold_metrics.json").write_text(json.dumps(summary,indent=2))
print("Created plots and comparison frames")
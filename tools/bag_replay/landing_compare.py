#!/usr/bin/env python3
"""Compare recorded vision with offline H-chain results on the SAME camera frames."""
import argparse
import json
import math
from pathlib import Path
import subprocess

from bag_replay import export, plain, stamp, Frames, Timeline


def prepare(a):
    import rosbag
    a.topics=None
    export(a)
    out=Path(a.out)
    source=json.loads((out/"data.json").read_text())
    rows={}
    with rosbag.Bag(a.reinferred) as bag:
        for topic,msg,receipt in bag.read_messages():
            data=plain(msg)
            rows.setdefault(topic,[]).append(dict(t=receipt.to_sec()-source["start"],
                stamp=stamp(data["header"])-source["start"],m=data))
    (out/"counterfactual.json").write_text(json.dumps(rows))


def render(a):
    import cv2
    import numpy as np
    cv2.setNumThreads(2)
    out=Path(a.out);d=json.loads((out/"data.json").read_text())
    rows=json.loads((out/"counterfactual.json").read_text())
    frames=Timeline(d["frames"])
    original={}
    for row in d["rows"]["raw"]:original.setdefault(row["m"]["source"],[]).append(row)
    original={k:Frames(v) for k,v in original.items()}
    raw=Frames(rows.get("/uav_vision/detections",[]))
    mapped=Frames(rows.get("/uav_vision/detections_mapped",[]))
    baseline=Frames(rows.get("/replay/baseline",[]))
    memory=Timeline(rows.get("/uav_vision/targets",[]))
    offsets=Timeline(rows.get("/uav_vision/drop_offset",[]))
    modes=Timeline(d["rows"]["mode"]);fc=Timeline(d["rows"]["fc"])
    ready=Timeline(rows.get("/uav_vision/drop_ready",[]))
    final=out/"h_landing_comparison.mp4"
    command=["ffmpeg","-v","error","-y","-f","rawvideo","-pix_fmt","bgr24","-s","1920x1080",
        "-r",str(a.fps),"-i","-","-an","-c:v","libx264","-preset","fast","-crf","21",
        "-pix_fmt","yuv420p","-movflags","+faststart",str(final)]
    encoder=subprocess.Popen(command,stdin=subprocess.PIPE)
    def text(im,s,x,y,color=(235,235,235),scale=.8):
        cv2.putText(im,str(s),(x,y),cv2.FONT_HERSHEY_SIMPLEX,scale,color,2,cv2.LINE_AA)
    last=None;image=None
    try:
        for i in range(math.ceil(d["duration"]*a.fps)):
            t=i/a.fps;frame=frames.row(t)
            panel=np.full((1080,1920,3),22,np.uint8)
            text(panel,"RECORDED FLIGHT: original detections",25,42)
            text(panel,"OFFLINE REPLAY: updated H vision chain",985,42,(80,225,255))
            if frame:
                if frame["file"]!=last:
                    image=cv2.imread(str(out/frame["file"]));last=frame["file"]
                if image is None:raise RuntimeError("missing exported image")
                h,w=image.shape[:2];sx=960/w;sy=540/h
                for x in (0,960):panel[65:605,x:x+960]=cv2.resize(image,(960,540))
                def draw(row,origin,color):
                    for det in (row or {}).get("m",{}).get("detections",[]):
                        roi=det["roi"];x=int(roi["x_offset"]*sx)+origin;y=int(roi["y_offset"]*sy)+65
                        cv2.rectangle(panel,(x,y),(x+int(roi["width"]*sx),y+int(roi["height"]*sy)),color,2)
                        center=det["center_px"];cx=int(center["x"]*sx)+origin;cy=int(center["y"]*sy)+65
                        cv2.drawMarker(panel,(cx,cy),color,cv2.MARKER_CROSS,18,2)
                        text(panel,"%s %.2f"%(det["class_name"],det["geometry_confidence"]),x,max(82,y-5),color,.6)
                for src,line in original.items():draw(line.match(frame["stamp"]),0,(70,210,70))
                revised=raw.match(frame["stamp"]);draw(revised,960,(80,225,255))
                old=baseline.match(frame["stamp"])
                text(panel,"Same-frame old H algorithm: %d"%len((old or {}).get("m",{}).get("detections",[])),25,650)
                text(panel,"New H geometry: %d"%len((revised or {}).get("m",{}).get("detections",[])),985,650)
                maps=(mapped.match(frame["stamp"]) or {}).get("m",{}).get("detections",[])
                for j,det in enumerate(maps[:2]):
                    mp=det["map_point"]
                    label="map_valid=%s %s (%.3f, %.3f, %.3f)"%(det["map_valid"],det["map_frame"],mp["x"],mp["y"],mp["z"])
                    text(panel,label,985,695+j*32,scale=.65)
            mode=modes.msg(t);mode=mode.get("data","?") if mode else "?"
            state=fc.msg(t) or {}
            text(panel,"t=%.2f / %.2fs | original mode=%s armed=%s"%(t,d["duration"],state.get("mode","?"),state.get("armed","?")),25,730,(235,235,235),.75)
            text(panel,"Recorded vision mode: "+mode,25,775)
            targets=(memory.msg(t) or {}).get("targets",[])
            labels=["id=%s %s %s"%(q["id"],q["class_name"],{0:"DETECTED",1:"OBSERVING",2:"CONFIRMED",3:"REJECTED",4:"EXPIRED"}.get(q["state"],"?")) for q in targets]
            text(panel,"Replay memory: "+(", ".join(labels[:2]) or "none"),985,775,scale=.7)
            offset=offsets.msg(t,.5);r=ready.msg(t,.5) or {}
            text(panel,"Replay offset: "+("dx=%.1f dy=%.1f px"%(offset["dx_px"],offset["dy_px"]) if offset else "no fresh result"),985,820,scale=.75)
            text(panel,"Replay aligned/ready: "+str(r.get("ready","?")),985,865,scale=.75)
            text(panel,"Recorded flight/TF only. Right panel is counterfactual perception, NOT a new landing.",25,945,(100,200,255),.85)
            text(panel,"H raw geometry shown for diagnosis; fusion/memory follow the recorded landing stage.",25,990,(190,190,190),.8)
            text(panel,"0.25s recorded TF look-ahead for interpolation; online latency is not measured here.",25,1033,(190,190,190),.8)
            encoder.stdin.write(panel.tobytes())
    finally:
        encoder.stdin.close()
        return_code = encoder.wait()
    if return_code!=0:raise RuntimeError("video encoding failed")
    subprocess.run(["ffmpeg","-v","error","-i",str(final),"-f","null","-"],check=True)
    meta=json.loads(subprocess.check_output(["ffprobe","-v","error","-show_format","-show_streams","-of","json",str(final)]))
    (out/"h_video_validation.json").write_text(json.dumps(meta,indent=2))
    (out/"h_index.html").write_text('<meta charset="utf-8"><title>H offline comparison</title><h2>Recorded / offline H comparison</h2><video controls style="width:100%" src="h_landing_comparison.mp4"></video><p>Offline perception only; original flight was not repeated.</p>')
    print(final,flush=True)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("mode",choices=["prepare","render"])
    p.add_argument("--bag");p.add_argument("--reinferred");p.add_argument("--out",required=True);p.add_argument("--fps",type=int,default=10)
    a=p.parse_args();(prepare if a.mode=="prepare" else render)(a)

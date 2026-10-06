#!/usr/bin/env python3
"""Summarize an offline comparison and draw largest centre changes for review."""
import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    report=json.loads((args.output/"summary.json").read_text())
    for group,value in report["groups"].items():
        print(group)
        for name,variant in value["variants"].items():
            print(name,json.dumps({key:variant[key] for key in (
                "consecutive_single_target_center_step_px","synthetic_truth_compensated_jitter_px",
                "ellipse_normal_residual_detector_px","visible_arc_fraction")}))
    print("perspective",json.dumps(report["perspective_center_demonstration"]))
    frames=json.loads((args.output/"frames.json").read_text())
    with (args.output/"geometry.tsv").open() as stream:
        observations=list(csv.DictReader(stream,delimiter="\t"))
    by_frame={}
    for row in observations:
        if row["detector"]!="circle" or int(row["index"])<0:
            continue
        by_frame.setdefault(int(row["frame"]),{}).setdefault(row["variant"],[]).append(row)
    changes=[]
    for i,variants in by_frame.items():
        if not frames[i]["group"].startswith("recorded_panzer"):
            continue
        # One-target images permit an unambiguous visual old/new overlay.
        if len(variants.get("old",[]))!=1 or len(variants.get("new",[]))!=1:
            continue
        a,b=variants["old"][0],variants["new"][0]
        delta=math.hypot(float(a["x"])-float(b["x"]),float(a["y"])-float(b["y"]))
        changes.append((delta,i,a,b))
    changes.sort(reverse=True)
    sheet=np.full((3*270,3*360,3),240,np.uint8)
    selections=[]
    for index,(delta,i,a,b) in enumerate(changes[:9]):
        image=cv2.imread(frames[i]["file"])
        x,y=float(b["x"]),float(b["y"])
        radius=max(float(a["radius"]),float(b["radius"]))
        half=max(70,int(radius*1.45))
        lo_x=max(0,int(x)-half);lo_y=max(0,int(y)-half)
        hi_x=min(image.shape[1],int(x)+half);hi_y=min(image.shape[0],int(y)+half)
        for row,color in ((a,(0,0,255)),(b,(0,180,0))):
            point=(round(float(row["x"])),round(float(row["y"])))
            cv2.drawMarker(image,point,color,cv2.MARKER_CROSS,20,2)
            cv2.circle(image,point,round(float(row["radius"])),color,2)
        patch=image[lo_y:hi_y,lo_x:hi_x]
        scale=min(360/patch.shape[1],230/patch.shape[0])
        patch=cv2.resize(patch,(round(patch.shape[1]*scale),round(patch.shape[0]*scale)))
        tile=np.full((270,360,3),240,np.uint8)
        tile[:patch.shape[0],:patch.shape[1]]=patch
        cv2.putText(tile,f"frame {i}: delta {delta:.2f} px",(5,247),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,0,0),1)
        cv2.putText(tile,f"old red q={float(a['quality']):.3f} / new green q={float(b['quality']):.3f}",
                    (5,264),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,0,0),1)
        sheet[index//3*270:(index//3+1)*270,index%3*360:(index%3+1)*360]=tile
        selections.append(dict(frame=i,file=frames[i]["file"],delta_px=delta,old=a,new=b))
    cv2.imwrite(str(args.output/"largest_changes.jpg"),sheet)
    (args.output/"largest_changes.json").write_text(json.dumps(selections,indent=2))


if __name__=="__main__":
    main()
